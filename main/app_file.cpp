#include "app_file.h"
#include "app_gpio.h"
#include "app_mic.h"
#include "app_mpu5060.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "portmacro.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <errno.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#define MOUNT_POINT "/spiffs"
#define MAX_CHAR_SIZE 64
#define MAX_FILE_SIZE 1024 * 1024 * 2  // 减小文件大小限制，适应SPIFFS空间
#define FLUSH_INTERVAL_MS 100
#define DATA_INTERVAL_MS 30 * 1000
#define DATA_INTERVAL_TIMES DATA_INTERVAL_MS / FLUSH_INTERVAL_MS

TaskHandle_t imu_get_handler = nullptr;
TaskHandle_t imu_run_handler = nullptr;
TaskHandle_t mic_run_handler = nullptr;
static uint32_t last_flush_time = 0;

// 静态成员变量定义
bool FileStorage::s_initialized = false;
SemaphoreHandle_t FileStorage::s_mutex = nullptr;

// IMU_Data 静态成员定义
TaskHandle_t IMU_Data::s_collection_task = nullptr;
TaskHandle_t IMU_Data::s_write_task = nullptr;
QueueHandle_t IMU_Data::s_write_queue = nullptr;

/**
 * @brief 构造函数，对变量进行初始化
 */
FileStorage::FileStorage(const char *tag) : mount_point(MOUNT_POINT) {
  TAG = tag;
  isReady = false;
  for (int i = 0; i < MAX_FILES; i++) {
    files.file[i] = nullptr;
    files.file_numbers[i] = 0;
  }
  files.index = 0;
  file_number = 0;
}

/**
 * @brief 析构函数，释放资源
 */
FileStorage::~FileStorage() {
  for (int i = 0; i < MAX_FILES; i++) {
    if (files.file[i] != nullptr) {
      fclose(files.file[i]);
    }
    files.file[i] = nullptr;
  }
  isReady = false;
  files.index = 0;
}

/**
 * @brief 只初始化一次SPIFFS
 */
void FileStorage::app_file_init_once() {
  if (s_initialized) {
    return;
  }

  // 初始化互斥锁
  s_mutex = xSemaphoreCreateMutex();
  if (s_mutex == nullptr) {
    ESP_LOGE("FileStorage", "Failed to create mutex");
    return;
  }

  // 初始化SPIFFS
  ESP_LOGI("FileStorage", "Initializing SPIFFS");
  
  esp_vfs_spiffs_conf_t conf = {
    .base_path = MOUNT_POINT,
    .partition_label = NULL,
    .max_files = MAX_FILES,
    .format_if_mount_failed = true
  };

  esp_err_t ret = esp_vfs_spiffs_register(&conf);
  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE("FileStorage", "Failed to mount or format SPIFFS");
    } else if (ret == ESP_ERR_NOT_FOUND) {
      ESP_LOGE("FileStorage", "SPIFFS partition not found");
    } else {
      ESP_LOGE("FileStorage", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
    }
    return;
  }

  size_t total = 0, used = 0;
  ret = esp_spiffs_info(NULL, &total, &used);
  if (ret != ESP_OK) {
    ESP_LOGE("FileStorage", "Failed to get SPIFFS info (%s)", esp_err_to_name(ret));
  } else {
    ESP_LOGI("FileStorage", "SPIFFS mounted successfully");
    ESP_LOGI("FileStorage", "Total: %d bytes, Used: %d bytes", total, used);
  }

  s_initialized = true;
}

/**
 * @brief 组件初始化
 */
void FileStorage::app_file_init() {
  app_file_init_once();
  isReady = true;
}

/**
 * @brief 通过文件名检查文件是否已存在，并返回文件大小
 */
bool FileStorage::app_file_check_file(const char *filename, size_t *file_size) {
  struct stat st;
  bool result = false;
  if (stat(filename, &st) == 0) {
    if (file_size) {
      *file_size = st.st_size;
    }
    result = true;
  }
  return result;
}

/**
 * @brief 确认可以写入的文件编号
 */
void FileStorage::app_file_check_file_number(const char *prefix) {
  char filename[64];
  snprintf(filename, sizeof(filename), "%s/%s_%d.csv", mount_point, prefix,
           file_number);
  size_t file_size = 0;
  while (app_file_check_file(filename, &file_size)) {
    printf("filename: %s,filesize:%d\n", filename, file_size);
    if (file_size >= MAX_FILE_SIZE) {
      file_number++;
      snprintf(filename, sizeof(filename), "%s/%s_%d.csv", mount_point, prefix,
               file_number);
    } else
      break;
  }
}

/**
 * @brief 新建和打开文件
 */
esp_err_t FileStorage::app_file_open_file(const char *prefix) {
  if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return ESP_FAIL;
  }

  char filename[64];
  snprintf(filename, sizeof(filename), "%s/%s_%d.csv", mount_point, prefix,
           file_number);

  files.file[files.index] = fopen(filename, "a");
  if (files.file[files.index] == NULL) {
    ESP_LOGE(TAG, "Failed to open file %s for writing", filename);
    xSemaphoreGive(s_mutex);
    return ESP_FAIL;
  }

  app_file_check_file(filename, &files.file_size[files.index]);
  files.file_numbers[files.index] = file_number;

  ESP_LOGI(TAG, "File %s opened successfully", filename);

  files.index = (files.index + 1) % MAX_FILES;
  file_number++;

  xSemaphoreGive(s_mutex);
  return ESP_OK;
}

/**
 * @brief 通用文件写入函数
 */
esp_err_t FileStorage::app_file_write_file(int index, void *data) {
  if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return ESP_FAIL;
  }

  if (files.file[index] == NULL) {
    ESP_LOGE(TAG, "File handle is invalid for index %d", index);
    xSemaphoreGive(s_mutex);
    return ESP_FAIL;
  }

  if (data == NULL) {
    ESP_LOGE(TAG, "Data pointer is null");
    xSemaphoreGive(s_mutex);
    return ESP_FAIL;
  }

  ESP_LOGW(TAG, "Base class write function called, should be overridden");

  xSemaphoreGive(s_mutex);
  return ESP_FAIL;
}

/**
 * @brief 保存文件
 */
esp_err_t FileStorage::app_file_save_file(int index) {
  if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return ESP_FAIL;
  }

  if (files.file[index] == NULL) {
    ESP_LOGE(TAG, "File handle is invalid for index %d", index);
    xSemaphoreGive(s_mutex);
    return ESP_FAIL;
  }

  fflush(files.file[index]);
  fclose(files.file[index]);
  files.file[index] = nullptr;

  ESP_LOGI(TAG,"File index %d saved and closed", index);

  xSemaphoreGive(s_mutex);
  return ESP_OK;
}

/**
 * @brief 检查组件状态
 */
bool FileStorage::app_file_check_module() {
  if (!isReady)
    ESP_LOGD(TAG, "storage is not ready");
  return isReady;
}

/**
 * @brief 启动模组
 */
void FileStorage::app_file_start() { isReady = true; }

/**
 * @brief 停止模组
 */
void FileStorage::app_file_stop() { isReady = false; }

/****************************** IMU_Data **************************************/

IMU_Data::IMU_Data(const char *tag) : FileStorage(tag) {
  imu_filename_prefix = "imu";
  last_write_time = 0;

  current_buffer = nullptr;
  current_buffer_id = 0;
  last_switch_time = 0;
  for (int i = 0; i < BUFFER_COUNT; i++) {
    buffers[i].count = 0;
    buffers[i].buffer_id = i;
  }
}

IMU_Data::~IMU_Data() {
  last_write_time = 0;
}

/**
 * @brief 初始化IMU数据缓冲区
 */
void IMU_Data::imu_data_init(size_t size) {
  app_file_init();

  for (int i = 0; i < BUFFER_COUNT; i++) {
    buffers[i].count = 0;
    buffers[i].buffer_id = i;
  }
  current_buffer = &buffers[0];
  current_buffer_id = 0;
  last_switch_time = esp_timer_get_time() / 1000;

  if (s_write_queue == nullptr) {
    s_write_queue = xQueueCreate(BUFFER_COUNT, sizeof(imu_buffer_t *));
    if (s_write_queue == nullptr) {
      ESP_LOGE(TAG, "Failed to create write queue");
      return;
    }
    ESP_LOGI(TAG, "Write queue created with depth %d", BUFFER_COUNT);
  }

  app_file_check_file_number(imu_filename_prefix);
  app_file_open_file(imu_filename_prefix);

  isReady = true;
  ESP_LOGI(TAG, "IMU data initialized with double buffering (%d buffers x %d samples)",
           BUFFER_COUNT, SAMPLES_PER_BUFFER);
}

/**
 * @brief 切换缓冲区
 */
imu_buffer_t *IMU_Data::switch_buffer() {
  current_buffer_id = (current_buffer_id + 1) % BUFFER_COUNT;
  return &buffers[current_buffer_id];
}

/**
 * @brief 批量写入缓冲区到存储
 */
void IMU_Data::write_buffer_to_storage(imu_buffer_t *buffer) {
  if (!buffer || buffer->count == 0) {
    ESP_LOGW(TAG, "Empty buffer received");
    return;
  }

  int current_index = (files.index - 1 + MAX_FILES) % MAX_FILES;

  if (files.file[current_index] == nullptr ||
      files.file_size[current_index] >= MAX_FILE_SIZE) {

    if (files.file[current_index] != nullptr) {
      ESP_LOGI(TAG, "File size limit reached, saving current file");
      app_file_save_file(current_index);
    }

    esp_err_t ret = app_file_open_file(imu_filename_prefix);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create new file");
      return;
    }
    current_index = (files.index - 1 + MAX_FILES) % MAX_FILES;
  }

  if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex for writing");
    return;
  }

  uint32_t write_start = esp_timer_get_time() / 1000;
  size_t bytes_written = 0;

  for (size_t i = 0; i < buffer->count; i++) {
    mpu6050_processed_data_t *data = &buffer->samples[i];

    int len = fprintf(
        files.file[current_index],
        "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.4f,%.4f,%s,%s\n",
        data->timestamp, data->filter_acce_value.acce_x,
        data->filter_acce_value.acce_y, data->filter_acce_value.acce_z,
        data->filter_gyro_value.gyro_x, data->filter_gyro_value.gyro_y,
        data->filter_gyro_value.gyro_z, data->angles.roll, data->angles.pitch,
        data->angles.yaw, data->combined_accel, data->combined_gyro,
        data->fall_pre_detected ? "1" : "0", data->special_marker ? "1" : "0");

    if (len > 0) {
      bytes_written += len;
      files.file_size[current_index] += len;
    }
  }

  fflush(files.file[current_index]);
  xSemaphoreGive(s_mutex);

  buffer->count = 0;
}

/**
 * @brief IMU数据采集，创建文件
 */
esp_err_t IMU_Data::app_file_open_file(const char *prefix) {
  if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return ESP_FAIL;
  }

  char filename[64];
  snprintf(filename, sizeof(filename), "%s/%s_%d.csv", mount_point, prefix,
           file_number);

  files.file[files.index] = fopen(filename, "a");
  if (files.file[files.index] == NULL) {
    ESP_LOGE(TAG, "Failed to open file %s for writing, error: %s (errno: %d)",
             filename, strerror(errno), errno);
    xSemaphoreGive(s_mutex);
    return ESP_FAIL;
  }

  app_file_check_file(filename, &files.file_size[files.index]);
  files.file_numbers[files.index] = file_number;

  fprintf(files.file[files.index],
          "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll,pitch,"
          "yaw,combined_accel,combined_gyro,pre_judge,btn_sign\n");

  ESP_LOGI(TAG, "New file created: %s", filename);

  files.index = (files.index + 1) % MAX_FILES;
  file_number++;

  xSemaphoreGive(s_mutex);
  return ESP_OK;
}

/**
 * @brief 数据采集任务主循环
 */
void IMU_Data::collection_task_loop() {
  mpu6050_processed_data_t data;
  size_t dropped_samples = 0;
  size_t total_received = 0;
  bool was_in_imu_mode = false;

  ESP_LOGI(TAG, "Collection task started (priority 5)");
  last_switch_time = esp_timer_get_time() / 1000;

  while (1) {
    if (mode_switch_event_group != nullptr) {
      EventBits_t bits = xEventGroupGetBits(mode_switch_event_group);
      if (bits & MODE_SWITCH_REQUEST_BIT) {
        ESP_LOGI(TAG, "Mode switch request detected, saving IMU buffer");
        
        if (current_buffer->count > 0) {
          imu_buffer_t *full_buffer = current_buffer;
          ESP_LOGI(TAG, "Saving buffer with %d samples", full_buffer->count);
          
          if (xQueueSend(s_write_queue, &full_buffer, portMAX_DELAY) == pdTRUE) {
            current_buffer = switch_buffer();
            current_buffer->count = 0;
            current_buffer->buffer_id = full_buffer->buffer_id + BUFFER_COUNT;
            ESP_LOGI(TAG, "IMU buffer saved successfully");
          } else {
            ESP_LOGW(TAG, "Failed to save IMU buffer, dropping %d samples", full_buffer->count);
            current_buffer->count = 0;
            dropped_samples += full_buffer->count;
          }
        }
        
        xEventGroupSetBits(mode_switch_event_group, MODE_SWITCH_IMU_READY_BIT);
        ESP_LOGI(TAG, "IMU ready bit set");
        
        while (xEventGroupGetBits(mode_switch_event_group) & MODE_SWITCH_REQUEST_BIT) {
          vTaskDelay(pdMS_TO_TICKS(10));
        }
        was_in_imu_mode = false;
        continue;
      }
    }

    if (!get_current_data_mode()) {
      if (was_in_imu_mode) {
        was_in_imu_mode = false;
        ESP_LOGI(TAG, "IMU mode disabled, stopped collection");
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    } else {
      if (!was_in_imu_mode) {
        was_in_imu_mode = true;
        ESP_LOGI(TAG, "IMU mode enabled, started collection");
      }
    }
    
    if (xQueueReceive(data_queue, &data, pdMS_TO_TICKS(10)) == pdTRUE) {
      total_received++;

      if (current_buffer->count < SAMPLES_PER_BUFFER) {
        current_buffer->samples[current_buffer->count++] = data;
      } else {
        dropped_samples++;
        ESP_LOGW(TAG, "Buffer overflow! Dropped %d samples so far",
                 dropped_samples);
      }

      uint32_t now = esp_timer_get_time() / 1000;
      bool buffer_full = (current_buffer->count >= SAMPLES_PER_BUFFER);
      bool timeout = (now - last_switch_time > BUFFER_TIMEOUT_MS);

      if (buffer_full || timeout) {
        if (current_buffer->count > 0) {
          imu_buffer_t *full_buffer = current_buffer;

          if (xQueueSend(s_write_queue, &full_buffer, 0) == pdTRUE) {
            current_buffer = switch_buffer();
            current_buffer->count = 0;
            current_buffer->buffer_id = full_buffer->buffer_id + BUFFER_COUNT;
            last_switch_time = now;

            ESP_LOGD(TAG, "Buffer switched: sent buffer %d with %d samples, total received: %d",
                     full_buffer->buffer_id, full_buffer->count, total_received);
          } else {
            ESP_LOGW(TAG, "Write queue full! Dropping buffer with %d samples",
                     full_buffer->count);
            current_buffer->count = 0;
            dropped_samples += full_buffer->count;
          }
        }
      }
    }
  }
}

/**
 * @brief 文件写入任务主循环
 */
void IMU_Data::write_task_loop() {
  imu_buffer_t *buffer;
  size_t buffers_written = 0;

  ESP_LOGI(TAG, "Write task started (priority 2)");

  while (1) {
    if (xQueueReceive(s_write_queue, &buffer, portMAX_DELAY) == pdTRUE) {
      buffers_written++;
      ESP_LOGD(TAG, "Received buffer %d for writing (%d samples, buffer#%d)",
               buffer->buffer_id, buffer->count, buffers_written);
      write_buffer_to_storage(buffer);
    }
  }
}

/**
 * @brief 静态任务入口 - 数据采集任务
 */
void IMU_Data::collection_task_entry(void *arg) {
  IMU_Data *instance = static_cast<IMU_Data *>(arg);
  instance->collection_task_loop();
}

/**
 * @brief 静态任务入口 - 文件写入任务
 */
void IMU_Data::write_task_entry(void *arg) {
  IMU_Data *instance = static_cast<IMU_Data *>(arg);
  instance->write_task_loop();
}

/**
 * @brief 创建IMU数据处理任务
 */
void IMU_Data::app_file_create_task() {
  if (s_collection_task == nullptr) {
    xTaskCreate(collection_task_entry, "imu_collect", 4096, this, 5,
                &s_collection_task);
    ESP_LOGI(TAG, "IMU collection task created successfully (priority 5)");
  } else {
    ESP_LOGW(TAG, "IMU collection task already exists");
  }

  if (s_write_task == nullptr) {
    xTaskCreate(write_task_entry, "imu_write", 4096, this, 2, &s_write_task);
    ESP_LOGI(TAG, "IMU write task created successfully (priority 2)");
  } else {
    ESP_LOGW(TAG, "IMU write task already exists");
  }
}

/***************************** MIC_Data **************************************/

MIC_Data::MIC_Data(const char *tag) : FileStorage(tag) {
  sample_rate = MIC_SAMPLE_RATE;
  bit_depth = 16;
  mic_filename_prefix = "mic";
  
  total_pcm_size = 0;
  wav_header_written = false;
  wav_header_position = 0;
  last_flush_time = 0;
  memset(wav_header, 0, WAV_HEADER_SIZE);
}

MIC_Data::~MIC_Data() {
}

/**
 * @brief 初始化麦克风数据存储
 */
void MIC_Data::mic_data_init(size_t size) {
  app_file_init();
  
  if (wav_queue == nullptr) {
    ESP_LOGE(TAG, "WAV queue not created, please initialize MEMS_MIC first");
    return;
  }
  
  app_file_check_file_number(mic_filename_prefix);
  
  esp_err_t ret = app_file_open_file(mic_filename_prefix);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create initial WAV file");
    return;
  }
  
  total_pcm_size = 0;
  wav_header_written = false;
  wav_header_position = 0;
  last_flush_time = 0;
  memset(wav_header, 0, WAV_HEADER_SIZE);
  
  isReady = true;
  ESP_LOGI(TAG, "MIC data initialized, ready for WAV streaming");
}

/**
 * @brief 麦克风创建文件函数
 */
esp_err_t MIC_Data::app_file_open_file(const char *prefix) {
  if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return ESP_FAIL;
  }
  
  char filename[64];
  snprintf(filename, sizeof(filename), "%s/%s_%d.wav", mount_point, prefix,
           file_number);

  int new_file_idx = files.index;
  files.file[new_file_idx] = fopen(filename, "w");
  if (files.file[new_file_idx] == NULL) {
    ESP_LOGE(TAG, "Failed to open file %s for writing", filename);
    xSemaphoreGive(s_mutex);
    return ESP_FAIL;
  }

  files.file_numbers[new_file_idx] = file_number;
  wav_header_position = ftell(files.file[new_file_idx]);
  
  uint8_t empty_header[WAV_HEADER_SIZE];
  memset(empty_header, 0, WAV_HEADER_SIZE);
  fwrite(empty_header, 1, WAV_HEADER_SIZE, files.file[new_file_idx]);
  
  files.file_size[new_file_idx] = WAV_HEADER_SIZE;
  wav_header_written = false;

  ESP_LOGI(TAG, "WAV file %s opened successfully (index=%d, size=%d)", filename, new_file_idx, files.file_size[new_file_idx]);

  files.index = (files.index + 1) % MAX_FILES;
  file_number++;

  xSemaphoreGive(s_mutex);
  return ESP_OK;
}

/**
 * @brief 确认文件编号
 */
void MIC_Data::app_file_check_file_number(const char *prefix) {
  char filename[64];
  snprintf(filename, sizeof(filename), "%s/%s_%d.wav", mount_point, prefix,
           file_number);
  size_t file_size = 0;
  while (app_file_check_file(filename, &file_size)) {
    printf("filename: %s,filesize:%d\n", filename, file_size);
    file_number++;
    snprintf(filename, sizeof(filename), "%s/%s_%d.wav", mount_point, prefix,
             file_number);
  }
}

/**
 * @brief 更新WAV头中的数据大小
 */
void MIC_Data::update_wav_header(int index) {
  if (files.file[index] == NULL) {
    ESP_LOGE(TAG, "File handle is invalid for index %d", index);
    return;
  }
  
  if (total_pcm_size == 0) {
    ESP_LOGW(TAG, "No PCM data written, skipping WAV header update");
    return;
  }
  
  uint32_t chunk_size = 36 + total_pcm_size;
  uint32_t byte_rate = sample_rate * 1 * (bit_depth / 8);
  uint16_t block_align = 1 * (bit_depth / 8);
  
  uint8_t header[WAV_HEADER_SIZE];
  
  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  header[4] = (uint8_t)(chunk_size & 0xFF);
  header[5] = (uint8_t)((chunk_size >> 8) & 0xFF);
  header[6] = (uint8_t)((chunk_size >> 16) & 0xFF);
  header[7] = (uint8_t)((chunk_size >> 24) & 0xFF);
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;
  header[20] = 1; header[21] = 0;
  header[22] = 1; header[23] = 0;
  header[24] = (uint8_t)(sample_rate & 0xFF);
  header[25] = (uint8_t)((sample_rate >> 8) & 0xFF);
  header[26] = (uint8_t)((sample_rate >> 16) & 0xFF);
  header[27] = (uint8_t)((sample_rate >> 24) & 0xFF);
  header[28] = (uint8_t)(byte_rate & 0xFF);
  header[29] = (uint8_t)((byte_rate >> 8) & 0xFF);
  header[30] = (uint8_t)((byte_rate >> 16) & 0xFF);
  header[31] = (uint8_t)((byte_rate >> 24) & 0xFF);
  header[32] = (uint8_t)(block_align & 0xFF);
  header[33] = (uint8_t)((block_align >> 8) & 0xFF);
  header[34] = (uint8_t)(bit_depth & 0xFF);
  header[35] = (uint8_t)((bit_depth >> 8) & 0xFF);
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  header[40] = (uint8_t)(total_pcm_size & 0xFF);
  header[41] = (uint8_t)((total_pcm_size >> 8) & 0xFF);
  header[42] = (uint8_t)((total_pcm_size >> 16) & 0xFF);
  header[43] = (uint8_t)((total_pcm_size >> 24) & 0xFF);
  
  long current_pos = ftell(files.file[index]);
  fseek(files.file[index], 0, SEEK_SET);
  fwrite(header, 1, WAV_HEADER_SIZE, files.file[index]);
  fseek(files.file[index], current_pos, SEEK_SET);
  
  ESP_LOGI(TAG, "WAV header updated: chunk_size=%u, data_size=%u, file_index=%d", 
           chunk_size, total_pcm_size, index);
}

/**
 * @brief 获取当前使用的文件索引
 */
int MIC_Data::get_current_file_index() {
  return (files.index - 1 + MAX_FILES) % MAX_FILES;
}

/**
 * @brief 写入WAV数据包到文件
 */
void MIC_Data::write_wav_packet(mic_wav_packet_t* packet) {
  if (packet == nullptr || packet->data == nullptr) {
    ESP_LOGW(TAG, "Invalid WAV packet");
    return;
  }
  
  if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex for writing");
    free(packet->data);
    return;
  }
  
  int current_index = get_current_file_index();
  
  if (files.file[current_index] == nullptr ||
      files.file_size[current_index] >= MAX_FILE_SIZE) {
    
    if (files.file[current_index] != nullptr) {
      update_wav_header(current_index);
      fflush(files.file[current_index]);
      fclose(files.file[current_index]);
      files.file[current_index] = nullptr;
    }
    
    xSemaphoreGive(s_mutex);
    
    esp_err_t ret = app_file_open_file(mic_filename_prefix);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create new WAV file");
      free(packet->data);
      return;
    }
    
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
      ESP_LOGE(TAG, "Failed to take mutex after creating new file");
      free(packet->data);
      return;
    }
    
    current_index = get_current_file_index();
    total_pcm_size = 0;
    wav_header_written = false;
    last_flush_time = 0;
  }
  
  if (files.file[current_index] == NULL) {
    ESP_LOGE(TAG, "File handle is invalid");
    xSemaphoreGive(s_mutex);
    free(packet->data);
    return;
  }
  
  size_t written = 0;
  
  if (packet->is_wav_header_included) {
    written = fwrite(packet->data + WAV_HEADER_SIZE, 1, 
                     packet->pcm_data_size, files.file[current_index]);
    total_pcm_size += packet->pcm_data_size;
    wav_header_written = true;
  } else {
    written = fwrite(packet->data, 1, packet->pcm_data_size, 
                     files.file[current_index]);
    total_pcm_size += packet->pcm_data_size;
  }
  
  files.file_size[current_index] += written;
  
  uint32_t now = esp_timer_get_time() / 1000;
  bool flushed = false;
  if (now - last_flush_time > 1000) {
    fflush(files.file[current_index]);
    last_flush_time = now;
    flushed = true;
  }
  
  xSemaphoreGive(s_mutex);
  free(packet->data);
  packet->data = nullptr;
}

/**
 * @brief 保存文件，更新WAV头
 */
esp_err_t MIC_Data::app_file_save_file(int index) {
  if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return ESP_FAIL;
  }

  if (files.file[index] == NULL) {
    ESP_LOGE(TAG, "File handle is invalid for index %d", index);
    xSemaphoreGive(s_mutex);
    return ESP_FAIL;
  }
  
  update_wav_header(index);
  fflush(files.file[index]);
  fclose(files.file[index]);
  files.file[index] = nullptr;

  ESP_LOGI(TAG, "WAV file index %d saved and closed, total PCM size: %d bytes", 
           index, total_pcm_size);

  xSemaphoreGive(s_mutex);
  return ESP_OK;
}

/**
 * @brief 麦克风写入任务主循环
 */
void MIC_Data::mic_write_task_loop() {
  mic_wav_packet_t packet;
  size_t packets_received = 0;
  
  ESP_LOGI(TAG, "MIC write task started");
  
  while (1) {
    if (xQueueReceive(wav_queue, &packet, portMAX_DELAY) == pdTRUE) {
      packets_received++;
      ESP_LOGD(TAG, "Received WAV packet: %d bytes (header: %s)",
               packet.total_size, packet.is_wav_header_included ? "yes" : "no");
      write_wav_packet(&packet);
    }
  }
}

/**
 * @brief 静态任务入口
 */
void MIC_Data::mic_write_task_entry(void* arg) {
  MIC_Data* instance = static_cast<MIC_Data*>(arg);
  instance->mic_write_task_loop();
}

/**
 * @brief 创建麦克风写入任务
 */
void MIC_Data::app_file_create_task() {
  if (mic_run_handler == nullptr) {
    xTaskCreate(mic_write_task_entry, "mic_write", 4096, this, 2, &mic_run_handler);
    ESP_LOGI(TAG, "MIC write task created (priority 2)");
  } else {
    ESP_LOGW(TAG, "MIC write task already exists");
  }
}