#include "app_mic.h"
#include "app_file.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <errno.h>
#include <inttypes.h>

// 全局队列定义
QueueHandle_t wav_queue = nullptr;

// 外部引用
extern MIC_Data mic_data;

MEMS_MIC::MEMS_MIC(const char* tag) : TAG(tag), isReady(false), mic_handler(nullptr), 
                                      adc_channel_num(MIC_ADC_CHANNEL), adc_unit_num(MIC_ADC_UNIT), 
                                      buffer_pos(0) {
    memset(sample_buffer, 0, sizeof(sample_buffer));
}

MEMS_MIC::~MEMS_MIC() {
    if (wav_queue != nullptr) {
        mic_wav_packet_t packet;
        while (xQueueReceive(wav_queue, &packet, 0) == pdTRUE) {
            if (packet.data != nullptr) {
                free(packet.data);
            }
        }
    }
}

esp_err_t MEMS_MIC::app_mic_init() {
    // 创建WAV队列
    if (wav_queue == nullptr) {
        wav_queue = xQueueCreate(WAV_QUEUE_SIZE, sizeof(mic_wav_packet_t));
        if (wav_queue == nullptr) {
            ESP_LOGE(TAG, "Failed to create WAV queue");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "WAV queue created successfully (depth=%d)", WAV_QUEUE_SIZE);
    }

    // 配置ADC
    if (adc_unit_num == ADC_UNIT_1) {
        ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
        ESP_ERROR_CHECK(adc1_config_channel_atten((adc1_channel_t)adc_channel_num, MIC_ADC_ATTEN));
    } else {
        ESP_ERROR_CHECK(adc2_config_channel_atten((adc2_channel_t)adc_channel_num, MIC_ADC_ATTEN));
    }

    ESP_LOGI(TAG, "Analog MIC initialized (ZTS6216)");
    ESP_LOGI(TAG, "ADC Unit: %d, Channel: %d (IO15)", adc_unit_num, adc_channel_num);
    ESP_LOGI(TAG, "Sample Rate: %d Hz", MIC_SAMPLE_RATE);
    ESP_LOGI(TAG, "Attenuation: 11dB (max ~3.3V)");

    isReady = true;
    return ESP_OK;
}

void MEMS_MIC::app_mic_start() {
    if (!isReady) {
        isReady = true;
        ESP_LOGI(TAG, "MIC ready for sampling");
    }
}

void MEMS_MIC::app_mic_stop() {
    if (isReady) {
        isReady = false;
        ESP_LOGI(TAG, "MIC stopped sampling");
    }
}

bool MEMS_MIC::app_mic_check_module() {
    if (!isReady) {
        ESP_LOGD(TAG, "mic is not ready");
    }
    return isReady;
}

/**
 * @brief 创建WAV数据包并发送到队列
 */
static bool send_wav_packet(QueueHandle_t queue, int16_t* pcm_data, size_t pcm_size,
                            bool include_header, uint32_t timestamp) {
    mic_wav_packet_t packet;

    size_t header_size = include_header ? WAV_HEADER_SIZE : 0;
    packet.total_size = header_size + pcm_size;
    packet.pcm_data_size = pcm_size;
    packet.timestamp = timestamp;
    packet.is_wav_header_included = include_header;

    packet.data = (uint8_t*)malloc(packet.total_size);
    if (packet.data == nullptr) {
        ESP_LOGE("MIC", "Failed to allocate WAV packet buffer");
        return false;
    }

    if (include_header) {
        uint32_t sample_rate = MIC_SAMPLE_RATE;
        uint16_t bits_per_sample = 16;
        uint16_t num_channels = 1;

        uint32_t byte_rate = sample_rate * num_channels * (bits_per_sample / 8);
        uint16_t block_align = num_channels * (bits_per_sample / 8);
        uint32_t chunk_size = 36 + pcm_size;

        uint8_t* wav_header = packet.data;
        wav_header[0] = 'R'; wav_header[1] = 'I'; wav_header[2] = 'F'; wav_header[3] = 'F';
        wav_header[4] = (uint8_t)(chunk_size & 0xFF);
        wav_header[5] = (uint8_t)((chunk_size >> 8) & 0xFF);
        wav_header[6] = (uint8_t)((chunk_size >> 16) & 0xFF);
        wav_header[7] = (uint8_t)((chunk_size >> 24) & 0xFF);
        wav_header[8] = 'W'; wav_header[9] = 'A'; wav_header[10] = 'V'; wav_header[11] = 'E';
        wav_header[12] = 'f'; wav_header[13] = 'm'; wav_header[14] = 't'; wav_header[15] = ' ';
        wav_header[16] = 16; wav_header[17] = 0; wav_header[18] = 0; wav_header[19] = 0;
        wav_header[20] = 1; wav_header[21] = 0;
        wav_header[22] = (uint8_t)(num_channels & 0xFF);
        wav_header[23] = (uint8_t)((num_channels >> 8) & 0xFF);
        wav_header[24] = (uint8_t)(sample_rate & 0xFF);
        wav_header[25] = (uint8_t)((sample_rate >> 8) & 0xFF);
        wav_header[26] = (uint8_t)((sample_rate >> 16) & 0xFF);
        wav_header[27] = (uint8_t)((sample_rate >> 24) & 0xFF);
        wav_header[28] = (uint8_t)(byte_rate & 0xFF);
        wav_header[29] = (uint8_t)((byte_rate >> 8) & 0xFF);
        wav_header[30] = (uint8_t)((byte_rate >> 16) & 0xFF);
        wav_header[31] = (uint8_t)((byte_rate >> 24) & 0xFF);
        wav_header[32] = (uint8_t)(block_align & 0xFF);
        wav_header[33] = (uint8_t)((block_align >> 8) & 0xFF);
        wav_header[34] = (uint8_t)(bits_per_sample & 0xFF);
        wav_header[35] = (uint8_t)((bits_per_sample >> 8) & 0xFF);
        wav_header[36] = 'd'; wav_header[37] = 'a'; wav_header[38] = 't'; wav_header[39] = 'a';
        wav_header[40] = (uint8_t)(pcm_size & 0xFF);
        wav_header[41] = (uint8_t)((pcm_size >> 8) & 0xFF);
        wav_header[42] = (uint8_t)((pcm_size >> 16) & 0xFF);
        wav_header[43] = (uint8_t)((pcm_size >> 24) & 0xFF);

        memcpy(packet.data + WAV_HEADER_SIZE, pcm_data, pcm_size);
    } else {
        memcpy(packet.data, pcm_data, pcm_size);
    }

    if (xQueueSend(queue, &packet, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW("MIC", "WAV queue full, dropping packet");
        free(packet.data);
        return false;
    }

    return true;
}

/**
 * @brief 读取ADC采样值
 */
static int adc_read_sample(adc_unit_t unit, adc_channel_t channel) {
    int raw_value = 0;
    if (unit == ADC_UNIT_1) {
        raw_value = adc1_get_raw((adc1_channel_t)channel);
    } else {
        // ADC2需要在critical section中读取，因为它与WiFi共享
        portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
        portENTER_CRITICAL(&spinlock);
        adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw_value);
        portEXIT_CRITICAL(&spinlock);
    }
    return raw_value;
}

/**
 * @brief 麦克风采集任务主循环 - 持续采集并存储到flash，同时串口实时打印
 */
void MEMS_MIC::mic_collection_loop() {
    ESP_LOGI(TAG, "MIC collection task started - continuous mode");

    const size_t target_packet_samples = MIC_SAMPLE_RATE / 10;  // 每包100ms数据
    const size_t target_packet_bytes = target_packet_samples * sizeof(int16_t);

    int16_t* accum_buffer = (int16_t*)malloc(target_packet_bytes);
    if (accum_buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate accumulation buffer");
        vTaskDelete(nullptr);
        return;
    }
    size_t accum_samples = 0;
    bool first_packet = true;
    size_t packets_sent = 0;
    size_t total_samples = 0;

    // 启动ADC
    app_mic_start();

    // 初始化变量用于串口打印和统计
    int print_counter = 0;
    const int PRINT_INTERVAL = 20;  // 每20个采样打印一次（约800Hz打印频率）
    int16_t min_pcm = INT16_MAX;
    int16_t max_pcm = INT16_MIN;
    int32_t sum_pcm = 0;
    int stats_counter = 0;
    const int STATS_INTERVAL = 160;  // 每160个采样（约10ms）计算一次统计
    
    uint32_t start_time = esp_timer_get_time() / 1000;

    // 串口打印启动信息
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║            MICROPHONE CONTINUOUS SAMPLING STARTED            ║\n");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║  Sample Rate: %6d Hz    Channel: IO15 (ADC2_CH2)             ║\n", MIC_SAMPLE_RATE);
    printf("║  Format: 16-bit PCM     Print: Every %d samples              ║\n", PRINT_INTERVAL);
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    while (1) {
        int raw_value = adc_read_sample(adc_unit_num, adc_channel_num);

        // 将ADC原始值转换为16位PCM
        // ESP32-S3 ADC是12位的，范围0-4095
        // 转换为有符号16位：0 -> -32768, 4095 -> 32767
        int16_t pcm_sample = (int16_t)((raw_value - 2048) * 16);
        
        // 统计计算
        sum_pcm += pcm_sample;
        if (pcm_sample < min_pcm) min_pcm = pcm_sample;
        if (pcm_sample > max_pcm) max_pcm = pcm_sample;
        stats_counter++;
        
        // 每PRINT_INTERVAL个采样打印一次详细数据
        if (print_counter++ >= PRINT_INTERVAL) {
            uint32_t current_time = esp_timer_get_time() / 1000;
            uint32_t elapsed = current_time - start_time;
            
            // 生成波形可视化（用字符表示信号幅度）
            char waveform[21] = "--------------------";  // 20个字符
            int wave_pos = (pcm_sample + 32768) * 19 / 65535;  // 映射到0-19
            waveform[wave_pos] = '*';
            
            printf("[MIC] [%6u ms] Raw:%4d | PCM:%6d | %s\n", 
                   (unsigned int)elapsed, raw_value, pcm_sample, waveform);
            print_counter = 0;
        }
        
        // 每STATS_INTERVAL个采样打印一次统计信息
        if (stats_counter >= STATS_INTERVAL) {
            uint32_t current_time = esp_timer_get_time() / 1000;
            uint32_t elapsed = current_time - start_time;
            int16_t avg_pcm = (int16_t)(sum_pcm / stats_counter);
            int32_t peak_to_peak = max_pcm - min_pcm;
            
            printf("[MIC] [%6u ms] Stats: Min=%6d Max=%6d Avg=%5d P2P=%6" PRIi32 "\n",
                   (unsigned int)elapsed, min_pcm, max_pcm, avg_pcm, peak_to_peak);
            
            // 重置统计
            min_pcm = INT16_MAX;
            max_pcm = INT16_MIN;
            sum_pcm = 0;
            stats_counter = 0;
        }

        // 累积到缓冲区
        accum_buffer[accum_samples++] = pcm_sample;
        total_samples++;

        // 如果缓冲区满，发送数据包
        if (accum_samples >= target_packet_samples) {
            uint32_t timestamp = (uint32_t)(esp_timer_get_time() / 1000);

            if (send_wav_packet(wav_queue, accum_buffer, target_packet_bytes, first_packet, timestamp)) {
                packets_sent++;
                
                // 打印数据包发送状态
                UBaseType_t queue_count = uxQueueMessagesWaiting(wav_queue);
                printf("[MIC] [%6u ms] Packet %u sent | Queue:%u pending | Total samples:%u\n", 
                       (unsigned int)(timestamp - start_time), (unsigned int)packets_sent, (unsigned int)queue_count, (unsigned int)total_samples);
                
                if (first_packet) {
                    ESP_LOGI(TAG, "First WAV packet sent, size=%d bytes", target_packet_bytes);
                    first_packet = false;
                }
            }

            accum_samples = 0;
        }

        // 控制采样速率（约16kHz）
        vTaskDelay(pdMS_TO_TICKS(62)); // 约16kHz采样率
    }

    free(accum_buffer);
}

void MEMS_MIC::app_mic_task_entry(void* arg) {
    MEMS_MIC* instance = static_cast<MEMS_MIC*>(arg);
    instance->mic_collection_loop();
}

void MEMS_MIC::app_mic_create_task() {
    if (mic_handler == nullptr) {
        xTaskCreate(app_mic_task_entry, "mic_collect", 4096, this, 4, &mic_handler);
        ESP_LOGI(TAG, "MIC collection task created (priority 4)");
    } else {
        ESP_LOGW(TAG, "MIC collection task already exists");
    }
}

/**
 * @brief 测试函数 - 读取ADC数据并通过日志输出
 */
void MEMS_MIC::adc_read_test(MEMS_MIC* instance) {
    ESP_LOGI(instance->TAG, "=== MICROPHONE ADC TEST START ===");

    // 启动ADC
    instance->app_mic_start();

    const int sample_count = MIC_SAMPLE_RATE * 3; // 3秒数据
    int16_t* pcm_buffer = (int16_t*)malloc(sample_count * sizeof(int16_t));
    if (pcm_buffer == nullptr) {
        ESP_LOGE(instance->TAG, "Failed to allocate test buffer");
        return;
    }

    ESP_LOGI(instance->TAG, "Sampling %d samples at %d Hz...", sample_count, MIC_SAMPLE_RATE);

    for (int i = 0; i < sample_count; i++) {
        int raw_value = adc_read_sample(instance->adc_unit_num, instance->adc_channel_num);
        
        // 转换为16位PCM
        pcm_buffer[i] = (int16_t)((raw_value - 2048) * 16);

        // 每100个采样打印一次（实时输出）
        if (i % 100 == 0) {
            printf("[MIC TEST] Sample %5d: Raw=%4d, PCM=%6d\n", i, raw_value, pcm_buffer[i]);
        }

        vTaskDelay(pdMS_TO_TICKS(62)); // 约16kHz
    }

    instance->app_mic_stop();

    // 统计分析
    int64_t sum = 0;
    int32_t max_val = INT32_MIN;
    int32_t min_val = INT32_MAX;
    
    for (int i = 0; i < sample_count; i++) {
        sum += abs(pcm_buffer[i]);
        if (pcm_buffer[i] > max_val) max_val = pcm_buffer[i];
        if (pcm_buffer[i] < min_val) min_val = pcm_buffer[i];
    }

    int32_t avg_amplitude = sample_count > 0 ? (int32_t)(sum / sample_count) : 0;

    ESP_LOGI(instance->TAG, "=== MICROPHONE TEST RESULTS ===");
    ESP_LOGI(instance->TAG, "Max value: %d", max_val);
    ESP_LOGI(instance->TAG, "Min value: %d", min_val);
    ESP_LOGI(instance->TAG, "Average amplitude: %d", avg_amplitude);

    if (avg_amplitude < 100) {
        ESP_LOGW(instance->TAG, "Warning: Low signal level, check microphone connection");
    } else {
        ESP_LOGI(instance->TAG, "Microphone working correctly");
    }

    // 保存测试数据到SPIFFS
    FileStorage::app_file_init_once();
    
    char filename[64];
    snprintf(filename, sizeof(filename), "/spiffs/mic_test.wav");
    
    struct stat st;
    if (stat(filename, &st) == 0) {
        remove(filename);
    }

    FILE* wav_file = fopen(filename, "wb");
    if (wav_file != nullptr) {
        uint32_t pcm_data_size = sample_count * sizeof(int16_t);
        uint32_t chunk_size = 36 + pcm_data_size;
        uint32_t byte_rate = MIC_SAMPLE_RATE * 1 * 2;

        uint8_t wav_header[44] = {
            'R','I','F','F',
            (uint8_t)(chunk_size&0xFF), (uint8_t)((chunk_size>>8)&0xFF), 
            (uint8_t)((chunk_size>>16)&0xFF), (uint8_t)((chunk_size>>24)&0xFF),
            'W','A','V','E','f','m','t',' ',16,0,0,0,1,0,1,0,
            (uint8_t)(MIC_SAMPLE_RATE&0xFF), (uint8_t)((MIC_SAMPLE_RATE>>8)&0xFF),
            (uint8_t)((MIC_SAMPLE_RATE>>16)&0xFF), (uint8_t)((MIC_SAMPLE_RATE>>24)&0xFF),
            (uint8_t)(byte_rate&0xFF), (uint8_t)((byte_rate>>8)&0xFF),
            (uint8_t)((byte_rate>>16)&0xFF), (uint8_t)((byte_rate>>24)&0xFF),
            2,0,16,0,'d','a','t','a',
            (uint8_t)(pcm_data_size&0xFF), (uint8_t)((pcm_data_size>>8)&0xFF),
            (uint8_t)((pcm_data_size>>16)&0xFF), (uint8_t)((pcm_data_size>>24)&0xFF)
        };

        fwrite(wav_header, 1, 44, wav_file);
        fwrite(pcm_buffer, sizeof(int16_t), sample_count, wav_file);
        fclose(wav_file);

        ESP_LOGI(instance->TAG, "WAV file saved: %s (%d bytes)", filename, 44 + pcm_data_size);
    }

    free(pcm_buffer);
    ESP_LOGI(instance->TAG, "=== MICROPHONE ADC TEST END ===");
}
