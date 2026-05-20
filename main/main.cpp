#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"
#include "app_wifi.h"
#include "app_mpu5060.h"
#include "app_mic.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>
#include <sys/unistd.h>

static const char *TAG = "main";

#define SAMPLE_RATE_HZ        50
#define SAMPLES_PER_UPLOAD    (SAMPLE_RATE_HZ * 20)  /* 1000 = 20 seconds */
#define UPLOAD_INTERVAL_MS    60000
#define MAX_RETRY_COUNT       3
#define RETRY_DELAY_MS        2000

#define DEFAULT_LAT           39.9
#define DEFAULT_LNG           116.3
#define DEFAULT_BATTERY       99

#define SPIFFS_MOUNT_POINT    "/spiffs"
#define FLASH_DATA_FILE       "/spiffs/sensor_buf.dat"
#define JSON_BUF_SIZE         65000

/* 每样本二进制存储: 6*float + 1*int16 = 26 bytes */
#define BYTES_PER_SAMPLE      26

/* 打印当前可用堆内存 */
#define LOG_FREE_HEAP() \
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size())

static APP_MPU6050 *mpu6050 = nullptr;
static MEMS_MIC *mic = nullptr;

static bool is_wifi_connected(void) {
    wifi_ap_record_t ap_info;
    return esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK && ap_info.ssid[0] != '\0';
}

static bool init_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = SPIFFS_MOUNT_POINT,
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        return false;
    }
    size_t total = 0, used = 0;
    esp_spiffs_info(NULL, &total, &used);
    ESP_LOGI(TAG, "SPIFFS ready: %u / %u bytes used", (unsigned)used, (unsigned)total);
    return true;
}

static bool write_sample_to_flash(FILE *f, float acc_x, float acc_y, float acc_z,
                                   float gyr_x, float gyr_y, float gyr_z,
                                   int16_t audio) {
    if (!f) return false;
    if (fwrite(&acc_x, 4, 1, f) != 1) return false;
    if (fwrite(&acc_y, 4, 1, f) != 1) return false;
    if (fwrite(&acc_z, 4, 1, f) != 1) return false;
    if (fwrite(&gyr_x, 4, 1, f) != 1) return false;
    if (fwrite(&gyr_y, 4, 1, f) != 1) return false;
    if (fwrite(&gyr_z, 4, 1, f) != 1) return false;
    if (fwrite(&audio, 2, 1, f) != 1) return false;
    return true;
}

static FILE *open_flash_for_upload(size_t *out_samples) {
    FILE *f = fopen(FLASH_DATA_FILE, "rb");
    if (!f) return nullptr;

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0 || (size % BYTES_PER_SAMPLE) != 0) {
        ESP_LOGW(TAG, "Flash file corrupted (%ld bytes), deleting", size);
        fclose(f);
        unlink(FLASH_DATA_FILE);
        return nullptr;
    }

    *out_samples = (size_t)size / BYTES_PER_SAMPLE;
    ESP_LOGI(TAG, "Found %u leftover samples on flash", (unsigned)*out_samples);
    return f;
}

static int load_samples_from_flash(FILE *f, size_t max_samples,
                                    float *acc_out, float *gyro_out,
                                    int16_t *audio_out) {
    if (!f || !acc_out || !gyro_out || !audio_out) return 0;
    int count = 0;
    for (size_t i = 0; i < max_samples; i++) {
        float ax, ay, az, gx, gy, gz;
        int16_t aud;
        if (fread(&ax, 4, 1, f) != 1) break;
        if (fread(&ay, 4, 1, f) != 1) break;
        if (fread(&az, 4, 1, f) != 1) break;
        if (fread(&gx, 4, 1, f) != 1) break;
        if (fread(&gy, 4, 1, f) != 1) break;
        if (fread(&gz, 4, 1, f) != 1) break;
        if (fread(&aud, 2, 1, f) != 1) break;

        int idx = count * 3;
        acc_out[idx]     = ax;
        acc_out[idx + 1] = ay;
        acc_out[idx + 2] = az;
        gyro_out[idx]     = gx;
        gyro_out[idx + 1] = gy;
        gyro_out[idx + 2] = gz;
        audio_out[count]  = aud;
        count++;
    }
    return count;
}

/*
 * 直接构建 JSON 字符串，避免 cJSON 为 18000 个数值创建节点的巨大内存开销。
 * 返回 JSON 字节数，失败返回 -1。
 */
static int build_periodic_json(char *buf, size_t buf_size,
                                uint64_t timestamp, int seq_id, int battery,
                                double lat, double lng, int duration_ms,
                                float *acc_data, int acc_floats,
                                float *gyro_data, int gyro_floats,
                                int16_t *audio_data, int audio_samples) {
    char *p = buf;
    char *end = buf + buf_size;
    int w = 0;

    /* 头部 */
    w = snprintf(p, end - p,
        "{\"device_id\":\"%s\",\"timestamp\":%llu,\"seq_id\":%d,"
        "\"battery\":%d,\"location\":{\"lat\":%.1f,\"lng\":%.1f},"
        "\"payload\":{\"duration_ms\":%d,\"sensor_data\":{",
        DEVICE_ID, (unsigned long long)timestamp, seq_id,
        battery, lat, lng, duration_ms);
    if (w < 0 || w >= end - p) return -1;
    p += w;

    /* acc 数组: [[x,y,z], ...] */
    w = snprintf(p, end - p, "\"acc\":[");
    if (w < 0 || w >= end - p) return -1;
    p += w;
    for (int i = 0; i < acc_floats; i += 3) {
        w = snprintf(p, end - p, "%s[%.2f,%.2f,%.2f]",
                     (i > 0) ? "," : "",
                     acc_data[i], acc_data[i + 1], acc_data[i + 2]);
        if (w < 0 || w >= end - p) return -1;
        p += w;
    }
    w = snprintf(p, end - p, "],");
    if (w < 0 || w >= end - p) return -1;
    p += w;

    /* gyro 数组 */
    w = snprintf(p, end - p, "\"gyro\":[");
    if (w < 0 || w >= end - p) return -1;
    p += w;
    for (int i = 0; i < gyro_floats; i += 3) {
        w = snprintf(p, end - p, "%s[%.2f,%.2f,%.2f]",
                     (i > 0) ? "," : "",
                     gyro_data[i], gyro_data[i + 1], gyro_data[i + 2]);
        if (w < 0 || w >= end - p) return -1;
        p += w;
    }
    w = snprintf(p, end - p, "],");
    if (w < 0 || w >= end - p) return -1;
    p += w;

    w = snprintf(p, end - p, "\"baro\":[]}");
    if (w < 0 || w >= end - p) return -1;
    p += w;

    /* 音频部分 */
    if (audio_data && audio_samples > 0) {
        char *b64 = encode_audio_base64(audio_data, audio_samples);
        if (b64) {
            w = snprintf(p, end - p, ",\"audio_data\":\"%s\"", b64);
            free(b64);
            if (w < 0 || w >= end - p) return -1;
            p += w;
        }
        w = snprintf(p, end - p, ",\"audio_format\":{\"sr\":%d,\"bit\":16}",
                     MIC_UPLOAD_SAMPLE_RATE);
        if (w < 0 || w >= end - p) return -1;
        p += w;
    }

    w = snprintf(p, end - p, ",\"is_abnormal\":0}}");
    if (w < 0 || w >= end - p) return -1;
    p += w;

    return (int)(p - buf);
}

static esp_err_t send_with_retry(const char *json_str, size_t json_len) {
    char url[128];
    snprintf(url, sizeof(url), "http://%s:%d/api/data", SERVER_IP, HTTP_PORT);

    for (int retry = 0; retry < MAX_RETRY_COUNT; retry++) {
        if (!is_wifi_connected()) {
            ESP_LOGW(TAG, "WiFi not connected, retry %d/%d", retry + 1, MAX_RETRY_COUNT);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            continue;
        }

        esp_err_t ret = app_http_send_data(url, json_str, json_len);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Upload failed (%s), retry %d/%d", esp_err_to_name(ret),
                 retry + 1, MAX_RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }
    return ESP_FAIL;
}

static int collect_to_buffers(float *acc_out, float *gyro_out,
                                   int16_t *audio_out, FILE *flash_f) {
    mpu6050_acce_value_t acce = {};
    mpu6050_gyro_value_t gyro_sample = {};
    int16_t pcm = 0;
    int count = 0;
    const int period_ms = 1000 / SAMPLE_RATE_HZ;

    ESP_LOGI(TAG, "Collecting %d samples (~20s)...", SAMPLES_PER_UPLOAD);
    uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);

    for (int i = 0; i < SAMPLES_PER_UPLOAD; i++) {
        bool got_imu = false;
        float ax = 0, ay = 0, az = 0;
        float gx = 0, gy = 0, gz = 0;

        if (mpu6050 && mpu6050->read_sample(&acce, &gyro_sample)) {
            ax = acce.acce_x;
            ay = acce.acce_y;
            az = acce.acce_z;
            gx = gyro_sample.gyro_x;
            gy = gyro_sample.gyro_y;
            gz = gyro_sample.gyro_z;
            got_imu = true;
        }

        int16_t audio_s = 0;
        bool got_mic = false;
        if (mic && mic->read_sample_pcm(&pcm)) {
            audio_s = pcm;
            got_mic = true;
            app_mic_append_sample(pcm);
        }

        if (got_imu) {
            int idx = count * 3;
            acc_out[idx]     = ax;
            acc_out[idx + 1] = ay;
            acc_out[idx + 2] = az;
            gyro_out[idx]     = gx;
            gyro_out[idx + 1] = gy;
            gyro_out[idx + 2] = gz;
            audio_out[count]  = audio_s;
            count++;
        }

        if (flash_f && got_imu) {
            write_sample_to_flash(flash_f, ax, ay, az, gx, gy, gz, audio_s);
        }

        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }

    uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - t0;
    ESP_LOGI(TAG, "Collection done: %d IMU samples in %lu ms", count, (unsigned long)elapsed);
    return count;
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "=== IMU+MIC 20s采集上传系统启动 ===");

    /* ---- 1. WiFi ---- */
    ESP_LOGI(TAG, "[1/5] 连接 WiFi: 清和科技...");
    app_wifi_init_sta("清和科技", "qinghekeji");
    /* HTTP client deferred to first upload (saves ~32KB heap at boot) */

    /* ---- 2. SPIFFS ---- */
    ESP_LOGI(TAG, "[2/5] 初始化 SPIFFS flash 存储...");
    if (!init_spiffs()) {
        ESP_LOGE(TAG, "SPIFFS 初始化失败，仅使用 RAM 缓冲");
    }

    /* ---- 3. MPU6050 ---- */
    ESP_LOGI(TAG, "[3/5] 初始化 MPU6050 (SDA=GPIO4, SCL=GPIO5)...");
    mpu6050 = new APP_MPU6050("MPU6050");
    if (!mpu6050->app_mpu6050_init()) {
        ESP_LOGE(TAG, "MPU6050 init failed!");
    } else {
        ESP_LOGI(TAG, "MPU6050 ready");
    }

    /* ---- 4. Mic ---- */
    ESP_LOGI(TAG, "[4/5] 初始化麦克风 (GPIO15/ADC)...");
    mic = new MEMS_MIC("MIC");
    if (mic->app_mic_init() != ESP_OK) {
        ESP_LOGE(TAG, "Mic init failed!");
    } else {
        ESP_LOGI(TAG, "Mic ready (%dHz sync)", MIC_UPLOAD_SAMPLE_RATE);
    }

    /* ---- 5. 分配缓冲区 ---- */
    ESP_LOGI(TAG, "[5/5] 分配内存缓冲区...");
    LOG_FREE_HEAP();
    float *acc_buf = (float *)malloc(SAMPLES_PER_UPLOAD * 3 * sizeof(float));
    float *gyro_buf = (float *)malloc(SAMPLES_PER_UPLOAD * 3 * sizeof(float));
    int16_t *audio_buf = (int16_t *)malloc(SAMPLES_PER_UPLOAD * sizeof(int16_t));
    char *json_buf = (char *)malloc(JSON_BUF_SIZE);

    if (!acc_buf || !gyro_buf || !audio_buf || !json_buf) {
        ESP_LOGE(TAG, "内存分配失败!");
        return;
    }
    ESP_LOGI(TAG, "缓冲区就绪 (acc:%u, gyro:%u, audio:%u, json:%u bytes)",
             (unsigned)(SAMPLES_PER_UPLOAD * 3 * 4),
             (unsigned)(SAMPLES_PER_UPLOAD * 3 * 4),
             (unsigned)(SAMPLES_PER_UPLOAD * 2),
             (unsigned)JSON_BUF_SIZE);

    /* 尝试恢复上次上传失败留下的数据 */
    size_t leftover_n = 0;
    FILE *leftover_f = open_flash_for_upload(&leftover_n);
    if (leftover_f && leftover_n > 0 && leftover_n <= (size_t)SAMPLES_PER_UPLOAD) {
        int recovered = load_samples_from_flash(leftover_f, leftover_n,
                                                 acc_buf, gyro_buf, audio_buf);
        fclose(leftover_f);
        if (recovered > 0) {
            uint64_t ts_ms = esp_timer_get_time() / 1000;
            int json_len = build_periodic_json(
                json_buf, JSON_BUF_SIZE,
                ts_ms, 0, DEFAULT_BATTERY, DEFAULT_LAT, DEFAULT_LNG,
                recovered * 1000 / SAMPLE_RATE_HZ,
                acc_buf, recovered * 3,
                gyro_buf, recovered * 3,
                audio_buf, recovered);
            if (json_len > 0) {
                ESP_LOGI(TAG, "尝试上传遗留数据 (%d samples)...", recovered);
                if (send_with_retry(json_buf, (size_t)json_len) == ESP_OK) {
                    unlink(FLASH_DATA_FILE);
                    ESP_LOGI(TAG, "遗留数据上传成功");
                }
            }
        }
    } else if (leftover_f) {
        fclose(leftover_f);
    }

    ESP_LOGI(TAG, "开始循环: 每60s上传一次(采集~20s), 数据同步写入flash");

    int seq_id = 0;
    while (1) {
        seq_id++;
        ESP_LOGI(TAG, "===== 第 %d 轮采集开始 =====", seq_id);

        /* 打开 flash 文件用于写入本轮数据 */
        FILE *flash_f = fopen(FLASH_DATA_FILE, "wb");
        if (!flash_f) {
            ESP_LOGW(TAG, "无法打开 flash 文件, 本轮不持久化");
        }

        /* 采集数据 */
        uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);
        app_mic_reset_ring();
        int n = collect_to_buffers(acc_buf, gyro_buf, audio_buf, flash_f);

        /* 关闭 flash 文件 */
        if (flash_f) {
            fflush(flash_f);
            fclose(flash_f);
        }

        uint32_t t1 = (uint32_t)(esp_timer_get_time() / 1000);
        int duration_ms = (int)(t1 - t0);

        /* 获取音频 */
        size_t audio_n = app_mic_get_upload_pcm(audio_buf, SAMPLES_PER_UPLOAD);

        ESP_LOGI(TAG, "本轮: %d IMU样本, %u 音频样本, 耗时 %d ms",
                 n, (unsigned)audio_n, duration_ms);

        if (n == 0) {
            ESP_LOGW(TAG, "无 IMU 数据, 跳过上传");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* 构建 JSON */
        uint64_t ts_ms = esp_timer_get_time() / 1000;
        int json_len = build_periodic_json(
            json_buf, JSON_BUF_SIZE,
            ts_ms, seq_id, DEFAULT_BATTERY, DEFAULT_LAT, DEFAULT_LNG,
            duration_ms,
            acc_buf, n * 3,
            gyro_buf, n * 3,
            audio_buf, (int)audio_n);

        if (json_len < 0) {
            ESP_LOGE(TAG, "JSON 构建失败 (buf overflow)");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "JSON size: %d bytes", json_len);

        /* 上传 */
        esp_err_t ret = send_with_retry(json_buf, (size_t)json_len);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "第 %d 轮上传成功", seq_id);
            /* 上传成功, 清除 flash 备份 */
            unlink(FLASH_DATA_FILE);
        } else {
            ESP_LOGE(TAG, "第 %d 轮上传失败, 数据已保留在 %s", seq_id, FLASH_DATA_FILE);
        }

        /* 计算到下一轮需要等待的时间 */
        uint32_t t2 = (uint32_t)(esp_timer_get_time() / 1000);
        int elapsed_since_start = (int)(t2 - t0);
        int wait_ms = UPLOAD_INTERVAL_MS - elapsed_since_start;
        if (wait_ms < 0) wait_ms = 0;
        if (wait_ms > 0) {
            ESP_LOGI(TAG, "等待 %d ms 到下一轮...", wait_ms);
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }
    }
}
