#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "app_wifi.h"
#include "app_mpu6050.h"
#include "app_mic.h"
#include "app_gps.h"
#include "app_gpio.h"
#include "app_battery.h"
#include "app_extflash.h"
#include "app_speaker.h"
#include "sdkconfig.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>
#include <sys/unistd.h>

static const char *TAG = "main";

#define SAMPLE_RATE_HZ        50
#define SAMPLES_PER_UPLOAD    (SAMPLE_RATE_HZ * CONFIG_COLLECT_DURATION_SEC)
#define AUDIO_COLLECT_SAMPLES (SAMPLE_RATE_HZ * 10)  /* first 10s audio */
#define UPLOAD_INTERVAL_MS    (CONFIG_COLLECT_DURATION_SEC * 1000)
#define MAX_RETRY_COUNT       3
#define RETRY_DELAY_MS        2000

#define SPIFFS_MOUNT_POINT    "/spiffs"
#define FLASH_DATA_FILE       "/spiffs/sensor_buf.dat"
#define LAST_JSON_FILE        "/spiffs/last_payload.json"
#define JSON_BUF_SIZE         72000
/* JSON 中 IMU 降采样: 采集 50Hz, JSON 每 N 点取 1 (60s/5=600 点, ~35KB JSON) */
#define IMU_JSON_DECIM        5
#define FALL_ALARM_EVENT_ID   500
#define FALL_ALARM_COOLDOWN_MS 60000

/* 每样本二进制存储: 6*float + 1*int16 = 26 bytes */
#define BYTES_PER_SAMPLE      26

/* 打印当前可用堆内存 */
#define LOG_FREE_HEAP() \
    ESP_LOGI(TAG, "Free heap: internal=%lu total=%lu", \
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), \
             (unsigned long)esp_get_free_heap_size())

static void *alloc_buffer(size_t size, const char *name) {
    void *p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!p) {
        ESP_LOGE(TAG, "分配 %s 失败 (%u bytes), internal free=%lu",
                 name, (unsigned)size,
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    }
    return p;
}

static APP_MPU6050 *mpu6050 = nullptr;
static MEMS_MIC *mic = nullptr;
static ExtFlash *ext_flash = nullptr;
static uint32_t s_last_fall_alarm_ms = 0;

static void fall_alert_task(void *arg) {
    (void)arg;
    speaker_beep(2000, 500);
    vTaskDelete(nullptr);
}

static void trigger_fall_alert_sound(void) {
    xTaskCreate(fall_alert_task, "fall_alert", 3072, nullptr, 4, nullptr);
}

static int read_battery_for_upload(void) {
    int pct = app_battery_read_percent();
    return (pct >= 0) ? pct : 0;
}

static void read_location_for_upload(double *lat, double *lng, bool *has_fix) {
#if CONFIG_GPS_ENABLE
    gps_data_t gps;
    app_gps_get_data(&gps);
    *has_fix = app_gps_has_fix();
    *lat = gps.latitude;
    *lng = gps.longitude;
#else
    if (lat) *lat = 0.0;
    if (lng) *lng = 0.0;
    if (has_fix) *has_fix = false;
#endif
}

static void save_json_snapshot(const char *json, size_t len) {
    FILE *f = fopen(LAST_JSON_FILE, "wb");
    if (!f) {
        ESP_LOGW(TAG, "无法保存 JSON 到 %s", LAST_JSON_FILE);
        return;
    }
    size_t w = fwrite(json, 1, len, f);
    fclose(f);
    ESP_LOGI(TAG, "JSON 已保存到 %s (%u bytes)", LAST_JSON_FILE, (unsigned)w);
}

static void log_round_summary(int imu_n, size_t audio_n, const int16_t *audio,
                              const float *acc, const float *gyro,
                              int battery, bool gps_fix, double lat, double lng,
                              int json_len, esp_err_t upload_ret) {
    ESP_LOGI(TAG, "========== 本轮采集汇总 ==========");
    ESP_LOGI(TAG, "IMU: %d samples", imu_n);
    if (imu_n > 0 && acc && gyro) {
        ESP_LOGI(TAG, "  acc[0]=[%.3f, %.3f, %.3f]", acc[0], acc[1], acc[2]);
        ESP_LOGI(TAG, "  gyro[0]=[%.3f, %.3f, %.3f]", gyro[0], gyro[1], gyro[2]);
    }
    ESP_LOGI(TAG, "Audio: %u samples", (unsigned)audio_n);
    if (audio_n > 0 && audio) {
        ESP_LOGI(TAG, "  pcm[0..2]=[%d, %d, %d]", (int)audio[0], (int)audio[1], (int)audio[2]);
    } else {
        ESP_LOGW(TAG, "  无音频 (检查 mic init / I2S IO15/16/3)");
    }
    ESP_LOGI(TAG, "Battery: %d%%", battery);
#if CONFIG_GPS_ENABLE
    if (gps_fix) {
        ESP_LOGI(TAG, "GPS: fix lat=%.6f lng=%.6f", lat, lng);
    } else {
        ESP_LOGI(TAG, "GPS: no fix, using lat/lng=0");
    }
#else
    ESP_LOGI(TAG, "GPS: disabled");
#endif
    ESP_LOGI(TAG, "JSON: %d bytes", json_len);
    ESP_LOGI(TAG, "Upload: %s", upload_ret == ESP_OK ? "成功" : "失败(网络不可用或4G未连接)");
    ESP_LOGI(TAG, "==================================");
}

static bool is_wifi_connected(void) {
    return app_network_is_connected();
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
                                const char *device_id,
                                uint64_t timestamp, int seq_id, int battery,
                                double lat, double lng, int duration_ms,
                                float *acc_data, int acc_floats,
                                float *gyro_data, int gyro_floats,
                                int16_t *audio_data, int audio_samples,
                                int is_abnormal) {
    char *p = buf;
    char *end = buf + buf_size;
    int w = 0;

    w = snprintf(p, end - p,
        "{\"device_id\":\"%s\",\"timestamp\":%llu,\"seq_id\":%d,"
        "\"battery\":%d,\"location\":{\"lat\":%.6f,\"lng\":%.6f},"
        "\"payload\":{\"duration_ms\":%d,\"sensor_data\":{",
        device_id, (unsigned long long)timestamp, seq_id,
        battery, lat, lng, duration_ms);
    if (w < 0 || w >= end - p) return -1;
    p += w;

    /* acc 数组: [[x,y,z], ...] (降采样 IMU_JSON_DECIM) */
    w = snprintf(p, end - p, "\"acc\":[");
    if (w < 0 || w >= end - p) return -1;
    p += w;
    for (int i = 0, n = 0; i < acc_floats; i += 3 * IMU_JSON_DECIM, n++) {
        w = snprintf(p, end - p, "%s[%.2f,%.2f,%.2f]",
                     (n > 0) ? "," : "",
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
    for (int i = 0, n = 0; i < gyro_floats; i += 3 * IMU_JSON_DECIM, n++) {
        w = snprintf(p, end - p, "%s[%.2f,%.2f,%.2f]",
                     (n > 0) ? "," : "",
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
                     AUDIO_PCM_SAMPLE_RATE);
        if (w < 0 || w >= end - p) return -1;
        p += w;
    }

    w = snprintf(p, end - p, ",\"is_abnormal\":%d}}", is_abnormal ? 1 : 0);
    if (w < 0 || w >= end - p) return -1;
    p += w;

    return (int)(p - buf);
}

static esp_err_t send_json_with_retry(const char *api_path, const char *json_str, size_t json_len) {
    char url[128];
    snprintf(url, sizeof(url), "http://%s:%d%s", SERVER_IP, HTTP_PORT, api_path);

    for (int retry = 0; retry < MAX_RETRY_COUNT; retry++) {
        if (!is_wifi_connected()) {
            ESP_LOGW(TAG, "Network not connected, retry %d/%d", retry + 1, MAX_RETRY_COUNT);
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

static esp_err_t send_periodic_with_retry(const char *json_str, size_t json_len) {
    return send_json_with_retry(HTTP_PATH_DATA, json_str, json_len);
}

/* 算法跌倒报警: 取最近 1 秒 IMU 快照 + 当前音频 */
static esp_err_t send_fall_alarm(float *acc, float *gyro, int imu_samples,
                                 int16_t *audio, int audio_samples) {
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (s_last_fall_alarm_ms != 0 &&
        (now_ms - s_last_fall_alarm_ms) < FALL_ALARM_COOLDOWN_MS) {
        ESP_LOGW(TAG, "跌倒报警冷却中, 跳过重复上报");
        return ESP_ERR_INVALID_STATE;
    }

    double lat = 0, lng = 0;
    bool has_fix = false;
    read_location_for_upload(&lat, &lng, &has_fix);

    int snap = imu_samples;
    if (snap > IMU_SAMPLE_RATE_HZ) snap = IMU_SAMPLE_RATE_HZ;
    if (snap <= 0) return ESP_ERR_INVALID_ARG;

    int offset = (imu_samples - snap) * 3;
    float *baro_empty = nullptr;

    esp_err_t ret = app_http_send_alarm(
        FALL_ALARM_EVENT_ID,
        (uint64_t)(esp_timer_get_time() / 1000),
        read_battery_for_upload(),
        lat, lng,
        0, /* status_confirm: 0=算法自动检测 */
        acc + offset, snap * 3,
        gyro + offset, snap * 3,
        baro_empty, 0,
        audio, audio_samples);

    if (ret == ESP_OK) {
        s_last_fall_alarm_ms = now_ms;
        trigger_fall_alert_sound();
    }
    return ret;
}

/* SW1 手动报警: 取最近 1 秒 IMU 快照 + 当前音频 */
static esp_err_t send_manual_alarm(float *acc, float *gyro, int imu_samples,
                                   int16_t *audio, int audio_samples) {
    double lat = 0, lng = 0;
    bool has_fix = false;
    read_location_for_upload(&lat, &lng, &has_fix);

    int snap = imu_samples;
    if (snap > IMU_SAMPLE_RATE_HZ) snap = IMU_SAMPLE_RATE_HZ;
    if (snap <= 0) return ESP_ERR_INVALID_ARG;

    int offset = (imu_samples - snap) * 3;
    float *baro_empty = nullptr;

    return app_http_send_alarm(
        501,
        (uint64_t)(esp_timer_get_time() / 1000),
        read_battery_for_upload(),
        lat, lng,
        1, /* status_confirm: 1=手动按键报警 */
        acc + offset, snap * 3,
        gyro + offset, snap * 3,
        baro_empty, 0,
        audio, audio_samples);
}

/* SW3 留言: 上传当前音频 */
static esp_err_t send_voice_message(int16_t *audio, int audio_samples) {
    double lat = 0, lng = 0;
    bool has_fix = false;
    read_location_for_upload(&lat, &lng, &has_fix);

    return app_http_send_message(
        (uint64_t)(esp_timer_get_time() / 1000),
        read_battery_for_upload(),
        lat, lng,
        audio, audio_samples);
}

static int collect_to_buffers(float *acc_out, float *gyro_out,
                                   int16_t *audio_out, FILE *flash_f,
                                   bool *fall_detected_out) {
    mpu6050_acce_value_t acce = {};
    mpu6050_gyro_value_t gyro_sample = {};
    int16_t pcm = 0;
    int count = 0;
    const int period_ms = 1000 / SAMPLE_RATE_HZ;

    ESP_LOGI(TAG, "Collecting %d samples (~60s), audio only first %d samples",
             SAMPLES_PER_UPLOAD, AUDIO_COLLECT_SAMPLES);
    uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);

    int16_t audio_min = 32767, audio_max = -32768;
    int64_t audio_sum = 0;
    int audio_samples = 0;
    int audio_read_fail = 0;
    bool fall_detected = false;

    if (!mic || !mic->app_mic_check_module()) {
        ESP_LOGW(TAG, "Mic not ready, audio will be skipped this round");
    }

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
        if (i < AUDIO_COLLECT_SAMPLES && mic && mic->read_sample_pcm(&pcm)) {
            audio_s = pcm;
            app_mic_append_sample(pcm);

            if ((i % 50) == 0) {
                if (got_imu) {
                    ESP_LOGI(TAG, "IMU[%d]: acc=(%.3f,%.3f,%.3f) gyro=(%.2f,%.2f,%.2f)",
                             i, ax, ay, az, gx, gy, gz);
                }
                ESP_LOGI(TAG, "Audio[%d]: pcm=%d raw=0x%08lx",
                         i, (int)audio_s, (unsigned long)(uint32_t)app_mic_get_last_raw());
            }
            if (audio_s < audio_min) audio_min = audio_s;
            if (audio_s > audio_max) audio_max = audio_s;
            audio_sum += audio_s;
            audio_samples++;
        } else if (i < AUDIO_COLLECT_SAMPLES && mic) {
            audio_read_fail++;
            if (audio_read_fail == 1) {
                ESP_LOGW(TAG, "Mic read failed at sample %d", i);
            }
        } else if (i == AUDIO_COLLECT_SAMPLES) {
            ESP_LOGI(TAG, "Audio window done: ok=%d fail=%d",
                     audio_samples, audio_read_fail);
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

            if (mpu6050->detect_fall(acce)) {
                fall_detected = true;
                ESP_LOGW(TAG, "!!! 跌倒算法触发 (sample %d) !!!", count);
                size_t audio_n = app_mic_get_upload_pcm(audio_out, count);
                esp_err_t ar = send_fall_alarm(acc_out, gyro_out, count,
                                               audio_out, (int)audio_n);
                ESP_LOGI(TAG, "跌倒报警上传 %s", ar == ESP_OK ? "成功" : "失败");
            }
        }

        if (flash_f && got_imu) {
            write_sample_to_flash(flash_f, ax, ay, az, gx, gy, gz, audio_s);
        }

        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }

    uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - t0;
    ESP_LOGI(TAG, "Collection done: %d IMU samples in %lu ms", count, (unsigned long)elapsed);

    /* 音频统计（仅前 10 秒） */
    if (audio_samples > 0) {
        ESP_LOGI(TAG, "Audio stats: min=%d, max=%d, avg=%lld, ok=%d, fail=%d",
                 (int)audio_min, (int)audio_max,
                 (long long)(audio_sum / audio_samples),
                 audio_samples, audio_read_fail);
    } else {
        ESP_LOGW(TAG, "Audio: no samples (fail=%d, mic=%s)", audio_read_fail,
                 (mic && mic->app_mic_check_module()) ? "ready" : "not ready");
    }
    if (fall_detected_out) {
        *fall_detected_out = fall_detected;
    }
    return count;
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "=== IMU+MIC 采集上传 (每轮 %ds) ===", CONFIG_COLLECT_DURATION_SEC);

    /* ---- 1. Network ---- */
    ESP_LOGI(TAG, "[1/8] 初始化 4G 网络 (ML307R)...");
    app_network_init(NET_MODE_ML307R, NULL, NULL);

    /* ---- 2. SPIFFS ---- */
    ESP_LOGI(TAG, "[2/8] 初始化 SPIFFS flash 存储...");
    if (!init_spiffs()) {
        ESP_LOGE(TAG, "SPIFFS 初始化失败，仅使用 RAM 缓冲");
    }

    /* ---- 3. GPS (optional) ---- */
#if CONFIG_GPS_ENABLE
    ESP_LOGI(TAG, "[3/8] 初始化 GPS (TX=IO%d, RX=IO%d, ON_OFF=IO%d, RST=IO%d)...",
             GPS_TX_PIN, GPS_RX_PIN, GPS_ON_OFF_PIN, GPS_RST_PIN);
    if (app_gps_init() == ESP_OK) {
        app_gps_power_on();
        xTaskCreate(app_gps_task, "gps_task", 3072, NULL, 5, NULL);
        ESP_LOGI(TAG, "GPS 模块已启动");
    }
#else
    ESP_LOGI(TAG, "[3/8] GPS 已禁用 (menuconfig: GPS_ENABLE), 上传 lat/lng=0");
#endif

    /* ---- 4. Keys, Speaker & Battery ---- */
    ESP_LOGI(TAG, "[4/8] 初始化按键/扬声器/电量检测...");
    app_keys_init();
    if (speaker_init() == ESP_OK) {
        ESP_LOGI(TAG, "Speaker ready (GPIO%d)", SPEAKER_GPIO);
    } else {
        ESP_LOGW(TAG, "Speaker init failed");
    }
    if (app_battery_init() == ESP_OK) {
        app_battery_led_update();
    } else {
        ESP_LOGW(TAG, "Battery ADC init failed");
    }

    /* ---- 5. External Flash ---- */
    ESP_LOGI(TAG, "[5/8] 初始化外置 Flash W25Q64...");
    ext_flash = new ExtFlash("ExtFlash");
    if (ext_flash->init() != ESP_OK) {
        ESP_LOGW(TAG, "外置 Flash 不可用, 继续使用 SPIFFS");
    }

    /* ---- 6. MPU9250/6050 ---- */
    ESP_LOGI(TAG, "[6/8] 初始化 IMU (SDA=IO%d, SCL=IO%d, INT=IO%d)...",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, MPU6050_INT_GPIO);
    mpu6050 = new APP_MPU6050("IMU");
    if (!mpu6050->app_mpu6050_init()) {
        ESP_LOGE(TAG, "IMU init failed!");
    } else {
        ESP_LOGI(TAG, "IMU ready");
    }

    /* ---- 7. Mic ---- */
    ESP_LOGI(TAG, "[7/8] 初始化麦克风 SPH0645 I2S (WS=IO%d BCLK=IO%d DATA=IO%d)...",
             MIC_I2S_WS_GPIO, MIC_I2S_BCLK_GPIO, MIC_I2S_DATA_GPIO);
    mic = new MEMS_MIC("MIC");
    if (mic->app_mic_init() != ESP_OK) {
        ESP_LOGE(TAG, "Mic init failed!");
    } else {
        ESP_LOGI(TAG, "Mic ready (I2S %dHz, upload sync %dHz)",
                 MIC_SAMPLE_RATE, MIC_UPLOAD_SAMPLE_RATE);
    }

    /* ---- 8. 分配缓冲区 ---- */
    ESP_LOGI(TAG, "[8/8] 分配内存缓冲区...");
    LOG_FREE_HEAP();

    size_t acc_bytes = SAMPLES_PER_UPLOAD * 3 * sizeof(float);
    size_t gyro_bytes = SAMPLES_PER_UPLOAD * 3 * sizeof(float);
    size_t audio_bytes = SAMPLES_PER_UPLOAD * sizeof(int16_t);

    float *acc_buf = (float *)alloc_buffer(acc_bytes, "acc_buf");
    float *gyro_buf = (float *)alloc_buffer(gyro_bytes, "gyro_buf");
    int16_t *audio_buf = (int16_t *)alloc_buffer(audio_bytes, "audio_buf");
    char *json_buf = (char *)alloc_buffer(JSON_BUF_SIZE, "json_buf");

    if (!acc_buf || !gyro_buf || !audio_buf || !json_buf) {
        ESP_LOGE(TAG, "内存分配失败! (无 PSRAM, 需约 %u bytes)",
                 (unsigned)(acc_bytes + gyro_bytes + audio_bytes + JSON_BUF_SIZE));
        return;
    }
    ESP_LOGI(TAG, "缓冲区就绪 (acc:%u, gyro:%u, audio:%u, json:%u, IMU decim:%d)",
             (unsigned)acc_bytes, (unsigned)gyro_bytes,
             (unsigned)audio_bytes, (unsigned)JSON_BUF_SIZE, IMU_JSON_DECIM);

    /* 尝试恢复上次上传失败留下的数据 */
    size_t leftover_n = 0;
    FILE *leftover_f = open_flash_for_upload(&leftover_n);
    if (leftover_f && leftover_n > 0 && leftover_n <= (size_t)SAMPLES_PER_UPLOAD) {
        int recovered = load_samples_from_flash(leftover_f, leftover_n,
                                                 acc_buf, gyro_buf, audio_buf);
        fclose(leftover_f);
        if (recovered > 0) {
            uint64_t ts_ms = esp_timer_get_time() / 1000;
            double lat = 0, lng = 0;
            bool has_fix = false;
            read_location_for_upload(&lat, &lng, &has_fix);
            int json_len = build_periodic_json(
                json_buf, JSON_BUF_SIZE, app_get_device_id(),
                ts_ms, 0, read_battery_for_upload(), lat, lng,
                recovered * 1000 / SAMPLE_RATE_HZ,
                acc_buf, recovered * 3,
                gyro_buf, recovered * 3,
                audio_buf, recovered, 0);
            if (json_len > 0) {
                ESP_LOGI(TAG, "尝试上传遗留数据 (%d samples)...", recovered);
                if (send_periodic_with_retry(json_buf, (size_t)json_len) == ESP_OK) {
                    unlink(FLASH_DATA_FILE);
                    ESP_LOGI(TAG, "遗留数据上传成功");
                }
            }
        }
    } else if (leftover_f) {
        fclose(leftover_f);
    }

    ESP_LOGI(TAG, "开始循环: 每60s上传一次(IMU全60s, 音频仅前10s), 同步写flash");

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
        if (mpu6050) {
            mpu6050->reset_fall_detector();
        }
        bool fall_detected = false;
        int n = collect_to_buffers(acc_buf, gyro_buf, audio_buf, flash_f,
                                   &fall_detected);

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

        /* 按键: SW1=手动报警, SW3=留言 */
        if (app_key_sw1_pressed()) {
            ESP_LOGI(TAG, "SW1 手动报警");
            esp_err_t ar = send_manual_alarm(acc_buf, gyro_buf, n, audio_buf, (int)audio_n);
            ESP_LOGI(TAG, "报警包上传 %s", ar == ESP_OK ? "成功" : "失败");
        }
        if (app_key_sw3_pressed()) {
            ESP_LOGI(TAG, "SW3 留言");
            esp_err_t mr = send_voice_message(audio_buf, (int)audio_n);
            ESP_LOGI(TAG, "留言包上传 %s", mr == ESP_OK ? "成功" : "失败");
        }

        double use_lat = 0, use_lng = 0;
        bool has_gps_fix = false;
        read_location_for_upload(&use_lat, &use_lng, &has_gps_fix);

        int battery = read_battery_for_upload();

        uint64_t ts_ms = esp_timer_get_time() / 1000;
        int json_len = build_periodic_json(
            json_buf, JSON_BUF_SIZE, app_get_device_id(),
            ts_ms, seq_id, battery, use_lat, use_lng,
            duration_ms,
            acc_buf, n * 3,
            gyro_buf, n * 3,
            audio_buf, (int)audio_n,
            fall_detected ? 1 : 0);

        if (json_len < 0) {
            ESP_LOGE(TAG, "JSON 构建失败 (buf overflow)");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        save_json_snapshot(json_buf, (size_t)json_len);

        esp_err_t ret = send_periodic_with_retry(json_buf, (size_t)json_len);
        log_round_summary(n, audio_n, audio_buf, acc_buf, gyro_buf,
                          battery, has_gps_fix, use_lat, use_lng,
                          json_len, ret);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "第 %d 轮上传成功", seq_id);
            unlink(FLASH_DATA_FILE);
        } else {
            ESP_LOGW(TAG, "第 %d 轮上传失败, JSON 已存 %s", seq_id, LAST_JSON_FILE);
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

        app_battery_led_update();
    }
}
