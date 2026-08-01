#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "app_ml307r.h"
#include "app_mpu6050.h"
#include "app_mic.h"
#include "app_gps.h"
#include "app_gpio.h"
#include "app_battery.h"
#include "app_extflash.h"
#include "app_speaker.h"
#include "app_sr.h"
#include "app_wifi.h"
#include "mbedtls/base64.h"
#include "sdkconfig.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>
#include <sys/unistd.h>

static const char *TAG = "main";

#define SAMPLE_RATE_HZ        50
#define SAMPLES_PER_UPLOAD    (SAMPLE_RATE_HZ * CONFIG_COLLECT_DURATION_SEC)
#define COLLECT_INTERVAL_MS   (CONFIG_COLLECT_DURATION_SEC * 1000)
#define AUDIO_PCM_SAMPLE_RATE 16000

#define SPIFFS_MOUNT_POINT    "/spiffs"
#define JSON_BUF_SIZE         72000
/* JSON 中 IMU 降采样: 采集 50Hz, JSON 每 N 点取 1 (60s/5=600 点, ~35KB JSON) */
#define IMU_JSON_DECIM        5

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

static char s_device_id[32] = ML307R_ESIM_ICCID;

static const char *get_device_id(void) {
    return s_device_id;
}

static void update_device_id_from_4g(void) {
    char iccid[32] = {0};
    if (ml307r_get_iccid(iccid, sizeof(iccid)) && strlen(iccid) >= 10) {
        strncpy(s_device_id, iccid, sizeof(s_device_id) - 1);
        s_device_id[sizeof(s_device_id) - 1] = '\0';
        ESP_LOGI(TAG, "Device ID from eSIM ICCID: %s", s_device_id);
    }
}

static char *encode_audio_base64_local(const int16_t *audio_data, int audio_samples) {
    if (!audio_data || audio_samples <= 0) {
        return nullptr;
    }

    size_t raw_len = (size_t)audio_samples * sizeof(int16_t);
    size_t base64_len = ((raw_len + 2) / 3) * 4 + 1;
    char *encoded = (char *)malloc(base64_len);
    if (!encoded) {
        return nullptr;
    }

    size_t out_len = 0;
    if (mbedtls_base64_encode((unsigned char *)encoded, base64_len, &out_len,
                              (const unsigned char *)audio_data, raw_len) != 0) {
        free(encoded);
        return nullptr;
    }

    encoded[out_len] = '\0';
    return encoded;
}

static APP_MPU6050 *mpu6050 = nullptr;
static MEMS_MIC *mic = nullptr;
static ExtFlash *ext_flash = nullptr;

#if CONFIG_APP_SR_ENABLE
static void on_sr_event(app_sr_event_t evt, int command_id, const char *cmd_str) {
    (void)command_id;
  switch (evt) {
    case APP_SR_EVT_WAKEWORD:
      ESP_LOGI(TAG, "SR: wake word detected");
      speaker_beep(1000, 150);
      break;
    case APP_SR_EVT_CMD_HELP:
    case APP_SR_EVT_CMD_ALARM:
      ESP_LOGI(TAG, "SR: voice alarm command (%s)", cmd_str ? cmd_str : "");
      break;
    case APP_SR_EVT_CMD_MESSAGE:
      ESP_LOGI(TAG, "SR: voice message command (%s)", cmd_str ? cmd_str : "");
      break;
    case APP_SR_EVT_TIMEOUT:
      ESP_LOGW(TAG, "SR: command timeout");
      break;
    default:
      break;
  }
}

static void handle_pending_sr_events(float *acc, float *gyro, int imu_n,
                                     int16_t *audio, int audio_n) {
  (void)acc;
  (void)gyro;
  (void)imu_n;
  (void)audio;
  (void)audio_n;
  app_sr_event_t evt;
  int cmd_id = 0;
  while (app_sr_poll_event(&evt, &cmd_id)) {
    switch (evt) {
      case APP_SR_EVT_CMD_HELP:
      case APP_SR_EVT_CMD_ALARM:
        ESP_LOGI(TAG, "SR alarm action disabled");
        break;
      case APP_SR_EVT_CMD_MESSAGE:
        ESP_LOGI(TAG, "SR message action disabled");
        break;
      default:
        break;
    }
  }
}
#endif

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

static void log_round_summary(int imu_n, size_t audio_n, const int16_t *audio,
                              const float *acc, const float *gyro,
                              int battery, bool gps_fix, double lat, double lng,
                              int json_len) {
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
    ESP_LOGI(TAG, "==================================");
}

static bool init_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = SPIFFS_MOUNT_POINT,
        .partition_label = "spiffs",
        .max_files = 5,
        .format_if_mount_failed = false
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        return false;
    }
    size_t total = 0, used = 0;
    esp_spiffs_info("spiffs", &total, &used);
    ESP_LOGI(TAG, "SPIFFS ready: %u / %u bytes used", (unsigned)used, (unsigned)total);
    return true;
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

    /* 外层: 协议通用字段 */
    w = snprintf(p, end - p,
        "{\"ver\":1,\"type\":\"stream\","
        "\"msg_id\":\"%s_%llu_%d\","
        "\"device_id\":\"%s\","
        "\"ts\":%llu,"
        "\"seq\":%d,"
        "\"battery\":%d,",
        device_id, (unsigned long long)timestamp, seq_id,
        device_id, (unsigned long long)timestamp, seq_id,
        battery);
    if (w < 0 || w >= end - p) return -1;
    p += w;

    /* loc: 有 GPS 信号时写入，无信号时省略 */
    if (lat != 0 || lng != 0) {
        w = snprintf(p, end - p, "\"loc\":{\"lat\":%.6f,\"lng\":%.6f},", lat, lng);
        if (w < 0 || w >= end - p) return -1;
        p += w;
    }

    w = snprintf(p, end - p,
        "\"payload\":{\"duration_ms\":%d,\"sensor_data\":{\"freq\":%d,",
        duration_ms, SAMPLE_RATE_HZ);
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

    w = snprintf(p, end - p, "\"baro\":[]},");
    if (w < 0 || w >= end - p) return -1;
    p += w;

    /* audio_ref: 音频走独立二进制通道，JSON 只传引用 ID */
    if (audio_data && audio_samples > 0) {
        w = snprintf(p, end - p,
            "\"audio_ref\":\"stream_%s_%llu_%d.opus\"",
            device_id, (unsigned long long)timestamp, seq_id);
        if (w < 0 || w >= end - p) return -1;
        p += w;
    }

    /* anomaly: 0=正常, 1=疑似异常(端侧预判) */
    w = snprintf(p, end - p, ",\"anomaly\":%d}}", is_abnormal ? 1 : 0);
    if (w < 0 || w >= end - p) return -1;
    p += w;

    return (int)(p - buf);
}

static int collect_to_buffers(float *acc_out, float *gyro_out,
                                   int16_t *audio_out,
                                   bool *fall_detected_out) {
    mpu6050_acce_value_t acce = {};
    mpu6050_gyro_value_t gyro_sample = {};
    int16_t pcm = 0;
    int count = 0;
    const int period_ms = 1000 / SAMPLE_RATE_HZ;

    ESP_LOGI(TAG, "Collecting %d IMU+audio samples (~%ds)",
             SAMPLES_PER_UPLOAD, CONFIG_COLLECT_DURATION_SEC);
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
        if (mic && mic->read_sample_pcm(&pcm)) {
            audio_s = pcm;
            app_mic_append_sample(pcm);

            if (audio_s < audio_min) audio_min = audio_s;
            if (audio_s > audio_max) audio_max = audio_s;
            audio_sum += audio_s;
            audio_samples++;
        } else if (mic) {
            audio_read_fail++;
            if (audio_read_fail == 1) {
                ESP_LOGW(TAG, "Mic read failed at sample %d", i);
            }
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
            }
        }


#if CONFIG_APP_SR_ENABLE
        if (got_imu && count > 0) {
            size_t sr_audio_n = app_mic_get_upload_pcm(audio_out, (size_t)count);
            handle_pending_sr_events(acc_out, gyro_out, count, audio_out, (int)sr_audio_n);
        }
#endif

        if ((i % 50) == 0) {
            if (got_imu) {
                ESP_LOGI(TAG, "IMU[%d]: acc=(%.3f,%.3f,%.3f) gyro=(%.2f,%.2f,%.2f)",
                         i, ax, ay, az, gx, gy, gz);
            }
            ESP_LOGI(TAG, "Audio[%d]: pcm=%d raw=0x%08lx",
                     i, (int)audio_s, (unsigned long)(uint32_t)app_mic_get_last_raw());
#if CONFIG_GPS_ENABLE
            {
                gps_data_t gps_snap;
                char gnrmc_raw[128];
                app_gps_get_data(&gps_snap);
                if (app_gps_get_last_gnrmc(gnrmc_raw, sizeof(gnrmc_raw))) {
                    ESP_LOGI(TAG, "GNRMC原始数据: %s", gnrmc_raw);
                }
                if (app_gps_has_fix()) {
                    ESP_LOGI(TAG,
                             "GPS经纬度: latitude=%.6f longitude=%.6f",
                             gps_snap.latitude, gps_snap.longitude);
                } else {
                    ESP_LOGI(TAG, "GPS没有接收到准确的数据");
                }
            }
#endif
            int adc_mv = app_battery_read_adc_mv();
            if (adc_mv >= 0) {
                int battery_mv = adc_mv * 2; /* 10K/10K divider */
                ESP_LOGI(TAG,
                         "Battery[%d]: GPIO7 ADC=%d mV (%.3f V), "
                         "battery=%d mV (%.3f V)",
                         i, adc_mv, adc_mv / 1000.0,
                         battery_mv, battery_mv / 1000.0);
            } else {
                ESP_LOGW(TAG, "Battery[%d]: ADC read failed", i);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }

    uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - t0;
    ESP_LOGI(TAG, "Collection done: %d IMU samples in %lu ms", count, (unsigned long)elapsed);

    /* 整轮音频统计 */
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
    ESP_LOGI(TAG, "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "=== IMU+MIC 本地采集 (每轮 %ds) ===", CONFIG_COLLECT_DURATION_SEC);

    /*
     * 首次分区表加载会暂时关闭双核 Flash cache。
     * 必须在创建应用任务、GPIO ISR 和 UART 驱动前完成 SPIFFS 初始化。
     */
    ESP_LOGI(TAG, "[1/8] 初始化 SPIFFS flash 存储...");
    if (!init_spiffs()) {
        ESP_LOGE(TAG, "SPIFFS 初始化失败，仅使用 RAM 缓冲");
    }

    app_keys_init();

    /* ---- 2. ML307R 4G only (no WiFi, no server connection/upload) ---- */
    ESP_LOGI(TAG, "[2/8] 初始化 ML307R 4G 网络...");
    if (ml307r_init() != ESP_OK) {
        ESP_LOGE(TAG, "ML307R init failed");
    } else {
        ml307r_power_on();
        if (!ml307r_is_alive()) {
            ESP_LOGE(TAG, "ML307R not responding");
        } else if (!ml307r_wait_network(ML307R_NETWORK_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "4G network registration timeout");
        } else {
            update_device_id_from_4g();
            ESP_LOGI(TAG, "4G ready; connecting TCP server 120.53.251.149:8007");
            if (ml307r_send_hello("120.53.251.149", 8007)) {
                ESP_LOGI(TAG, "TCP hello sent successfully");
            } else {
                ESP_LOGW(TAG, "无法连接服务器，继续执行后续程序");
            }
        }
        /* 通知 app_wifi 模块使用 4G 模式，后续 app_http_send_data 会走 ml307r_http_post */
        app_network_init(NET_MODE_ML307R, NULL, NULL);
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
    ESP_LOGI(TAG, "[3/8] GPS 已禁用 (menuconfig: GPS_ENABLE)");
#endif

    /* ---- 4. Keys, Speaker & Battery ---- */
    ESP_LOGI(TAG, "[4/8] 初始化按键/扬声器/电量检测...");
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

    /* ---- 5. External SPI PSRAM ---- */
    ESP_LOGI(TAG, "[5/8] init external SPI PSRAM APS1604M...");
    ext_flash = new ExtFlash("ExtPSRAM");
    esp_err_t psram_ret = ext_flash->init();
    if (psram_ret == ESP_OK && ext_flash->is_ready()) {
        ESP_LOGI(TAG, "PSRAM 初始化成功");
    } else {
        ESP_LOGW(TAG, "PSRAM 初始化失败: %s, continue with internal RAM/SPIFFS",
                 esp_err_to_name(psram_ret));
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

#if CONFIG_APP_SR_ENABLE
    if (mic && mic->app_mic_check_module()) {
        app_sr_bind_mic(mic);
        esp_err_t sr_ret = app_sr_start(on_sr_event);
        if (sr_ret != ESP_OK) {
            ESP_LOGW(TAG, "ESP-SR start failed: %s", esp_err_to_name(sr_ret));
        } else {
            ESP_LOGI(TAG, "ESP-SR running — wake: 你好小智; commands: 救命/报警/留言");
        }
    }
#endif

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

    ESP_LOGI(TAG, "开始循环: 每60s本地采集一次(IMU和音频全程采集), 同步写flash");

    int seq_id = 0;
    while (1) {
        seq_id++;
        ESP_LOGI(TAG, "===== 第 %d 轮采集开始 =====", seq_id);

        /* 采集数据 */
        uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);
        app_mic_reset_ring();
        if (mpu6050) {
            mpu6050->reset_fall_detector();
        }
        bool fall_detected = false;
        int n = collect_to_buffers(acc_buf, gyro_buf, audio_buf,
                                   &fall_detected);

        uint32_t t1 = (uint32_t)(esp_timer_get_time() / 1000);
        int duration_ms = (int)(t1 - t0);

        /* 获取音频 */
        size_t audio_n = app_mic_get_upload_pcm(audio_buf, SAMPLES_PER_UPLOAD);

        ESP_LOGI(TAG, "本轮: %d IMU样本, %u 音频样本, 耗时 %d ms",
                 n, (unsigned)audio_n, duration_ms);

        if (n == 0) {
            ESP_LOGW(TAG, "无 IMU 数据, 跳过本轮处理");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        double use_lat = 0, use_lng = 0;
        bool has_gps_fix = false;
        read_location_for_upload(&use_lat, &use_lng, &has_gps_fix);

        int battery = read_battery_for_upload();

        uint64_t ts_ms = esp_timer_get_time() / 1000;
        int json_len = build_periodic_json(
            json_buf, JSON_BUF_SIZE, get_device_id(),
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

        /* 发送到服务器 */
        char upload_url[128];
        snprintf(upload_url, sizeof(upload_url),
                 "http://%s:%d%s", SERVER_IP, HTTP_PORT, HTTP_PATH_DATA);
        esp_err_t upload_err = app_http_send_data(upload_url, json_buf, (size_t)json_len);
        if (upload_err != ESP_OK) {
            ESP_LOGE(TAG, "上传失败: %s", esp_err_to_name(upload_err));
        } else {
            ESP_LOGI(TAG, "上传成功 (%d bytes)", json_len);
        }

        log_round_summary(n, audio_n, audio_buf, acc_buf, gyro_buf,
                          battery, has_gps_fix, use_lat, use_lng,
                          json_len);
        ESP_LOGI(TAG, "第 %d 轮采集完成", seq_id);

        /* 计算到下一轮需要等待的时间 */
        uint32_t t2 = (uint32_t)(esp_timer_get_time() / 1000);
        int elapsed_since_start = (int)(t2 - t0);
        int wait_ms = COLLECT_INTERVAL_MS - elapsed_since_start;
        if (wait_ms < 0) wait_ms = 0;
        if (wait_ms > 0) {
            ESP_LOGI(TAG, "等待 %d ms 到下一轮...", wait_ms);
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }

        app_battery_led_update();
    }
}
