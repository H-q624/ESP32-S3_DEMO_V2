#ifndef _APP_WIFI_H_
#define _APP_WIFI_H_

#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 服务器配置
#define SERVER_IP "182.92.156.138"
#define HTTP_PORT 8007
#define MQTT_PORT 1883

/* HTTP 上传路径 (与服务器协议一致) */
#define HTTP_PATH_DATA     "/api/data"
#define HTTP_PATH_ALARM    "/api/alarm"
#define HTTP_PATH_MESSAGE  "/api/message"

/* IMU 采样率 / 音频 PCM 采样率 */
#define IMU_SAMPLE_RATE_HZ     50
#define AUDIO_PCM_SAMPLE_RATE  16000

// 设备配置 (ICCID 运行时从 ML307R 读取, 此为 fallback)
#define DEVICE_ID "898604293624D0028511"
#define DEVICE_IMEI "898604293624D0028511"

const char *app_get_device_id(void);

/* 与 IMU 同步降采样后的上传音频包采样率 (非 PCM 原始采样率) */
#ifndef MIC_UPLOAD_SAMPLE_RATE
#define MIC_UPLOAD_SAMPLE_RATE IMU_SAMPLE_RATE_HZ
#endif

/* 网络模式选择 */
typedef enum {
    NET_MODE_WIFI = 0,
    NET_MODE_ML307R = 1,  /* 4G 模组 */
} network_mode_t;

/**
 * @brief 统一网络初始化 (自动选择 WiFi 或 ML307R)
 * @param mode   网络模式
 * @param param1 WiFi: ssid / ML307R: 不使用
 * @param param2 WiFi: password / ML307R: 不使用
 */
void app_network_init(network_mode_t mode, const char* param1, const char* param2);

/**
 * @brief 检查当前网络是否已连接
 */
bool app_network_is_connected(void);

/**
 * @brief [兼容] Initialize WiFi station mode and connect to AP
 */
void app_wifi_init_sta(const char* ssid, const char* password);

/**
 * @brief [兼容] Connect to HTTP server and send test request
 */
esp_err_t app_http_connect_to_server();

/**
 * @brief Send data to HTTP server
 * 
 * @param path Request path
 * @param data Data to send
 * @param data_len Length of data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t app_http_send_data(const char* path, const char* data, size_t data_len);

/**
 * @brief Send alarm event to server (异常触发包)
 * 
 * @param event_id Event ID
 * @param timestamp Timestamp
 * @param battery Battery level (0-100)
 * @param lat Latitude
 * @param lng Longitude
 * @param status_confirm 0:auto, 1:manual
 * @param acc_data Accelerometer data array
 * @param acc_len Number of acc samples
 * @param gyro_data Gyroscope data array
 * @param gyro_len Number of gyro samples
 * @param audio_data Audio data (PCM)
 * @param audio_len Length of audio data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t app_http_send_alarm(const char *event, const char *trigger,
                              float confidence, uint64_t timestamp, int seq,
                              int battery,
                              double lat, double lng,
                              float* acc_data, int acc_len,
                              float* gyro_data, int gyro_len,
                              float* baro_data, int baro_len,
                              int duration_ms,
                              const char *audio_ref);

esp_err_t app_http_send_message(uint64_t timestamp, int seq, int battery,
                                double lat, double lng,
                                int duration_ms, const char *audio_ref);

/**
 * @brief Encode audio PCM data to base64 string
 * Caller must free the returned string.
 */
char *encode_audio_base64(const int16_t *audio_data, int audio_samples);

/**
 * @brief Send periodic data to server (持续上传包)
 *
 * @param timestamp Timestamp
 * @param seq_id Sequence ID
 * @param battery Battery level (0-100)
 * @param lat Latitude
 * @param lng Longitude
 * @param duration_ms Duration in milliseconds
 * @param acc_data Accelerometer data array
 * @param acc_len Number of acc samples
 * @param gyro_data Gyroscope data array
 * @param gyro_len Number of gyro samples
 * @param baro_data Barometer data array
 * @param baro_len Number of baro samples
 * @param audio_data Audio data (PCM)
 * @param audio_len Length of audio data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t app_http_send_periodic(uint64_t timestamp, int seq_id, int battery,
                                 double lat, double lng, int duration_ms,
                                 float* acc_data, int acc_len,
                                 float* gyro_data, int gyro_len,
                                 float* baro_data, int baro_len,
                                 int16_t* audio_data, int audio_len);

#ifdef __cplusplus
}
#endif

#endif /* _APP_WIFI_H_ */
