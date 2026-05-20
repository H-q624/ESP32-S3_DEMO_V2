#ifndef _APP_WIFI_H_
#define _APP_WIFI_H_

#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 服务器配置
#define SERVER_IP "182.92.156.138"
#define HTTP_PORT 8007
#define MQTT_PORT 1883

// 设备配置
#define DEVICE_ID "CARD_001"
#define DEVICE_IMEI "CARD360005000"

/* 与 IMU 同步上传的音频采样率（app_mic 模块） */
#ifndef MIC_UPLOAD_SAMPLE_RATE
#define MIC_UPLOAD_SAMPLE_RATE 50
#endif

/**
 * @brief Initialize WiFi station mode and connect to AP
 * 
 * @param ssid WiFi SSID
 * @param password WiFi password
 */
void app_wifi_init_sta(const char* ssid, const char* password);

/**
 * @brief Connect to HTTP server and send test request
 * 
 * @return esp_err_t ESP_OK on success
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
esp_err_t app_http_send_alarm(int event_id, uint64_t timestamp, int battery,
                              double lat, double lng, int status_confirm,
                              float* acc_data, int acc_len,
                              float* gyro_data, int gyro_len,
                              float* baro_data, int baro_len,
                              int16_t* audio_data, int audio_len);

esp_err_t app_http_send_message(uint64_t timestamp, int battery,
                                double lat, double lng,
                                int16_t* audio_data, int audio_len);

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
