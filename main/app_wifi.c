#include "app_wifi.h"

#include "esp_wifi.h"

#include "esp_event.h"

#include "esp_log.h"

#include "nvs_flash.h"

#include "esp_http_client.h"

#include "esp_err.h"

#include "cJSON.h"

#include "mbedtls/base64.h"



static const char *TAG = "app_wifi";



static bool s_wifi_connected = false;

static esp_http_client_handle_t s_http_client = NULL;



static void event_handler(void* arg, esp_event_base_t event_base,

                          int32_t event_id, void* event_data) {

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {

        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {

        s_wifi_connected = false;

        ESP_LOGI(TAG, "WiFi disconnected, reconnecting...");

        esp_wifi_connect();

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;

        s_wifi_connected = true;

        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));

    }

}



void app_wifi_init_sta(const char* ssid, const char* password) {

    ESP_ERROR_CHECK(nvs_flash_init());



    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());



    esp_netif_create_default_wifi_sta();



    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));



    esp_event_handler_instance_t instance_any_id;

    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,

                                                        ESP_EVENT_ANY_ID,

                                                        &event_handler,

                                                        NULL,

                                                        &instance_any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,

                                                        IP_EVENT_STA_GOT_IP,

                                                        &event_handler,

                                                        NULL,

                                                        &instance_got_ip));



    wifi_config_t wifi_config = {

        .sta = {

            .ssid = "",

            .password = "",

            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,

            .pmf_cfg = {

                .capable = true,

                .required = false

            }

        }

    };

    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);

    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);



    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());



    ESP_LOGI(TAG, "WiFi STA initialized, connecting to %s (async)", ssid);
}



static esp_err_t http_client_reconnect(void) {

    if (s_http_client) {

        esp_http_client_cleanup(s_http_client);

        s_http_client = NULL;

    }



    char url[128];

    snprintf(url, sizeof(url), "http://%s:%d", SERVER_IP, HTTP_PORT);



    esp_http_client_config_t config = {

        .url = url,

        .method = HTTP_METHOD_POST,

        .timeout_ms = 60000,

        .buffer_size = 8192,

        .buffer_size_tx = 8192,

        .keep_alive_enable = true,

    };



    s_http_client = esp_http_client_init(&config);

    if (!s_http_client) {

        ESP_LOGE(TAG, "Failed to create HTTP client");

        return ESP_FAIL;

    }



    ESP_LOGI(TAG, "HTTP client initialized");

    return ESP_OK;

}



esp_err_t app_http_connect_to_server(void) {

    if (!s_wifi_connected) {

        ESP_LOGE(TAG, "WiFi not connected");

        return ESP_ERR_HTTP_CONNECT;

    }



    ESP_LOGI(TAG, "HTTP server: %s:%d", SERVER_IP, HTTP_PORT);

    return http_client_reconnect();

}



/* 写一个 chunk: <hex_len>\r\n<data>\r\n */
static esp_err_t http_write_chunk(esp_http_client_handle_t client,
                                   const char *data, size_t len) {
    char hdr[16];
    int hdr_len = snprintf(hdr, sizeof(hdr), "%X\r\n", (unsigned)len);
    if (esp_http_client_write(client, hdr, hdr_len) < 0) return ESP_FAIL;
    if (len > 0 && esp_http_client_write(client, data, (int)len) < 0) return ESP_FAIL;
    if (esp_http_client_write(client, "\r\n", 2) < 0) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t app_http_send_data(const char* path, const char* data, size_t data_len) {

    if (!s_wifi_connected) {

        ESP_LOGE(TAG, "WiFi not connected");

        return ESP_ERR_HTTP_CONNECT;

    }



    if (!s_http_client) {

        esp_err_t err = http_client_reconnect();

        if (err != ESP_OK) {

            return err;

        }

    }



    esp_http_client_set_url(s_http_client, path);

    esp_http_client_set_method(s_http_client, HTTP_METHOD_POST);

    esp_http_client_set_header(s_http_client, "Content-Type", "application/json");

    esp_http_client_set_header(s_http_client, "Transfer-Encoding", "chunked");

    esp_http_client_set_header(s_http_client, "Connection", "keep-alive");



    esp_err_t err = esp_http_client_open(s_http_client, 0);

    if (err != ESP_OK) {

        ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(err));

        http_client_reconnect();

        return err;

    }



    /* 分块写入, 每块 4KB, 避免大块导致 TCP 窗口阻塞 */

    size_t offset = 0;

    const size_t max_chunk = 4096;

    while (offset < data_len) {

        size_t n = data_len - offset;

        if (n > max_chunk) n = max_chunk;

        if (http_write_chunk(s_http_client, data + offset, n) != ESP_OK) {

            ESP_LOGE(TAG, "Chunk write failed at offset %u", (unsigned)offset);

            esp_http_client_close(s_http_client);

            http_client_reconnect();

            return ESP_FAIL;

        }

        offset += n;

    }



    /* 结束块 */

    if (http_write_chunk(s_http_client, NULL, 0) != ESP_OK) {

        ESP_LOGE(TAG, "Final chunk write failed");

        esp_http_client_close(s_http_client);

        http_client_reconnect();

        return ESP_FAIL;

    }



    int content_length = esp_http_client_fetch_headers(s_http_client);

    int status_code = esp_http_client_get_status_code(s_http_client);

    esp_http_client_close(s_http_client);



    if (status_code > 0) {

        ESP_LOGI(TAG, "HTTP chunked OK, status: %d, resp_len: %d", status_code, content_length);

        return ESP_OK;

    } else {

        ESP_LOGE(TAG, "HTTP chunked failed, status: %d", status_code);

        return ESP_FAIL;

    }

}



static cJSON *build_xyz_array(float *data, int float_len) {

    cJSON *arr = cJSON_CreateArray();

    if (!arr) {

        return NULL;

    }

    for (int i = 0; i < float_len; i += 3) {

        cJSON *item = cJSON_CreateArray();

        if (!item) {

            continue;

        }

        cJSON_AddItemToArray(item, cJSON_CreateNumber(data[i]));

        cJSON_AddItemToArray(item, cJSON_CreateNumber(data[i + 1]));

        cJSON_AddItemToArray(item, cJSON_CreateNumber(data[i + 2]));

        cJSON_AddItemToArray(arr, item);

    }

    return arr;

}



static void add_location(cJSON *root, double lat, double lng) {

    cJSON *location = cJSON_CreateObject();

    cJSON_AddNumberToObject(location, "lat", lat);

    cJSON_AddNumberToObject(location, "lng", lng);

    cJSON_AddItemToObject(root, "location", location);

}



char *encode_audio_base64(const int16_t *audio_data, int audio_samples) {

    if (!audio_data || audio_samples <= 0) {

        return NULL;

    }

    size_t raw_len = (size_t)audio_samples * sizeof(int16_t);

    size_t base64_len = ((raw_len + 2) / 3) * 4 + 1;

    char *base64_audio = (char *)malloc(base64_len);

    if (!base64_audio) {

        return NULL;

    }

    size_t out_len = 0;

    if (mbedtls_base64_encode((unsigned char *)base64_audio, base64_len, &out_len,

                              (const unsigned char *)audio_data, raw_len) != 0) {

        free(base64_audio);

        return NULL;

    }

    base64_audio[out_len] = '\0';

    return base64_audio;

}



static void add_audio_format(cJSON *parent) {

    cJSON *fmt = cJSON_CreateObject();

    cJSON_AddNumberToObject(fmt, "sr", MIC_UPLOAD_SAMPLE_RATE);

    cJSON_AddNumberToObject(fmt, "bit", 16);

    cJSON_AddItemToObject(parent, "audio_format", fmt);

}



esp_err_t app_http_send_alarm(int event_id, uint64_t timestamp, int battery,

                              double lat, double lng, int status_confirm,

                              float *acc_data, int acc_len,

                              float *gyro_data, int gyro_len,

                              float *baro_data, int baro_len,

                              int16_t *audio_data, int audio_len) {

    cJSON *root = cJSON_CreateObject();

    if (!root) {

        return ESP_FAIL;

    }



    cJSON_AddStringToObject(root, "type", "alarm");

    cJSON_AddNumberToObject(root, "event_id", event_id);

    cJSON_AddStringToObject(root, "device_id", DEVICE_ID);

    cJSON_AddNumberToObject(root, "timestamp", (double)timestamp);

    cJSON_AddNumberToObject(root, "battery", battery);

    add_location(root, lat, lng);

    cJSON_AddNumberToObject(root, "status_confirm", status_confirm);



    cJSON *payload = cJSON_CreateObject();

    cJSON *snapshot = cJSON_CreateObject();

    cJSON_AddNumberToObject(snapshot, "freq", 50);



    cJSON *acc = build_xyz_array(acc_data, acc_len);

    cJSON *gyro = build_xyz_array(gyro_data, gyro_len);

    if (acc) {

        cJSON_AddItemToObject(snapshot, "acc", acc);

    }

    if (gyro) {

        cJSON_AddItemToObject(snapshot, "gyro", gyro);

    }



    cJSON *baro = cJSON_CreateArray();

    for (int i = 0; i < baro_len; i++) {

        cJSON_AddItemToArray(baro, cJSON_CreateNumber(baro_data[i]));

    }

    cJSON_AddItemToObject(snapshot, "baro", baro);

    cJSON_AddItemToObject(payload, "sensor_snapshot", snapshot);



    char *audio_b64 = encode_audio_base64(audio_data, audio_len);

    if (audio_b64) {

        cJSON_AddStringToObject(payload, "audio_clip", audio_b64);

        free(audio_b64);

    }

    add_audio_format(payload);

    cJSON_AddNumberToObject(payload, "is_abnormal", 1);

    cJSON_AddItemToObject(root, "payload", payload);



    char url[128];

    snprintf(url, sizeof(url), "http://%s:%d/api/alarm", SERVER_IP, HTTP_PORT);

    char *json_str = cJSON_PrintUnformatted(root);

    esp_err_t err = app_http_send_data(url, json_str, strlen(json_str));



    cJSON_Delete(root);

    free(json_str);

    return err;

}



esp_err_t app_http_send_message(uint64_t timestamp, int battery, double lat, double lng,

                                int16_t *audio_data, int audio_len) {

    cJSON *root = cJSON_CreateObject();

    if (!root) {

        return ESP_FAIL;

    }



    cJSON_AddStringToObject(root, "type", "message");

    cJSON_AddStringToObject(root, "device_id", DEVICE_ID);

    cJSON_AddNumberToObject(root, "timestamp", (double)timestamp);

    cJSON_AddNumberToObject(root, "battery", battery);

    add_location(root, lat, lng);



    cJSON *payload = cJSON_CreateObject();

    char *audio_b64 = encode_audio_base64(audio_data, audio_len);

    if (audio_b64) {

        cJSON_AddStringToObject(payload, "audio_data", audio_b64);

        free(audio_b64);

    }

    add_audio_format(payload);

    cJSON_AddItemToObject(root, "payload", payload);



    char url[128];

    snprintf(url, sizeof(url), "http://%s:%d/api/message", SERVER_IP, HTTP_PORT);

    char *json_str = cJSON_PrintUnformatted(root);

    esp_err_t err = app_http_send_data(url, json_str, strlen(json_str));



    cJSON_Delete(root);

    free(json_str);

    return err;

}



esp_err_t app_http_send_periodic(uint64_t timestamp, int seq_id, int battery,

                                 double lat, double lng, int duration_ms,

                                 float *acc_data, int acc_len,

                                 float *gyro_data, int gyro_len,

                                 float *baro_data, int baro_len,

                                 int16_t *audio_data, int audio_len) {

    cJSON *root = cJSON_CreateObject();

    if (!root) {

        return ESP_FAIL;

    }



    cJSON_AddStringToObject(root, "device_id", DEVICE_ID);

    cJSON_AddNumberToObject(root, "timestamp", (double)timestamp);

    cJSON_AddNumberToObject(root, "seq_id", seq_id);

    cJSON_AddNumberToObject(root, "battery", battery);

    add_location(root, lat, lng);



    cJSON *payload = cJSON_CreateObject();

    cJSON_AddNumberToObject(payload, "duration_ms", duration_ms);



    cJSON *sensor_data = cJSON_CreateObject();

    cJSON *acc = build_xyz_array(acc_data, acc_len);

    cJSON *gyro = build_xyz_array(gyro_data, gyro_len);

    if (acc) {

        cJSON_AddItemToObject(sensor_data, "acc", acc);

    }

    if (gyro) {

        cJSON_AddItemToObject(sensor_data, "gyro", gyro);

    }



    cJSON *baro = cJSON_CreateArray();

    for (int i = 0; i < baro_len; i++) {

        cJSON_AddItemToArray(baro, cJSON_CreateNumber(baro_data[i]));

    }

    cJSON_AddItemToObject(sensor_data, "baro", baro);

    cJSON_AddItemToObject(payload, "sensor_data", sensor_data);



    char *audio_b64 = encode_audio_base64(audio_data, audio_len);

    if (audio_b64) {

        cJSON_AddStringToObject(payload, "audio_data", audio_b64);

        free(audio_b64);

        add_audio_format(payload);

    }

    cJSON_AddNumberToObject(payload, "is_abnormal", 0);

    cJSON_AddItemToObject(root, "payload", payload);



    char url[128];

    snprintf(url, sizeof(url), "http://%s:%d/api/data", SERVER_IP, HTTP_PORT);

    char *json_str = cJSON_PrintUnformatted(root);

    ESP_LOGI(TAG, "Sending periodic JSON, %d bytes", (int)strlen(json_str));



    esp_err_t err = app_http_send_data(url, json_str, strlen(json_str));



    cJSON_Delete(root);

    free(json_str);

    return err;

}


