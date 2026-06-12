#include "app_gps.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

static const char *TAG = "GPS";

/* 全局 GPS 数据 */
gps_data_t g_gps_data = {0};
static portMUX_TYPE gps_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* ---- NMEA 解析 ---- */

/* 将 ddmm.mmmm 格式转为十进制度数 */
static double nmea_to_degrees(double nmea_val) {
    if (nmea_val < 1.0) return 0.0;
    int deg = (int)(nmea_val / 100.0);
    double minutes = nmea_val - (deg * 100.0);
    return (double)deg + minutes / 60.0;
}

static int nmea_parse_int(const char *s, int len) {
    char buf[16] = {0};
    if (len <= 0 || len >= 16) return 0;
    memcpy(buf, s, (size_t)len);
    return atoi(buf);
}

static double nmea_parse_float(const char *s, int len) {
    char buf[32] = {0};
    if (len <= 0 || len >= 32) return 0.0;
    memcpy(buf, s, (size_t)len);
    return atof(buf);
}

/*
 * 解析 $GPGGA 语句
 * 例: $GPGGA,092204.000,3909.1234,N,11623.5678,E,1,12,1.0,50.5,M,-5.7,M,,*7F
 */
static void parse_gpgga(char *fields[], int count) {
    if (count < 10) return;

    /* 时间 */
    if (fields[1] && strlen(fields[1]) >= 6) {
        g_gps_data.hour   = nmea_parse_int(fields[1] + 0, 2);
        g_gps_data.minute = nmea_parse_int(fields[1] + 2, 2);
        g_gps_data.second = nmea_parse_int(fields[1] + 4, 2);
    }

    /* 纬度 */
    double lat = nmea_to_degrees(nmea_parse_float(fields[2], (int)strlen(fields[2])));
    if (fields[3] && fields[3][0] == 'S') lat = -lat;
    g_gps_data.latitude = lat;

    /* 经度 */
    double lng = nmea_to_degrees(nmea_parse_float(fields[4], (int)strlen(fields[4])));
    if (fields[5] && fields[5][0] == 'W') lng = -lng;
    g_gps_data.longitude = lng;

    /* 定位质量 */
    g_gps_data.fix_quality = (uint8_t)nmea_parse_int(fields[6], (int)strlen(fields[6]));

    /* 卫星数 */
    g_gps_data.satellites = (uint8_t)nmea_parse_int(fields[7], (int)strlen(fields[7]));

    /* 海拔 */
    g_gps_data.altitude = (float)nmea_parse_float(fields[9], (int)strlen(fields[9]));

    if (g_gps_data.fix_quality >= 1) {
        g_gps_data.valid = true;
    }
}

/*
 * 解析 $GPRMC 语句
 * 例: $GPRMC,092204.000,A,3909.1234,N,11623.5678,E,0.5,180.0,010624,,,D*6A
 */
static void parse_gprmc(char *fields[], int count) {
    if (count < 8) return;

    /* 有效标志 */
    bool active = (fields[2] && fields[2][0] == 'A');
    if (!active) {
        g_gps_data.valid = false;
        return;
    }

    /* 速度 (节 -> km/h) */
    g_gps_data.speed_kmh = (float)(nmea_parse_float(fields[7], (int)strlen(fields[7])) * 1.852);

    /* 航向 */
    g_gps_data.heading = (float)nmea_parse_float(fields[8], (int)strlen(fields[8]));

    /* 日期 */
    if (fields[9] && strlen(fields[9]) >= 6) {
        g_gps_data.day   = nmea_parse_int(fields[9] + 0, 2);
        g_gps_data.month = nmea_parse_int(fields[9] + 2, 2);
        g_gps_data.year  = (uint16_t)(2000 + nmea_parse_int(fields[9] + 4, 2));
    }

    g_gps_data.valid = true;
}

/*
 * 解析一条 NMEA 语句
 */
static void parse_nmea_line(char *line) {
    if (!line || line[0] != '$') return;

    /* 去除 \r\n */
    int len = (int)strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) {
        line[--len] = '\0';
    }

    /* 校验和检查 (可选) */
    char *ast = strchr(line, '*');
    if (ast) *ast = '\0'; /* 截断校验和部分 */

    /* 分割字段 */
    char *fields[32] = {0};
    int field_cnt = 0;
    char *tok = strtok(line, ",");
    while (tok && field_cnt < 32) {
        fields[field_cnt++] = tok;
        tok = strtok(NULL, ",");
    }
    if (field_cnt == 0) return;

    /* 根据语句类型分发 */
    if (strcmp(fields[0], "$GPGGA") == 0 || strcmp(fields[0], "$GNGGA") == 0) {
        parse_gpgga(fields, field_cnt);
    } else if (strcmp(fields[0], "$GPRMC") == 0 || strcmp(fields[0], "$GNRMC") == 0) {
        parse_gprmc(fields, field_cnt);
    }
}

/* ---- 公开函数 ---- */

esp_err_t app_gps_init(void) {
    /* ON/OFF + RST 控制引脚 */
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << GPS_ON_OFF_PIN) | (1ULL << GPS_RST_PIN);
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    /* RST 高 = 不复位; 先释放复位再开电源 */
    gpio_set_level((gpio_num_t)GPS_RST_PIN, 1);
    gpio_set_level((gpio_num_t)GPS_ON_OFF_PIN, 0);

    /* 1PPS 引脚 (仅用于检测, 输入) */
    io_conf.pin_bit_mask = (1ULL << GPS_1PPS_PIN);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);

    /* ---- UART 配置 ---- */
    uart_config_t uart_cfg = {
        .baud_rate           = GPS_BAUDRATE,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(GPS_UART_NUM, GPS_UART_BUF_SIZE, GPS_UART_BUF_SIZE,
                                         0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(GPS_UART_NUM, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "GPS UART%d init: TX=IO%d, RX=IO%d, baud=%d",
             GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUDRATE);
    ESP_LOGI(TAG, "GPS control: ON_OFF=IO%d, RST=IO%d, 1PPS=IO%d",
             GPS_ON_OFF_PIN, GPS_RST_PIN, GPS_1PPS_PIN);

    return ESP_OK;
}

void app_gps_power_on(void) {
    gpio_set_level((gpio_num_t)GPS_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level((gpio_num_t)GPS_ON_OFF_PIN, 1);
    ESP_LOGI(TAG, "GPS power ON (ON_OFF=IO%d)", GPS_ON_OFF_PIN);
}

void app_gps_power_off(void) {
    gpio_set_level((gpio_num_t)GPS_ON_OFF_PIN, 0);
    ESP_LOGI(TAG, "GPS power OFF");
}

void app_gps_get_data(gps_data_t *out) {
    if (!out) return;
    portENTER_CRITICAL(&gps_spinlock);
    memcpy(out, &g_gps_data, sizeof(gps_data_t));
    portEXIT_CRITICAL(&gps_spinlock);
}

bool app_gps_has_fix(void) {
    gps_data_t snap;
    app_gps_get_data(&snap);
    return snap.valid && snap.fix_quality >= 1 &&
           (snap.latitude != 0.0 || snap.longitude != 0.0);
}

/*
 * GPS 解析任务: 每 200ms 轮询 UART 缓冲区，逐行解析 NMEA
 */
void app_gps_task(void *pvParameters) {
    (void)pvParameters;
    char line[256];
    int line_pos = 0;

    while (1) {
        uint8_t ch;
        int len = uart_read_bytes(GPS_UART_NUM, &ch, 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            if (ch == '\n') {
                line[line_pos] = '\0';
                if (line_pos > 0) {
                    portENTER_CRITICAL(&gps_spinlock);
                    g_gps_data.last_update_ms = (uint32_t)(esp_timer_get_time() / 1000);
                    portEXIT_CRITICAL(&gps_spinlock);
                    parse_nmea_line(line);
                }
                line_pos = 0;
            } else if (ch != '\r' && line_pos < 255) {
                line[line_pos++] = (char)ch;
            }
        }

        /* 定期打印 GPS 状态 */
        static uint32_t last_print = 0;
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - last_print > 5000) {
            last_print = now;
            gps_data_t snap;
            app_gps_get_data(&snap);
            if (snap.valid) {
                ESP_LOGI(TAG, "Fix: lat=%.5f lng=%.5f alt=%.1fm sats=%d speed=%.1fkm/h",
                         snap.latitude, snap.longitude,
                         (double)snap.altitude, snap.satellites,
                         (double)snap.speed_kmh);
            } else {
                ESP_LOGD(TAG, "No fix yet, sats=%d", snap.satellites);
            }
        }
    }
}
