#include "app_gps.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

static const char *TAG = "GPS";

/* 全局 GPS 数据 */
gps_data_t g_gps_data = {0};
static portMUX_TYPE gps_spinlock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_gps_uart_queue = nullptr;
static char s_last_gnrmc[128] = {0};

/* ---- NMEA 解析 ---- */

/* 将 ddmm.mmmm 格式转为十进制度数 */
static double nmea_to_degrees(double nmea_val) {
    if (nmea_val < 1.0) return 0.0;
    int deg = (int)(nmea_val / 100.0);
    double minutes = nmea_val - (deg * 100.0);
    return (double)deg + minutes / 60.0;
}

static double nmea_parse_float(const char *s, int len) {
    char buf[32] = {0};
    if (len <= 0 || len >= 32) return 0.0;
    memcpy(buf, s, (size_t)len);
    return atof(buf);
}

static int nmea_split_fields(char *line, char *fields[], int max_fields) {
    if (!line || !fields || max_fields <= 0) return 0;

    int count = 0;
    char *p = line;
    while (count < max_fields) {
        fields[count++] = p;
        char *comma = strchr(p, ',');
        if (!comma) break;

        *comma = '\0';
        p = comma + 1;
    }

    return count;
}

static void gps_mark_invalid(void) {
    portENTER_CRITICAL(&gps_spinlock);
    g_gps_data.valid = false;
    g_gps_data.fix_quality = 0;
    portEXIT_CRITICAL(&gps_spinlock);
}

static int nmea_hex_value(char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

static bool nmea_checksum_valid(const char *line, const char *asterisk) {
    if (!line || line[0] != '$' || !asterisk ||
        asterisk[1] == '\0' || asterisk[2] == '\0') {
        return false;
    }

    uint8_t checksum = 0;
    for (const char *p = line + 1; p < asterisk; ++p) {
        checksum ^= (uint8_t)*p;
    }

    int high = nmea_hex_value(asterisk[1]);
    int low = nmea_hex_value(asterisk[2]);
    return high >= 0 && low >= 0 &&
           checksum == (uint8_t)((high << 4) | low);
}

/*
 * 仅解析 $GNRMC：
 * $GNRMC,092204.000,A,3909.1234,N,11623.5678,E,0.5,180.0,010624,,,D*6A
 */
static bool parse_gnrmc(char *fields[], int count) {
    if (count < 7) {
        gps_mark_invalid();
        return false;
    }

    /* RMC 状态：A=定位有效，V=定位无效。 */
    bool active = (fields[2] && fields[2][0] == 'A');
    if (!active) {
        gps_mark_invalid();
        return false;
    }

    if (!fields[3] || fields[3][0] == '\0' ||
        !fields[4] || (fields[4][0] != 'N' && fields[4][0] != 'S') ||
        !fields[5] || fields[5][0] == '\0' ||
        !fields[6] || (fields[6][0] != 'E' && fields[6][0] != 'W')) {
        gps_mark_invalid();
        return false;
    }

    double lat = nmea_to_degrees(nmea_parse_float(fields[3], (int)strlen(fields[3])));
    if (fields[4][0] == 'S') lat = -lat;

    double lng = nmea_to_degrees(nmea_parse_float(fields[5], (int)strlen(fields[5])));
    if (fields[6][0] == 'W') lng = -lng;

    if (std::fabs(lat) > 90.0 || std::fabs(lng) > 180.0) {
        gps_mark_invalid();
        return false;
    }

    portENTER_CRITICAL(&gps_spinlock);
    g_gps_data.latitude = lat;
    g_gps_data.longitude = lng;
    /* GNRMC 没有 GGA 的 fix_quality 字段，状态 A 对应有效定位。 */
    g_gps_data.fix_quality = 1;
    g_gps_data.valid = true;
    g_gps_data.last_update_ms = (uint32_t)(esp_timer_get_time() / 1000);
    portEXIT_CRITICAL(&gps_spinlock);
    return true;
}

/*
 * 解析一条 NMEA 语句
 */
static bool parse_nmea_line(char *line) {
    if (!line || line[0] != '$') return false;

    /* 去除 \r\n */
    int len = (int)strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) {
        line[--len] = '\0';
    }

    /* 忽略 GPTXT/GGA/GSA/GSV 等所有非 GNRMC 语句。 */
    if (strncmp(line, "$GNRMC,", 7) != 0) {
        return false;
    }

    /* Preserve the complete GNRMC, including checksum, before parsing mutates it. */
    portENTER_CRITICAL(&gps_spinlock);
    strncpy(s_last_gnrmc, line, sizeof(s_last_gnrmc) - 1);
    s_last_gnrmc[sizeof(s_last_gnrmc) - 1] = '\0';
    portEXIT_CRITICAL(&gps_spinlock);

    /* 校验失败时仍保留并打印原始语句，但不使用其中的坐标。 */
    char *ast = strchr(line, '*');
    if (!nmea_checksum_valid(line, ast)) {
        gps_mark_invalid();
        return false;
    }
    *ast = '\0';

    /* 分割字段 */
    char *fields[32] = {0};
    int field_cnt = nmea_split_fields(line, fields, 32);
    if (field_cnt == 0) return false;

    return strcmp(fields[0], "$GNRMC") == 0 &&
           parse_gnrmc(fields, field_cnt);
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

    /* ON/OFF 和 nRESET 都是低有效: 保持高电平表示常开且不复位 */
    gpio_set_level((gpio_num_t)GPS_RST_PIN, 1);
    gpio_set_level((gpio_num_t)GPS_ON_OFF_PIN, 1);

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
                                         20, &s_gps_uart_queue, 0);
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

    ESP_LOGI(TAG, "GPS UART%d interrupt mode: TX=IO%d, RX=IO%d, baud=%d",
             GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUDRATE);
    ESP_LOGI(TAG, "GPS control: ON_OFF=IO%d, RST=IO%d, 1PPS=IO%d",
             GPS_ON_OFF_PIN, GPS_RST_PIN, GPS_1PPS_PIN);

    return ESP_OK;
}

void app_gps_power_on(void) {
    gpio_set_level((gpio_num_t)GPS_RST_PIN, 1);
    gpio_set_level((gpio_num_t)GPS_ON_OFF_PIN, 1);
    ESP_LOGI(TAG, "GPS power ON (ON_OFF=IO%d)", GPS_ON_OFF_PIN);
}

void app_gps_power_off(void) {
    gpio_set_level((gpio_num_t)GPS_RST_PIN, 1);
    gpio_set_level((gpio_num_t)GPS_ON_OFF_PIN, 1);
    ESP_LOGI(TAG, "GPS always-on mode: power off ignored");
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
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    return snap.valid && snap.fix_quality >= 1 &&
           (snap.latitude != 0.0 || snap.longitude != 0.0) &&
           (now_ms - snap.last_update_ms <= 3000);
}

bool app_gps_get_last_gnrmc(char *out, size_t out_size) {
    if (!out || out_size == 0) {
        return false;
    }

    portENTER_CRITICAL(&gps_spinlock);
    bool received = s_last_gnrmc[0] != '\0';
    if (received) {
        strncpy(out, s_last_gnrmc, out_size - 1);
        out[out_size - 1] = '\0';
    } else {
        out[0] = '\0';
    }
    portEXIT_CRITICAL(&gps_spinlock);
    return received;
}

/*
 * GPS 解析任务:
 * UART 驱动在接收中断中写入环形缓冲区并投递事件，本任务被事件唤醒后
 * 批量读取数据、组装 NMEA 行并提取经纬度。
 */
void app_gps_task(void *pvParameters) {
    (void)pvParameters;
    if (s_gps_uart_queue == nullptr) {
        ESP_LOGE(TAG, "GPS UART event queue is not initialized");
        vTaskDelete(nullptr);
        return;
    }

    char line[256];
    int line_pos = 0;
    uint8_t rx_buf[256];
    uint32_t rx_count = 0;
    uint32_t last_rx_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t last_no_data_log_ms = last_rx_ms;

    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        uart_event_t event;
        if (xQueueReceive(s_gps_uart_queue, &event, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (event.type == UART_DATA) {
                size_t remaining = event.size;
                while (remaining > 0) {
                    size_t request = remaining < sizeof(rx_buf) ? remaining : sizeof(rx_buf);
                    int len = uart_read_bytes(GPS_UART_NUM, rx_buf, request, 0);
                    if (len <= 0) {
                        break;
                    }

                    remaining -= (size_t)len;
                    rx_count += (uint32_t)len;
                    last_rx_ms = (uint32_t)(esp_timer_get_time() / 1000);

                    for (int i = 0; i < len; ++i) {
                        uint8_t ch = rx_buf[i];
                        if (ch == '\n') {
                            line[line_pos] = '\0';
                            if (line_pos > 0) {
                                parse_nmea_line(line);
                            }
                            line_pos = 0;
                        } else if (ch != '\r' && line_pos < (int)sizeof(line) - 1) {
                            line[line_pos++] = (char)ch;
                        } else if (line_pos >= (int)sizeof(line) - 1) {
                            line_pos = 0;
                        }
                    }
                }
            } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                ESP_LOGW(TAG, "GPS UART0 RX overflow, resetting buffer");
                uart_flush_input(GPS_UART_NUM);
                xQueueReset(s_gps_uart_queue);
                line_pos = 0;
            } else if (event.type == UART_PARITY_ERR) {
                ESP_LOGW(TAG, "GPS UART0 parity error");
            } else if (event.type == UART_FRAME_ERR) {
                ESP_LOGW(TAG, "GPS UART0 frame error");
            }
        } else {
            if (rx_count == 0 && now_ms - last_no_data_log_ms >= 5000) {
                last_no_data_log_ms = now_ms;
                ESP_LOGW(TAG, "GPS UART0 no data: RX=IO%d baud=%d", GPS_RX_PIN, GPS_BAUDRATE);
            } else if (line_pos > 0 && now_ms - last_rx_ms >= 1000) {
                ESP_LOGW(TAG, "GPS UART0 partial RX discarded");
                line_pos = 0;
            }
        }
    }
}
