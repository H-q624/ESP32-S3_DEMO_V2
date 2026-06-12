#include "app_ml307r.h"
#include "app_wifi.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ML307R";

#ifdef CONFIG_ML307R_APN
#undef ML307R_APN
#define ML307R_APN CONFIG_ML307R_APN
#endif

static bool s_initialized = false;
static bool s_network_ready = false;
static char s_iccid[32] = {0};

/* ---- 低层 UART ---- */

static void uart_flush_rx(void) {
    uint8_t dummy[256];
    while (uart_read_bytes(ML307R_UART_NUM, dummy, sizeof(dummy), pdMS_TO_TICKS(10)) > 0) {}
}

static void uart_write_raw(const char *data, size_t len) {
    if (!data || len == 0) return;
    uart_write_bytes(ML307R_UART_NUM, data, (int)len);
}

static void uart_write_cmd(const char *cmd) {
    uart_write_raw(cmd, strlen(cmd));
}

/* 累积 UART 响应, 支持匹配 expect 或 '>' 提示符 */
static bool uart_read_until(const char *expect, int timeout_ms,
                            char *out_buf, size_t out_size,
                            bool wait_prompt) {
    char acc[ML307R_RX_BUF_SIZE];
    int total = 0;
    uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);

    while (1) {
        uint8_t ch;
        int n = uart_read_bytes(ML307R_UART_NUM, &ch, 1, pdMS_TO_TICKS(50));
        if (n > 0) {
            if (total < (int)(sizeof(acc) - 1)) {
                acc[total++] = (char)ch;
                acc[total] = '\0';
            }
            if (out_buf && total <= (int)out_size - 1) {
                out_buf[total - 1] = (char)ch;
                out_buf[total] = '\0';
            }
        }

        if (wait_prompt && total > 0 && acc[total - 1] == '>') {
            return true;
        }
        if (expect && strstr(acc, expect) != NULL) {
            return true;
        }

        uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - t0;
        if (elapsed >= (uint32_t)timeout_ms) {
            if (out_buf && total < (int)out_size) out_buf[total] = '\0';
            if (total > 0) {
                ESP_LOGD(TAG, "RX timeout (%d ms), got: %.120s%s",
                         timeout_ms, acc, total > 120 ? "..." : "");
            }
            return false;
        }
    }
}

bool ml307r_at_send(const char *cmd, const char *expect, int timeout_ms,
                    char *out_buf, size_t out_size) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "ML307R not initialized");
        return false;
    }

    if (out_buf && out_size > 0) out_buf[0] = '\0';
    uart_flush_rx();

    if (cmd && cmd[0] != '\0') {
        ESP_LOGD(TAG, "TX: %.*s", (int)strcspn(cmd, "\r\n"), cmd);
        uart_write_cmd(cmd);
    }

    bool wait_prompt = (expect && strcmp(expect, ">") == 0);
    return uart_read_until(expect, timeout_ms, out_buf, out_size, wait_prompt);
}

static bool ml307r_at_retry(const char *cmd, const char *expect,
                            int timeout_ms, char *buf, size_t buf_size) {
    for (int i = 0; i < ML307R_AT_RETRY; i++) {
        if (ml307r_at_send(cmd, expect, timeout_ms, buf, buf_size)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return false;
}

/* ---- URL 解析 ---- */

typedef struct {
    char host[64];
    char path[128];
    int port;
    bool ssl;
} url_parts_t;

static bool parse_http_url(const char *url, url_parts_t *out) {
    if (!url || !out) return false;
    memset(out, 0, sizeof(*out));
    out->port = 80;

    const char *p = url;
    if (strncmp(p, "https://", 8) == 0) {
        out->ssl = true;
        out->port = 443;
        p += 8;
    } else if (strncmp(p, "http://", 7) == 0) {
        p += 7;
    } else {
        return false;
    }

    const char *slash = strchr(p, '/');
    const char *colon = strchr(p, ':');
    if (colon && (!slash || colon < slash)) {
        size_t hlen = (size_t)(colon - p);
        if (hlen >= sizeof(out->host)) return false;
        memcpy(out->host, p, hlen);
        out->host[hlen] = '\0';
        out->port = atoi(colon + 1);
        if (slash) {
            strncpy(out->path, slash, sizeof(out->path) - 1);
        } else {
            strcpy(out->path, "/");
        }
    } else {
        if (slash) {
            size_t hlen = (size_t)(slash - p);
            if (hlen >= sizeof(out->host)) return false;
            memcpy(out->host, p, hlen);
            out->host[hlen] = '\0';
            strncpy(out->path, slash, sizeof(out->path) - 1);
        } else {
            strncpy(out->host, p, sizeof(out->host) - 1);
            strcpy(out->path, "/");
        }
    }
    if (out->path[0] == '\0') strcpy(out->path, "/");
    return out->host[0] != '\0';
}

/* ---- 初始化 / 电源 ---- */

static void ml307r_set_baud(int baud) {
    uart_set_baudrate(ML307R_UART_NUM, baud);
}

static bool ml307r_wait_boot_banner(int timeout_ms) {
    char acc[160];
    int total = 0;
    uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);

    acc[0] = '\0';
    while ((uint32_t)(esp_timer_get_time() / 1000) - t0 < (uint32_t)timeout_ms) {
        uint8_t ch;
        int n = uart_read_bytes(ML307R_UART_NUM, &ch, 1, pdMS_TO_TICKS(50));
        if (n <= 0) {
            continue;
        }
        if (total < (int)(sizeof(acc) - 1)) {
            acc[total++] = (char)ch;
            acc[total] = '\0';
        }
        if (strstr(acc, "RDY") != NULL || strstr(acc, "+MATREADY") != NULL) {
            ESP_LOGI(TAG, "Module boot: %.100s", acc);
            return true;
        }
    }
    if (total > 0) {
        ESP_LOGI(TAG, "Boot UART data: %.100s", acc);
    }
    return false;
}

static bool ml307r_probe_link(int tx_pin, int rx_pin, int baud) {
    uart_set_pin(ML307R_UART_NUM, tx_pin, rx_pin,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ml307r_set_baud(baud);
    uart_flush_rx();
    return ml307r_at_send("AT\r\n", "OK", 2000, NULL, 0);
}

esp_err_t ml307r_init(void) {
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << ML307R_PWR_PIN) | (1ULL << ML307R_RST_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_cfg);
    /* PWRKEY 空闲为高 (释放); RESET 高 = 不复位 */
    gpio_set_level((gpio_num_t)ML307R_PWR_PIN, 1);
    gpio_set_level((gpio_num_t)ML307R_RST_PIN, 1);

    uart_config_t uart_cfg = {
        .baud_rate  = ML307R_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(ML307R_UART_NUM, ML307R_RX_BUF_SIZE * 2,
                                         ML307R_RX_BUF_SIZE * 2, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed");
        return ret;
    }

    ret = uart_param_config(ML307R_UART_NUM, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed");
        return ret;
    }

    ret = uart_set_pin(ML307R_UART_NUM, ML307R_TX_PIN, ML307R_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed");
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "ML307R UART%d: TX=IO%d RX=IO%d PWR=IO%d RST=IO%d baud=%d APN=%s",
             ML307R_UART_NUM, ML307R_TX_PIN, ML307R_RX_PIN, ML307R_PWR_PIN,
             ML307R_RST_PIN, ML307R_BAUDRATE, ML307R_APN);
    return ESP_OK;
}

void ml307r_power_on(void) {
    ESP_LOGI(TAG, "Hardware reset pulse (RST=IO%d)...", ML307R_RST_PIN);
    gpio_set_level((gpio_num_t)ML307R_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level((gpio_num_t)ML307R_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* ML307R PWRKEY: 拉低 >=1.2s 后释放 (高电平空闲) */
    ESP_LOGI(TAG, "Power on sequence (PWR=IO%d, active-low pulse)...", ML307R_PWR_PIN);
    gpio_set_level((gpio_num_t)ML307R_PWR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level((gpio_num_t)ML307R_PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1500));
    gpio_set_level((gpio_num_t)ML307R_PWR_PIN, 1);

    ESP_LOGI(TAG, "Waiting for module boot (+MATREADY/RDY)...");
    ml307r_wait_boot_banner(8000);
    vTaskDelay(pdMS_TO_TICKS(2000)); /* +MATREADY 后建议等待 2s */

    static const int baud_list[] = {115200, 9600};
    bool linked = false;
    for (int b = 0; b < 2 && !linked; b++) {
        if (ml307r_probe_link(ML307R_TX_PIN, ML307R_RX_PIN, baud_list[b])) {
            ESP_LOGI(TAG, "AT OK: TX=IO%d RX=IO%d baud=%d",
                     ML307R_TX_PIN, ML307R_RX_PIN, baud_list[b]);
            linked = true;
            break;
        }
        ESP_LOGW(TAG, "No AT @ TX=IO%d RX=IO%d baud=%d, try swapped",
                 ML307R_TX_PIN, ML307R_RX_PIN, baud_list[b]);
        if (ml307r_probe_link(ML307R_RX_PIN, ML307R_TX_PIN, baud_list[b])) {
            ESP_LOGW(TAG, "AT OK with swapped UART: TX=IO%d RX=IO%d baud=%d",
                     ML307R_RX_PIN, ML307R_TX_PIN, baud_list[b]);
            linked = true;
            break;
        }
    }

    if (!linked) {
        for (int i = 0; i < 5 && !linked; i++) {
            ESP_LOGW(TAG, "No AT response, retry %d/5", i + 1);
            vTaskDelay(pdMS_TO_TICKS(2000));
            if (ml307r_probe_link(ML307R_TX_PIN, ML307R_RX_PIN, ML307R_BAUDRATE)) {
                linked = true;
            }
        }
    }

    if (linked) {
        ESP_LOGI(TAG, "Module alive");
        s_network_ready = false;
    } else {
        ESP_LOGE(TAG, "Module not responding after power-on");
    }

    ml307r_release_pwr_pin();
}

void ml307r_release_pwr_pin(void) {
    gpio_set_level((gpio_num_t)ML307R_PWR_PIN, 1);
#if defined(CONFIG_BATTERY_ADC_GPIO) && defined(CONFIG_ML307R_PWR_PIN)
    if (CONFIG_BATTERY_ADC_GPIO == CONFIG_ML307R_PWR_PIN) {
        gpio_reset_pin((gpio_num_t)ML307R_PWR_PIN);
        ESP_LOGI(TAG, "PWR IO%d released for battery ADC", ML307R_PWR_PIN);
    }
#endif
}

void ml307r_power_off(void) {
    ml307r_at_send("AT+MIPCALL=0,1\r\n", "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);
    /* PWRKEY 再次拉低关机 */
    gpio_set_level((gpio_num_t)ML307R_PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1500));
    gpio_set_level((gpio_num_t)ML307R_PWR_PIN, 1);
    s_network_ready = false;
}

bool ml307r_is_alive(void) {
    return ml307r_at_send("AT\r\n", "OK", ML307R_AT_SHORT_TIMEOUT, NULL, 0);
}

int ml307r_get_csq(void) {
    char buf[128] = {0};
    if (!ml307r_at_send("AT+CSQ\r\n", "+CSQ:", ML307R_AT_SHORT_TIMEOUT, buf, sizeof(buf))) {
        return 99;
    }
    char *p = strstr(buf, "+CSQ:");
    if (!p) return 99;
    p += 5;
    while (*p == ' ') p++;
    return atoi(p);
}

static void ml307r_parse_iccid(const char *resp) {
    const char *p = resp;
    while (*p) {
        if ((*p >= '0' && *p <= '9') ||
            (*p >= 'A' && *p <= 'F') ||
            (*p >= 'a' && *p <= 'f')) {
            size_t i = 0;
            while (*p && i < sizeof(s_iccid) - 1 &&
                   ((*p >= '0' && *p <= '9') ||
                    (*p >= 'A' && *p <= 'F') ||
                    (*p >= 'a' && *p <= 'f'))) {
                s_iccid[i++] = *p++;
            }
            s_iccid[i] = '\0';
            if (i >= 15) return;
        } else {
            p++;
        }
    }
}

static void ml307r_log_iccid(void) {
    char buf[128] = {0};
    const char *cmds[] = {"AT+ICCID\r\n", "AT+CICCID\r\n", "AT+QCCID\r\n"};
    for (int i = 0; i < 3; i++) {
        if (ml307r_at_send(cmds[i], "ICCID", ML307R_AT_SHORT_TIMEOUT, buf, sizeof(buf))) {
            ml307r_parse_iccid(buf);
            ESP_LOGI(TAG, "SIM/eSIM ICCID: %s", s_iccid[0] ? s_iccid : buf);
            if (s_iccid[0] && strstr(s_iccid, ML307R_ESIM_ICCID) != NULL) {
                ESP_LOGI(TAG, "ICCID matched expected eSIM");
            } else if (s_iccid[0]) {
                ESP_LOGW(TAG, "ICCID mismatch (expected prefix %s)", ML307R_ESIM_ICCID);
            }
            return;
        }
    }
    ESP_LOGW(TAG, "Could not read ICCID");
}

bool ml307r_get_iccid(char *buf, size_t buf_size) {
    if (!buf || buf_size == 0) return false;
    if (s_iccid[0] == '\0') {
        ml307r_log_iccid();
    }
    if (s_iccid[0] == '\0') return false;
    strncpy(buf, s_iccid, buf_size - 1);
    buf[buf_size - 1] = '\0';
    return true;
}

static bool ml307r_activate_pdp(void) {
    char buf[256] = {0};
    char cmd[96];

    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IPV4V6\",\"%s\"\r\n", ML307R_APN);
    if (!ml307r_at_retry(cmd, "OK", ML307R_AT_TIMEOUT_MS, NULL, 0)) {
        ESP_LOGW(TAG, "CGDCONT failed, trying IPV4");
        snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", ML307R_APN);
        ml307r_at_retry(cmd, "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);
    }

    /* 关闭已有 PDP 再激活 */
    ml307r_at_send("AT+MIPCALL=0,1\r\n", "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    if (ml307r_at_send("AT+MIPCALL=1,1\r\n", "+MIPCALL:", ML307R_NETWORK_TIMEOUT_MS,
                       buf, sizeof(buf))) {
        ESP_LOGI(TAG, "PDP active: %s", buf);
        return true;
    }

    /* 部分固件只返回 OK */
    if (ml307r_at_send("AT+MIPCALL=1,1\r\n", "OK", ML307R_NETWORK_TIMEOUT_MS, buf, sizeof(buf))) {
        ESP_LOGI(TAG, "PDP activated (OK)");
        return true;
    }

    return false;
}

bool ml307r_wait_network(int timeout_ms) {
    char buf[256] = {0};
    uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);
    int last_csq = -1;

    ml307r_at_send("ATE0\r\n", "OK", ML307R_AT_SHORT_TIMEOUT, NULL, 0);
    ml307r_at_send("AT+IPR=115200\r\n", "OK", ML307R_AT_SHORT_TIMEOUT, NULL, 0);

    ESP_LOGI(TAG, "Waiting for SIM/eSIM (expect ICCID %s)...", ML307R_ESIM_ICCID);

    while (1) {
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - t0;

        if (ml307r_at_send("AT+CPIN?\r\n", "READY", ML307R_AT_SHORT_TIMEOUT,
                           buf, sizeof(buf))) {
            ESP_LOGI(TAG, "SIM/eSIM ready");
            ml307r_log_iccid();

            if (ml307r_activate_pdp()) {
                s_network_ready = true;
                int csq = ml307r_get_csq();
                ESP_LOGI(TAG, "4G online, CSQ=%d (%lu ms)", csq, (unsigned long)elapsed);
                return true;
            }
        } else if (elapsed < 5000) {
            ESP_LOGW(TAG, "Waiting for SIM/eSIM READY...");
        }

        int csq = ml307r_get_csq();
        if (csq != last_csq && csq != 99) {
            ESP_LOGI(TAG, "CSQ: %d", csq);
            last_csq = csq;
        }

        if ((int)elapsed >= timeout_ms) {
            ESP_LOGE(TAG, "Network registration timeout");
            s_network_ready = false;
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

bool ml307r_is_connected(void) {
    if (!s_network_ready) return false;
    char buf[64] = {0};
    if (ml307r_at_send("AT+MIPCALL?\r\n", "+MIPCALL:", ML307R_AT_SHORT_TIMEOUT,
                       buf, sizeof(buf))) {
        return strstr(buf, ",1,") != NULL || strstr(buf, ",1\r") != NULL;
    }
    return s_network_ready;
}

/* ---- TCP 裸 HTTP POST (支持大 JSON) ---- */

static int parse_http_status(const char *resp) {
    if (!resp) return 0;
    const char *p = strstr(resp, "HTTP/1.");
    if (!p) {
        p = strstr(resp, "HTTP/2");
    }
    if (!p) return 0;
    p = strchr(p, ' ');
    if (!p) return 0;
    while (*p == ' ') p++;
    return atoi(p);
}

static esp_err_t ml307r_http_post_tcp(const url_parts_t *url,
                                      const char *data, size_t data_len) {
    char cmd[160];
    char header[512];
    char resp[ML307R_RX_BUF_SIZE];

    if (url->ssl) {
        ESP_LOGE(TAG, "HTTPS not supported via TCP path, use http://");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "TCP POST %s:%d%s (%u bytes)", url->host, url->port, url->path,
             (unsigned)data_len);

    ml307r_at_send("AT+MIPCLOSE=0\r\n", "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    snprintf(cmd, sizeof(cmd), "AT+MIPOPEN=0,\"TCP\",\"%s\",%d\r\n", url->host, url->port);
    if (!ml307r_at_send(cmd, "OK", ML307R_TCP_OPEN_TIMEOUT, resp, sizeof(resp))) {
        ESP_LOGE(TAG, "MIPOPEN failed: %.80s", resp);
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    int hdr_len = snprintf(header, sizeof(header),
        "POST %s HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n"
        "\r\n",
        url->path, url->host, url->port, (unsigned)data_len);
    if (hdr_len <= 0 || hdr_len >= (int)sizeof(header)) {
        ml307r_at_send("AT+MIPCLOSE=0\r\n", "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);
        return ESP_FAIL;
    }

    size_t total = (size_t)hdr_len + data_len;
    char send_cmd[48];
    snprintf(send_cmd, sizeof(send_cmd), "AT+MIPSEND=0,%u\r\n", (unsigned)total);
    if (!ml307r_at_send(send_cmd, ">", ML307R_AT_TIMEOUT_MS, NULL, 0)) {
        ESP_LOGE(TAG, "MIPSEND prompt failed");
        ml307r_at_send("AT+MIPCLOSE=0\r\n", "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);
        return ESP_FAIL;
    }

    uart_write_raw(header, (size_t)hdr_len);
    uart_write_raw(data, data_len);

    resp[0] = '\0';
    bool got_resp = uart_read_until("+MIPURC:", ML307R_HTTP_TIMEOUT_MS, resp, sizeof(resp), false);
    if (!got_resp) {
        got_resp = uart_read_until("HTTP/1.", ML307R_HTTP_TIMEOUT_MS, resp, sizeof(resp), false);
    }

    int status = parse_http_status(resp);
    ESP_LOGI(TAG, "TCP response status=%d, snippet: %.120s", status,
             resp[0] ? resp : "(empty)");

    ml307r_at_send("AT+MIPCLOSE=0\r\n", "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);

    if (status >= 200 && status < 300) {
        return ESP_OK;
    }
    if (status == 0 && got_resp) {
        /* 部分服务器响应格式不同, 有数据即认为成功 */
        return ESP_OK;
    }
    return ESP_FAIL;
}

/* ---- 公开 HTTP POST ---- */

esp_err_t ml307r_http_post(const char *url, const char *data, size_t data_len) {
    if (!s_network_ready) {
        ESP_LOGW(TAG, "Network flag false, retry PDP...");
        if (!ml307r_activate_pdp()) {
            ESP_LOGE(TAG, "Network not ready");
            return ESP_FAIL;
        }
        s_network_ready = true;
    }

    url_parts_t parts;
    if (!parse_http_url(url, &parts)) {
        ESP_LOGE(TAG, "Invalid URL: %s", url);
        return ESP_ERR_INVALID_ARG;
    }

    /* 持续上传 JSON 体积大 (~65KB), 统一走 TCP 裸 HTTP */
    return ml307r_http_post_tcp(&parts, data, data_len);
}

/* ---- 协议 JSON 上传 (持续包 / 报警 / 留言) ---- */

static esp_err_t ml307r_post_api(const char *api_path, const char *json, size_t len) {
    char url[160];
    snprintf(url, sizeof(url), "http://%s:%d%s", SERVER_IP, HTTP_PORT, api_path);
    ESP_LOGI(TAG, "ML307R POST %s (%u bytes)", api_path, (unsigned)len);
    return ml307r_http_post(url, json, len);
}

esp_err_t ml307r_upload_data(const char *json, size_t len) {
    return ml307r_post_api(HTTP_PATH_DATA, json, len);
}

esp_err_t ml307r_upload_alarm(const char *json, size_t len) {
    return ml307r_post_api(HTTP_PATH_ALARM, json, len);
}

esp_err_t ml307r_upload_message(const char *json, size_t len) {
    return ml307r_post_api(HTTP_PATH_MESSAGE, json, len);
}
