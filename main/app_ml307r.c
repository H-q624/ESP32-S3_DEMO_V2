#include "app_ml307r.h"
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
/*
 * AT responses can be up to ML307R_RX_BUF_SIZE bytes.  Do not put this
 * buffer on app_main's stack: the previous 4096-byte local array was larger
 * than CONFIG_ESP_MAIN_TASK_STACK_SIZE (3584) and corrupted the task stack.
 * All AT access is serialized by the current modem implementation.
 */
static char s_at_response[ML307R_RX_BUF_SIZE];

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
    int total = 0;
    uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);

    s_at_response[0] = '\0';
    while (1) {
        uint8_t ch;
        int n = uart_read_bytes(ML307R_UART_NUM, &ch, 1, pdMS_TO_TICKS(50));
        if (n > 0) {
            if (total < (int)(sizeof(s_at_response) - 1)) {
                s_at_response[total++] = (char)ch;
                s_at_response[total] = '\0';
            }
            if (out_buf && total <= (int)out_size - 1) {
                out_buf[total - 1] = (char)ch;
                out_buf[total] = '\0';
            }
        }

        if (wait_prompt && total > 0 && s_at_response[total - 1] == '>') {
            return true;
        }
        if (expect && strstr(s_at_response, expect) != NULL) {
            return true;
        }

        uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - t0;
        if (elapsed >= (uint32_t)timeout_ms) {
            if (out_buf && total < (int)out_size) out_buf[total] = '\0';
            if (total > 0) {
                ESP_LOGD(TAG, "RX timeout (%d ms), got: %.120s%s",
                         timeout_ms, s_at_response, total > 120 ? "..." : "");
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
            if (s_iccid[0]) {
                ESP_LOGI(TAG, "Using eSIM ICCID: %s", s_iccid);
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

bool ml307r_send_hello(const char *server_ip, uint16_t server_port) {
    static const int socket_id = 0;
    static const char payload[] = "hello";
    char cmd[96];
    char response[256];

    if (!server_ip || server_ip[0] == '\0' || server_port == 0 || !s_network_ready) {
        return false;
    }

    /* 清理可能由上次异常退出遗留的 0 号 socket。 */
    snprintf(cmd, sizeof(cmd), "AT+MIPCLOSE=%d\r\n", socket_id);
    ml307r_at_send(cmd, "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);

    snprintf(cmd, sizeof(cmd), "AT+MIPOPEN=%d,\"TCP\",\"%s\",%u\r\n",
             socket_id, server_ip, (unsigned)server_port);
    if (!ml307r_at_send(cmd, "OK", 10000, response, sizeof(response))) {
        return false;
    }

    /* MIPOPEN 在部分固件上异步完成，通过状态查询确认真正连接成功。 */
    bool connected = false;
    for (int i = 0; i < 5; ++i) {
        snprintf(cmd, sizeof(cmd), "AT+MIPSTATE=%d\r\n", socket_id);
        if (ml307r_at_send(cmd, "CONNECTED", 2000, response, sizeof(response))) {
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!connected) {
        snprintf(cmd, sizeof(cmd), "AT+MIPCLOSE=%d\r\n", socket_id);
        ml307r_at_send(cmd, "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);
        return false;
    }

    snprintf(cmd, sizeof(cmd), "AT+MIPSEND=%d,%u\r\n",
             socket_id, (unsigned)(sizeof(payload) - 1));
    bool sent = false;
    if (ml307r_at_send(cmd, ">", ML307R_AT_TIMEOUT_MS, NULL, 0)) {
        uart_write_raw(payload, sizeof(payload) - 1);
        sent = uart_read_until("OK", ML307R_AT_TIMEOUT_MS,
                               response, sizeof(response), false);
    }

    snprintf(cmd, sizeof(cmd), "AT+MIPCLOSE=%d\r\n", socket_id);
    ml307r_at_send(cmd, "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);
    return sent;
}

esp_err_t ml307r_http_post(const char *path, const char *data, size_t data_len) {
    static const int socket_id = 0;
    static const char server_ip[] = "120.53.251.149";
    static const uint16_t server_port = 8007;
    char cmd[96];
    char response[256];

    if (!path || !data || data_len == 0 || !s_network_ready) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 关闭可能遗留的 socket */
    snprintf(cmd, sizeof(cmd), "AT+MIPCLOSE=%d\r\n", socket_id);
    ml307r_at_send(cmd, "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);

    /* 建立 TCP 连接 */
    snprintf(cmd, sizeof(cmd), "AT+MIPOPEN=%d,\"TCP\",\"%s\",%u\r\n",
             socket_id, server_ip, (unsigned)server_port);
    if (!ml307r_at_send(cmd, "OK", 10000, response, sizeof(response))) {
        return ESP_FAIL;
    }

    /* 等待连接真正建立 */
    bool connected = false;
    for (int i = 0; i < 5; i++) {
        snprintf(cmd, sizeof(cmd), "AT+MIPSTATE=%d\r\n", socket_id);
        if (ml307r_at_send(cmd, "CONNECTED", 2000, response, sizeof(response))) {
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!connected) {
        snprintf(cmd, sizeof(cmd), "AT+MIPCLOSE=%d\r\n", socket_id);
        ml307r_at_send(cmd, "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);
        return ESP_FAIL;
    }

    /* 构造 HTTP POST 请求并通过 MIPSEND 发送 */
    char *http_req = malloc(256 + data_len);
    if (!http_req) return ESP_ERR_NO_MEM;

    int header_len = snprintf(http_req, 256,
        "POST %s HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n"
        "\r\n",
        path, server_ip, server_port, (unsigned)data_len);

    memcpy(http_req + header_len, data, data_len);
    int total_len = header_len + (int)data_len;

    snprintf(cmd, sizeof(cmd), "AT+MIPSEND=%d,%d\r\n", socket_id, total_len);
    if (!ml307r_at_send(cmd, ">", ML307R_AT_TIMEOUT_MS, NULL, 0)) {
        free(http_req);
        snprintf(cmd, sizeof(cmd), "AT+MIPCLOSE=%d\r\n", socket_id);
        ml307r_at_send(cmd, "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);
        return ESP_FAIL;
    }

    uart_write_raw(http_req, total_len);
    free(http_req);

    /* 等待发送完成 */
    bool ok = uart_read_until("OK", ML307R_AT_TIMEOUT_MS, response, sizeof(response), false);

    /* 关闭连接 */
    snprintf(cmd, sizeof(cmd), "AT+MIPCLOSE=%d\r\n", socket_id);
    ml307r_at_send(cmd, "OK", ML307R_AT_TIMEOUT_MS, NULL, 0);

    return ok ? ESP_OK : ESP_FAIL;
}
