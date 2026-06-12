#ifndef _APP_ML307R_H_
#define _APP_ML307R_H_

#include "sdkconfig.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  ML307R 4G 模组引脚 (通过 TXS0108EPWR 电平转换)
 * ================================================================ */
#define ML307R_UART_NUM       UART_NUM_1
#define ML307R_TX_PIN         CONFIG_ML307R_UART_TX_PIN
#define ML307R_RX_PIN         CONFIG_ML307R_UART_RX_PIN
#define ML307R_BAUDRATE       CONFIG_ML307R_BAUDRATE
#define ML307R_PWR_PIN        CONFIG_ML307R_PWR_PIN
#define ML307R_RST_PIN        CONFIG_ML307R_RST_PIN

/* eSIM / SIM 卡 ICCID (用于启动校验) */
#ifndef ML307R_ESIM_ICCID
#define ML307R_ESIM_ICCID     "898604293624D0028511"
#endif

/* 中移物联网 APN (898604 开头 ICCID 通常用 cmnet) */
#ifndef ML307R_APN
#define ML307R_APN            "cmnet"
#endif

/* ================================================================
 *  AT 指令超时与重试
 * ================================================================ */
#define ML307R_AT_TIMEOUT_MS      3000
#define ML307R_AT_SHORT_TIMEOUT   500
#define ML307R_NETWORK_TIMEOUT_MS 60000
#define ML307R_HTTP_TIMEOUT_MS    120000
#define ML307R_TCP_OPEN_TIMEOUT   30000
#define ML307R_RX_BUF_SIZE        4096
#define ML307R_AT_RETRY           3

/* MHTTP 内联 POST 上限 (超过则走 TCP 裸 HTTP) */
#define ML307R_MHTTP_MAX_BODY     1500

/*
 * 服务器 JSON 协议 (ML307R TCP POST):
 *   持续上传包 -> ml307r_upload_data()   POST /api/data
 *   异常报警包 -> ml307r_upload_alarm()  POST /api/alarm
 *   留言包     -> ml307r_upload_message() POST /api/message
 * 也可通过 app_http_send_periodic/alarm/message() 自动路由到 ML307R。
 */

/* ================================================================
 *  公开 API
 * ================================================================ */

esp_err_t ml307r_init(void);
void ml307r_power_on(void);
void ml307r_power_off(void);
bool ml307r_is_alive(void);
bool ml307r_wait_network(int timeout_ms);
bool ml307r_is_connected(void);
int ml307r_get_csq(void);

/**
 * @brief 通过 ML307R 发送 HTTP POST (自动选择 MHTTP 或 TCP)
 * @param url      完整 URL, 如 "http://182.92.156.138:8007/api/data"
 * @param data     JSON 请求体
 * @param data_len 请求体长度
 */
esp_err_t ml307r_http_post(const char *url, const char *data, size_t data_len);

/** 读取模组 ICCID (eSIM), 失败返回 false */
bool ml307r_get_iccid(char *buf, size_t buf_size);

/** 按协议路径上传 JSON (TCP HTTP POST) */
esp_err_t ml307r_upload_data(const char *json, size_t len);
esp_err_t ml307r_upload_alarm(const char *json, size_t len);
esp_err_t ml307r_upload_message(const char *json, size_t len);

bool ml307r_at_send(const char *cmd, const char *expect, int timeout_ms,
                    char *out_buf, size_t out_size);

/** 4G 开机完成后释放 PWR 引脚 (与电池 ADC 共用 IO7 时) */
void ml307r_release_pwr_pin(void);

#ifdef __cplusplus
}
#endif

#endif /* _APP_ML307R_H_ */
