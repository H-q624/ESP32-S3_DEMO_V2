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
#define ML307R_RX_BUF_SIZE        4096
#define ML307R_AT_RETRY           3

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

/** 连接指定 TCP 服务器并发送一次 hello；任何失败均返回 false */
bool ml307r_send_hello(const char *server_ip, uint16_t server_port);

/** 读取模组 ICCID (eSIM), 失败返回 false */
bool ml307r_get_iccid(char *buf, size_t buf_size);

bool ml307r_at_send(const char *cmd, const char *expect, int timeout_ms,
                    char *out_buf, size_t out_size);

/** 4G 开机完成后释放 PWR 引脚 (与电池 ADC 共用 IO7 时) */
void ml307r_release_pwr_pin(void);

esp_err_t ml307r_http_post(const char *path, const char *data, size_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* _APP_ML307R_H_ */
