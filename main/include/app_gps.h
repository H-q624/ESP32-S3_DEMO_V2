#ifndef _APP_GPS_H_
#define _APP_GPS_H_

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_UART_NUM          UART_NUM_0
#define GPS_TX_PIN            CONFIG_GPS_UART_TX_PIN   /* ESP UART0 TX -> GPS RXD */
#define GPS_RX_PIN            CONFIG_GPS_UART_RX_PIN   /* ESP UART0 RX <- GPS TXD */
#define GPS_BAUDRATE          CONFIG_GPS_UART_BAUDRATE
#define GPS_UART_BUF_SIZE     1024
#define GPS_ON_OFF_PIN        CONFIG_GPS_ON_OFF_PIN
#define GPS_RST_PIN           CONFIG_GPS_RST_PIN
#define GPS_1PPS_PIN          CONFIG_GPS_1PPS_PIN

typedef struct {
    double   latitude;
    double   longitude;
    float    altitude;
    float    speed_kmh;
    float    heading;
    uint8_t  satellites;
    uint8_t  fix_quality;
    uint8_t  hour, minute, second;
    uint16_t year;
    uint8_t  month, day;
    bool     valid;
    uint32_t last_update_ms;
} gps_data_t;

extern gps_data_t g_gps_data;

esp_err_t app_gps_init(void);
void app_gps_task(void *pvParameters);
void app_gps_power_on(void);
void app_gps_power_off(void);
void app_gps_get_data(gps_data_t *out);
bool app_gps_has_fix(void);
bool app_gps_get_last_gnrmc(char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* _APP_GPS_H_ */
