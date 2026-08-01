#ifndef _APP_BATTERY_H_
#define _APP_BATTERY_H_

#include "sdkconfig.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* BATTERY_ADC 分压后接入 IO7 (10K+10K, Vbat = Vadc * 2) */
#define BATTERY_ADC_GPIO        CONFIG_BATTERY_ADC_GPIO
#define BATTERY_LED1_GPIO       CONFIG_BATTERY_LED1_GPIO
#define BATTERY_LED2_GPIO       CONFIG_BATTERY_LED2_GPIO
#define BATTERY_LED3_GPIO       CONFIG_BATTERY_LED3_GPIO

/* 单节锂电电压阈值 (mV) */
#define BATTERY_VOLT_EMPTY_MV   CONFIG_BATTERY_VOLT_EMPTY_MV
#define BATTERY_VOLT_FULL_MV    CONFIG_BATTERY_VOLT_FULL_MV
#define BATTERY_LED2_MV         CONFIG_BATTERY_LED2_MV
#define BATTERY_LED3_MV         CONFIG_BATTERY_LED3_MV

esp_err_t app_battery_init(void);
int app_battery_read_adc_mv(void);
int app_battery_read_mv(void);
int app_battery_read_percent(void);
void app_battery_led_update(void);

#ifdef __cplusplus
}
#endif

#endif /* _APP_BATTERY_H_ */
