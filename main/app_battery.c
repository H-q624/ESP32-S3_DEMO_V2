#include "app_battery.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "BATTERY";

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_channel_t s_adc_channel = ADC_CHANNEL_6; /* GPIO7 = ADC1_CH6 on ESP32-S3 */
static bool s_ready = false;

/* 共阳 LED: 拉低点亮, 拉高熄灭 */
static void bat_led_set(gpio_num_t pin, bool on) {
    gpio_set_level(pin, on ? 0 : 1);
}

static int mv_to_percent(int mv) {
    if (mv <= BATTERY_VOLT_EMPTY_MV) return 0;
    if (mv >= BATTERY_VOLT_FULL_MV) return 100;
    return (mv - BATTERY_VOLT_EMPTY_MV) * 100 /
           (BATTERY_VOLT_FULL_MV - BATTERY_VOLT_EMPTY_MV);
}

esp_err_t app_battery_init(void) {
    /* 电量 LED: IO46/47/48, 共阳, 默认全灭 */
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << BATTERY_LED1_GPIO) |
                        (1ULL << BATTERY_LED2_GPIO) |
                        (1ULL << BATTERY_LED3_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_cfg);
    bat_led_set((gpio_num_t)BATTERY_LED1_GPIO, false);
    bat_led_set((gpio_num_t)BATTERY_LED2_GPIO, false);
    bat_led_set((gpio_num_t)BATTERY_LED3_GPIO, false);

    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&adc_cfg, &s_adc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

#if CONFIG_BATTERY_ADC_GPIO == 7
    s_adc_channel = ADC_CHANNEL_6;
#elif CONFIG_BATTERY_ADC_GPIO == 1
    s_adc_channel = ADC_CHANNEL_0;
#else
    s_adc_channel = ADC_CHANNEL_6;
#endif

    ret = adc_oneshot_config_channel(s_adc, s_adc_channel, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
        return ret;
    }

    s_ready = true;
    ESP_LOGI(TAG, "Battery ADC on IO%d (CH%d), LEDs IO%d/IO%d/IO%d (common-anode)",
             BATTERY_ADC_GPIO, (int)s_adc_channel,
             BATTERY_LED1_GPIO, BATTERY_LED2_GPIO, BATTERY_LED3_GPIO);
    return ESP_OK;
}

int app_battery_read_mv(void) {
    if (!s_ready || !s_adc) return -1;

    int raw_sum = 0;
    const int samples = 8;
    for (int i = 0; i < samples; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, s_adc_channel, &raw) != ESP_OK) {
            return -1;
        }
        raw_sum += raw;
    }
    int raw_avg = raw_sum / samples;

    /* ESP32-S3 ADC @12dB: 约 0~3300mV 量程, 12bit */
    int adc_mv = (raw_avg * 3300) / 4095;
    int bat_mv = adc_mv * 2; /* 10K/10K 分压 */

    return bat_mv;
}

int app_battery_read_percent(void) {
    int mv = app_battery_read_mv();
    if (mv < 0) return -1;
    return mv_to_percent(mv);
}

void app_battery_led_update(void) {
    int mv = app_battery_read_mv();
    if (mv < 0) {
        bat_led_set((gpio_num_t)BATTERY_LED1_GPIO, false);
        bat_led_set((gpio_num_t)BATTERY_LED2_GPIO, false);
        bat_led_set((gpio_num_t)BATTERY_LED3_GPIO, false);
        return;
    }

    bool led1 = (mv >= BATTERY_VOLT_EMPTY_MV);
    bool led2 = (mv >= BATTERY_LED2_MV);
    bool led3 = (mv >= BATTERY_LED3_MV);

    bat_led_set((gpio_num_t)BATTERY_LED1_GPIO, led1);
    bat_led_set((gpio_num_t)BATTERY_LED2_GPIO, led2);
    bat_led_set((gpio_num_t)BATTERY_LED3_GPIO, led3);

    ESP_LOGD(TAG, "Battery %d mV (%d%%), LEDs %d%d%d",
             mv, mv_to_percent(mv), led1, led2, led3);
}
