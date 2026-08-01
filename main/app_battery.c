#include "app_battery.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "BATTERY";

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_channel_t s_adc_channel = ADC_CHANNEL_6; /* GPIO7 = ADC1_CH6 on ESP32-S3 */
static adc_cali_handle_t s_adc_cali = NULL;
static bool s_ready = false;

static bool bat_led_pin_available(gpio_num_t pin) {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        return false;
    }
#if CONFIG_EXTFLASH_ENABLE
    if ((int)pin == CONFIG_EXTFLASH_CS_PIN ||
        (int)pin == CONFIG_EXTFLASH_CLK_PIN ||
        (int)pin == CONFIG_EXTFLASH_MOSI_PIN ||
        (int)pin == CONFIG_EXTFLASH_MISO_PIN) {
        return false;
    }
#endif
    return true;
}

/* 共阳 LED: 拉低点亮, 拉高熄灭 */
static void bat_led_set(gpio_num_t pin, bool on) {
    if (!bat_led_pin_available(pin)) {
        return;
    }
    gpio_set_level(pin, on ? 0 : 1);
}

static int mv_to_percent(int mv) {
    if (mv <= BATTERY_VOLT_EMPTY_MV) return 0;
    if (mv >= BATTERY_VOLT_FULL_MV) return 100;
    return (mv - BATTERY_VOLT_EMPTY_MV) * 100 /
           (BATTERY_VOLT_FULL_MV - BATTERY_VOLT_EMPTY_MV);
}

esp_err_t app_battery_init(void) {
    /*
     * IO46/IO47 are now APS1604M SI/SO.  Build the LED mask dynamically so
     * battery indication can never reconfigure or drive an external-RAM pin.
     */
    const gpio_num_t led_pins[] = {
        (gpio_num_t)BATTERY_LED1_GPIO,
        (gpio_num_t)BATTERY_LED2_GPIO,
        (gpio_num_t)BATTERY_LED3_GPIO,
    };
    uint64_t led_mask = 0;
    for (size_t i = 0; i < sizeof(led_pins) / sizeof(led_pins[0]); ++i) {
        if (bat_led_pin_available(led_pins[i])) {
            led_mask |= 1ULL << led_pins[i];
        } else {
            ESP_LOGW(TAG, "Battery LED GPIO%d disabled (reserved by APS1604M)",
                     (int)led_pins[i]);
        }
    }

    gpio_config_t led_cfg = {
        .pin_bit_mask = led_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (led_mask != 0) {
        gpio_config(&led_cfg);
    }
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

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = s_adc_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali);
    if (ret != ESP_OK) {
        s_adc_cali = NULL;
        ESP_LOGW(TAG, "ADC calibration unavailable (%s), using raw conversion",
                 esp_err_to_name(ret));
    }

    s_ready = true;
    ESP_LOGI(TAG, "Battery ADC enabled: GPIO%d = ADC1_CH%d, calibration=%s",
             BATTERY_ADC_GPIO, (int)s_adc_channel,
             s_adc_cali ? "curve-fitting" : "raw fallback");
    ESP_LOGI(TAG, "Battery LEDs: GPIO%d/GPIO%d/GPIO%d (common-anode)",
             BATTERY_LED1_GPIO, BATTERY_LED2_GPIO, BATTERY_LED3_GPIO);
    return ESP_OK;
}

int app_battery_read_adc_mv(void) {
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

    int adc_mv = (raw_avg * 3300) / 4095;
    if (s_adc_cali &&
        adc_cali_raw_to_voltage(s_adc_cali, raw_avg, &adc_mv) != ESP_OK) {
        adc_mv = (raw_avg * 3300) / 4095;
    }
    return adc_mv;
}

int app_battery_read_mv(void) {
    int adc_mv = app_battery_read_adc_mv();
    if (adc_mv < 0) return -1;

    /*
     * 10K/10K voltage divider:
     * Vadc = Vbattery * Rbottom / (Rtop + Rbottom)
     * Vbattery = Vadc * (Rtop + Rbottom) / Rbottom
     *          = Vadc * (10K + 10K) / 10K = Vadc * 2
     */
    return adc_mv * 2;
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
