#include "app_speaker.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "speaker";

esp_err_t speaker_init(void) {
    const gpio_num_t gpio = static_cast<gpio_num_t>(SPEAKER_GPIO);

    /* Detach any peripheral output, then drive GPIO10 at a constant low level. */
    esp_err_t ret = gpio_reset_pin(gpio);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d reset failed: %s", SPEAKER_GPIO, esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_set_level(gpio, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d set low failed: %s", SPEAKER_GPIO, esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d output mode failed: %s", SPEAKER_GPIO, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "GPIO%d configured as output and held low", SPEAKER_GPIO);
    return ESP_OK;
}

void speaker_off(void) {
    gpio_set_level(static_cast<gpio_num_t>(SPEAKER_GPIO), 0);
}

void speaker_play_pcm16(const int16_t *samples, size_t num_samples, uint32_t sample_rate_hz) {
    (void)samples;
    (void)num_samples;
    (void)sample_rate_hz;
}

void speaker_beep(uint32_t freq_hz, uint32_t duration_ms) {
    (void)freq_hz;
    (void)duration_ms;
}
