#include "app_speaker.h"
#include "driver/sdm.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "speaker";
static sdm_channel_handle_t s_sdm_chan = NULL;

esp_err_t speaker_init(void) {
    sdm_config_t cfg = {
        .gpio_num       = SPEAKER_GPIO,
        .clk_src        = SDM_CLK_SRC_DEFAULT,
        .sample_rate_hz = 100000, /* 100 kHz carrier — reduces switching loss vs 1MHz */
    };
    esp_err_t ret = sdm_new_channel(&cfg, &s_sdm_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sdm_new_channel failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = sdm_channel_enable(s_sdm_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sdm_channel_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    sdm_channel_set_duty(s_sdm_chan, 0);
    ESP_LOGI(TAG, "Sigma-Delta speaker init OK (GPIO%d, 1MHz carrier)", SPEAKER_GPIO);
    return ESP_OK;
}

void speaker_off(void) {
    if (s_sdm_chan) {
        sdm_channel_set_duty(s_sdm_chan, 0);
    }
}

/*
 * Play 16-bit PCM mono buffer via sigma-delta output.
 * Scales int16 [-32768,32767] to SDM duty int8 [-128,127].
 * Uses esp_timer for sample-accurate pacing.
 */
void speaker_play_pcm16(const int16_t *samples, size_t num_samples, uint32_t sample_rate_hz) {
    if (!s_sdm_chan || !samples || num_samples == 0) return;

    const int64_t period_us = 1000000LL / sample_rate_hz;
    int64_t next_tick = esp_timer_get_time();

    for (size_t i = 0; i < num_samples; i++) {
        int8_t duty = (int8_t)(samples[i] >> 8);
        sdm_channel_set_duty(s_sdm_chan, duty);

        next_tick += period_us;
        int64_t wait_us = next_tick - esp_timer_get_time();
        if (wait_us > 1000) {
            vTaskDelay(pdMS_TO_TICKS(wait_us / 1000));
        }
        while (esp_timer_get_time() < next_tick) {}
    }
    speaker_off();
}

/*
 * Blocking beep: synthesises a sine at freq_hz for duration_ms.
 */
void speaker_beep(uint32_t freq_hz, uint32_t duration_ms) {
    if (!s_sdm_chan) return;

    const uint32_t sr = 16000;
    const uint32_t num_samples = sr * duration_ms / 1000;
    const float step = 2.0f * (float)M_PI * (float)freq_hz / (float)sr;

    const int64_t period_us = 1000000LL / sr;
    int64_t next_tick = esp_timer_get_time();

    for (uint32_t i = 0; i < num_samples; i++) {
        int8_t duty = (int8_t)(sinf(step * i) * 120.0f);
        sdm_channel_set_duty(s_sdm_chan, duty);

        next_tick += period_us;
        int64_t wait_us = next_tick - esp_timer_get_time();
        if (wait_us > 1000) vTaskDelay(pdMS_TO_TICKS(wait_us / 1000));
        while (esp_timer_get_time() < next_tick) {}
    }
    speaker_off();
}
