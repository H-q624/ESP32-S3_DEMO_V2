#include "app_mic.h"
#include "app_gpio.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>

QueueHandle_t wav_queue = nullptr;

static int16_t s_pcm_ring[AUDIO_SAMPLES_PER_UPLOAD];
static size_t s_ring_len = 0;
static int32_t s_last_raw = 0;

static void ring_append(const int16_t *pcm, size_t nsamples) {
    for (size_t i = 0; i < nsamples; i++) {
        if (s_ring_len < AUDIO_SAMPLES_PER_UPLOAD) {
            s_pcm_ring[s_ring_len++] = pcm[i];
        } else {
            memmove(&s_pcm_ring[0], &s_pcm_ring[1],
                    (AUDIO_SAMPLES_PER_UPLOAD - 1) * sizeof(int16_t));
            s_pcm_ring[AUDIO_SAMPLES_PER_UPLOAD - 1] = pcm[i];
        }
    }
}

extern "C" void app_mic_reset_ring(void) { s_ring_len = 0; }

extern "C" bool app_mic_append_sample(int16_t sample) {
    ring_append(&sample, 1);
    return true;
}

extern "C" size_t app_mic_get_upload_pcm(int16_t *out, size_t max_samples) {
    if (!out || max_samples == 0) return 0;
    size_t n = s_ring_len < max_samples ? s_ring_len : max_samples;
    if (n > 0) memcpy(out, s_pcm_ring, n * sizeof(int16_t));
    return n;
}

extern "C" int32_t app_mic_get_last_raw(void) { return s_last_raw; }

/* SPH0645: 18-bit signed, MSB-aligned in 32-bit slot (bits 31:14) */
static int16_t sph0645_raw_to_pcm(int32_t raw) {
    int32_t s = raw >> 14;
    if (s & 0x20000) {
        s |= ~0x3FFFF;
    }
    s >>= 2;
    if (s > 32767) s = 32767;
    if (s < -32768) s = -32768;
    return (int16_t)s;
}

static void mic_flush_rx(i2s_chan_handle_t rx_chan, int rounds) {
    int32_t dummy[64];
    size_t bytes_read = 0;
    for (int i = 0; i < rounds; i++) {
        (void)i2s_channel_read(rx_chan, dummy, sizeof(dummy), &bytes_read,
                               pdMS_TO_TICKS(20));
    }
}

typedef struct {
    const char *name;
    i2s_std_slot_config_t slot;
    bool ws_inv;
} mic_fmt_candidate_t;

static i2s_std_slot_config_t mic_slot_philips_left(void) {
    i2s_std_slot_config_t c = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
    c.slot_mask = I2S_STD_SLOT_LEFT;
    c.ws_width = 32;
    return c;
}

static i2s_std_slot_config_t mic_slot_msb_left(void) {
    i2s_std_slot_config_t c = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
    c.slot_mask = I2S_STD_SLOT_LEFT;
    c.ws_width = 32;
    c.bit_shift = false;
    return c;
}

static i2s_std_slot_config_t mic_slot_philips_right(void) {
    i2s_std_slot_config_t c = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
    c.slot_mask = I2S_STD_SLOT_RIGHT;
    c.ws_width = 32;
    return c;
}

static i2s_std_slot_config_t mic_slot_msb_right(void) {
    i2s_std_slot_config_t c = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
    c.slot_mask = I2S_STD_SLOT_RIGHT;
    c.ws_width = 32;
    c.bit_shift = false;
    return c;
}

static uint32_t mic_score_format(i2s_chan_handle_t rx_chan) {
    int32_t raw_min = INT32_MAX;
    int32_t raw_max = INT32_MIN;
    int16_t pcm_min = INT16_MAX;
    int16_t pcm_max = INT16_MIN;
    int got = 0;

    mic_flush_rx(rx_chan, 4);

    for (int i = 0; i < 48; i++) {
        int32_t raw = 0;
        size_t bytes_read = 0;
        if (i2s_channel_read(rx_chan, &raw, sizeof(raw), &bytes_read,
                             pdMS_TO_TICKS(50)) != ESP_OK ||
            bytes_read != sizeof(raw)) {
            continue;
        }
        int16_t pcm = sph0645_raw_to_pcm(raw);
        if (raw < raw_min) raw_min = raw;
        if (raw > raw_max) raw_max = raw;
        if (pcm < pcm_min) pcm_min = pcm;
        if (pcm > pcm_max) pcm_max = pcm;
        got++;
    }

    if (got == 0) {
        return 0;
    }

    uint32_t raw_range = (uint32_t)(raw_max - raw_min);
    uint32_t pcm_range = (uint32_t)(pcm_max - pcm_min);
    /* 优先 raw 动态范围；pcm 全挤在同一量化级(如全-1)时降权 */
    if (pcm_range <= 1 && raw_range < 65536) {
        return raw_range / 4;
    }
    return raw_range + ((uint32_t)pcm_range << 16);
}

static void mic_log_samples(i2s_chan_handle_t rx_chan, const char *tag) {
    for (int i = 0; i < 4; i++) {
        int32_t raw = 0;
        size_t bytes_read = 0;
        if (i2s_channel_read(rx_chan, &raw, sizeof(raw), &bytes_read,
                             pdMS_TO_TICKS(100)) != ESP_OK ||
            bytes_read != sizeof(raw)) {
            ESP_LOGW(tag, "self-test[%d]: read failed", i);
            continue;
        }
        ESP_LOGI(tag, "self-test[%d]: raw=0x%08lx pcm=%d",
                 i, (unsigned long)(uint32_t)raw, (int)sph0645_raw_to_pcm(raw));
    }
}

MEMS_MIC::MEMS_MIC(const char *tag)
    : TAG(tag), isReady(false), i2s_rx_handle(nullptr),
      latest_pcm(0), decim_counter(0) {}

MEMS_MIC::~MEMS_MIC() {
    if (i2s_rx_handle) {
        i2s_channel_disable((i2s_chan_handle_t)i2s_rx_handle);
        i2s_del_channel((i2s_chan_handle_t)i2s_rx_handle);
        i2s_rx_handle = nullptr;
    }
}

esp_err_t MEMS_MIC::app_mic_init() {
    if (GPIO_PIN_BEEP == MIC_I2S_DATA_GPIO) {
        ESP_LOGW(TAG, "BEEP=IO%d conflicts with MIC DATA, beep disabled", GPIO_PIN_BEEP);
    }
    gpio_reset_pin((gpio_num_t)MIC_I2S_DATA_GPIO);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = 256;
    chan_cfg.auto_clear = true;

    i2s_chan_handle_t rx_chan = nullptr;
    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S channel create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    const mic_fmt_candidate_t candidates[] = {
        { "Philips/STEREO/LEFT",      mic_slot_philips_left(),  false },
        { "MSB/STEREO/LEFT",          mic_slot_msb_left(),      false },
        { "Philips/STEREO/RIGHT",     mic_slot_philips_right(), false },
        { "MSB/STEREO/RIGHT",         mic_slot_msb_right(),     false },
        { "Philips/STEREO/LEFT/WS_INV", mic_slot_philips_left(), true },
    };

    i2s_std_gpio_config_t gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = (gpio_num_t)MIC_I2S_BCLK_GPIO,
        .ws   = (gpio_num_t)MIC_I2S_WS_GPIO,
        .dout = I2S_GPIO_UNUSED,
        .din  = (gpio_num_t)MIC_I2S_DATA_GPIO,
        .invert_flags = {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv   = false,
        },
    };

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = candidates[0].slot,
        .gpio_cfg = gpio_cfg,
    };

    ret = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S std init failed: %s", esp_err_to_name(ret));
        i2s_del_channel(rx_chan);
        return ret;
    }

    uint32_t best_score = 0;
    int best_idx = 0;

    for (int i = 0; i < (int)(sizeof(candidates) / sizeof(candidates[0])); i++) {
        if (i > 0) {
            i2s_channel_disable(rx_chan);
        }

        gpio_cfg.invert_flags.ws_inv = candidates[i].ws_inv;
        std_cfg.gpio_cfg = gpio_cfg;
        std_cfg.slot_cfg = candidates[i].slot;

        if (i == 0) {
            ret = ESP_OK;
        } else {
            ret = i2s_channel_reconfig_std_slot(rx_chan, &std_cfg.slot_cfg);
            if (ret == ESP_OK) {
                ret = i2s_channel_reconfig_std_gpio(rx_chan, &std_cfg.gpio_cfg);
            }
        }
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Format %s reconfig failed: %s",
                     candidates[i].name, esp_err_to_name(ret));
            continue;
        }

        ret = i2s_channel_enable(rx_chan);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Format %s enable failed: %s",
                     candidates[i].name, esp_err_to_name(ret));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(30));
        uint32_t score = mic_score_format(rx_chan);
        ESP_LOGI(TAG, "Format probe %s: score=%lu",
                 candidates[i].name, (unsigned long)score);

        if (score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }

    i2s_channel_disable(rx_chan);
    gpio_cfg.invert_flags.ws_inv = candidates[best_idx].ws_inv;
    std_cfg.gpio_cfg = gpio_cfg;
    std_cfg.slot_cfg = candidates[best_idx].slot;
    ret = i2s_channel_reconfig_std_slot(rx_chan, &std_cfg.slot_cfg);
    if (ret == ESP_OK) {
        ret = i2s_channel_reconfig_std_gpio(rx_chan, &std_cfg.gpio_cfg);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Apply best format failed: %s", esp_err_to_name(ret));
        i2s_del_channel(rx_chan);
        return ret;
    }

    ret = i2s_channel_enable(rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S enable failed: %s", esp_err_to_name(ret));
        i2s_del_channel(rx_chan);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
    mic_log_samples(rx_chan, TAG);
    ESP_LOGI(TAG, "Selected format: %s (score=%lu)",
             candidates[best_idx].name, (unsigned long)best_score);
    if (best_score < 256) {
        ESP_LOGW(TAG, "Mic signal weak (score=%lu). Check 3.3V / WS=IO15 BCLK=IO16 DATA=IO3",
                 (unsigned long)best_score);
    }

    i2s_rx_handle = rx_chan;
    isReady = true;
    ESP_LOGI(TAG, "SPH0645 ready: WS=IO%d BCLK=IO%d DATA=IO%d @ %dHz",
             MIC_I2S_WS_GPIO, MIC_I2S_BCLK_GPIO, MIC_I2S_DATA_GPIO, MIC_SAMPLE_RATE);
    return ESP_OK;
}

bool MEMS_MIC::app_mic_check_module() {
    return isReady && i2s_rx_handle != nullptr;
}

bool MEMS_MIC::read_sample_pcm(int16_t *out) {
    if (!out || !app_mic_check_module()) return false;

    const int decim = MIC_SAMPLE_RATE / MIC_UPLOAD_SAMPLE_RATE;
    int32_t chunk[64];
    int64_t sum = 0;
    int got = 0;

    while (got < decim) {
        int want = decim - got;
        if (want > (int)(sizeof(chunk) / sizeof(chunk[0]))) {
            want = (int)(sizeof(chunk) / sizeof(chunk[0]));
        }
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read((i2s_chan_handle_t)i2s_rx_handle,
                                         chunk, want * sizeof(int32_t), &bytes_read,
                                         pdMS_TO_TICKS(500));
        if (ret != ESP_OK || bytes_read == 0) {
            return false;
        }
        int n = (int)(bytes_read / sizeof(int32_t));
        for (int i = 0; i < n; i++) {
            sum += sph0645_raw_to_pcm(chunk[i]);
        }
        got += n;
        s_last_raw = chunk[n - 1];
    }

    *out = (int16_t)(sum / decim);
    latest_pcm = *out;
    return true;
}
