#include "app_mic.h"
#include "app_gpio.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>

QueueHandle_t wav_queue = nullptr;

/* Upload rate must evenly divide hardware sample rate. */
static_assert(MIC_SAMPLE_RATE % MIC_UPLOAD_SAMPLE_RATE == 0,
              "MIC_SAMPLE_RATE must be an integer multiple of MIC_UPLOAD_SAMPLE_RATE");
#define MIC_DECIM_FACTOR (MIC_SAMPLE_RATE / MIC_UPLOAD_SAMPLE_RATE)

#define UPLOAD_RING_CAP       AUDIO_SAMPLES_PER_UPLOAD
#define UPLOAD_DECIM_QUEUE_LEN 32

/* ---- O(1) ring buffer for upload PCM history ---- */
typedef struct {
    int16_t buf[UPLOAD_RING_CAP];
    size_t write_idx;
    size_t count;
    SemaphoreHandle_t lock;
} upload_ring_t;

static upload_ring_t s_upload_ring;
static int32_t s_last_raw = 0;

static volatile bool s_sr_feed_active = false;
static QueueHandle_t s_decim_queue = nullptr;
static int64_t s_decim_sum = 0;
static int s_decim_count = 0;

/* First-order DC blocking: y = x - x_z1 + y_z1 - (y_z1 >> 8)  (~0.996 feedback) */
static int32_t s_dc_x_z1 = 0;
static int32_t s_dc_y_z1 = 0;

static void upload_ring_init(void) {
    if (!s_upload_ring.lock) {
        s_upload_ring.lock = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(s_upload_ring.lock, portMAX_DELAY);
    s_upload_ring.write_idx = 0;
    s_upload_ring.count = 0;
    xSemaphoreGive(s_upload_ring.lock);
}

extern "C" void app_mic_reset_ring(void) {
    if (!s_upload_ring.lock) {
        return;
    }
    xSemaphoreTake(s_upload_ring.lock, portMAX_DELAY);
    s_upload_ring.write_idx = 0;
    s_upload_ring.count = 0;
    xSemaphoreGive(s_upload_ring.lock);
}

static void upload_ring_push(int16_t sample) {
    xSemaphoreTake(s_upload_ring.lock, portMAX_DELAY);
    s_upload_ring.buf[s_upload_ring.write_idx] = sample;
    s_upload_ring.write_idx = (s_upload_ring.write_idx + 1) % UPLOAD_RING_CAP;
    if (s_upload_ring.count < UPLOAD_RING_CAP) {
        s_upload_ring.count++;
    }
    xSemaphoreGive(s_upload_ring.lock);
}

static size_t upload_ring_copy(int16_t *out, size_t max_samples) {
    if (!out || max_samples == 0) {
        return 0;
    }

    xSemaphoreTake(s_upload_ring.lock, portMAX_DELAY);
    size_t n = s_upload_ring.count < max_samples ? s_upload_ring.count : max_samples;
    if (n > 0) {
        size_t start = (s_upload_ring.write_idx + UPLOAD_RING_CAP - s_upload_ring.count) % UPLOAD_RING_CAP;
        for (size_t i = 0; i < n; i++) {
            out[i] = s_upload_ring.buf[(start + i) % UPLOAD_RING_CAP];
        }
    }
    xSemaphoreGive(s_upload_ring.lock);
    return n;
}

extern "C" bool app_mic_sr_feed_active(void) { return s_sr_feed_active; }

extern "C" bool app_mic_take_upload_sample(int16_t *out, uint32_t timeout_ms) {
    if (!out || !s_decim_queue) {
        return false;
    }
    return xQueueReceive(s_decim_queue, out, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

static int16_t mic_dc_block(int16_t pcm) {
    int32_t x = pcm;
    int32_t y = x - s_dc_x_z1 + s_dc_y_z1 - (s_dc_y_z1 >> 8);
    s_dc_x_z1 = x;
    s_dc_y_z1 = y;
    if (y > 32767) {
        y = 32767;
    } else if (y < -32768) {
        y = -32768;
    }
    return (int16_t)y;
}

static void mic_reset_dc_block(void) {
    s_dc_x_z1 = 0;
    s_dc_y_z1 = 0;
}

static bool mic_push_decimated_sample(int16_t pcm16k, int16_t *decim_out) {
    pcm16k = mic_dc_block(pcm16k);

    s_decim_sum += pcm16k;
    s_decim_count++;
    if (s_decim_count < MIC_DECIM_FACTOR) {
        return false;
    }

    int16_t decimated = (int16_t)(s_decim_sum / s_decim_count);
    s_decim_sum = 0;
    s_decim_count = 0;

    if (decim_out) {
        *decim_out = decimated;
    }

    if (s_sr_feed_active && s_decim_queue) {
        if (xQueueSend(s_decim_queue, &decimated, 0) != pdTRUE) {
            int16_t drop;
            (void)xQueueReceive(s_decim_queue, &drop, 0);
            (void)xQueueSend(s_decim_queue, &decimated, 0);
        }
    }
    return true;
}

void app_mic_sr_on_pcm_sample(int16_t pcm) {
    if (!s_sr_feed_active) {
        return;
    }
    (void)mic_push_decimated_sample(pcm, nullptr);
}

void app_mic_sr_set_feed_active(bool active) {
    s_sr_feed_active = active;
    s_decim_sum = 0;
    s_decim_count = 0;
    mic_reset_dc_block();

    if (active && !s_decim_queue) {
        s_decim_queue = xQueueCreate(UPLOAD_DECIM_QUEUE_LEN, sizeof(int16_t));
    }

    if (s_decim_queue) {
        int16_t drop;
        while (xQueueReceive(s_decim_queue, &drop, 0) == pdTRUE) {
        }
    }
}

extern "C" bool app_mic_append_sample(int16_t sample) {
    upload_ring_push(sample);
    return true;
}

extern "C" size_t app_mic_get_upload_pcm(int16_t *out, size_t max_samples) {
    return upload_ring_copy(out, max_samples);
}

extern "C" int32_t app_mic_get_last_raw(void) { return s_last_raw; }

/*
 * SPH0645: 18-bit signed PCM, MSB-aligned in 32-bit I2S slot (bits 31:14).
 * Shift down to int16 with saturation (>>2 maps 18-bit into 16-bit headroom).
 */
static int16_t sph0645_raw_to_pcm(int32_t raw) {
    int32_t s = (int32_t)raw >> 14;
    s >>= 2;
    if (s > 32767) {
        s = 32767;
    } else if (s < -32768) {
        s = -32768;
    }
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

#if CONFIG_MIC_I2S_AUTO_PROBE
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
    if (pcm_range <= 1 && raw_range < 65536) {
        return raw_range / 4;
    }
    return raw_range + ((uint32_t)pcm_range << 16);
}
#endif

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

static esp_err_t mic_apply_format(i2s_chan_handle_t rx_chan,
                                  i2s_std_config_t *std_cfg,
                                  const mic_fmt_candidate_t *fmt,
                                  const char *tag) {
    esp_err_t ret = i2s_channel_reconfig_std_slot(rx_chan, &std_cfg->slot_cfg);
    if (ret == ESP_OK) {
        ret = i2s_channel_reconfig_std_gpio(rx_chan, &std_cfg->gpio_cfg);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Format %s reconfig failed: %s", fmt->name, esp_err_to_name(ret));
        return ret;
    }
    ret = i2s_channel_enable(rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Format %s enable failed: %s", fmt->name, esp_err_to_name(ret));
    }
    return ret;
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
    upload_ring_init();

    if (GPIO_PIN_BEEP == MIC_I2S_DATA_GPIO) {
        ESP_LOGW(TAG, "BEEP=IO%d conflicts with MIC DATA, beep disabled", GPIO_PIN_BEEP);
    }
    gpio_reset_pin((gpio_num_t)MIC_I2S_DATA_GPIO);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = CONFIG_MIC_I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = CONFIG_MIC_I2S_DMA_FRAME_NUM;
    chan_cfg.auto_clear = true;

    i2s_chan_handle_t rx_chan = nullptr;
    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S channel create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    const mic_fmt_candidate_t fixed_fmt = {
        "Philips/STEREO/LEFT", mic_slot_philips_left(), false
    };

#if CONFIG_MIC_I2S_AUTO_PROBE
    const mic_fmt_candidate_t candidates[] = {
        { "Philips/STEREO/LEFT",      mic_slot_philips_left(),  false },
        { "MSB/STEREO/LEFT",          mic_slot_msb_left(),      false },
        { "Philips/STEREO/RIGHT",     mic_slot_philips_right(), false },
        { "MSB/STEREO/RIGHT",         mic_slot_msb_right(),     false },
        { "Philips/STEREO/LEFT/WS_INV", mic_slot_philips_left(), true },
    };
#endif

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
        .slot_cfg = fixed_fmt.slot,
        .gpio_cfg = gpio_cfg,
    };

    ret = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S std init failed: %s", esp_err_to_name(ret));
        i2s_del_channel(rx_chan);
        return ret;
    }

#if CONFIG_MIC_I2S_AUTO_PROBE
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
    ret = mic_apply_format(rx_chan, &std_cfg, &candidates[best_idx], TAG);
    if (ret != ESP_OK) {
        i2s_del_channel(rx_chan);
        return ret;
    }

    ESP_LOGI(TAG, "Selected format: %s (score=%lu)",
             candidates[best_idx].name, (unsigned long)best_score);
    if (best_score < 256) {
        ESP_LOGW(TAG, "Mic signal weak (score=%lu). Check 3.3V / WS=IO15 BCLK=IO16 DATA=IO3",
                 (unsigned long)best_score);
    }
#else
    ret = mic_apply_format(rx_chan, &std_cfg, &fixed_fmt, TAG);
    if (ret != ESP_OK) {
        i2s_del_channel(rx_chan);
        return ret;
    }
    ESP_LOGI(TAG, "Using fixed I2S format: %s", fixed_fmt.name);
#endif

    vTaskDelay(pdMS_TO_TICKS(50));
    mic_log_samples(rx_chan, TAG);

    i2s_rx_handle = rx_chan;
    isReady = true;
    ESP_LOGI(TAG, "SPH0645 ready: WS=IO%d BCLK=IO%d DATA=IO%d @ %dHz -> upload %dHz (decim=%d)",
             MIC_I2S_WS_GPIO, MIC_I2S_BCLK_GPIO, MIC_I2S_DATA_GPIO,
             MIC_SAMPLE_RATE, MIC_UPLOAD_SAMPLE_RATE, MIC_DECIM_FACTOR);
    return ESP_OK;
}

bool MEMS_MIC::app_mic_check_module() {
    return isReady && i2s_rx_handle != nullptr;
}

bool MEMS_MIC::read_raw_pcm(int16_t *buf, int nsamples) {
    if (!buf || nsamples <= 0 || !app_mic_check_module()) {
        return false;
    }

    int got = 0;
    while (got < nsamples) {
        int32_t chunk[64];
        int want = nsamples - got;
        if (want > (int)(sizeof(chunk) / sizeof(chunk[0]))) {
            want = (int)(sizeof(chunk) / sizeof(chunk[0]));
        }
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read((i2s_chan_handle_t)i2s_rx_handle,
                                       chunk, want * sizeof(int32_t), &bytes_read,
                                       pdMS_TO_TICKS(500));
        if (ret != ESP_OK || bytes_read == 0) {
            return got > 0;
        }
        int n = (int)(bytes_read / sizeof(int32_t));
        for (int i = 0; i < n; i++) {
            int16_t pcm = sph0645_raw_to_pcm(chunk[i]);
            buf[got++] = pcm;
            s_last_raw = chunk[i];
            app_mic_sr_on_pcm_sample(pcm);
        }
    }
    return true;
}

bool MEMS_MIC::read_sample_pcm(int16_t *out) {
    if (!out || !app_mic_check_module()) {
        return false;
    }

    if (s_sr_feed_active) {
        return app_mic_take_upload_sample(out, 500);
    }

    int32_t chunk[64];
    while (true) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read((i2s_chan_handle_t)i2s_rx_handle,
                                         chunk, sizeof(chunk), &bytes_read,
                                         pdMS_TO_TICKS(500));
        if (ret != ESP_OK || bytes_read == 0) {
            return false;
        }
        int n = (int)(bytes_read / sizeof(int32_t));
        for (int i = 0; i < n; i++) {
            int16_t ready = 0;
            s_last_raw = chunk[i];
            if (mic_push_decimated_sample(sph0645_raw_to_pcm(chunk[i]), &ready)) {
                *out = ready;
                latest_pcm = ready;
                return true;
            }
        }
    }
}
