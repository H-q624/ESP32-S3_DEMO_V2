#include "app_mic.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

QueueHandle_t wav_queue = nullptr;

static int16_t s_pcm_ring[AUDIO_SAMPLES_PER_UPLOAD];
static size_t s_ring_len = 0;

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
    if (!out || max_samples == 0) {
        return 0;
    }
    size_t n = s_ring_len < max_samples ? s_ring_len : max_samples;
    if (n > 0) {
        memcpy(out, s_pcm_ring, n * sizeof(int16_t));
    }
    return n;
}

static int adc_read_raw(adc_oneshot_unit_handle_t adc_handle,
                        adc_channel_t channel) {
    int raw_value = 2048;
    adc_oneshot_read(adc_handle, channel, &raw_value);
    return raw_value;
}

MEMS_MIC::MEMS_MIC(const char *tag)
    : TAG(tag), isReady(false), adc_handle(nullptr),
      adc_channel_num(MIC_ADC_CHANNEL), adc_unit_num(MIC_ADC_UNIT) {
    memset(sample_buffer, 0, sizeof(sample_buffer));
}

MEMS_MIC::~MEMS_MIC() {
    if (adc_handle != nullptr) {
        adc_oneshot_del_unit(adc_handle);
        adc_handle = nullptr;
    }
}

esp_err_t MEMS_MIC::app_mic_init() {
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = adc_unit_num,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&adc_cfg, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = MIC_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_oneshot_config_channel(adc_handle, adc_channel_num, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MIC ready on IO15 (ADC2), upload sync %d Hz",
             MIC_UPLOAD_SAMPLE_RATE);
    isReady = true;
    return ESP_OK;
}

bool MEMS_MIC::app_mic_check_module() { return isReady && adc_handle != nullptr; }

bool MEMS_MIC::read_sample_pcm(int16_t *out) {
    if (!out || !app_mic_check_module()) {
        return false;
    }
    int raw = adc_read_raw(adc_handle, adc_channel_num);
    *out = (int16_t)((raw - 2048) * 16);
    return true;
}

void MEMS_MIC::adc_read_test(MEMS_MIC *instance) {
    if (!instance) {
        return;
    }
    ESP_LOGI(instance->TAG, "MIC ADC test (sync mode, no background task)");
    int16_t pcm = 0;
    for (int i = 0; i < 100; i++) {
        instance->read_sample_pcm(&pcm);
        if (i % 20 == 0) {
            ESP_LOGI(instance->TAG, "sample[%d] pcm=%d", i, pcm);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
