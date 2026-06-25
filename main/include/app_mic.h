#ifndef APP_MIC_H
#define APP_MIC_H

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MIC_SAMPLE_RATE        CONFIG_MIC_SAMPLE_RATE
#define MIC_UPLOAD_SAMPLE_RATE 50
#define MIC_I2S_WS_GPIO        CONFIG_MIC_I2S_WS_GPIO
#define MIC_I2S_BCLK_GPIO      CONFIG_MIC_I2S_CLK_GPIO
#define MIC_I2S_DATA_GPIO      CONFIG_MIC_I2S_DATA_GPIO
#define MIC_BUFFER_SIZE        1024
#define AUDIO_SAMPLES_PER_UPLOAD 3000

#define WAV_HEADER_SIZE 44
typedef struct {
    uint8_t *data;
    size_t total_size;
    size_t pcm_data_size;
    uint32_t timestamp;
    bool is_wav_header_included;
} mic_wav_packet_t;

extern QueueHandle_t wav_queue;

void app_mic_reset_ring(void);
size_t app_mic_get_upload_pcm(int16_t *out, size_t max_samples);
bool app_mic_append_sample(int16_t sample);
int32_t app_mic_get_last_raw(void);

/** True while ESP-SR feed task owns I2S reads (upload path uses decimated samples). */
bool app_mic_sr_feed_active(void);

/** Block until a decimated upload sample is ready (used when SR feed is active). */
bool app_mic_take_upload_sample(int16_t *out, uint32_t timeout_ms);

/** Called by app_sr when feed/detect tasks start or stop. */
void app_mic_sr_set_feed_active(bool active);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class MEMS_MIC {
public:
    MEMS_MIC(const char *tag);
    ~MEMS_MIC();

    esp_err_t app_mic_init();
    bool app_mic_check_module();
    bool read_sample_pcm(int16_t *out);

    /** Read nsamples of 16 kHz mono PCM from I2S (for ESP-SR feed task). */
    bool read_raw_pcm(int16_t *buf, int nsamples);

    void *get_i2s_handle() const { return i2s_rx_handle; }

private:
    const char *TAG;
    bool isReady;
    void *i2s_rx_handle;
    int16_t latest_pcm;
    int32_t decim_counter;
};

#endif /* __cplusplus */

#endif /* APP_MIC_H */
