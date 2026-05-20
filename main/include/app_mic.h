#ifndef APP_MIC_H
#define APP_MIC_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MIC_SAMPLE_RATE 16000
#define MIC_UPLOAD_SAMPLE_RATE 50
#define MIC_ADC_UNIT ADC_UNIT_2
#define MIC_ADC_CHANNEL ADC_CHANNEL_2
#define MIC_ADC_ATTEN ADC_ATTEN_DB_11
#define MIC_BUFFER_SIZE 1024
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

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "esp_adc/adc_oneshot.h"

class MEMS_MIC {
public:
    MEMS_MIC(const char *tag);
    ~MEMS_MIC();

    esp_err_t app_mic_init();
    bool app_mic_check_module();
    bool read_sample_pcm(int16_t *out);

    static void adc_read_test(MEMS_MIC *instance);

private:
    const char *TAG;
    bool isReady;
    adc_oneshot_unit_handle_t adc_handle;
    adc_channel_t adc_channel_num;
    adc_unit_t adc_unit_num;
    int16_t sample_buffer[MIC_BUFFER_SIZE];
};

#endif /* __cplusplus */

#endif /* APP_MIC_H */
