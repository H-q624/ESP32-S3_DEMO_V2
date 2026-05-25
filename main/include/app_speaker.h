#ifndef APP_SPEAKER_H
#define APP_SPEAKER_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#define SPEAKER_GPIO        10
#define SPEAKER_SAMPLE_RATE 16000  /* must match audio source */

esp_err_t speaker_init(void);
void      speaker_beep(uint32_t freq_hz, uint32_t duration_ms);
void      speaker_play_pcm16(const int16_t *samples, size_t num_samples, uint32_t sample_rate_hz);
void      speaker_off(void);

#endif
