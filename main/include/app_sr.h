#ifndef APP_SR_H
#define APP_SR_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_SR_EVT_WAKEWORD = 1,
    APP_SR_EVT_CMD_ALARM = 2,
    APP_SR_EVT_CMD_HELP = 3,
    APP_SR_EVT_CMD_MESSAGE = 4,
    APP_SR_EVT_TIMEOUT = 5,
} app_sr_event_t;

typedef void (*app_sr_event_cb_t)(app_sr_event_t evt, int command_id, const char *command_str);

esp_err_t app_sr_start(app_sr_event_cb_t callback);
void app_sr_stop(void);
bool app_sr_is_running(void);

/** Non-blocking: returns true and fills evt if an SR event is pending. */
bool app_sr_poll_event(app_sr_event_t *evt, int *command_id);

#ifdef __cplusplus
class MEMS_MIC;
void app_sr_bind_mic(MEMS_MIC *mic);
#endif

#ifdef __cplusplus
}
#endif

#endif /* APP_SR_H */
