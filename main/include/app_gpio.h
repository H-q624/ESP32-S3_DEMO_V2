#ifndef _APP_BUTTON_H_
#define _APP_BUTTON_H_

#include "sdkconfig.h"
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define CONTINUE_INTERVAL_MS 10000
#define LONG_THRESHOlD 500

#define GPIO_PIN_BTN  CONFIG_BTN_PIN_IO
#define GPIO_PIN_BEEP CONFIG_BEEP_PIN_IO
#define GPIO_PIN_LED  CONFIG_LED_PIN_IO
#define GPIO_PIN_LED2 CONFIG_LED2_PIN_IO

#define GPIO_KEY_SW1  CONFIG_KEY_SW1_PIN
#define GPIO_KEY_SW3  CONFIG_KEY_SW3_PIN
#define GPIO_KEY_SW6  CONFIG_KEY_SW6_PIN
#define GPIO_KEY_NEW1  CONFIG_KEY_NEW1_PIN
#define GPIO_KEY_NEW2  CONFIG_KEY_NEW2_PIN
#ifndef CONFIG_PA_CTRL_GPIO
#define CONFIG_PA_CTRL_GPIO -1
#endif
#define GPIO_PA_CTRL    CONFIG_PA_CTRL_GPIO

typedef enum {
    KEY_IDLE = 0,
    KEY_PRESSED,
    KEY_LONG_PRESS,
    KEY_SHORT_PRESS,
} key_status_t;

#define MODE_SWITCH_REQUEST_BIT   (1 << 0)
#define MODE_SWITCH_IMU_READY_BIT (1 << 1)
#define MODE_SWITCH_MIC_READY_BIT (1 << 2)
#define MODE_SWITCH_ALL_READY_BITS (MODE_SWITCH_IMU_READY_BIT | MODE_SWITCH_MIC_READY_BIT)
#define MODE_SWITCH_TIMEOUT_MS    2000

extern bool global_btn_sign;
extern EventGroupHandle_t mode_switch_event_group;

typedef enum {
    BTN_IDLE = 0,
    BTN_PRESSED,
    BTN_LONG_PRESS,
    BTN_SHORT_PRESS,
    BTN_MAX
} btn_status;

class Button {
private:
    const char *TAG;
    uint32_t timestamp;
    btn_status status;
    bool isReady;
    int last_pin_level;
    int pin_level;
    uint32_t press_start_time;
    uint32_t press_duration;
    bool data_mode;
    uint32_t long_press_threshold;

public:
    Button(const char *tag);
    ~Button();
    void app_btn_init();
    bool app_btn_check_module();
    void app_btn_create_task();
    void run_once();
    bool is_long_press();
    bool is_short_press();
    void toggle_data_mode();
    bool get_current_data_mode();
};

extern Button button;

void app_keys_init(void);
void app_led_init(void);
void app_led_set(int led_index, bool on);
bool app_key_privacy_mode(void);

extern "C" bool get_current_data_mode();
extern "C" void set_buzzer_on();
extern "C" void set_buzzer_off();

#endif
