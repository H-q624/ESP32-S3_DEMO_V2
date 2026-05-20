#ifndef _APP_BUTTON_H_
#define _APP_BUTTON_H_

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define CONTINUE_INTERVAL_MS 10000 // 暂定10秒
#define LONG_THRESHOlD 500  // 500ms

#define GPIO_PIN_BTN CONFIG_BTN_PIN_IO
#define GPIO_PIN_BEEP CONFIG_BEEP_PIN_IO
#define GPIO_PIN_LED CONFIG_LED_PIN_IO

// 模式切换事件组位定义
#define MODE_SWITCH_REQUEST_BIT   (1 << 0)  // 请求模式切换
#define MODE_SWITCH_IMU_READY_BIT (1 << 1)  // IMU 数据保存完成
#define MODE_SWITCH_MIC_READY_BIT (1 << 2)  // MIC 数据保存完成
#define MODE_SWITCH_ALL_READY_BITS (MODE_SWITCH_IMU_READY_BIT | MODE_SWITCH_MIC_READY_BIT)

// 模式切换超时时间（毫秒）
#define MODE_SWITCH_TIMEOUT_MS    2000

extern bool global_btn_sign; // 直接采用全局标志位传递数据

// 全局事件组，用于模式切换同步
extern EventGroupHandle_t mode_switch_event_group;

typedef enum{
    BTN_IDLE=0,        // 空闲状态
    BTN_PRESSED,       // 按钮按下状态
    BTN_LONG_PRESS,    // 长按状态
    BTN_SHORT_PRESS,   // 短按状态
    BTN_MAX
}btn_status;

class Button {
private:
    const char* TAG;
    uint32_t timestamp; // 按钮按下时间戳
    btn_status status;  // 当前按钮状态
    bool isReady;       // 组件状态
    int last_pin_level; // 前一次状态
    int pin_level;      // 当前状态
    uint32_t press_start_time; // 按压开始时间
    uint32_t press_duration;   // 按压持续时间
    bool data_mode;        // 当前数据模式：true=MPU, false=MIC
    uint32_t long_press_threshold; // 长按阈值（毫秒）
    
public:
    Button(const char* tag);
    ~Button();
    void app_btn_init(); // 初始化
    bool app_btn_check_module(); //检查状态
    void app_btn_create_task(); // 创建任务扫描按键状态
    void run_once(); // 任务函数的一次运行
    
    // 新增接口
    bool is_long_press(); // 检查是否为长按
    bool is_short_press(); // 检查是否为短按
    void toggle_data_mode(); // 切换数据模式
    bool get_current_data_mode(); // 获取当前数据模式
};

extern Button button;

// 全局函数
extern "C" bool get_current_data_mode();
extern "C" void set_buzzer_on();
extern "C" void set_buzzer_off();

#endif 