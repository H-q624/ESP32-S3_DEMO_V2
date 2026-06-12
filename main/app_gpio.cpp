#include "app_gpio.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_num.h"

#define GPIO_PIN_MASK_IN (1ULL << GPIO_PIN_BTN)
#define GPIO_PIN_MASK_OUT ((1ULL << GPIO_PIN_LED) | \
    ((GPIO_PIN_BEEP >= 0) ? (1ULL << GPIO_PIN_BEEP) : 0ULL))

bool global_btn_sign = false;
TaskHandle_t gpio_handler = nullptr;
Button button("app_button");

// 全局事件组定义
EventGroupHandle_t mode_switch_event_group = nullptr;
/**
 * @brief 构造函数，初始化变量
 */
Button::Button(const char *tag)
    : timestamp(0), status(BTN_IDLE), long_press_threshold(LONG_THRESHOlD) {
  TAG = tag;
  last_pin_level = 1;
  pin_level = 1;
  isReady = false;
  data_mode = true; // 初始化设置为IMU
}
/**
 * @brief 析构函数，可由编译器自动生成
 */
Button::~Button() {
  timestamp = 0;
  last_pin_level = 1;
  pin_level = 1;
  isReady = false;
}

/**
 * @brief 初始化按钮对应引脚
 */
void Button::app_btn_init() {
  gpio_config_t config_in = {.pin_bit_mask = GPIO_PIN_MASK_IN,
                          .mode = GPIO_MODE_INPUT,
                          .pull_up_en = GPIO_PULLUP_ENABLE,
                          .pull_down_en = GPIO_PULLDOWN_DISABLE,
                          .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&config_in);
  gpio_config_t config_out = {.pin_bit_mask = GPIO_PIN_MASK_OUT,
                              .mode = GPIO_MODE_OUTPUT,
                              .pull_up_en = GPIO_PULLUP_DISABLE,
                              .pull_down_en = GPIO_PULLDOWN_DISABLE,
                              .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&config_out);
  ESP_LOGI(TAG, "GPIO Set successfully");
  
  // 创建模式切换事件组
  if (mode_switch_event_group == nullptr) {
    mode_switch_event_group = xEventGroupCreate();
    if (mode_switch_event_group == nullptr) {
      ESP_LOGE(TAG, "Failed to create mode switch event group");
      return;
    }
    ESP_LOGI(TAG, "Mode switch event group created");
  }
  
  isReady = true;
}
/**
 * @brief 检查组件状态
 */
bool Button::app_btn_check_module() {
  if (!isReady)
    ESP_LOGD(TAG, "gpio is not ready");
  return isReady;
}
/**
 * @brief freertos任务函数
 */
static void gpio_task_function(void *arg) {
  Button *instance = static_cast<Button *>(arg);
  while (1) {
    if (instance->app_btn_check_module())
      instance->run_once();
    vTaskDelay(pdMS_TO_TICKS(10)); // 延时10ms，基础消抖
  }
  vTaskDelete(NULL);
}
/**
 * @brief 运行函数
 */
void Button::run_once() {
  // 获取当前level
  pin_level = gpio_get_level((gpio_num_t)GPIO_PIN_BTN);
  switch (status) {
  case BTN_IDLE:
    if (last_pin_level == 1 && pin_level == 0) {
      // 按钮按下
      status = BTN_PRESSED;
      press_start_time = esp_timer_get_time() / 1000;
    }
    break;
  case BTN_PRESSED:
    press_duration = esp_timer_get_time() / 1000 - press_start_time;
    if (pin_level == 1) {
      // 按钮释放
      if (press_duration < long_press_threshold) {
        status = BTN_SHORT_PRESS;
        global_btn_sign = true;
      } else {
        status = BTN_LONG_PRESS;
        toggle_data_mode();
      }
      status = BTN_IDLE;
    }
    break;
  default:
    status = BTN_IDLE;
    break;
  }
  last_pin_level = pin_level;
}
/**
 * @brief 任务函数
 */
void Button::app_btn_create_task() {
  if (nullptr == gpio_handler) {
    xTaskCreate(gpio_task_function, "gpio task", 2048, this, 6, &gpio_handler);
    ESP_LOGI(TAG, "gpio task create successfully");
  } else {
    ESP_LOGW(TAG, "gpio task is already exists");
  }
}

/** 
* @brief 返回按钮当前状态 
*/
bool Button::is_long_press(){
  return status == BTN_LONG_PRESS ? true : false;
}
bool Button::is_short_press(){
  return status == BTN_SHORT_PRESS ? true: false;
}
/**
* @brief 切换数据模式（使用事件同步，确保数据保存完成后再切换）
*/
void Button::toggle_data_mode(){
  // 长按是切换数据模式
  if(is_long_press()) {
    bool target_mode = !data_mode;
    
    ESP_LOGI(TAG, "Mode switch requested: %s -> %s", 
             data_mode ? "IMU" : "MIC",
             target_mode ? "IMU" : "MIC");
    
    // 清除之前的事件位
    if (mode_switch_event_group != nullptr) {
      xEventGroupClearBits(mode_switch_event_group, MODE_SWITCH_ALL_READY_BITS);
      
      // 发送模式切换请求（设置请求位）
      xEventGroupSetBits(mode_switch_event_group, MODE_SWITCH_REQUEST_BIT);
      
      ESP_LOGI(TAG, "Waiting for data save completion...");
      
      // 等待 IMU 和 MIC 都完成数据保存（带超时）
      EventBits_t bits = xEventGroupWaitBits(
          mode_switch_event_group,
          MODE_SWITCH_ALL_READY_BITS,
          pdTRUE,  // 清除事件位
          pdTRUE,  // 等待所有位都设置
          pdMS_TO_TICKS(MODE_SWITCH_TIMEOUT_MS)
      );
      
      if ((bits & MODE_SWITCH_ALL_READY_BITS) == MODE_SWITCH_ALL_READY_BITS) {
        ESP_LOGI(TAG, "All data saved, switching mode");
      } else {
        ESP_LOGW(TAG, "Mode switch timeout, some data may be lost");
      }
    }
    
    // 实际切换模式
    data_mode = target_mode;
    
    if(data_mode) {
      // 切换回MPU模式
      gpio_set_level((gpio_num_t)GPIO_PIN_LED, 0);
      ESP_LOGI(TAG, "Switched to MPU mode");
    } else {
      // 切换到MIC模式
      gpio_set_level((gpio_num_t)GPIO_PIN_LED, 1);
      ESP_LOGI(TAG, "Switched to MIC mode");
    }
    
    // 清除请求位，允许下一次切换
    if (mode_switch_event_group != nullptr) {
      xEventGroupClearBits(mode_switch_event_group, MODE_SWITCH_REQUEST_BIT);
    }
  }
}

/**
* @brief 获取数据模式 
*/
bool Button::get_current_data_mode(){
  return data_mode;
}

/**
* @brief 全局访问函数，获取状态 
*/
extern "C" bool get_current_data_mode(){
  return button.get_current_data_mode();
}

/*********************** BEEP ************************/
// 初始化在app_btn_init中一起完成
void set_buzzer_on(){
  if (GPIO_PIN_BEEP < 0) return;
  gpio_set_level((gpio_num_t)GPIO_PIN_BEEP, 1);
}
void set_buzzer_off(){
  if (GPIO_PIN_BEEP < 0) return;
  gpio_set_level((gpio_num_t)GPIO_PIN_BEEP, 0);
}

/*********************** 多按键支持 ************************/
/*
 * SW1 → IO41 (功能键1, 低电平有效, 内部上拉)
 * SW3 → IO40 (功能键2, 低电平有效, 内部上拉)
 * SW6 → IO45 (隐私模式拨动开关, 高=隐私开, 低=隐私关, 内部下拉)
 */

static int s_sw1_last = 1;
static int s_sw3_last = 1;

void app_keys_init(void) {
    /* SW1 + SW3: 按键输入, 上拉, 按下为低 */
    gpio_config_t key_cfg = {
        .pin_bit_mask = (1ULL << GPIO_KEY_SW1) | (1ULL << GPIO_KEY_SW3),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&key_cfg);

    /* SW6: 隐私模式拨动开关, 下拉, 拨到ON为高 */
    gpio_config_t sw6_cfg = {
        .pin_bit_mask = (1ULL << GPIO_KEY_SW6),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&sw6_cfg);

    s_sw1_last = gpio_get_level((gpio_num_t)GPIO_KEY_SW1);
    s_sw3_last = gpio_get_level((gpio_num_t)GPIO_KEY_SW3);

    ESP_LOGI("KEYS", "Keys ready: SW1=IO%d, SW3=IO%d, SW6(隐私)=IO%d",
             GPIO_KEY_SW1, GPIO_KEY_SW3, GPIO_KEY_SW6);
}

/* SW1 短按检测 (下降沿触发, 调用一次返回1, 之后清0) */
int app_key_sw1_pressed(void) {
    int cur = gpio_get_level((gpio_num_t)GPIO_KEY_SW1);
    int pressed = (s_sw1_last == 1 && cur == 0);
    s_sw1_last = cur;
    return pressed;
}

/* SW3 短按检测 */
int app_key_sw3_pressed(void) {
    int cur = gpio_get_level((gpio_num_t)GPIO_KEY_SW3);
    int pressed = (s_sw3_last == 1 && cur == 0);
    s_sw3_last = cur;
    return pressed;
}

/* SW6 隐私模式: 高电平 = 隐私开启 */
bool app_key_privacy_mode(void) {
    return gpio_get_level((gpio_num_t)GPIO_KEY_SW6) == 1;
}

void app_led_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << GPIO_PIN_LED) | (1ULL << GPIO_PIN_LED2),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level((gpio_num_t)GPIO_PIN_LED, 0);
    gpio_set_level((gpio_num_t)GPIO_PIN_LED2, 0);
    ESP_LOGI("LED", "LED ready: LED1=IO%d LED2=IO%d", GPIO_PIN_LED, GPIO_PIN_LED2);
}

void app_led_set(int led_index, bool on) {
    gpio_num_t pin = (led_index == 0) ? (gpio_num_t)GPIO_PIN_LED : (gpio_num_t)GPIO_PIN_LED2;
    gpio_set_level(pin, on ? 1 : 0);
}