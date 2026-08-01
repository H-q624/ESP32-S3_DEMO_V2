#include "app_gpio.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/gpio_num.h"

#define GPIO_PIN_MASK_IN (1ULL << GPIO_PIN_BTN)

static bool gpio_output_available(int pin) {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        return false;
    }
#if CONFIG_EXTFLASH_ENABLE
    if (pin == CONFIG_EXTFLASH_CS_PIN ||
        pin == CONFIG_EXTFLASH_CLK_PIN ||
        pin == CONFIG_EXTFLASH_MOSI_PIN ||
        pin == CONFIG_EXTFLASH_MISO_PIN) {
        return false;
    }
#endif
    return true;
}

static uint64_t gpio_output_mask(int pin) {
    return gpio_output_available(pin) ? (1ULL << (unsigned)pin) : 0ULL;
}

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
  uint64_t output_mask = gpio_output_mask(GPIO_PIN_LED) |
                         gpio_output_mask(GPIO_PIN_BEEP);
  gpio_config_t config_out = {.pin_bit_mask = output_mask,
                              .mode = GPIO_MODE_OUTPUT,
                              .pull_up_en = GPIO_PULLUP_DISABLE,
                              .pull_down_en = GPIO_PULLDOWN_DISABLE,
                              .intr_type = GPIO_INTR_DISABLE};
  if (output_mask != 0) {
    gpio_config(&config_out);
  }
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
    
    if(data_mode && gpio_output_available(GPIO_PIN_LED)) {
      // 切换回MPU模式
      gpio_set_level((gpio_num_t)GPIO_PIN_LED, 0);
      ESP_LOGI(TAG, "Switched to MPU mode");
    } else if (gpio_output_available(GPIO_PIN_LED)) {
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

#define KEY_DEBOUNCE_MS 30
#define KEY_RETRIGGER_GUARD_MS 50

typedef struct {
    gpio_num_t gpio_num;
    TickType_t interrupt_tick;
} key_interrupt_event_t;

static QueueHandle_t s_key_event_queue = nullptr;
static TaskHandle_t s_key_event_task = nullptr;
static bool s_keys_initialized = false;

static void key_gpio_isr(void *arg) {
    if (s_key_event_queue == nullptr) {
        return;
    }

    key_interrupt_event_t event = {
        .gpio_num = static_cast<gpio_num_t>(reinterpret_cast<uintptr_t>(arg)),
        .interrupt_tick = xTaskGetTickCountFromISR(),
    };
    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueSendFromISR(s_key_event_queue, &event, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void key_event_task(void *arg) {
    (void)arg;
    TickType_t last_io41_tick = 0;
    TickType_t last_io40_tick = 0;
    bool io41_seen = false;
    bool io40_seen = false;
    key_interrupt_event_t event;

    while (true) {
        if (xQueueReceive(s_key_event_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        TickType_t *last_tick = nullptr;
        bool *seen = nullptr;
        if (event.gpio_num == static_cast<gpio_num_t>(GPIO_KEY_SW1)) {
            last_tick = &last_io41_tick;
            seen = &io41_seen;
        } else if (event.gpio_num == static_cast<gpio_num_t>(GPIO_KEY_SW3)) {
            last_tick = &last_io40_tick;
            seen = &io40_seen;
        } else {
            continue;
        }

        if (*seen &&
            (event.interrupt_tick - *last_tick) < pdMS_TO_TICKS(KEY_RETRIGGER_GUARD_MS)) {
            continue;
        }

        /* 软件消抖：下降沿触发后 30ms，按键仍为低电平才确认。 */
        vTaskDelay(pdMS_TO_TICKS(KEY_DEBOUNCE_MS));
        if (gpio_get_level(event.gpio_num) != 0) {
            continue;
        }

        *last_tick = event.interrupt_tick;
        *seen = true;
        if (event.gpio_num == static_cast<gpio_num_t>(GPIO_KEY_SW1)) {
            ESP_LOGI("main", "io41按下");
        } else {
            ESP_LOGI("main", "io40按下");
        }
    }
}

void app_keys_init(void) {
    if (s_keys_initialized) {
        return;
    }

    /* SW1 + SW3: 按键输入, 上拉, 按下为低 */
    gpio_config_t key_cfg = {
        .pin_bit_mask = (1ULL << GPIO_KEY_SW1) | (1ULL << GPIO_KEY_SW3),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    esp_err_t ret = gpio_config(&key_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE("KEYS", "Key GPIO config failed: %s", esp_err_to_name(ret));
        return;
    }

    /* SW6: 隐私模式拨动开关, 下拉, 拨到ON为高 */
    gpio_config_t sw6_cfg = {
        .pin_bit_mask = (1ULL << GPIO_KEY_SW6),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&sw6_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE("KEYS", "SW6 GPIO config failed: %s", esp_err_to_name(ret));
        return;
    }

    s_key_event_queue = xQueueCreate(8, sizeof(key_interrupt_event_t));
    if (s_key_event_queue == nullptr) {
        ESP_LOGE("KEYS", "Failed to create key interrupt queue");
        return;
    }

    if (xTaskCreate(key_event_task, "key_event", 2048, nullptr, 6,
                    &s_key_event_task) != pdPASS) {
        ESP_LOGE("KEYS", "Failed to create key event task");
        return;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("KEYS", "GPIO ISR service install failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = gpio_isr_handler_add(
        static_cast<gpio_num_t>(GPIO_KEY_SW1), key_gpio_isr,
        reinterpret_cast<void *>(static_cast<uintptr_t>(GPIO_KEY_SW1)));
    if (ret != ESP_OK) {
        ESP_LOGE("KEYS", "IO%d ISR add failed: %s", GPIO_KEY_SW1, esp_err_to_name(ret));
        return;
    }

    ret = gpio_isr_handler_add(
        static_cast<gpio_num_t>(GPIO_KEY_SW3), key_gpio_isr,
        reinterpret_cast<void *>(static_cast<uintptr_t>(GPIO_KEY_SW3)));
    if (ret != ESP_OK) {
        ESP_LOGE("KEYS", "IO%d ISR add failed: %s", GPIO_KEY_SW3, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI("KEYS", "Keys ready: SW1=IO%d, SW3=IO%d, SW6(隐私)=IO%d",
             GPIO_KEY_SW1, GPIO_KEY_SW3, GPIO_KEY_SW6);
    s_keys_initialized = true;
}

/* SW6 隐私模式: 高电平 = 隐私开启 */
bool app_key_privacy_mode(void) {
    return gpio_get_level((gpio_num_t)GPIO_KEY_SW6) == 1;
}

void app_led_init(void) {
    uint64_t led_mask = gpio_output_mask(GPIO_PIN_LED) |
                        gpio_output_mask(GPIO_PIN_LED2);
    gpio_config_t cfg = {
        .pin_bit_mask = led_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (led_mask != 0) {
        gpio_config(&cfg);
    }
    if (gpio_output_available(GPIO_PIN_LED)) {
        gpio_set_level((gpio_num_t)GPIO_PIN_LED, 0);
    }
    if (gpio_output_available(GPIO_PIN_LED2)) {
        gpio_set_level((gpio_num_t)GPIO_PIN_LED2, 0);
    }
    /* PA_CTRL (IO21) 上电默认拉低，避免功放上电响爆音 */
    if (GPIO_PA_CTRL >= 0) {
        gpio_set_level((gpio_num_t)GPIO_PA_CTRL, 0);
        ESP_LOGI("LED", "PA_CTRL=IO%d pulled low", GPIO_PA_CTRL);
    }
    ESP_LOGI("LED", "LED ready: LED1=IO%d LED2=IO%d", GPIO_PIN_LED, GPIO_PIN_LED2);
}

void app_led_set(int led_index, bool on) {
    gpio_num_t pin = (led_index == 0) ? (gpio_num_t)GPIO_PIN_LED : (gpio_num_t)GPIO_PIN_LED2;
    if (gpio_output_available((int)pin)) {
        gpio_set_level(pin, on ? 1 : 0);
    }
}
