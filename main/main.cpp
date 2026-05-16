#include "app_file.h"
#include "app_mpu5060.h"
#include "app_gpio.h"
#include "app_mic.h"
#include "app_wifi.h"
#include "app_extflash.h"
#include "driver/gpio.h"
#include <cstdio>
#include <inttypes.h>

static const char *TAG = "app_main";
APP_MPU6050 mpu6050("app_mpu6050"); // 类变量放在外层，防止因main函数结束受影响
Button button("app_button");
IMU_Data imu_data("app_imu_data");
MEMS_MIC mic("app_mic");
MIC_Data mic_data("app_mic_data");
ExtFlash ext_flash("app_extflash");

// WiFi配置
#define WIFI_SSID "清和科技"
#define WIFI_PASSWORD "qinghekeji"

/**
 * @brief 外部Flash测试函数
 */
void test_extflash() {
    printf("\n\n=== EXTERNAL FLASH TEST START ===\n");
    printf("External Flash module is disabled\n");
    printf("To enable, please implement the SPI driver properly\n");
    printf("=== EXTERNAL FLASH TEST END ===\n\n");
}

extern "C" void app_main(void) {
  // 测试功能：烧录后立刻配置IO引脚
  // IO14 拉高 (WP引脚)
  gpio_config_t io14_config = {
      .pin_bit_mask = (1ULL << 14),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io14_config);
  gpio_set_level((gpio_num_t)14, 1);
  printf("Test: IO14 pin set to HIGH\n");
  
  // IO21 拉低
  gpio_config_t io21_config = {
      .pin_bit_mask = (1ULL << 21),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io21_config);
  gpio_set_level((gpio_num_t)21, 0);
  printf("Test: IO21 pin set to LOW\n");
  
  // ========== 外部Flash测试 ==========
  test_extflash();
  
  // ========== WiFi自动连接 ==========
  printf("\n\n=== WiFi连接开始 ===\n");
  printf("SSID: %s\n", WIFI_SSID);
  printf("正在连接...\n");
  app_wifi_init_sta(WIFI_SSID, WIFI_PASSWORD);
  printf("=== WiFi连接结束 ===\n\n");
  
  // 初始化
  button.app_btn_init(); // 按钮
  imu_data.imu_data_init(10); // IMU文件系统
  mpu6050.app_mpu6050_init(); // IMU传感器
  mic.app_mic_init(); // 麦克风
  mic_data.mic_data_init(0); // 麦克风文件系统
  
  // ========== 麦克风测试功能 ==========
  printf("\n\n=== MICROPHONE TEST START ===\n");
  MEMS_MIC::adc_read_test(&mic);
  printf("=== MICROPHONE TEST END ===\n\n");
  
  // 任务创建
  button.app_btn_create_task();
  mpu6050.app_mpu6050_create_task();
  imu_data.app_file_create_task();
  mic.app_mic_create_task();      // 麦克风采集任务
  mic_data.app_file_create_task(); // 麦克风写入任务
}
