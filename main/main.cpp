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
    
    // 初始化外部Flash
    esp_err_t ret = ext_flash.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "External Flash initialization failed");
        printf("=== EXTERNAL FLASH TEST FAILED ===\n\n");
        return;
    }
    
    // 读取设备ID
    uint32_t id;
    ret = ext_flash.read_id(&id);
    if (ret == ESP_OK) {
        printf("Flash ID: 0x%06" PRIX32 "\n", id);
        // W25Q64JVSIQ 的制造商ID是0xEF，设备ID是0x4017
        if ((id & 0xFF) == 0x17 && ((id >> 8) & 0xFF) == 0x40 && ((id >> 16) & 0xFF) == 0xEF) {
            printf("Detected: Winbond W25Q64JVSIQ (8MB)\n");
        } else {
            printf("Unknown flash device\n");
        }
    }
    
    // 测试读写
    uint8_t test_data[256];
    uint8_t read_back[256];
    
    // 准备测试数据
    for (int i = 0; i < 256; i++) {
        test_data[i] = i;
    }
    
    // 擦除扇区
    printf("Erasing sector at 0x10000...\n");
    ret = ext_flash.erase_sector(0x10000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sector erase failed");
        printf("=== EXTERNAL FLASH TEST FAILED ===\n\n");
        return;
    }
    printf("Sector erase completed\n");
    
    // 写入数据
    printf("Writing test data to 0x10000...\n");
    ret = ext_flash.write(0x10000, test_data, 256);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write failed");
        printf("=== EXTERNAL FLASH TEST FAILED ===\n\n");
        return;
    }
    printf("Write completed\n");
    
    // 读取数据
    printf("Reading data from 0x10000...\n");
    ret = ext_flash.read(0x10000, read_back, 256);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed");
        printf("=== EXTERNAL FLASH TEST FAILED ===\n\n");
        return;
    }
    printf("Read completed\n");
    
    // 验证数据
    bool verify_ok = true;
    for (int i = 0; i < 256; i++) {
        if (read_back[i] != test_data[i]) {
            printf("Data mismatch at offset %d: expected 0x%02X, got 0x%02X\n", i, test_data[i], read_back[i]);
            verify_ok = false;
            break;
        }
    }
    
    if (verify_ok) {
        printf("Data verification PASSED!\n");
        printf("External Flash W25Q64JVSIQ is working correctly\n");
        printf("Connection: CS=IO12, SCK=IO11, MOSI=IO10, MISO=IO13, WP=IO14\n");
    } else {
        printf("Data verification FAILED!\n");
    }
    
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
