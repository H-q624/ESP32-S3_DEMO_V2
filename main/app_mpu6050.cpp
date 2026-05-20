#include "app_mpu5060.h"
#include "driver/gpio.h"

QueueHandle_t data_queue = nullptr;
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief 默认构造函数
 */
APP_MPU6050::APP_MPU6050(const char *tag)
    : acce_fs(ACCE_FS_4G), gyro_fs(GYRO_FS_500DPS) {
  deviceid = 0;
  handle = nullptr;
  isReady = false;
  TAG = tag;
}

APP_MPU6050::APP_MPU6050(mpu6050_acce_fs_t set_acce_fs,
                         mpu6050_gyro_fs_t set_gyro_fs, const char *tag) {
  acce_fs = set_acce_fs;
  gyro_fs = set_gyro_fs;
  deviceid = 0;
  handle = nullptr;
  isReady = false;
  TAG = tag;
}

APP_MPU6050::~APP_MPU6050() {
  isReady = false;
  if (handle) {
    mpu6050_delete(handle);
    handle = nullptr;
  }
}

static bool is_mpu6050_who_am_i(uint8_t id) {
  return id == MPU6050_WHO_AM_I_VAL || id == MPU6050_WHO_AM_I_CLONE_VAL;
}

static esp_err_t try_wake_and_verify(mpu6050_handle_t handle, uint8_t *deviceid) {
  esp_err_t ret = mpu6050_wake_up(handle);
  if (ret != ESP_OK) {
    return ret;
  }
  return mpu6050_get_deviceid(handle, deviceid);
}

bool APP_MPU6050::app_mpu6050_init() {
  i2c_config_t config = {.mode = I2C_MODE_MASTER,
                         .sda_io_num = I2C_MASTER_SDA_IO,
                         .scl_io_num = I2C_MASTER_SCL_IO,
                         .sda_pullup_en = GPIO_PULLUP_ENABLE,
                         .scl_pullup_en = GPIO_PULLUP_ENABLE,
                         .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL};
  config.master.clk_speed = I2C_MASTER_FREQ_HZ;

  esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
    return false;
  }

  ret = i2c_driver_install(I2C_MASTER_NUM, config.mode, 0, 0, 0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "I2C init OK (SDA=%d, SCL=%d)", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

  gpio_config_t int_cfg = {.pin_bit_mask = (1ULL << MPU6050_INT_GPIO),
                           .mode = GPIO_MODE_INPUT,
                           .pull_up_en = GPIO_PULLUP_ENABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&int_cfg);

  const uint16_t addrs[] = {MPU6050_I2C_ADDRESS, MPU6050_I2C_ADDRESS_1};
  for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
    handle = mpu6050_create(I2C_MASTER_NUM, addrs[i]);
    if (!handle) {
      continue;
    }

    ret = try_wake_and_verify(handle, &deviceid);
    if (ret == ESP_OK && is_mpu6050_who_am_i(deviceid)) {
      ESP_LOGI(TAG, "MPU6050 found at 0x%02x, WHO_AM_I=0x%02x%s", addrs[i],
               deviceid,
               deviceid == MPU6050_WHO_AM_I_CLONE_VAL ? " (clone)" : "");
      ret = mpu6050_config(handle, acce_fs, gyro_fs);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 config failed: %s", esp_err_to_name(ret));
        mpu6050_delete(handle);
        handle = nullptr;
        return false;
      }
      isReady = true;
      return true;
    }

    ESP_LOGW(TAG, "MPU6050 not at 0x%02x (ret=%s, id=0x%02x)", addrs[i],
             esp_err_to_name(ret), deviceid);
    mpu6050_delete(handle);
    handle = nullptr;
  }

  ESP_LOGE(TAG, "MPU6050 not found, check wiring (SDA=GPIO%d, SCL=GPIO%d)",
           I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
  return false;
}

bool APP_MPU6050::app_mpu6050_check_module() { return isReady && handle != nullptr; }

bool APP_MPU6050::read_sample(mpu6050_acce_value_t *acc, mpu6050_gyro_value_t *gyro) {
  if (!app_mpu6050_check_module() || !acc || !gyro) {
    return false;
  }

  if (mpu6050_get_acce(handle, acc) != ESP_OK) {
    return false;
  }
  if (mpu6050_get_gyro(handle, gyro) != ESP_OK) {
    return false;
  }
  return true;
}
