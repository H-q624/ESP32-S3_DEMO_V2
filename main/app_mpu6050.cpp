#include "app_mpu6050.h"
#include "driver/gpio.h"
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

static bool is_mpu_imu_who_am_i(uint8_t id) {
  return id == MPU6050_WHO_AM_I_VAL || id == MPU6050_WHO_AM_I_CLONE_VAL ||
         id == MPU6050_WHO_AM_I_ALT_VAL || id == MPU9250_WHO_AM_I_VAL;
}

static esp_err_t try_wake_and_verify(mpu6050_handle_t handle, uint8_t *deviceid) {
  esp_err_t ret = mpu6050_wake_up(handle);
  if (ret != ESP_OK) {
    return ret;
  }
  vTaskDelay(pdMS_TO_TICKS(10));
  return mpu6050_get_deviceid(handle, deviceid);
}

static bool probe_imu_data(mpu6050_handle_t handle) {
  mpu6050_acce_value_t acc = {};
  mpu6050_gyro_value_t gyro = {};
  if (mpu6050_get_acce(handle, &acc) != ESP_OK) {
    return false;
  }
  if (mpu6050_get_gyro(handle, &gyro) != ESP_OK) {
    return false;
  }
  return (acc.acce_x != 0.0f || acc.acce_y != 0.0f || acc.acce_z != 0.0f);
}

static esp_err_t i2c_bus_init(int sda, int scl) {
  i2c_config_t config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = sda,
      .scl_io_num = scl,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
  };
  config.master.clk_speed = I2C_MASTER_FREQ_HZ;

  esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &config);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = i2c_driver_install(I2C_MASTER_NUM, config.mode, 0, 0, 0);
  if (ret == ESP_ERR_INVALID_STATE) {
    ret = ESP_OK;
  }
  return ret;
}

static void i2c_scan_bus(const char *tag) {
  ESP_LOGI(tag, "I2C scan (SDA=%d, SCL=%d):", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
      ESP_LOGI(tag, "  device at 0x%02x", addr);
      found++;
    }
  }
  if (found == 0) {
    ESP_LOGW(tag, "  no I2C devices found");
  }
}

bool APP_MPU6050::app_mpu6050_init() {
  struct {
    int sda;
    int scl;
  } pin_pairs[] = {
      {I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO},
      {I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO},
  };

  for (size_t p = 0; p < sizeof(pin_pairs) / sizeof(pin_pairs[0]); p++) {
    int sda = pin_pairs[p].sda;
    int scl = pin_pairs[p].scl;

    esp_err_t ret = i2c_bus_init(sda, scl);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "I2C init failed (SDA=%d, SCL=%d): %s", sda, scl,
               esp_err_to_name(ret));
      continue;
    }

    ESP_LOGI(TAG, "I2C init OK (SDA=%d, SCL=%d, INT=%d)", sda, scl,
             MPU6050_INT_GPIO);

    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << MPU6050_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&int_cfg);

    const uint16_t addrs[] = {MPU6050_I2C_ADDRESS, MPU6050_I2C_ADDRESS_1};
    for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
      uint8_t id = 0;
      mpu6050_handle_t h = mpu6050_create(I2C_MASTER_NUM, addrs[i]);
      if (!h) {
        continue;
      }

      ret = try_wake_and_verify(h, &id);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 not at 0x%02x (ret=%s, id=0x%02x)", addrs[i],
                 esp_err_to_name(ret), id);
        mpu6050_delete(h);
        continue;
      }

      bool id_ok = is_mpu_imu_who_am_i(id);
      ret = mpu6050_config(h, acce_fs, gyro_fs);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Config failed at 0x%02x: %s", addrs[i],
                 esp_err_to_name(ret));
        mpu6050_delete(h);
        continue;
      }

      if (!id_ok && !probe_imu_data(h)) {
        ESP_LOGW(TAG, "WHO_AM_I=0x%02x at 0x%02x, data probe failed", id,
                 addrs[i]);
        mpu6050_delete(h);
        continue;
      }

      handle = h;
      deviceid = id;
      isReady = true;
      fall_detector.reset_state();
      ESP_LOGI(TAG, "IMU found at 0x%02x, WHO_AM_I=0x%02x%s (SDA=%d SCL=%d INT=%d)",
               addrs[i], deviceid,
               deviceid == MPU9250_WHO_AM_I_VAL ? " MPU9250" :
               deviceid == MPU6050_WHO_AM_I_CLONE_VAL ? " clone" :
               deviceid == MPU6050_WHO_AM_I_ALT_VAL ? " alt/clone" : "",
               sda, scl, MPU6050_INT_GPIO);
      return true;
    }

    if (p == 0) {
      i2c_scan_bus(TAG);
    }
  }

  ESP_LOGE(TAG, "MPU6050 not found (schematic: SDA=IO4 SCL=IO5 INT=IO2 AD0=GND->0x68)");
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

bool APP_MPU6050::detect_fall(const mpu6050_acce_value_t &acc) {
  return fall_detector.process_sample(acc.acce_x, acc.acce_y, acc.acce_z);
}

void APP_MPU6050::reset_fall_detector() {
  fall_detector.reset_state();
}
