#ifndef _APP_MPU6050_H_
#define _APP_MPU6050_H_

#include <stdint.h>
#include <stdbool.h>

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY
#define I2C_MASTER_TIMEOUT_MS       1000
#define MPU6050_INT_GPIO            CONFIG_MPU6050_INT_PIN
/* 部分国产克隆 MPU6050 返回 0x98，功能与 0x68 兼容 */
#define MPU6050_WHO_AM_I_CLONE_VAL  0x98

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "mpu6050.h"
#include "app_fall_new.h"

extern QueueHandle_t data_queue;

typedef struct {
    float roll, pitch, yaw;
} mpu6050_euler_angles;

typedef struct {
    mpu6050_acce_value_t filter_acce_value;
    mpu6050_gyro_value_t filter_gyro_value;
    mpu6050_euler_angles angles;
    float combined_accel;
    float combined_gyro;
    bool fall_pre_detected;
    bool special_marker;
    uint32_t timestamp;
} mpu6050_processed_data_t;

class APP_MPU6050 {
private:
    uint8_t deviceid;
    bool isReady;
    const char *TAG;
    mpu6050_acce_fs_t acce_fs;
    mpu6050_gyro_fs_t gyro_fs;
    mpu6050_handle_t handle;
    NewFallDetector fall_detector;

public:
    APP_MPU6050(const char *tag);
    APP_MPU6050(mpu6050_acce_fs_t acce_fs, mpu6050_gyro_fs_t gyro_fs, const char *tag);
    ~APP_MPU6050();

    bool app_mpu6050_init();
    bool app_mpu6050_check_module();
    bool read_sample(mpu6050_acce_value_t *acc, mpu6050_gyro_value_t *gyro);
    bool detect_fall(const mpu6050_acce_value_t &acc);
    void reset_fall_detector();
};

#endif
