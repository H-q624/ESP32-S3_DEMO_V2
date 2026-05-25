#ifndef APP_IMU_PROCESSING_H
#define APP_IMU_PROCESSING_H

/*
 * Standalone IMU signal-processing library extracted from app_mpu6050_old.cpp.
 * Not compiled into the main build (not listed in CMakeLists.txt).
 * Include / link only when you want to reuse moving-average filtering,
 * complementary-filter attitude estimation, or the legacy fall pre-detector.
 */

#include <stdint.h>
#include <stdbool.h>
#include "mpu6050.h"

#define IMU_WINDOW_SIZE 10

typedef struct {
    float buffer_x[IMU_WINDOW_SIZE];
    float buffer_y[IMU_WINDOW_SIZE];
    float buffer_z[IMU_WINDOW_SIZE];
    int   index;
    bool  filled;
} imu_moving_avg_t;

typedef struct {
    float roll, pitch, yaw;
} imu_euler_angles_t;

typedef struct {
    mpu6050_acce_value_t filter_acce_value;
    mpu6050_gyro_value_t filter_gyro_value;
    imu_euler_angles_t   angles;
    float    combined_accel;
    float    combined_gyro;
    bool     fall_pre_detected;
    uint32_t timestamp;
} imu_processed_data_t;

class ImuProcessor {
public:
    ImuProcessor();

    /* Feed one raw sample; updates internal filtered data. */
    void update(const mpu6050_acce_value_t &acce,
                const mpu6050_gyro_value_t &gyro,
                uint32_t timestamp_ms);

    /* Returns the most recently computed processed data. */
    const imu_processed_data_t &get_data() const { return data; }

    /* Legacy threshold-based fall pre-detector (3-condition AND). */
    bool fall_pre_detect();

private:
    imu_moving_avg_t   acce_filter;
    imu_moving_avg_t   gyro_filter;
    imu_processed_data_t data;

    imu_euler_angles_t prev_angles;
    uint32_t           prev_timestamp;

    void compute_moving_average(const mpu6050_acce_value_t &acce,
                                const mpu6050_gyro_value_t &gyro);
    void compute_combined();
    void compute_attitude();
};

#endif /* APP_IMU_PROCESSING_H */
