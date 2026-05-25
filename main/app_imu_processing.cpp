/*
 * app_imu_processing.cpp
 *
 * Standalone IMU signal-processing routines, NOT linked into the main build.
 * To use: add this file to CMakeLists.txt SRCS and #include "app_imu_processing.h".
 *
 * Algorithms extracted from app_mpu6050_old.cpp:
 *   - Moving-average filter (accel + gyro, circular buffer)
 *   - Combined magnitude computation
 *   - Complementary-filter attitude (roll / pitch / yaw)
 *   - Legacy fall pre-detector (low-accel AND rapid-tilt AND high-gyro)
 */

#include "app_imu_processing.h"
#include "esp_timer.h"
#include <cmath>
#include <cstring>

ImuProcessor::ImuProcessor() {
    memset(&acce_filter, 0, sizeof(acce_filter));
    memset(&gyro_filter, 0, sizeof(gyro_filter));
    memset(&data, 0, sizeof(data));
    memset(&prev_angles, 0, sizeof(prev_angles));
    prev_timestamp = 0;
}

/* ------------------------------------------------------------------ */
/* Moving-average filter                                                */
/* ------------------------------------------------------------------ */
void ImuProcessor::compute_moving_average(const mpu6050_acce_value_t &acce,
                                          const mpu6050_gyro_value_t  &gyro) {
    acce_filter.buffer_x[acce_filter.index] = acce.acce_x;
    acce_filter.buffer_y[acce_filter.index] = acce.acce_y;
    acce_filter.buffer_z[acce_filter.index] = acce.acce_z;
    acce_filter.index = (acce_filter.index + 1) % IMU_WINDOW_SIZE;

    gyro_filter.buffer_x[gyro_filter.index] = gyro.gyro_x;
    gyro_filter.buffer_y[gyro_filter.index] = gyro.gyro_y;
    gyro_filter.buffer_z[gyro_filter.index] = gyro.gyro_z;
    gyro_filter.index = (gyro_filter.index + 1) % IMU_WINDOW_SIZE;

    if (acce_filter.filled) {
        float sx = 0, sy = 0, sz = 0;
        for (int i = 0; i < IMU_WINDOW_SIZE; i++) {
            sx += acce_filter.buffer_x[i];
            sy += acce_filter.buffer_y[i];
            sz += acce_filter.buffer_z[i];
        }
        data.filter_acce_value.acce_x = sx / IMU_WINDOW_SIZE;
        data.filter_acce_value.acce_y = sy / IMU_WINDOW_SIZE;
        data.filter_acce_value.acce_z = sz / IMU_WINDOW_SIZE;
    } else {
        if (acce_filter.index == 0) acce_filter.filled = true;
    }

    if (gyro_filter.filled) {
        float sx = 0, sy = 0, sz = 0;
        for (int i = 0; i < IMU_WINDOW_SIZE; i++) {
            sx += gyro_filter.buffer_x[i];
            sy += gyro_filter.buffer_y[i];
            sz += gyro_filter.buffer_z[i];
        }
        data.filter_gyro_value.gyro_x = sx / IMU_WINDOW_SIZE;
        data.filter_gyro_value.gyro_y = sy / IMU_WINDOW_SIZE;
        data.filter_gyro_value.gyro_z = sz / IMU_WINDOW_SIZE;
    } else {
        if (gyro_filter.index == 0) gyro_filter.filled = true;
    }
}

/* ------------------------------------------------------------------ */
/* Combined magnitude                                                   */
/* ------------------------------------------------------------------ */
void ImuProcessor::compute_combined() {
    float ax = data.filter_acce_value.acce_x;
    float ay = data.filter_acce_value.acce_y;
    float az = data.filter_acce_value.acce_z;
    data.combined_accel = sqrtf(ax*ax + ay*ay + az*az);

    float gx = data.filter_gyro_value.gyro_x;
    float gy = data.filter_gyro_value.gyro_y;
    float gz = data.filter_gyro_value.gyro_z;
    data.combined_gyro = sqrtf(gx*gx + gy*gy + gz*gz);
}

/* ------------------------------------------------------------------ */
/* Complementary-filter attitude (Y-axis down mounting)                 */
/* ------------------------------------------------------------------ */
void ImuProcessor::compute_attitude() {
    static float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
    const float alpha = 0.98f;

    static uint64_t last_time = 0;
    uint64_t now = esp_timer_get_time() / 1000;
    float dt = (now - last_time) / 1000.0f;
    last_time = now;

    if (dt <= 0 || dt > 1.0f) return;

    float ax = data.filter_acce_value.acce_x;
    float ay = data.filter_acce_value.acce_y;
    float az = data.filter_acce_value.acce_z;
    float gx = data.filter_gyro_value.gyro_x;
    float gy = data.filter_gyro_value.gyro_y;
    float gz = data.filter_gyro_value.gyro_z;

    float acc_roll  = atan2f(ay, az) * 57.2957795f;
    float acc_pitch = atan2f(-ax, az) * 57.2957795f;
    gyro_yaw += gy * dt;

    data.angles.roll  = alpha * (gyro_roll  + gx * dt) + (1.0f - alpha) * acc_roll;
    data.angles.pitch = alpha * (gyro_pitch + gz * dt) + (1.0f - alpha) * acc_pitch;
    data.angles.yaw   = gyro_yaw;

    if (data.angles.roll  >  180.0f) data.angles.roll  -= 360.0f;
    if (data.angles.roll  < -180.0f) data.angles.roll  += 360.0f;
    if (data.angles.pitch >  180.0f) data.angles.pitch -= 360.0f;
    if (data.angles.pitch < -180.0f) data.angles.pitch += 360.0f;

    gyro_roll  = data.angles.roll;
    gyro_pitch = data.angles.pitch;
}

/* ------------------------------------------------------------------ */
/* Public: update with one raw sample                                   */
/* ------------------------------------------------------------------ */
void ImuProcessor::update(const mpu6050_acce_value_t &acce,
                          const mpu6050_gyro_value_t  &gyro,
                          uint32_t timestamp_ms) {
    data.timestamp = timestamp_ms;
    compute_moving_average(acce, gyro);
    compute_combined();
    compute_attitude();
}

/* ------------------------------------------------------------------ */
/* Legacy fall pre-detector                                             */
/* Triggers when ALL three conditions hold simultaneously:             */
/*   1. combined_accel < 0.3 g  (free-fall / impact drop)             */
/*   2. max angular rate > 80 deg/s                                    */
/*   3. combined_gyro > 400 deg/s                                      */
/* ------------------------------------------------------------------ */
bool ImuProcessor::fall_pre_detect() {
    const float ACCEL_THRESHOLD       = 0.3f;
    const float ANGLE_RATE_THRESHOLD  = 80.0f;
    const float GYRO_THRESHOLD        = 400.0f;
    const uint32_t MIN_DT_MS          = 10;

    uint32_t dt_ms = data.timestamp - prev_timestamp;
    if (dt_ms < MIN_DT_MS || dt_ms > 500) {
        prev_angles    = data.angles;
        prev_timestamp = data.timestamp;
        return false;
    }

    float dt_s = dt_ms / 1000.0f;
    float dpitch = fabsf(data.angles.pitch - prev_angles.pitch);
    float droll  = fabsf(data.angles.roll  - prev_angles.roll);
    float max_rate = fmaxf(dpitch, droll) / dt_s;

    bool low_accel  = data.combined_accel < ACCEL_THRESHOLD;
    bool rapid_tilt = max_rate > ANGLE_RATE_THRESHOLD;
    bool high_gyro  = data.combined_gyro  > GYRO_THRESHOLD;

    prev_angles    = data.angles;
    prev_timestamp = data.timestamp;

    data.fall_pre_detected = low_accel && rapid_tilt && high_gyro;
    return data.fall_pre_detected;
}
