#ifndef _APP_MPU6050_H_
#define _APP_MPU6050_H_

#include <stdint.h>
#include <stdbool.h>

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY       /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0 /*!< I2C master doesn't need buffer */ 
#define I2C_MASTER_TIMEOUT_MS       1000

#define WINDOW_SIZE 5 // 滤波器大小

#include "mpu6050.h"
#include "freertos/queue.h"

extern QueueHandle_t data_queue;

typedef struct{
    float roll,pitch,yaw;
}mpu6050_euler_angles;

typedef struct{
    mpu6050_acce_value_t filter_acce_value;     // 滤波后的加速度数据
    mpu6050_gyro_value_t filter_gyro_value;     // 滤波后的角速度数据
    mpu6050_euler_angles angles;                // 计算后的欧拉角数据
    float combined_accel;                       // 合角速度
    float combined_gyro;                        // 合加速度
    bool fall_pre_detected;                     // 预跌倒判断标志
    bool special_marker;                        // 特殊标注标志
    uint32_t timestamp;                         // 时间戳（毫秒）
}mpu6050_processed_data_t; // 定义结构体用于传输需要存储的数据

// 滑动平均滤波器结构体
typedef struct {
    float buffer_x[WINDOW_SIZE];
    float buffer_y[WINDOW_SIZE];
    float buffer_z[WINDOW_SIZE];
    int index;
    bool filled;
} moving_average_filter_t;

class APP_MPU6050 {
private:
    uint8_t deviceid;          // Who I am 的设备id
    bool isReady;              // 组件状态
    const char *TAG;           // 日志提示模组名

    mpu6050_acce_fs_t acce_fs; // 加速度量程
    mpu6050_gyro_fs_t gyro_fs; // 角速度量程

    mpu6050_acce_value_t raw_acce_value;        // 组件处理后的原始加速度数据
    mpu6050_gyro_value_t raw_gyro_value;        // 组件处理后的原始角速度数据
    moving_average_filter_t acce_filter;        // 加速度滤波器
    moving_average_filter_t gyro_filter;        // 角速度滤波器
    mpu6050_temp_value_t temp_value;            // 温度数据
    mpu6050_handle_t handle;                    // 组件句柄
    mpu6050_euler_angles prev_angles;           // 前一帧欧拉角
    uint32_t prev_timestamp;                    // 上一帧时间戳（ms）
    mpu6050_processed_data_t data;              // 所有需要传输的数据
public:

    APP_MPU6050(const char* tag);  // 构造函数
    APP_MPU6050(mpu6050_acce_fs_t acce_fs,mpu6050_gyro_fs_t gyro_fs,const char* tag);
    ~APP_MPU6050(); // 析构函数，释放资源

    void app_mpu6050_init(); // 初始化mpu6050
    void app_mpu6050_update_moving_average(); // 更新滤波数据
    void app_mpu6050_compute_attitude(); // 姿态解算，即计算欧拉角
    void app_mpu6050_compute_combined_accel_gyro(); // 计算合加速度和角速度
    bool app_mpu6050_fall_pre_detection(); // 跌倒算法，进行判断
    void app_mpu6050_create_task(); // 创建freertos任务
    bool app_mpu6050_check_module(); // 检查模组状态
    void app_mpu6050_start();   // 启动模组
    void app_mpu6050_stop();    // 停止模组
    void run_once(); // 任务运行的实际逻辑

};

#endif