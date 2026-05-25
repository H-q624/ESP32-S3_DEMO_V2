// #include "app_gpio.h"
// #include "app_mpu5060.h"
// #include "app_fall.h"
// #include "driver/i2c.h"
// #include "esp_err.h"
// #include "esp_log.h"
// #include "esp_timer.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/idf_additions.h"
// #include "freertos/projdefs.h"
// #include "freertos/task.h"
// #include "portmacro.h"
// #include "app_fall_new.h"

// #include <cmath>
// #include <cstdio>

// TaskHandle_t mpu6050_handler = nullptr; // 全局函数，防止重复创建任务
// QueueHandle_t data_queue = nullptr;

// // // 蜂鸣器非阻塞控制相关变量
// // static bool buzzer_active = false;
// // static uint32_t buzzer_start_time = 0;
// // static const uint32_t BUZZER_DURATION_MS = 500; // 蜂鸣器持续0.5s

// /**
//  * @brief 更新蜂鸣器状态（非阻塞方式）
//  * @param fall_detected 是否检测到跌倒
//  */
// static void update_buzzer(bool fall_detected) {
//     uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒
    
//     if (fall_detected) {
//         // 检测到跌倒，打开蜂鸣器并重新计时
//         if (!buzzer_active) {
//             set_buzzer_on();
//             buzzer_active = true;
//         }
//         buzzer_start_time = current_time; // 重新计时
//     } else {
//         // 检查是否需要关闭蜂鸣器
//         if (buzzer_active) {
//             if ((current_time - buzzer_start_time) >= BUZZER_DURATION_MS) {
//                 set_buzzer_off();
//                 buzzer_active = false;
//             }
//         }
//     }
// }

// /**
//  * @brief 默认构造函数
//  * @param tag 日志打印时对应的目标设置
//  * @note 设置加速度和角速度量程为默认值，设置状态为不可用
//  */
// APP_MPU6050::APP_MPU6050(const char *tag)
//     : acce_fs(ACCE_FS_4G), gyro_fs(GYRO_FS_500DPS) {
//   deviceid = 0;
//   raw_acce_value = {0};
//   raw_gyro_value = {0};
//   acce_filter.index = 0;
//   acce_filter.filled = false; // 未满标志，防止在启动时滤波不准确
//   gyro_filter.index = 0;
//   gyro_filter.filled = false;
//   temp_value.temp = 0.0;
//   handle = nullptr;
//   isReady = false;
//   TAG = tag;
// }
// /**
//  * @brief 设置量程的构造函数
//  * @param tag 日志打印时对应的目标设置
//  * @param set_acce_fs 设置加速度量程，见头文件定义
//  * @param set_gyro_fs 设置角速度量程，将头文件定义
//  * @note 设置加速度和角速度量程为默认值，设置状态为不可用
//  */
// APP_MPU6050::APP_MPU6050(mpu6050_acce_fs_t set_acce_fs,
//                          mpu6050_gyro_fs_t set_gyro_fs, const char *tag) {
//   acce_fs = set_acce_fs;
//   gyro_fs = set_gyro_fs;
//   deviceid = 0;
//   raw_acce_value = {};
//   raw_gyro_value = {};
//   acce_filter.index = 0;
//   acce_filter.filled = false; // 未满标志，防止在启动时滤波不准确
//   gyro_filter.index = 0;
//   gyro_filter.filled = false;
//   temp_value.temp = 0.0;
//   handle = nullptr;
//   isReady = false;
//   TAG = tag;
// }
// APP_MPU6050::~APP_MPU6050() {
//   isReady = false;
//   mpu6050_delete(handle);
// }
// /**
//  * @brief 扫描 I2C 总线上的所有设备地址
//  * @param i2c_num I2C 端口号
//  */
// static void i2c_scan(i2c_port_t i2c_num) {
//   ESP_LOGI("I2C_SCAN", "Scanning I2C bus...");
//   uint8_t address;
//   int found = 0;
//   for (address = 1; address < 127; address++) {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_stop(cmd);

//     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(10));
//     i2c_cmd_link_delete(cmd);

//     if (ret == ESP_OK) {
//       ESP_LOGI("I2C_SCAN", "Found device at 0x%02x", address);
//       found++;
//     }
//   }
//   if (found == 0) {
//     ESP_LOGW("I2C_SCAN", "No I2C devices found!");
//   } else {
//     ESP_LOGI("I2C_SCAN", "Found %d device(s)", found);
//   }
// }
// /**
//  * @brief 启动模组，设置标志位
//  * @note 单独使用必须在init后使用
//  */
// void APP_MPU6050::app_mpu6050_start() {
//   if (!isReady) {
//     mpu6050_wake_up(handle);
//     ESP_LOGI(TAG, "mpu6050 running normal");
//     isReady = true;

//     // ⬅️ 新增这几行：唤醒后立刻复位所有缓冲，防止脏数据误触发
//     acce_filter.filled = false;
//     acce_filter.index = 0;
//     gyro_filter.filled = false;
//     gyro_filter.index = 0;
//     fall_detector.reset_state(); 

//     mpu6050_get_deviceid(handle, &deviceid);
//     printf("MPU6050 Device id : 0x%x\n", deviceid);
//     isReady = true;
//   }
// }
// /**
//  * @brief 停止模组，取消标志位
//  */
// void APP_MPU6050::app_mpu6050_stop() {
//   if(isReady){
//     mpu6050_sleep(handle);
//     ESP_LOGI(TAG,"mpu6050 alreay asleep");
//     isReady = false;
//   }
// }
// /**
//  * @brief mpu6050初始化函数，完成i2c初始化和连接检查
//  */
// void APP_MPU6050::app_mpu6050_init() {
//   // I2C初始化
//   i2c_config_t config = {.mode = I2C_MODE_MASTER,
//                          .sda_io_num = I2C_MASTER_SDA_IO,
//                          .scl_io_num = I2C_MASTER_SCL_IO,
//                          .sda_pullup_en = GPIO_PULLUP_ENABLE,
//                          .scl_pullup_en = GPIO_PULLUP_ENABLE,
//                          .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL};
//   config.master.clk_speed = I2C_MASTER_FREQ_HZ; // 单独配置
//   ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &config));
//   ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, config.mode, 0, 0, 0));
//   ESP_LOGI(TAG, "I2C Initialization successed");
//   // i2c_scan(I2C_MASTER_NUM);
//   // 连接MPU6050(这的地址是默认地址，没有进行控制引脚的GPIO设置)
//   handle = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
//   mpu6050_config(handle, acce_fs, gyro_fs);
//   app_mpu6050_start();
// }
// /**
//  * @brief mpu6050数据处理函数，对获取的数据进行滤波
//  */
// void APP_MPU6050::app_mpu6050_update_moving_average() {
//   // 确认模组可用
//   if (!isReady)
//     ESP_LOGE(TAG, "mpu6050 is not available");

//   // 向滤波器中写入新值(环形缓冲)
//   acce_filter.buffer_x[acce_filter.index] = raw_acce_value.acce_x;
//   acce_filter.buffer_y[acce_filter.index] = raw_acce_value.acce_y;
//   acce_filter.buffer_z[acce_filter.index] = raw_acce_value.acce_z;
//   acce_filter.index = (acce_filter.index + 1) % WINDOW_SIZE;

//   gyro_filter.buffer_x[gyro_filter.index] = raw_gyro_value.gyro_x;
//   gyro_filter.buffer_y[gyro_filter.index] = raw_gyro_value.gyro_y;
//   gyro_filter.buffer_z[gyro_filter.index] = raw_gyro_value.gyro_z;
//   gyro_filter.index = (gyro_filter.index + 1) % WINDOW_SIZE;

//   // 进行滤波
//   if (acce_filter.filled) {
//     // 填满后进行滤波，由于只有最开始的时候不满，不重复设置filled
//     float sum_acce_x = 0.0;
//     float sum_acce_y = 0.0;
//     float sum_acce_z = 0.0;
//     for (int i = 0; i < WINDOW_SIZE; i++) {
//       sum_acce_x += acce_filter.buffer_x[i];
//       sum_acce_y += acce_filter.buffer_y[i];
//       sum_acce_z += acce_filter.buffer_z[i];
//     }
//     data.filter_acce_value.acce_x = sum_acce_x / WINDOW_SIZE;
//     data.filter_acce_value.acce_y = sum_acce_y / WINDOW_SIZE;
//     data.filter_acce_value.acce_z = sum_acce_z / WINDOW_SIZE;
//   } else {
//     if (acce_filter.index == 0)
//       acce_filter.filled = true;
//   }

//   if (gyro_filter.filled) {
//     // 填满后进行滤波，由于只有最开始的时候不满，不重复设置filled
//     float sum_gyro_x = 0.0;
//     float sum_gyro_y = 0.0;
//     float sum_gyro_z = 0.0;
//     for (int i = 0; i < WINDOW_SIZE; i++) {
//       sum_gyro_x += gyro_filter.buffer_x[i];
//       sum_gyro_y += gyro_filter.buffer_y[i];
//       sum_gyro_z += gyro_filter.buffer_z[i];
//     }
//     data.filter_gyro_value.gyro_x = sum_gyro_x / WINDOW_SIZE;
//     data.filter_gyro_value.gyro_y = sum_gyro_y / WINDOW_SIZE;
//     data.filter_gyro_value.gyro_z = sum_gyro_z / WINDOW_SIZE;
//   } else {
//     if (gyro_filter.index == 0)
//       gyro_filter.filled = true;
//   }

//   data.timestamp = esp_timer_get_time() / 1000; // 更新时间戳
// }
// /**
//  * @brief mpu6050姿态解算函数，根据滤波后的数据计算欧拉角 */
// void APP_MPU6050::app_mpu6050_compute_attitude() {
//   // 姿态解算 - 适配Y轴竖直向下的IMU安装方式
//   static float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
//   const float alpha = 0.98f; // 互补滤波权重

//   // 获取当前时间间隔
//   static uint64_t last_time = 0;
//   uint64_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒
//   float dt = (current_time - last_time) / 1000.0f;     // 转换为秒
//   last_time = current_time;

//   // 如果时间间隔过大，可能是首次运行或系统延迟，跳过本次计算
//   if (dt <= 0 || dt > 1.0f) {
//     return;
//   }

//   // 使用滤波后的数据
//   float acce_x = data.filter_acce_value.acce_x;
//   float acce_y = data.filter_acce_value.acce_y;
//   float acce_z = data.filter_acce_value.acce_z;

//   float gyro_x = data.filter_gyro_value.gyro_x;
//   float gyro_y = data.filter_gyro_value.gyro_y;
//   float gyro_z = data.filter_gyro_value.gyro_z;

//   // 计算加速度计角度 (适配Y轴竖直向下)
//   // 对于Y轴竖直向下的IMU：
//   // - Roll (绕X轴)：由Y和Z轴分量决定
//   // - Pitch (绕Y轴)：由X和Z轴分量决定
//   // - Yaw (绕Y轴)：由X和Y轴分量决定

//   // 计算Roll角 (绕X轴) - 适配Y轴竖直向下
//   // 当设备向右倾斜时，Y轴分量增加，Z轴分量减少
//   float acc_roll = atan2f(acce_y, acce_z) * 57.2957795f; // 转换为度数

//   // 计算Pitch角 (绕Y轴) - 适配Y轴竖直向下
//   // 当设备向前倾斜时，X轴分量增加，Z轴分量减少
//   float acc_pitch =
//       atan2f(-acce_x, acce_z) * 57.2957795f; // 负号是为了适配坐标系

//   // 计算Yaw角 (绕Y轴) - 适配Y轴竖直向下
//   // 由于加速度计无法测量绕Y轴旋转，使用陀螺仪积分
//   // 注意：这里我们假设设备在水平面内旋转，因此yaw角主要由陀螺仪积分得到
//   gyro_yaw += gyro_y * dt;

//   // 互补滤波融合
//   // Roll角融合
//   data.angles.roll =
//       alpha * (gyro_roll + gyro_x * dt) + (1.0f - alpha) * acc_roll;
//   // Pitch角融合
//   data.angles.pitch =
//       alpha * (gyro_pitch + gyro_z * dt) + (1.0f - alpha) * acc_pitch;
//   // Yaw角融合 - 由于加速度计无法测量，直接使用陀螺仪积分
//   data.angles.yaw = gyro_yaw;

//   // 限制角度范围，防止异常值
//   if (data.angles.roll > 180.0f)
//     data.angles.roll -= 360.0f;
//   if (data.angles.roll < -180.0f)
//     data.angles.roll += 360.0f;
//   if (data.angles.pitch > 180.0f)
//     data.angles.pitch -= 360.0f;
//   if (data.angles.pitch < -180.0f)
//     data.angles.pitch += 360.0f;

//   // 更新陀螺仪积分值
//   gyro_roll = data.angles.roll;
//   gyro_pitch = data.angles.pitch;
// }
// /**
//  * @brief mpu6050合加速度合角速度计算
//  */
// void APP_MPU6050::app_mpu6050_compute_combined_accel_gyro() {
//   // 合加速度
//   float ax2, ay2, az2;
//   ax2 = data.filter_acce_value.acce_x * data.filter_acce_value.acce_x;
//   ay2 = data.filter_acce_value.acce_y * data.filter_acce_value.acce_y;
//   az2 = data.filter_acce_value.acce_z * data.filter_acce_value.acce_z;
//   data.combined_accel = sqrtf(ax2 + ay2 + az2);
//   // 合角速度
//   float gx2, gy2, gz2;
//   gx2 = data.filter_gyro_value.gyro_x * data.filter_gyro_value.gyro_x;
//   gy2 = data.filter_gyro_value.gyro_y * data.filter_gyro_value.gyro_y;
//   gz2 = data.filter_gyro_value.gyro_z * data.filter_gyro_value.gyro_z;
//   data.combined_gyro = sqrtf(gx2 + gy2 + gz2);
// }

// /**
//  * @brief 跌倒算法的预判断
//  * 判断依据：
//  *   1. 合加速度骤降（< 0.3g）
//  *   2. 合角速度突增（> 阈值，如 200 deg/s）
//  *   3. 不依赖绝对角度，只检测短时间内角度剧烈变化
//  * 满足任意两项即可触发预跌倒标志
//  */
// bool APP_MPU6050::app_mpu6050_fall_pre_detection() {
//   // 阈值定义（可根据实际调试调整）
//   const float ACCEL_FALL_THRESHOLD =
//       0.3f; // g 单位（假设 combined_accel 已归一化为 g）
//   const float ANGLE_CHANGE_RATE_THRESHOLD = 80.0f; // 角度变化率阈值（度/秒）
//   const float COMBINED_GYRO_THRESHOLD = 400.0f;    // 合角速度阈值（度/秒）
//   const uint32_t MIN_TIME_DELTA_MS = 10;

//   // 计算时间差（秒）
//   uint32_t dt_ms = data.timestamp - prev_timestamp;
//   if (dt_ms < MIN_TIME_DELTA_MS || dt_ms > 500) { // 数据异常或间隔过大
//     prev_angles = data.angles;
//     prev_timestamp = data.timestamp;
//     return false;
//   }
//   float dt_s = dt_ms / 1000.0f;

//   // 计算角度变化量（考虑角度环绕问题，但短时变化通常 < 180°，可简化）
//   float delta_pitch = fabs(data.angles.pitch - prev_angles.pitch);
//   float delta_roll = fabs(data.angles.roll - prev_angles.roll);

//   // 计算最大角度变化率（度/秒）
//   float max_angle_rate = fmaxf(delta_pitch, delta_roll) / dt_s;

//   // 判断条件
//   bool low_accel = (data.combined_accel < ACCEL_FALL_THRESHOLD);
//   bool rapid_tilt = (max_angle_rate > ANGLE_CHANGE_RATE_THRESHOLD);
//   bool high_gyro = (data.combined_gyro > COMBINED_GYRO_THRESHOLD);

//   bool pre_detected = low_accel && rapid_tilt && high_gyro;

//   // 更新历史数据
//   prev_angles = data.angles;
//   prev_timestamp = data.timestamp;

//   // 更新标志位
//   data.fall_pre_detected = pre_detected;
//   if (pre_detected) {
//     data.fall_pre_detected = true; // 可用于日志或报警
//     ESP_LOGI(TAG, "Pre-detaction fall");
//   } else {
//     data.fall_pre_detected = false;
//   }

//   return pre_detected;
// }
// /**
//  * @brief 任务进行一次操作
//  */
// void APP_MPU6050::run_once() {
//   // 1. 获取原始数据
//   if (mpu6050_get_acce(handle, &raw_acce_value) != ESP_OK) {
//     ESP_LOGE(TAG, "Failed to get accelerometer value");
//   }

//   if (mpu6050_get_gyro(handle, &raw_gyro_value) != ESP_OK) {
//     ESP_LOGE(TAG, "Failed to get gyroscope value");
//   }

//   // 2. 数据滤波
//   app_mpu6050_update_moving_average();

//   // 3. 计算合加速度和合角速度
//   app_mpu6050_compute_combined_accel_gyro();

//   // 4. 计算欧拉角
//   app_mpu6050_compute_attitude();

//   // 5. 其他处理（如跌倒检测等）
//   data.special_marker = global_btn_sign;
  
//   // 使用原来的跌倒检测算法（已注释掉新的自适应阈值算法）
//   // bool fall_detected = app_fall_detect(
//   //     data.filter_acce_value.acce_x, data.filter_acce_value.acce_y, data.filter_acce_value.acce_z,
//   //     data.filter_gyro_value.gyro_x, data.filter_gyro_value.gyro_y, data.filter_gyro_value.gyro_z,
//   //     data.angles.roll, data.angles.pitch, data.angles.yaw,
//   //     data.combined_accel, data.combined_gyro, data.timestamp);
  
//   // 原来的跌倒预检测算法
//   // bool fall_detected = app_mpu6050_fall_pre_detection();

//   // 新的自适应阈值跌倒检测算法
//   bool fall_detected = fall_detector.process_100hz_data(
//       data.filter_acce_value.acce_x, data.filter_acce_value.acce_y, data.filter_acce_value.acce_z);
  
//   data.fall_pre_detected = fall_detected;
  
//   // 更新蜂鸣器状态（非阻塞方式）
//   update_buzzer(fall_detected);

//   // printf("mpu6050 filter data:
//   // accex:%.4f,accey:%.4f,accz:%.4f\n",data.filter_acce_value.acce_x,data.filter_acce_value.acce_y,data.filter_acce_value.acce_z);
//   // printf("mpu6050 filter data:
//   // gyrox:%.4f,gyroy%.4f,gyroz:%.4f\n",data.filter_gyro_value.gyro_x,data.filter_gyro_value.gyro_y,data.filter_gyro_value.gyro_z);
//   // printf("combineed_acce: %.4f, combined_gyro:
//   // %.4f\n",data.combined_accel,data.combined_gyro);
//   // printf("roll:%.2f,pitch:%.2f,yaw:%.2f\n",data.angles.roll,data.angles.pitch,data.angles.yaw);

//   // 队列发送数据
//   if (nullptr != data_queue) {
//     BaseType_t ret = xQueueSend(data_queue, &data, pdMS_TO_TICKS(10));
//     if (ret != pdTRUE)
//       ESP_LOGW(TAG, "Failed to send data to queue");
//   }
// }
// /**
//  * @brief mpu6050任务函数
//  * @note 实际任务逻辑在run_once函数中
//  */
// static void mpu6050_task_function(void *arg) {
//   APP_MPU6050 *instance = static_cast<APP_MPU6050 *>(arg);
//   while (1) {
//     if (instance->app_mpu6050_check_module())
//       instance->run_once();
//     if (!get_current_data_mode())
//       instance->app_mpu6050_stop(); // 根据数据模式停止或启用模组
//     else
//       instance->app_mpu6050_start();
//     vTaskDelay(pdMS_TO_TICKS(10)); // 延时10ms
//   }
//   vTaskDelete(NULL);
// }

// /**
//  * @brief 创建freertos任务，获取数据并处理
//  * @note 从IMU获取数据并进行跌倒判断
//  */
// void APP_MPU6050::app_mpu6050_create_task() {
//   // app_fall_init();  // 注释掉新的跌倒检测模块初始化
//   if (nullptr == data_queue) {
//     data_queue = xQueueCreate(5, sizeof(mpu6050_processed_data_t));
//     if (data_queue == nullptr) {
//       ESP_LOGE(TAG, "Failed to create queue");
//       return;
//     }
//   }
//   if (nullptr == mpu6050_handler) {
//     xTaskCreate(mpu6050_task_function, "mpu6050 task", 4096, this, 5,
//                 &mpu6050_handler);
//     ESP_LOGI(TAG, "mpu6050 task create successfully");
//   } else {
//     ESP_LOGW(TAG, "mpu6050 task is already exists");
//   }
// }
  

// /** 
// * @brief 检测模组状态
// */
// bool APP_MPU6050::app_mpu6050_check_module(){
//   return isReady;
// }