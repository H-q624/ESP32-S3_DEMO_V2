# APP_MPU6050 组件

## 职责

MPU6050传感器驱动、数据滤波、姿态解算、跌倒预检测及蜂鸣器报警控制。

## 核心设计

### 数据处理流水线

```
原始数据 → 滑动平均滤波 → 合值计算 → 姿态解算 → 跌倒检测 → 蜂鸣器控制 → 队列输出
```

### 滑动平均滤波

- **窗口大小**: 5
- **独立缓冲**: 加速度/角速度各一套环形缓冲
- **预热策略**: `filled`标志避免启动阶段滤波失真

### 姿态解算 (互补滤波)

```
角度 = 0.98 × (陀螺仪积分) + 0.02 × (加速度计计算)
```

- **适配安装**: Y轴竖直向下的IMU安装方式
- **输出**: Roll(绕X), Pitch(绕Y), Yaw(陀螺积分)

### 跌倒预检测

| 条件 | 阈值 | 说明 |
|------|------|------|
| 合加速度骤降 | < 0.3g | 失重状态 |
| 角度变化率 | > 80°/s | 剧烈倾斜 |

- **触发**: 两条件同时满足
- **输出**: `fall_pre_detected`标志位

### 蜂鸣器控制

跌倒预检测触发后，蜂鸣器鸣响500ms:
- **非阻塞**: 使用时间戳判断，无需延时阻塞任务
- **重新计时**: 检测到跌倒时重置计时器

```
if (fall_detected) {
    buzzer_on();
    buzzer_start_time = current_time;
} else if (current_time - buzzer_start_time >= 500ms) {
    buzzer_off();
}
```

### 数据结构

```
mpu6050_processed_data_t
├── timestamp          // 毫秒时间戳
├── filter_acce_value  // 滤波后加速度(x,y,z)
├── filter_gyro_value  // 滤波后角速度(x,y,z)
├── angles             // 欧拉角(roll,pitch,yaw)
├── combined_accel     // 合加速度
├── combined_gyro      // 合角速度
├── fall_pre_detected  // 跌倒预检测标志
└── special_marker     // 按钮标记
```

### 任务模型

| 参数 | 值 |
|------|-----|
| 周期 | 10ms (100Hz采样) |
| 优先级 | 5 |
| 输出 | FreeRTOS队列(`data_queue`) |

### 量程配置

| 参数 | 默认值 | 配置 |
|------|--------|------|
| 加速度 | ±4g | CONFIG (ACCE_FS_4G) |
| 角速度 | ±500°/s | CONFIG (GYRO_FS_500DPS) |

## 依赖

- I2C: `CONFIG_I2C_MASTER_SCL/SDA`
- 队列: `data_queue` (全局)
- 蜂鸣器: `CONFIG_BEEP_PIN_IO`
