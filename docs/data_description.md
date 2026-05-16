# SD卡数据存储说明

本文档详细说明了ESP32 IMU数据记录 demo 项目在SD卡中存储的数据格式和结构。

## 目录结构

SD卡挂载点为 `/sdcard`，文件按数据类型分类存储：

| 文件类型 | 文件名前缀 | 文件格式 | 说明 |
|---------|-----------|---------|------|
| IMU数据 | `imu_*` | CSV | 陀螺仪和加速度计数据 |
| 麦克风数据 | `mic_*` | WAV | 音频数据 |

---

## 1. IMU数据文件 (CSV格式)

### 文件命名规则

```
imu_0.csv   # 第一个文件
imu_1.csv   # 第二个文件
imu_2.csv   # 第三个文件
...
```

当文件大小达到 **5MB** 时，会自动创建新文件继续存储。

### CSV头部

```csv
timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll,pitch,yaw,combined_accel,combined_gyro,pre_judge,btn_sign
```

### 数据字段说明

| 序号 | 字段名 | 数据类型 | 单位 | 说明 |
|-----|--------|---------|------|------|
| 1 | timestamp | uint32 | 毫秒(ms) | 系统运行时间，从ESP32启动开始计数 |
| 2 | accel_x | float | g | X轴加速度，经过滑动平均滤波 |
| 3 | accel_y | float | g | Y轴加速度，经过滑动平均滤波 |
| 4 | accel_z | float | g | Z轴加速度，经过滑动平均滤波 |
| 5 | gyro_x | float | deg/s | X轴角速度，经过滑动平均滤波 |
| 6 | gyro_y | float | deg/s | Y轴角速度，经过滑动平均滤波 |
| 7 | gyro_z | float | deg/s | Z轴角速度，经过滑动平均滤波 |
| 8 | roll | float | 度(°) | 横滚角，通过互补滤波计算 |
| 9 | pitch | float | 度(°) | 俯仰角，通过互补滤波计算 |
| 10 | yaw | float | 度(°) | 偏航角，通过陀螺仪积分获得 |
| 11 | combined_accel | float | g | 合加速度 (√(x²+y²+z²)) |
| 12 | combined_gyro | float | deg/s | 合角速度 (√(x²+y²+z²)) |
| 13 | pre_judge | 0/1 | 布尔 | 预跌倒检测标志，1表示检测到跌倒倾向 |
| 14 | btn_sign | 0/1 | 布尔 | 按钮标注标志，按下按钮时为1 |

### 数据采样率

- **采样频率**: 100Hz (每10ms采集一次)
- **每行数据大小**: 约88字节
- **每秒数据量**: 约8.6KB
- **每文件存储时长**: 约10分钟

### 数据处理流程

1. MPU6050传感器采集原始加速度和角速度数据
2. 应用滑动平均滤波器（窗口大小=5）进行降噪
3. 通过互补滤波算法计算欧拉角（Roll, Pitch, Yaw）
4. 计算合加速度和合角速度
5. 执行跌倒预检测算法
6. 将处理后的数据写入CSV文件

### 跌倒检测算法说明

预跌倒判断条件（满足两项即触发）：
- 合加速度骤降 (< 0.3g)
- 角度变化率突增 (> 80°/s)

---

## 2. 麦克风数据文件 (WAV格式)

### 文件命名规则

```
mic_0.wav   # 第一个文件
mic_1.wav   # 第二个文件
mic_2.wav   # 第三个文件
...
```

### WAV文件参数

| 参数 | 值 | 说明 |
|-----|-----|------|
| 音频格式 | PCM | 脉冲编码调制 |
| 采样率 | 16000 Hz | 可通过配置修改 |
| 位深度 | 16 bit | 采样精度 |
| 声道数 | 1 (单声道) | Mono |
| 字节率 | 32000 Bytes/s | SampleRate × Channels × Bits/8 |
| 数据块对齐 | 2 Bytes | Channels × Bits/8 |

### 数据存储特点

- 每个音频数据段保存为一个独立的WAV文件
- 文件大小达到 **5MB** 时自动创建新文件
- WAV文件头在文件关闭时自动更新，确保数据大小正确

### 存储时长计算

```
每文件容量: 5MB = 5 × 1024 × 1024 ≈ 5,242,880 字节
每秒数据量: 16000 × 2 = 32,000 字节
存储时长: 5,242,880 / 32,000 ≈ 164 秒 ≈ 2.7 分钟
```

---

## 3. 数据文件管理

### 文件切换机制

- IMU文件：当文件大小 ≥ 5MB 时自动创建新文件
- MIC文件：当文件大小 ≥ 5MB 时自动创建新文件

### 文件索引

- 文件编号从0开始递增
- 系统会自动检查已存在的文件编号，避免数据覆盖

### 缓冲区机制

- IMU数据：采用双缓冲机制
  - 缓冲区大小：100个样本
  - 缓冲区超时：500ms
  - 采集任务优先级：5
  - 写入任务优先级：2

- 麦克风数据：流式写入
  - 定期刷新周期：1秒

---

## 4. 数据读取示例

### Python读取IMU数据

```python
import pandas as pd

# 读取IMU数据
df = pd.read_csv('imu_0.csv')

# 查看数据结构
print(df.head())

# 访问各字段
timestamps = df['timestamp']
accel_x = df['accel_x']
roll = df['roll']
```

### Python读取WAV音频

```python
import wave

# 打开WAV文件
with wave.open('mic_0.wav', 'rb') as wav:
    # 获取音频参数
    sample_rate = wav.getframerate()      # 16000
    channels = wav.getnchannels()          # 1
    bit_depth = wav.getsampwidth() * 8    # 16
    n_frames = wav.getnframes()            # 帧数
    
    # 读取音频数据
    audio_data = wav.readframes(n_frames)
    print(f"音频时长: {n_frames / sample_rate:.2f} 秒")
```

---

## 5. 注意事项

1. **时间戳**：timestamp 是ESP32系统运行时间（毫秒），不是实时时钟
2. **Yaw角漂移**：由于Yaw角通过陀螺仪积分计算，长时间运行会有漂移
3. **跌倒检测**：pre_judge 是预检测结果，非最终跌倒判断
4. **WAV文件完整性**：WAV文件关闭时才会更新正确的文件头
5. **文件系统**：使用FATFS文件系统

---

## 6. 硬件配置

| 硬件 | 配置 |
|-----|------|
| 主控芯片 | ESP32-C3 |
| IMU传感器 | MPU6050 (I2C) |
| 麦克风 | INMP441 (I2S) |
| 存储介质 | TF卡 (SPI) |
