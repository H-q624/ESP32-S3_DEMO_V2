# FallDetector

单头文件 C++ 跌倒检测器，基于峰度冲击 + 静止检测 + Pitch 姿态角。  
SisFall 数据集 Neck_data 训练，**Acc=96.9% / Prec=100% / Rec=93.9% / F1=96.9%**。

## 快速开始

```cpp
#include "FallDetector.hpp"

FallDetector detector;

// 每来一个 IMU 样本调用一次
bool isFall = detector.update(ax, ay, az, gx, gy, gz);

if (isFall) {
    // 触发报警
    detector.clearFall();  // 复位
}
```

## 算法流程

```
样本 → 固定归一化(6常数) → 119pt滑动峰度 → 三轴和
                                     ↓
                              k1_A>4.5 && k1_G>6.0 ?
                                     ↓ 是
                              ┌─ 1~3s后 k2 静止检测 (0.5s连续)
                              └─ 1~3s后 Pitch > -45°
                                     ↓
                              双确认 → 输出跌倒
```

## 移植到新硬件

需要重新标定 **6 个归一化常数**（ACC_MEAN/STD、GYR_MEAN/STD）。

1. 把 IMU 放在佩戴位置，采集 10 分钟以上日常活动数据（走、跑、坐、站、跳、转身）
2. 求 ACC 和 GYR 每个轴的总均值和标准差
3. 填入 `FallDetector.hpp` 顶部的 `ACC_MEAN_X`、`ACC_STD_X` 等常量

如果传感器量程不同，峰度阈值 `K1_A/K1_G/K2_A/K2_G` 可能需要微调。

## 内存与性能

| 项目 | 量级 |
|------|------|
| RAM | ~6.3KB (RingBuffer) |
| 每样本 CPU | ~2100 MAC |
| @238Hz 总负载 | <0.2% (ESP32-S3 240MHz) |
| 依赖 | `<cmath>` `<cstdint>` `<cstring>` |

## 文件

| 文件 | 说明 |
|------|------|
| FallDetector.hpp | 核心库，单头文件，拿过去 include 就行 |
