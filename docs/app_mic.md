# MEMS_MIC 组件

## 职责

INMP441 MEMS麦克风音频数据采集、24位→16位PCM转换、WAV文件流式封装与存储。

## 核心设计

### 数据处理流水线

```
INMP441(I2S) → 24位原始数据 → 符号扩展 → 16位PCM转换 → 累积50ms → WAV包封装 → 队列 → SD卡
```

### I2S配置

| 参数 | 值 | 说明 |
|------|-----|------|
| 模式 | I2S STD (Left Aligned) | 兼容INMP441 |
| 采样率 | 16kHz | CONFIG_MIC_SAMPLE_RATE |
| 数据位宽 | 32-bit slot, 24-bit data | DMA frame: 1023 |
| 声道 | Mono (Left only) | I2S_STD_SLOT_LEFT |

### 数据格式转换

INMP441输出24位音频数据，存储在32位槽的低24位：

```
┌────────────────────────────────────────────┐
│  32-bit slot (little-endian)               │
│  ┌────────┬────────┬────────┬────────┐     │
│  │ B3(高8)│ B2    │ B1    │ B0(低8)│     │
│  └────────┴────────┴────────┴────────┘     │
│       符号扩展    ← 低24位有效 →          │
└────────────────────────────────────────────┘
```

转换步骤:
1. 取低24位: `raw & 0x00FFFFFF`
2. 符号扩展: 如果bit23=1，则`raw | 0xFF000000`
3. 转16位: `sample >> 8`

### WAV包封装

- **包大小**: 50ms音频 = 800采样点 = 1600字节
- **WAV头**: 44字节 (首次包含，后续仅PCM数据)
- **队列**: `wav_queue` (深度20，约1秒缓冲)

### 任务模型

| 任务 | 优先级 | 职责 |
|------|--------|------|
| mic_collect | 4 | I2S采集、转换、WAV打包 |
| mic_write | 2 | WAV文件流式写入SD卡 |

### 模式切换

MIC采集仅在MIC模式(false)下工作:
- 进入MIC模式: 启动I2S通道，开始采集
- 退出MIC模式: 停止I2S，保存当前WAV文件
- 切换时通过事件组同步，确保文件完整保存

### 文件格式

- **命名**: `mic_{序号}.wav`
- **格式**: WAV (16-bit PCM, 16kHz, Mono)
- **大小限制**: 5MB/文件

WAV头结构 (44字节):
```
Offset  Size  Field
0       4     "RIFF"
4       4     ChunkSize (36 + data_size)
8       4     "WAVE"
12      4     "fmt "
16      4     SubChunk1Size (16)
20      2     AudioFormat (1 = PCM)
22      2     NumChannels (1 = Mono)
24      4     SampleRate (16000)
28      4     ByteRate (32000)
32      2     BlockAlign (2)
34      2     BitsPerSample (16)
36      4     "data"
40      4     data_size
```

### 关键接口

- `app_mic_init()`: I2S初始化、队列创建
- `app_mic_start()`: 使能I2S通道
- `app_mic_stop()`: 禁用I2S通道
- `mic_collection_loop()`: 采集任务主循环
- `mic_write_task_loop()`: 写入任务主循环

## 依赖

- I2S: `CONFIG_MIC_I2S_*` GPIO配置
- 队列: `wav_queue` (全局)
- 模式: `get_current_data_mode()`
- 事件组: `mode_switch_event_group`

## 测试日志 (参考)

麦克风正常工作时的数据特征:
- 有效采样比例: ~37%
- 信号范围: ±8388607 (24位)
- 平均绝对值: >1000 表示有有效信号
