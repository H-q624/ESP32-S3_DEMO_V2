# Button 组件

## 职责

GPIO输入检测与交互控制，管理按键状态机、系统模式切换及蜂鸣器/LED控制。

## 核心设计

### 状态机

```
┌─────────┐    按下(1→0)     ┌───────────┐
│  IDLE   │ ───────────────► │  PRESSED  │
│  (空闲)  │                  │  (按下中)  │
└─────────┘                  └─────┬─────┘
    ▲                            │
    └────────────────────────────┘
         释放(0→1)
         ├─ 短按(<500ms) → 标记事件(global_btn_sign)
         └─ 长按(>500ms) → 切换模式(事件组同步)
```

### 模式切换同步 (EventGroup)

长按切换IMU/MIC模式时，通过事件组确保数据完整保存:

```
Button任务                          IMU_Data/MIC_Data任务
    │                                      │
    │─ set MODE_SWITCH_REQUEST_BIT ──────►│
    │                                      │ 收到请求
    │                                      │ 保存当前缓冲区
    │◄── set MODE_SWITCH_IMU/MIC_READY ──┤
    │ (等待双方都完成)                      │
    │                                      │
    │ 切换 data_mode                       │
    │ 控制LED显示                          │
    │─ clear MODE_SWITCH_REQUEST_BIT ───►│ (退出等待循环)
```

**事件位定义**:
| 位 | 名称 | 说明 |
|---|------|------|
| BIT0 | MODE_SWITCH_REQUEST_BIT | 模式切换请求 |
| BIT1 | MODE_SWITCH_IMU_READY_BIT | IMU数据已保存 |
| BIT2 | MODE_SWITCH_MIC_READY_BIT | MIC数据已保存 |

**超时**: 2000ms

### 关键特性

| 特性 | 实现 |
|------|------|
| 消抖 | 10ms周期采样 |
| 长按阈值 | 500ms |
| 全局标志 | `global_btn_sign` 用于数据标记 |
| LED指示 | LED ON=MIC模式, LED OFF=IMU模式 |
| 蜂鸣器 | 跌倒检测时鸣响500ms |

### 输出控制

- **GPIO_PIN_LED**: 模式指示灯
- **GPIO_PIN_BEEP**: 蜂鸣器 (跌倒报警)

### 接口

- `app_btn_init()`: GPIO初始化(输入上拉+输出)
- `run_once()`: 状态机处理(10ms周期)
- `get_current_data_mode()`: 获取当前模式(true=IMU, false=MIC)
- `toggle_data_mode()`: 切换模式(带事件组同步)

## 依赖

- 输入: `CONFIG_BTN_PIN_IO`
- 输出: `CONFIG_BEEP_PIN_IO`, `CONFIG_LED_PIN_IO`
- 事件组: `mode_switch_event_group` (全局)
