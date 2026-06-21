# FSR402 五路压力传感器检测系统

基于 STM32F103C8T6 (Blue Pill) + FSR402 薄膜压力传感器 + 压力转换模块的五路实时压力检测系统。

## 硬件连接

### 传感器通道

| 通道 | FSR402 AO | STM32 ADC 引脚 |
|---|---|---|
| CH0 | AO | **PA0** (ADC1_IN0) |
| CH1 | AO | **PA1** (ADC1_IN1) |
| CH2 | AO | **PA2** (ADC1_IN2) |
| CH3 | AO | **PA3** (ADC1_IN3) |
| CH4 | AO | **PA4** (ADC1_IN4) |
| 全部 | VCC | 3.3V |
| 全部 | GND | GND |

### LED 指示

| LED | STM32 引脚 | 说明 |
|---|---|---|
| 板载 LED | **PC13** | 任意一路按压即亮，低电平有效 |

### 串口通信

| CH340 USB-TTL | STM32 Blue Pill |
|---|---|
| RX | **PA9** (USART1 TX) |
| TX | **PA10** (USART1 RX) |
| GND | GND |

### 烧录

| ST-Link | STM32 Blue Pill |
|---|---|
| SWDIO | PA13 |
| SWCLK | PA14 |
| 3.3V | 3.3V |
| GND | GND |

## 系统架构

```
┌──────────────────┐    5ch ADC     ┌────────────┐    串口USART1    ┌──────────┐    Python    ┌──────────────┐
│  FSR402 ×5       │ ──────────────►│ STM32F103  │ ◄──────────────►│  CH340   │ ◄───────────►│  上位机/PC   │
│  压力转换模块 ×5   │    PA0~PA4     │  Blue Pill │   简洁逐行输出    │ USB-TTL  │  fsr402.py  │              │
└──────────────────┘                └────────────┘                  └──────────┘              └──────────────┘
```

## 压力转换

**公式**：`Vout = 0.0004 × F(g) + 0.4749`

```
F(g) = (V_FSR − 0.4749) / 0.0004
  其中 V_FSR = VCC − V_AO
```

- 量程：约 6 kg / 路
- ADC：12-bit (0–4095)，参考电压 3.3V，8 次滑动平均
- 按压阈值：50g（可在源码中修改 `PRESS_THRESHOLD_G`）
- 异常检测：连续 3 次采样超出 [-100, 6500] g 范围则上报 -1

## 串口协议

每 200ms 输出一组数据，每行对应一个传感器通道：

```
PA0,60.0
PA1,-1
PA2,-1
PA3,-1
PA4,-1
```

| 格式 | 说明 |
|---|---|
| `PAx,<数值>` | 正常通道，数值为压力 (g)，保留一位小数 |
| `PAx,-1` | 异常/未连接通道 |

## 编译烧录

```bash
pio run                    # 编译
pio run --target upload    # 烧录 (ST-Link)
```

## Python SDK

### 安装

```bash
pip install pyserial
```

### 快速开始

```python
from fsr402_sensor import FSR402Sensor

with FSR402Sensor(port="COM7") as sensor:
    while True:
        frame = sensor.read()
        if frame:
            # 五路压力值（-1 表示异常）
            print(frame.forces)      # [60.0, -1.0, -1.0, -1.0, -1.0]
            # 按压状态
            print(frame.contacts)    # [True, False, False, False, False]
            # 是否有人正在按压
            print(frame.any_contact) # True
            # 按压路数
            print(frame.contact_count) # 1
```

### SensorFrame 属性

| 属性 / 方法 | 类型 | 说明 |
|---|---|---|
| `channels` | `list[ChannelReading]` | 五路读数（固定 5 个元素） |
| `forces` | `list[float]` | 五路压力 (g)，-1 表示异常 |
| `contacts` | `list[bool]` | 五路按压状态（>50g 为 True） |
| `any_contact` | `bool` | 任意一路按压 |
| `contact_count` | `int` | 按压路数 |
| `timestamp` | `float` | 接收时间 (Unix) |
| `to_dict()` | `dict` | 字典化 |
| `to_json()` | `str` | JSON 字符串 |

### ChannelReading 属性

| 属性 | 类型 | 说明 |
|---|---|---|
| `channel` | `int` | 通道号 0–4 |
| `force_g` | `float` | 压力 (g)，-1 表示异常/未连接 |
| `is_anomaly` | `bool` | 是否异常通道 |
| `contact` | `bool` | 是否按压（>50g） |

### 命令行测试

```bash
python fsr402_sensor.py --port COM7           # 文本输出
python fsr402_sensor.py --port COM7 --json    # JSON 输出
```

输出示例：
```
[14:30:02] CH0:    60g ● | CH1: ERR | CH2: ERR | CH3: ERR | CH4: ERR
[14:30:02] CH0:   100g ● | CH1:  50g ● | CH2: ERR | CH3: ERR | CH4:   30g ○
```

## 项目文件

| 文件 | 说明 |
|---|---|
| `src/main.cpp` | STM32 五路采集固件 |
| `fsr402_sensor.py` | Python SDK |
| `platformio.ini` | PlatformIO 构建配置 |
