# FlexSensor — 灵巧手 5 路压力传感器 Python 接口

基于 STM32F103C8T6 + FSR402 压力传感器的数据采集模块，通过串口实时读取灵巧手 5 路压力触发状态。

## 硬件连接

```
FSR402 × 5  →  STM32F103C8T6  →  USB串口  →  PC (Python)
```

- **传感器**: FSR402 薄膜压力传感器 × 5，通过分压电路接入 STM32 GPIO
- **主控**: STM32F103C8T6，采集 5 路 GPIO 数字电平
- **通信**: USB 虚拟串口，默认波特率 **115200**
- **数据格式**: STM32 固件以 CSV 格式持续输出 `seq,d0,d1,d2,d3,d4\r\n`

## 依赖

- Python 3.7+
- [pyserial](https://pypi.org/project/pyserial/) (`pip install pyserial`)

## 快速开始

```python
from flexsensor import FlexSensor

# 使用上下文管理器（推荐）
with FlexSensor(port="COM7", baudrate=115200) as sensor:
    # 读取全部 5 路传感器
    results = sensor.read_all()
    print(results)  # 例如: [1, 1, 0, 1, 1] — 传感器 2 触发

    # 查看当前哪些传感器被触发
    triggered = sensor.read_triggered()
    print(triggered)  # 例如: [2]
```

## 数据含义

| 返回值 | 含义 |
|--------|------|
| `0` | **压力触发** — FSR402 受力超过阈值，DO 输出低电平 |
| `1` | **无压力** — FSR402 未受力或受力不足，DO 输出高电平 |
| `-1` | **读取错误** — 串口断开 / 超时 / 数据解析失败 |

## API 参考

### `FlexSensor(port="COM7", baudrate=115200, timeout=1.0)`

构造函数。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `port` | `str` | `"COM7"` | 串口号，Windows 如 `COM7`，Linux 如 `/dev/ttyUSB0` |
| `baudrate` | `int` | `115200` | 波特率，需与 STM32 固件一致 |
| `timeout` | `float` | `1.0` | 串口读取超时（秒） |

### 连接管理

| 方法 | 说明 |
|------|------|
| `open()` | 打开串口连接，自动清空缓冲区 |
| `close()` | 关闭串口连接 |
| `is_open` | 属性，返回串口是否已打开 |

### 数据读取

#### `read(index: int) -> int`

读取单个传感器。

- `index`: 传感器索引 (0 ~ 4)
- 返回: `0` 触发 / `1` 正常 / `-1` 错误或索引越界

```python
val = sensor.read(0)  # 读取第 1 路传感器
```

#### `read_all() -> List[int]`

批量读取全部 5 路传感器。

- 返回: 长度为 5 的列表，每项为 `0` / `1` / `-1`

```python
results = sensor.read_all()  # [1, 0, 1, 1, 1]
```

#### `read_triggered() -> List[int]`

返回当前触发中的传感器索引列表（即值为 `0` 的通道）。

```python
triggered = sensor.read_triggered()  # [0, 3] 表示第 1、4 路被触发
```

### 辅助方法

| 方法 | 说明 |
|------|------|
| `flush()` | 清空串口接收缓冲区 |

### 异常

| 异常类 | 说明 |
|--------|------|
| `FlexSensorError` | 基类异常 |
| `ConnectionError` | 串口打开失败（端口不存在 / 被占用） |
| `ReadError` | 读取超时或数据异常 |

## 读取机制

- `read()` / `read_all()` / `read_triggered()` 内部调用 `_update()`，会尝试从串口缓冲区读取最新的有效数据行
- **非阻塞设计**：如果没有新数据到达，保持上一次的值不变
- **超时保护**：最多等待 50ms 获取新数据；若连续超过 2 秒无有效数据，所有通道标记为 `-1`

## 原始数据协议

STM32 固件以固定频率输出 CSV 行：

```
<seq>,<d0>,<d1>,<d2>,<d3>,<d4>\r\n
```

| 字段 | 说明 |
|------|------|
| `seq` | 递增序号 |
| `d0` ~ `d4` | 5 路传感器状态，取值 `0` 或 `1` |

示例原始数据: `3,1,1,0,1,1`

Python 侧只保留最新一条有效数据供上层轮询。

## 传感器索引参考

| 索引 | 建议对应手指 |
|------|-------------|
| 0 | 拇指 (thumb) |
| 1 | 食指 (index) |
| 2 | 中指 (middle) |
| 3 | 无名指 (ring) |
| 4 | 小指 (pinky) |

> 实际对应关系取决于硬件接线，请根据项目布线确认。

## 完整示例

```python
from flexsensor import FlexSensor, ConnectionError

try:
    sensor = FlexSensor(port="COM7")
    sensor.open()
    print(f"连接成功: {sensor}")

    for _ in range(10):
        # 方式 1: 逐路读取
        for i in range(5):
            v = sensor.read(i)
            print(f"S{i}={v} ", end="")
        print()

        # 方式 2: 只看触发的
        # triggered = sensor.read_triggered()
        # if triggered:
        #     print(f"触发: {triggered}")

        time.sleep(0.1)

    sensor.close()

except ConnectionError as e:
    print(f"连接失败: {e}")
```

## Linux 串口权限

Linux 下访问串口可能需要权限：

```bash
# 将用户加入 dialout 组（需重新登录生效）
sudo usermod -a -G dialout $USER

# 或临时赋予权限
sudo chmod 666 /dev/ttyUSB0
```
