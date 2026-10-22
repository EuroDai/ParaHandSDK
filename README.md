# ParaHandSDK

ParaHand 灵巧手 Python SDK，包含电机控制、hand-position 控制，以及两种可切换的触觉传感器：

- **fsr402**：五路 FSR402 薄膜压力传感器，输出单位为克（g）。
- **hwk**：华威科阵列触觉传感器，输出内部标定后的克重压力矩阵，以及 `Fx / Fy / Fnormal_sum` 三项克重力汇总。

传感器模式可以通过 config_hand.yaml、Python API 或 GUI 选择。两种驱动都通过统一的 start、stop、read 生命周期接入 ParaHand。

## 1. 安装

基础串口和配置依赖：

~~~powershell
python -m pip install pyserial pyyaml
~~~

运行 GUI 还需要：

~~~powershell
python -m pip install PySide6
~~~

本仓库目前没有 setup.py 或 pyproject.toml，请在仓库根目录运行示例和测试。

## 2. 触觉模式配置

两种传感器分别保存连接参数，切换模式时不需要反复修改波特率：

~~~yaml
tactile:
  enabled: true
  mode: fsr402              # fsr402 或 hwk

  fsr402:
    port: COM8              # auto 表示自动选择
    baudrate: 115200
    timeout_s: 1.0
    contact_threshold_g: 50.0
    channel_fingers:
    - thumb
    - index
    - middle
    - ring
    - little

  hwk:
    port: COM9
    baudrate: 921600
    response_timeout_s: 0.2
    write_timeout_s: 0.2
    poll_interval_s: 0.0
    contact_threshold_g: 100
    device_regions:
      1: thumb
      2: index
      3: middle
      4: ring
      5: little
      6: palm
~~~

device_regions 是设备 ID 到手指或掌心的显式映射。上例采用 1..6 对应五指加掌心的默认顺序；如果实物地址不同，应按实际接线修改。

旧版 FSR402 平铺配置仍可读取。一旦通过 GUI 或 update_tactile_config() 保存，会迁移为新的分模式结构。

## 3. 统一 Python API

### 按配置启动

~~~python
import time
from parahand import ParaHand

hand = ParaHand("config_hand.yaml")

if not hand.start_tactile():
    raise RuntimeError(hand.last_tactile_error)

try:
    while True:
        snapshot = hand.get_tactile_snapshot()
        if snapshot is not None:
            print(snapshot.mode, snapshot.unit, snapshot.value_map)
        time.sleep(0.05)
finally:
    hand.stop_tactile()
~~~

当 tactile.enabled 为 true 时，hand.connect() 也会自动启动所选触觉模式。触觉串口与电机串口相互独立。

调用时可以覆盖模式和串口：

~~~python
hand.start_tactile(mode="fsr402", port="COM8")
hand.start_tactile(mode="hwk", port="COM9")
~~~

正在运行时切换模式，SDK 会先停止旧驱动，再启动新驱动。

### 统一快照

get_tactile_snapshot() 返回 TactileSnapshot：

| 字段 | 说明 |
|---|---|
| mode | fsr402 或 hwk |
| timestamp | 最新区域数据的 Unix 时间 |
| unit | 两种模式均为 g |
| regions | 区域名到 TactileRegion 的映射 |
| value_map | 区域名到代表值的映射 |
| matrices | 区域名到空间压力二维矩阵的映射；不包含力汇总包 |
| force_summaries | 区域名到 Fx、Fy、Fnormal_sum 的映射，单位为 g |
| raw | 对应驱动的原生数据 |

TactileRegion.representative_value 用于 GUI 和阈值判断：

- FSR402：该通道的克重值。
- 华威科：主空间压力矩阵中的最大克重值。

华威科代表值只用于状态显示。需要空间分布时，应读取完整矩阵：

~~~python
snapshot = hand.get_tactile_snapshot()
if snapshot and snapshot.mode == "hwk":
    for region, matrices in snapshot.matrices.items():
        print(region, matrices)
    for region, summary in snapshot.force_summaries.items():
        print(region, summary)  # Fx、Fy、Fnormal_sum，单位 g

matrix_map = hand.get_tactile_matrices()
force_summary_map = hand.get_tactile_force_summaries()
~~~

### 五指兼容接口

~~~python
values = hand.get_tactile_values()
# 固定顺序：thumb, index, middle, ring, little
~~~

以下旧接口继续可用：

~~~python
hand.get_tactile_forces()
hand.get_fingertip_forces()
hand.get_tactile_force_map()
hand.get_tactile_frame()
~~~

旧方法名继续可用；华威科模式返回主压力矩阵最大克重值，单位为 g。

### 示例和 GUI

~~~powershell
python example.py
python example.py --tactile-mode fsr402 --tactile-port COM8
python example.py --tactile-mode hwk --tactile-port COM9
~~~

GUI 的“控制器配置”区域提供 FSR402 / 华威科下拉框。华威科模式显示五指压力矩阵，并分别显示 `Fx / Fy / Fnormal_sum`；所有数值单位均为 g。

## 4. FSR402 接口

### 硬件连接

五路输出连接到 STM32F103C8T6 ADC：

| 通道 | FSR402 AO | STM32 ADC 引脚 |
|---|---|---|
| CH0 | AO | PA0 / ADC1_IN0 |
| CH1 | AO | PA1 / ADC1_IN1 |
| CH2 | AO | PA2 / ADC1_IN2 |
| CH3 | AO | PA3 / ADC1_IN3 |
| CH4 | AO | PA4 / ADC1_IN4 |
| 全部 | VCC | 3.3V |
| 全部 | GND | GND |

CH340 USB-TTL 连接：

| CH340 | STM32 |
|---|---|
| RX | PA9 / USART1 TX |
| TX | PA10 / USART1 RX |
| GND | GND |

压力换算：

~~~text
Vout = 0.0004 × F(g) + 0.4749
F(g) = (V_FSR - 0.4749) / 0.0004
V_FSR = VCC - V_AO
~~~

串口约每 200 ms 输出一组逐行数据：

~~~text
PA0,60.0
PA1,-1
PA2,-1
PA3,-1
PA4,-1
~~~

- PAx,数值：压力克重。
- PAx,-1：异常或未连接。
- FSR402 驱动默认接触阈值为 50 g；ParaHand 和 GUI 使用 contact_threshold_g 配置。

### 原生 API

~~~python
from fsr402_sensor import FSR402Sensor

with FSR402Sensor(port="COM8") as sensor:
    frame = sensor.read_blocking(timeout_s=5.0)
    if frame:
        print(frame.forces)
        print(frame.contacts)
        print(frame.any_contact)
        print(frame.contact_count)
~~~

SensorFrame 提供 channels、forces、contacts、any_contact、contact_count、timestamp、to_dict() 和 to_json()。

原生命令行监视：

~~~powershell
python fsr402_sensor.py --port COM8
python fsr402_sensor.py --port COM8 --json
~~~

## 5. 华威科接口

### 串口和协议

默认行为：

- 921600 / 8N1，无软硬件流控。
- 主从模式，设备 ID 在高 4 位、通道在低 4 位。
- 数据通道为 0x02，轮询负载为 0x01，归零通道为 0x07。
- CRC16 初值为 0，多项式为 0xA001，校验范围为负载。
- 同时接受协议正文 ACK 0b11 和示例设备 ACK 0b10。
- 增量解析器支持拆包、粘包、前导噪声、CRC 错误和重新同步。

Windows 存在多个 COM 口时，应显式配置 port，避免连接到电机或其他串口。

### 原生 API

~~~python
from hwk_tactile import TactileSensorBus

with TactileSensorBus("COM9", response_timeout=0.2) as bus:
    reading = bus.poll_device(1)
    for packet in reading.packets:
        print(packet.rows, packet.columns, packet.bytes_per_value)
        print(packet.matrix)
~~~

轮询多个设备：

~~~python
with TactileSensorBus("COM9") as bus:
    readings = bus.poll_devices(range(1, 7), ignore_timeouts=True)
~~~

持续读取：

~~~python
with TactileSensorBus("COM9") as bus:
    for reading in bus.iter_readings(range(1, 7)):
        for packet in reading.packets:
            if packet.data_type == "pressure_map":
                consume_pressure(reading.device_id, packet.matrix, packet.unit)
            elif packet.data_type == "force_summary":
                consume_force(reading.device_id, dict(zip(packet.labels, packet.values)))
~~~

归零校准：

~~~python
hand.start_tactile(mode="hwk")
hand.calibrate_tactile(device_id=1, command=2)
~~~

command 为 0 表示关闭归零，1 表示开启归零，2 表示重新校准。

### 数据含义和限制

基础负载格式：

~~~text
总包数 | 当前包序号 | 列数 | 行数 | rows × columns 个 uint16 小端值
~~~

实机也可能在同一负载中连续返回多个数据块：

~~~text
总块数 | 块1序号 | 列 | 行 | 块1数据 | 块2序号 | 列 | 行 | 块2数据 | ...
~~~

接口同时支持单帧单块、多帧分块和同帧复合块，并动态使用响应中的行列数和 PressurePacket.bytes_per_value。

2026-07-14 的 ID 1 实机响应包含 `8×4 / uint16 / g` 空间压力块与一个力汇总包。力汇总包头部的 `03 04` 表示三个分量、每项四字节，后续 12 字节依次按小端 IEEE-754 `float32` 解析为 `Fx`、`Fy` 和 `Fnormal_sum`，单位均为 g；不能按三个 `int32` 解析。

厂家确认传感器内部已经完成标定，主机端不应再次应用零点或增益标定：

- PressurePacket.values 是一维传感器输出。
- PressurePacket.matrix 是按行优先整理的二维矩阵。
- PressurePacket.unit 给出物理单位；当前空间矩阵和力汇总均为 g。
- PressurePacket.data_type 区分 `pressure_map` 与 `force_summary`。
- `force_summary` 的 PressurePacket.labels 固定为 `Fx / Fy / Fnormal_sum`。
- 空间扫描方向和传感器有效量程仍待确认。

## 6. ParaHand hand-position 接口

ParaHand 保留两类电机控制语义：

- set_joint_positions() / get_joint_feedback()：关节角度，单位为度。
- set_hand_position_map() / get_hand_position_map()：整手姿态；普通关节为弧度，pip_dip 为 tendon length，单位为米。

~~~python
import math
from parahand import ParaHand

hand = ParaHand("config_hand.yaml")
hand.connect()
hand.start_polling()
hand.enable()

positions = {name: 0.0 for name in hand.get_hand_joint_order()}
positions["thumb.cmc_1"] = math.radians(10.0)
positions["index.mcp_2"] = math.radians(35.0)
positions["index.pip_dip"] = 0.012
hand.set_hand_position_map(positions)
~~~

固定顺序数组可使用 set_hand_positions()。MuJoCo 部署应使用 hand-position 系列接口；pip_dip 直接传 tendon length，不要套用 teleop 中的反向变换。

## 7. 测试

统一传感器层测试不需要硬件：

~~~powershell
python -m unittest discover -s tests -v
~~~

测试覆盖 FSR402 映射和克重快照、华威科后台轮询和矩阵、归零调用、模式切换以及两套配置互不覆盖。

真实串口验证仍需对应硬件，并应重点确认 COM 口、华威科设备 ID 映射和压力矩阵方向。

## 8. 相关文件

| 文件 | 说明 |
|---|---|
| parahand.py | 灵巧手主接口和触觉模式接入 |
| tactile_sensor.py | 统一模型、适配器和工厂 |
| fsr402_sensor.py | FSR402 原生驱动 |
| hwk_tactile/ | 华威科协议、串口总线和异常类型 |
| config_hand.yaml | 电机和双模式触觉配置 |
| example.py | 控制及模式选择示例 |
| gui_app.py | 带模式选择的 GUI |
| tests/ | 无硬件单元测试 |


## 原始串口记录（GUI / deploy 对比）

在启动 GUI 或部署进程前设置 `PARAHAND_TRACE_DIR` 为日志目录，
可用 `PARAHAND_TRACE_LABEL=gui` 或 `deploy-reset` 区分实验。
每次连接自动生成独立 JSONL，断开连接时结束；不额外打开串口或发送命令。
未设置环境变量时不创建记录器。

记录包含原始收发字节、实际写入返回值、CRC 校验、状态解析、查询结果与
反馈存储时间。时间戳为同一主机的 monotonic 时间，metadata 另含墙钟时间。
写入串口成功不等于电机收到或执行；坏包中的电机 ID 也不可信。
记录采用有限队列和异步写盘，单次上限 512 MiB；丢记录或写盘失败会标记为
不完整。没有 trace_end 的日志不能据此证明“没有回包”。

在 SDK 目录离线汇总：

```sh
uv run python serial_trace.py /path/gui.jsonl /path/deploy.jsonl --motor 3
```

完整比较需同时查看 query_result、tx_written、rx_chunk、rx_frame、status
及 feedback_store；尤其注意运动前后回复间隔和查询超时窗口。
