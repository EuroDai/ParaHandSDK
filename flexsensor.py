"""
灵巧手 FlexSensor Python 接口

通过串口读取 STM32F103C8T6 上 5 路 FSR402 压力传感器数据。
STM32 固件以 CSV 格式持续输出: seq,d0,d1,d2,d3,d4\\r\\n
"""

import serial
import time
from typing import List, Optional, Tuple


class FlexSensorError(Exception):
    """FlexSensor 异常基类"""
    pass


class ConnectionError(FlexSensorError):
    """串口连接失败"""
    pass


class ReadError(FlexSensorError):
    """读取失败 (超时 / 数据异常)"""
    pass


class FlexSensor:
    """
    灵巧手 5 路压力传感器接口。

    使用示例::

        sensor = FlexSensor(port="COM7", baudrate=115200)
        sensor.open()

        # 单路读取
        val = sensor.read(0)  # 0=触发, 1=正常, -1=错误

        # 批量读取
        results = sensor.read_all()  # [1, 0, 1, 1, 1]

        sensor.close()

    上下文管理器::

        with FlexSensor(port="COM7") as sensor:
            print(sensor.read_all())
    """

    SENSOR_COUNT = 5
    ERROR_VALUE = -1

    def __init__(self, port: str = "COM7", baudrate: int = 115200,
                 timeout: float = 1.0):
        """
        :param port: 串口号，Windows 如 "COM7"，Linux 如 "/dev/ttyUSB0"
        :param baudrate: 波特率，需与 STM32 固件一致 (默认 115200)
        :param timeout: 串口读取超时 (秒)
        """
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._seq_last = -1
        self._last_data: List[int] = [self.ERROR_VALUE] * self.SENSOR_COUNT
        self._last_read_time = 0.0

    # ── 连接管理 ──────────────────────────────────────

    def open(self) -> None:
        """打开串口连接"""
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout
            )
        except serial.SerialException as e:
            raise ConnectionError(f"无法打开串口 {self._port}: {e}") from e

        # 丢弃缓冲区中可能存在的半行数据
        time.sleep(0.1)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        """关闭串口连接"""
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    @property
    def is_open(self) -> bool:
        """串口是否已打开"""
        return self._ser is not None and self._ser.is_open

    # ── 底层读取 ──────────────────────────────────────

    @staticmethod
    def _parse(line: str) -> Optional[List[int]]:
        """
        解析一行 CSV 数据。
        :param line: 原始行字符串，如 "3,1,1,0,1,1"
        :return: [d0,d1,d2,d3,d4] 或 None (解析失败)
        """
        line = line.strip()
        if not line:
            return None

        parts = line.split(',')
        if len(parts) != FlexSensor.SENSOR_COUNT + 1:  # seq + 5 values
            return None

        try:
            seq = int(parts[0])
            values = [int(p) for p in parts[1:6]]
            # 校验: 每个值只能是 0 或 1
            if any(v not in (0, 1) for v in values):
                return None
        except (ValueError, IndexError):
            return None

        return values

    def _read_raw_line(self) -> Optional[str]:
        """从串口读取一行原始数据，失败返回 None"""
        if not self.is_open:
            return None
        try:
            line = self._ser.readline()
            if not line:
                return None
            try:
                return line.decode('utf-8', errors='replace').strip()
            except UnicodeDecodeError:
                return line.decode('latin-1', errors='replace').strip()
        except serial.SerialException:
            return None

    # ── 公开接口 ──────────────────────────────────────

    def read(self, index: int) -> int:
        """
        读取单个传感器。

        :param index: 传感器索引 (0 ~ 4)
        :return:
              0 — 压力触发 (DO 低电平)
              1 — 无压力   (DO 高电平)
             -1 — 读取失败 (超时 / 解析错误 / 索引越界)
        """
        if index < 0 or index >= self.SENSOR_COUNT:
            return self.ERROR_VALUE

        self._update()
        return self._last_data[index]

    def read_all(self) -> List[int]:
        """
        批量读取全部 5 路传感器。

        :return: 长度为 5 的列表，每项 0/1/-1
        """
        self._update()
        return list(self._last_data)

    def read_triggered(self) -> List[int]:
        """
        返回当前触发中的传感器索引列表。

        :return: 例如 [0, 3] 表示传感器 0 和 3 当前有压力触发
        """
        self._update()
        return [i for i, v in enumerate(self._last_data) if v == 0]

    def _update(self) -> None:
        """
        从串口缓冲区读取最新一行有效数据。
        非阻塞：如果没有新数据，保持上次的值不变。
        """
        if not self.is_open:
            for i in range(self.SENSOR_COUNT):
                self._last_data[i] = self.ERROR_VALUE
            return

        got_valid = False
        timeout_start = time.time()
        max_wait = 0.05  # 最多等 50ms

        while True:
            line = self._read_raw_line()
            if line is None:
                break

            values = self._parse(line)
            if values is not None:
                self._last_data = values
                self._last_read_time = time.time()
                got_valid = True

            if time.time() - timeout_start > max_wait:
                break

        # 超过 2s 没读到有效数据，标记错误
        if not got_valid and time.time() - self._last_read_time > 2.0:
            for i in range(self.SENSOR_COUNT):
                self._last_data[i] = self.ERROR_VALUE

    def flush(self) -> None:
        """清空串口缓冲区"""
        if self.is_open:
            self._ser.reset_input_buffer()

    # ── 上下文管理器 ──────────────────────────────────

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    def __repr__(self):
        state = "open" if self.is_open else "closed"
        return f"<FlexSensor port={self._port} state={state} data={self._last_data}>"
