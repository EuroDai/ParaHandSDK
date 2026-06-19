"""
FSR402 五路压力传感器 Python 接口

协议格式（每行输出五路传感器，逗号分隔）：
  正常帧 (5 路):
    N0,V0.1234,F0.0,S0,C0,N1,V1.2345,F100.2,S1,C1,...,N4,V4.xxxx,Fxxx,Sx,Cx
  异常帧 (单路):
    ERR,<通道号>,SIG_FAULT,<电压>,<克力>

用法:
  from fsr402_sensor import FSR402Sensor
  sensor = FSR402Sensor(port="COM7")
  sensor.start()
  while True:
      readings = sensor.read()  # list[ChannelReading]
      for ch in readings:
          print(ch.force_g)
"""

from __future__ import annotations

import json
import re
import threading
import time
from dataclasses import dataclass, asdict
from typing import List, Optional

import serial
import serial.tools.list_ports


# ── 数据模型 ──────────────────────────────────────────────────────────────────

@dataclass
class ChannelReading:
    """单路传感器读数"""
    channel: int            # 通道号 0-4
    voltage_V: float        # AO 电压 (V)
    force_g: float          # 压力 (g)
    contact: bool           # 是否按压
    contact_label: str      # "PRESSED" | "RELEASED"

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class AnomalyReading:
    """异常帧"""
    channel: int            # 通道号
    code: str               # 异常码
    voltage_V: float
    force_g: float

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class SensorFrame:
    """一帧完整数据"""
    channels: List[ChannelReading]   # 0-5 路正常读数
    anomalies: List[AnomalyReading]  # 异常通道
    any_contact: bool                # 任意一路按压
    contact_count: int               # 按压路数
    timestamp: float                 # 接收时间

    def to_dict(self) -> dict:
        return {
            "channels": [ch.to_dict() for ch in self.channels],
            "anomalies": [a.to_dict() for a in self.anomalies],
            "any_contact": self.any_contact,
            "contact_count": self.contact_count,
            "timestamp": self.timestamp,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)

    @property
    def forces(self) -> List[float]:
        """各通道压力值 (g)"""
        return [ch.force_g for ch in self.channels]

    @property
    def contacts(self) -> List[bool]:
        """各通道按压状态"""
        return [ch.contact for ch in self.channels]


# ── 解析 ─────────────────────────────────────────────────────────────────────

def _list_serial_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


def _parse_channel(token: str) -> Optional[ChannelReading]:
    """解析单个通道字段: N0,V1.234,F456.7,S1,CP"""
    m = re.match(r'N(\d),V([-\d.]+),F([-\d.]+),S(\d),C(\w)', token)
    if not m:
        return None
    return ChannelReading(
        channel=int(m.group(1)),
        voltage_V=float(m.group(2)),
        force_g=float(m.group(3)),
        contact=(m.group(4) == '1'),
        contact_label="PRESSED" if m.group(5) == 'P' else "RELEASED",
    )


def _parse_line(line: str) -> Optional[SensorFrame]:
    """解析一行串口输出，返回 SensorFrame 或 None"""
    line = line.strip()
    if not line or line.startswith('#'):
        return None

    ts = time.time()
    channels: List[ChannelReading] = []
    anomalies: List[AnomalyReading] = []

    tokens = line.split(',')
    i = 0
    while i < len(tokens):
        token = tokens[i].strip()
        if token.startswith('N'):
            # 通道段: N0,V,...,CP (5 tokens)
            if i + 4 < len(tokens):
                combined = ','.join(tokens[i:i + 5])
                ch = _parse_channel(combined)
                if ch is not None:
                    channels.append(ch)
                i += 5
            else:
                i += 1
        elif token == 'ERR':
            # 异常段: ERR,chan,code,V,F (5 tokens)
            if i + 4 < len(tokens):
                try:
                    anomalies.append(AnomalyReading(
                        channel=int(tokens[i + 1]),
                        code=tokens[i + 2],
                        voltage_V=float(tokens[i + 3]),
                        force_g=float(tokens[i + 4]),
                    ))
                except (ValueError, IndexError):
                    pass
                i += 5
            else:
                i += 1
        else:
            i += 1

    if not channels and not anomalies:
        return None

    any_ct = any(ch.contact for ch in channels)
    ct_cnt = sum(1 for ch in channels if ch.contact)

    return SensorFrame(
        channels=channels,
        anomalies=anomalies,
        any_contact=any_ct,
        contact_count=ct_cnt,
        timestamp=ts,
    )


# ── 传感器 ───────────────────────────────────────────────────────────────────

class FSR402Sensor:
    """FSR402 五路压力传感器驱动器"""

    def __init__(self, port: Optional[str] = None,
                 baudrate: int = 115200, timeout: float = 1.0):
        if port is None:
            ports = _list_serial_ports()
            if not ports:
                raise RuntimeError("未检测到串口设备")
            port = ports[-1]
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._latest: Optional[SensorFrame] = None
        self._lock = threading.Lock()
        self._running = False

    def start(self) -> None:
        self._ser = serial.Serial(port=self._port, baudrate=self._baudrate,
                                  timeout=self._timeout)
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            self._ser.close()

    def read(self) -> Optional[SensorFrame]:
        """返回最新一帧数据，无新数据返回 None"""
        with self._lock:
            return self._latest

    def read_blocking(self, timeout_s: float = 5.0) -> Optional[SensorFrame]:
        deadline = time.time() + timeout_s
        last_ts = self._latest.timestamp if self._latest else 0
        while time.time() < deadline:
            with self._lock:
                cur = self._latest
            if cur and cur.timestamp > last_ts:
                return cur
            time.sleep(0.05)
        return None

    def _reader(self) -> None:
        buf = b""
        while self._running:
            try:
                if self._ser and self._ser.in_waiting:
                    buf += self._ser.read(self._ser.in_waiting)
                else:
                    time.sleep(0.05)
                    continue
            except serial.SerialException:
                break
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                try:
                    line = raw.decode('utf-8', errors='replace')
                except Exception:
                    continue
                frame = _parse_line(line)
                if frame is not None:
                    with self._lock:
                        self._latest = frame

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()


# ── 自测 ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="FSR402 五路压力监视")
    p.add_argument("--port", type=str, default=None)
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--json", action="store_true", default=False)
    args = p.parse_args()

    if args.port is None:
        ports = _list_serial_ports()
        if not ports:
            print("错误: 未检测到串口")
            exit(1)
        args.port = ports[-1]
        print(f"自动选择: {args.port}")

    with FSR402Sensor(port=args.port, baudrate=args.baudrate) as sensor:
        print("FSR402 五路监视中 … Ctrl+C 退出\n")
        while True:
            frame = sensor.read()
            if frame:
                if args.json:
                    print(frame.to_json())
                else:
                    # 文本格式
                    parts = []
                    for ch in frame.channels:
                        stat = "●" if ch.contact else "○"
                        parts.append(f"CH{ch.channel}:{ch.force_g:6.0f}g {stat}")
                    anom = ""
                    if frame.anomalies:
                        anom = f"  ⚠️ {len(frame.anomalies)}路异常"
                    print(f"[{time.strftime('%H:%M:%S')}] {' | '.join(parts)}{anom}")
            else:
                time.sleep(0.05)
