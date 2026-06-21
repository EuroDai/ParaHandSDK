"""
FSR402 五路压力传感器 Python 接口

协议格式（每行一个传感器）：
  正常: PA0,60.0
  异常: PA0,-1

用法:
  from fsr402_sensor import FSR402Sensor
  sensor = FSR402Sensor(port="COM7")
  sensor.start()
  while True:
      frame = sensor.read()  # SensorFrame (5路)
      for ch in frame.channels:
          print(f"CH{ch.channel}: {ch.force_g}g")
"""

from __future__ import annotations

import json
import re
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import serial
import serial.tools.list_ports


# ── 常量 ──────────────────────────────────────────────────────────────────────
PRESS_THRESHOLD_G = 50.0   # 按压阈值 (g)，与硬件端一致


# ── 数据模型 ──────────────────────────────────────────────────────────────────

@dataclass
class ChannelReading:
    """单路传感器读数"""
    channel: int            # 通道号 0-4
    force_g: float          # 压力 (g), -1 表示异常/未连接

    @property
    def is_anomaly(self) -> bool:
        """是否为异常通道"""
        return self.force_g < 0

    @property
    def contact(self) -> bool:
        """是否按压（压力超过阈值）"""
        return self.force_g > PRESS_THRESHOLD_G

    def to_dict(self) -> dict:
        return {
            "channel": self.channel,
            "force_g": self.force_g,
            "is_anomaly": self.is_anomaly,
            "contact": self.contact,
        }


@dataclass
class SensorFrame:
    """一帧完整数据（5 路传感器）"""
    channels: List[ChannelReading]   # 长度固定为 5，对应 PA0~PA4
    any_contact: bool                # 任意一路按压
    contact_count: int               # 按压路数
    timestamp: float                 # 接收时间

    def to_dict(self) -> dict:
        return {
            "channels": [ch.to_dict() for ch in self.channels],
            "any_contact": self.any_contact,
            "contact_count": self.contact_count,
            "timestamp": self.timestamp,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)

    @property
    def forces(self) -> List[float]:
        """各通道压力值 (g)，-1 表示异常"""
        return [ch.force_g for ch in self.channels]

    @property
    def contacts(self) -> List[bool]:
        """各通道按压状态"""
        return [ch.contact for ch in self.channels]


# ── 解析 ─────────────────────────────────────────────────────────────────────

def _list_serial_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


# 匹配格式: PA0,60.0 或 PA1,-1
_LINE_RE = re.compile(r'PA(\d),(-?\d+\.?\d*)')


def _parse_line(line: str) -> Optional[ChannelReading]:
    """解析一行串口输出，返回 ChannelReading 或 None"""
    line = line.strip()
    if not line or line.startswith('#'):
        return None
    m = _LINE_RE.match(line)
    if not m:
        return None
    return ChannelReading(
        channel=int(m.group(1)),
        force_g=float(m.group(2)),
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
        """阻塞等待直到有新帧，超时返回 None"""
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
        # 用 -1 占位初始化 5 路，有新数据即更新
        latest_map: Dict[int, ChannelReading] = {
            i: ChannelReading(i, -1.0) for i in range(5)
        }
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
                ch = _parse_line(line)
                if ch is not None:
                    latest_map[ch.channel] = ch
                    # 每收到一个通道读数就组装完整帧并发布
                    channels = [latest_map[i] for i in range(5)]
                    frame = SensorFrame(
                        channels=channels,
                        any_contact=any(c.contact for c in channels),
                        contact_count=sum(1 for c in channels if c.contact),
                        timestamp=time.time(),
                    )
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
                    parts = []
                    for ch in frame.channels:
                        if ch.is_anomaly:
                            parts.append(f"CH{ch.channel}: ERR")
                        else:
                            stat = "●" if ch.contact else "○"
                            parts.append(f"CH{ch.channel}:{ch.force_g:6.0f}g {stat}")
                    print(f"[{time.strftime('%H:%M:%S')}] {' | '.join(parts)}")
            else:
                time.sleep(0.05)
