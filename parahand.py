from __future__ import annotations

import copy
import logging
import math
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import motor
from tactile_sensor import (
    FSR402_MODE,
    HWK_MODE,
    TactileSnapshot,
    create_tactile_sensor,
    normalize_tactile_mode,
)


CONFIG_KEYS = {"ctrl_frequency", "serial", "baudrate", "tactile"}
COUPLED_MCP1_JOINTS = (
    "index.mcp_1",
    "middle.mcp_1",
    "ring.mcp_1",
    "little.mcp_1",
)
FINGERTIP_FORCE_ORDER = ("thumb", "index", "middle", "ring", "little")
DEFAULT_CONFIG_PATH = Path(__file__).with_name("config_hand.yaml")
DEFAULT_FEEDBACK = {
    "position_deg": None,
    "position_updated_at_monotonic_s": None,
    "speed_pct": None,
    "current_mA": None,
    "online": None,
    "fsm_state": None,
    "error_code": None,
    "temp": None,
    "bus_voltage": None,
}


@dataclass(frozen=True)
class JointDefinition:
    motor_id: int
    min_deg: float
    max_deg: float
    reverse: bool
    enabled: bool


@dataclass(frozen=True)
class FSR402TactileDefinition:
    port: Optional[str]
    baudrate: int
    timeout_s: float
    channel_fingers: tuple[str, ...]
    contact_threshold_g: float


@dataclass(frozen=True)
class HWKTactileDefinition:
    port: Optional[str]
    baudrate: int
    response_timeout_s: float
    write_timeout_s: float
    poll_interval_s: float
    device_regions: tuple[tuple[int, str], ...]
    contact_threshold_g: float

    @property
    def contact_threshold_raw(self) -> float:
        """兼容旧字段名；华威科输出现已确认单位为 g。"""

        return self.contact_threshold_g


@dataclass(frozen=True)
class TactileDefinition:
    enabled: bool
    mode: str
    fsr402: FSR402TactileDefinition
    hwk: HWKTactileDefinition

    @property
    def active(self) -> FSR402TactileDefinition | HWKTactileDefinition:
        return self.fsr402 if self.mode == FSR402_MODE else self.hwk

    @property
    def port(self) -> Optional[str]:
        return self.active.port

    @property
    def baudrate(self) -> int:
        return self.active.baudrate

    @property
    def timeout_s(self) -> float:
        if self.mode == FSR402_MODE:
            return self.fsr402.timeout_s
        return self.hwk.response_timeout_s

    @property
    def channel_fingers(self) -> tuple[str, ...]:
        return self.fsr402.channel_fingers

    @property
    def contact_threshold_g(self) -> float:
        """返回当前触觉模式以克为单位的接触阈值。"""

        if self.mode == FSR402_MODE:
            return self.fsr402.contact_threshold_g
        return self.hwk.contact_threshold_g


class ParaHand:
    def __init__(self, config_path: Optional[str] = None):
        self.config_path = Path(config_path) if config_path is not None else DEFAULT_CONFIG_PATH
        self.config = self._load_config(self.config_path)
        port, baudrate, timeout_s, write_timeout_s = self._extract_connection_config(self.config)

        self.motor = motor.Motor(
            port=port,
            baudrate=baudrate,
            timeout_s=timeout_s,
            write_timeout_s=write_timeout_s,
        )
        self._feedback_lock = threading.Lock()
        self._motor_feedback: Dict[int, Dict[str, Any]] = {}
        self._poll_stop_event = threading.Event()
        self._poll_thread: Optional[threading.Thread] = None
        self._poll_failure_counts: Dict[int, int] = {}
        self._poll_next_allowed_at: Dict[int, float] = {}
        self._poll_bus_pause_until: float = 0.0
        self._poll_cursor = 0
        self._poll_error_count = 0
        self._last_poll_error = None
        self._last_poll_warning_at = float("-inf")
        self._callbacks_registered = False
        self.ctrl_frequency = self._parse_ctrl_frequency(self.config.get("ctrl_frequency", 50))
        self.joint_to_motor: Dict[str, JointDefinition] = {}
        self.motor_to_joint: Dict[int, str] = {}
        self._last_joint_targets_deg: Dict[str, float] = {}
        self.tactile_config = self._extract_tactile_config(self.config)
        self._tactile_lock = threading.Lock()
        self._tactile_sensor: Optional[Any] = None
        self._tactile_started = False
        self._tactile_mode: Optional[str] = None
        self._last_tactile_error: Optional[BaseException] = None
        self._rebuild_from_config(self.config)

    @property
    def connected(self) -> bool:
        '''返回当前电机连接状态。'''
        return self.motor.connected

    @property
    def tactile_connected(self) -> bool:
        '''返回触觉传感器后台读取线程是否已启动。'''
        with self._tactile_lock:
            sensor = self._tactile_sensor
        if not self._tactile_started or sensor is None:
            return False
        return bool(getattr(sensor, "is_running", True))

    @property
    def tactile_mode(self) -> str:
        '''返回当前运行或配置选中的触觉传感器模式。'''
        return self._tactile_mode or self.tactile_config.mode

    @property
    def tactile_unit(self) -> str:
        '''返回当前模式的代表值单位。'''
        return "g"

    @property
    def last_tactile_error(self) -> Optional[BaseException]:
        with self._tactile_lock:
            sensor = self._tactile_sensor
        runtime_error = getattr(sensor, "last_error", None) if sensor is not None else None
        return runtime_error or self._last_tactile_error

    def connect(self) -> bool:
        '''连接电机并注册反馈回调；触觉传感器独立启动不受电机连接影响。'''
        if self.motor.connected:
            self._register_callbacks()
        else:
            motor_ok = self.motor.connect()
            if motor_ok:
                self._register_callbacks()

        self._poll_failure_counts.clear()
        self._poll_next_allowed_at.clear()
        self._poll_bus_pause_until = 0.0
        self._poll_cursor = 0

        if self.tactile_config.enabled:
            self.start_tactile()

        return self.motor.connected

    def disconnect(self):
        '''停止轮询并断开电机和触觉传感器连接。'''
        self.stop_polling()
        self.stop_tactile()
        self._unregister_callbacks()
        self.motor.disconnect()

    def wait_for_commands(self, timeout_s: float = 1.0) -> None:
        '''等待电机串口发送队列排空。'''
        self.motor.wait_for_commands(timeout_s)

    def enable(self, joints: Optional[Iterable[str]] = None) -> str:
        '''使能指定关节或全部已启用关节。'''
        motor_ids = self._resolve_joint_motor_ids(self._active_joint_names() if joints is None else joints)
        if not motor_ids:
            return ""
        return self.motor.enable_motors_broadcast({motor_id: True for motor_id in motor_ids})

    def disable(self, joints: Optional[Iterable[str]] = None) -> str:
        '''失能指定关节或全部关节。'''
        joint_names = self.joint_to_motor.keys() if joints is None else joints
        motor_ids = self._resolve_joint_motor_ids(joint_names, include_disabled=True)
        if not motor_ids:
            return ""
        return self.motor.enable_motors_broadcast({motor_id: False for motor_id in motor_ids})

    def enable_motor(self, motor_id: int, enable: bool = True) -> str:
        '''设置单个电机的使能状态。'''
        self._validate_motor_id(motor_id)
        return self.motor.enable_motor(motor_id, enable)

    def get_hand_joint_order(self) -> list[str]:
        '''返回整手位置接口使用的固定关节顺序。'''
        return list(self.joint_to_motor.keys())

    def is_pip_dip_joint(self, joint_name: str) -> bool:
        '''判断指定关节是否为 pip_dip。'''
        return joint_name.endswith(".pip_dip")

    def get_hand_position_units(self) -> Dict[str, str]:
        '''返回整手位置接口中各关节的单位：普通关节为 rad，pip_dip 为 m。'''
        return {
            joint_name: "m" if self.is_pip_dip_joint(joint_name) else "rad"
            for joint_name in self.get_hand_joint_order()
        }

    def set_joint_positions(self, targets_deg: Dict[str, float], speed: Optional[Any] = None) -> Dict[str, str]:
        '''按关节名批量设置目标角度。'''
        return self._dispatch_joint_positions(self.resolve_joint_targets(targets_deg), speed)

    def _dispatch_joint_positions(self, targets_deg: Dict[str, float], speed: Optional[Any] = None) -> Dict[str, str]:
        if not targets_deg:
            raise ValueError("targets_deg 不能为空")

        filtered_targets = {
            joint_name: angle_deg
            for joint_name, angle_deg in targets_deg.items()
            if self._get_joint_definition(joint_name).enabled
        }
        if not filtered_targets:
            return {}

        speed_ratios = self._build_joint_speed_ratios(filtered_targets, speed)
        command_ids: Dict[str, str] = {}
        for joint_name, angle_deg in filtered_targets.items():
            definition = self._get_joint_definition(joint_name)
            command_id = self.set_motor_target(
                definition.motor_id,
                angle_deg,
                speed=speed_ratios.get(joint_name),
            )
            if command_id:
                command_ids[joint_name] = command_id
        return command_ids

    def resolve_joint_targets(self, targets_deg: Dict[str, float]) -> Dict[str, float]:
        '''解析关节目标并应用 mcp_1 联动限位。'''
        resolved_targets = {joint_name: float(angle_deg) for joint_name, angle_deg in targets_deg.items()}
        if any(joint_name in COUPLED_MCP1_JOINTS for joint_name in resolved_targets):
            resolved_targets.update(self._resolve_coupled_mcp1_targets(resolved_targets))
        self._last_joint_targets_deg.update(resolved_targets)
        return resolved_targets

    def hand_position_map_to_joint_targets_deg(self, positions: Any) -> Dict[str, float]:
        '''将按关节名组织的整手位置语义值转换为关节角度目标。'''
        joint_names = self.get_hand_joint_order()
        if not joint_names:
            raise RuntimeError("当前配置里没有可用的 joint 映射")
        if not isinstance(positions, dict):
            raise TypeError("positions 必须是按关节名组织的 dict")

        raw_targets: Dict[str, float] = {}
        for joint_name in joint_names:
            if joint_name not in positions:
                raise KeyError(f"缺少关节 {joint_name} 的位置输入")
            raw_targets[joint_name] = float(positions[joint_name])
        return self._raw_hand_positions_to_joint_targets_deg(raw_targets)

    def set_hand_position_map(self, positions: Dict[str, float], speed: Optional[Any] = None) -> Dict[str, str]:
        '''按关节名控制整手位置：普通关节 rad，pip_dip 为 tendon length(m)。'''
        return self.set_joint_positions(
            self.hand_position_map_to_joint_targets_deg(positions),
            speed,
        )

    def set_hand_position_map_broadcast(self, positions: Dict[str, float]) -> str:
        '''按关节名广播控制整手位置：普通关节 rad，pip_dip 为 tendon length(m)。'''
        return self.set_joint_positions_broadcast(
            self.hand_position_map_to_joint_targets_deg(positions)
        )

    def joint_targets_deg_to_hand_position_map(self, targets_deg: Dict[str, Any]) -> Dict[str, float]:
        '''将整手关节角度目标转换回整手位置语义值：普通关节 rad，pip_dip m。'''
        if not isinstance(targets_deg, dict):
            raise TypeError("targets_deg 必须是按关节名组织的 dict")

        positions: Dict[str, float] = {}
        for joint_name in self.get_hand_joint_order():
            if joint_name not in targets_deg:
                raise KeyError(f"缺少关节 {joint_name} 的角度输入")
            joint_value_deg = float(targets_deg[joint_name])
            if self.is_pip_dip_joint(joint_name):
                finger_name, _ = self._split_joint_name(joint_name)
                mcp_2_name = f"{finger_name}.mcp_2"
                if mcp_2_name not in targets_deg:
                    raise KeyError(f"未找到 {joint_name} 对应的 {mcp_2_name} 角度输入")
                positions[joint_name] = self.pip_dip_angle_to_tendon_length_m(
                    math.radians(float(targets_deg[mcp_2_name])),
                    joint_value_deg,
                )
            else:
                positions[joint_name] = math.radians(joint_value_deg)
        return positions

    def joint_feedback_to_hand_position_map(
        self,
        joint_feedback: Dict[str, Dict[str, Any]],
    ) -> Dict[str, Optional[float]]:
        '''将 get_joint_feedback 的角度反馈转换为整手位置语义值。'''
        positions: Dict[str, Optional[float]] = {}
        for joint_name in self.get_hand_joint_order():
            definition = self._get_joint_definition(joint_name)
            if not definition.enabled:
                positions[joint_name] = None
                continue

            joint_position_deg = joint_feedback.get(joint_name, {}).get("position_deg")
            if joint_position_deg is None:
                positions[joint_name] = None
                continue

            if self.is_pip_dip_joint(joint_name):
                finger_name, _ = self._split_joint_name(joint_name)
                mcp_2_name = f"{finger_name}.mcp_2"
                mcp_2_deg = joint_feedback.get(mcp_2_name, {}).get("position_deg")
                if mcp_2_deg is None:
                    positions[joint_name] = None
                    continue
                positions[joint_name] = self.pip_dip_angle_to_tendon_length_m(
                    math.radians(float(mcp_2_deg)),
                    float(joint_position_deg),
                )
            else:
                positions[joint_name] = math.radians(float(joint_position_deg))
        return positions

    def get_hand_position_map(self) -> Dict[str, Optional[float]]:
        '''按关节名返回整手位置反馈：普通关节 rad，pip_dip 为补偿后的 tendon length(m)。'''
        return self.joint_feedback_to_hand_position_map(self.get_joint_feedback())

    def get_hand_positions(self) -> list[Optional[float]]:
        '''按固定顺序返回整手位置反馈，顺序同 get_hand_joint_order。'''
        position_map = self.get_hand_position_map()
        return [position_map[joint_name] for joint_name in self.get_hand_joint_order()]

    def set_hand_positions(self, positions: Iterable[float], speed: Optional[Any] = None) -> Dict[str, str]:
        '''按固定顺序用输入数组控制整手关节：列表 0~15 依次对应 thumb.cmc_1(rad)、thumb.cmc_2(rad)、thumb.mcp(rad)、thumb.ip(rad)、index.mcp_1(rad)、index.mcp_2(rad)、index.pip_dip(m)、middle.mcp_1(rad)、middle.mcp_2(rad)、middle.pip_dip(m)、ring.mcp_1(rad)、ring.mcp_2(rad)、ring.pip_dip(m)、little.mcp_1(rad)、little.mcp_2(rad)、little.pip_dip(m)。'''
        joint_names = self.get_hand_joint_order()
        targets_deg = self._coerce_hand_positions_to_joint_targets_deg(positions)

        speed_input: Optional[Any] = speed
        if isinstance(speed, dict):
            raise TypeError("speed 不能是 dict，请传入单个速度值或速度数组")
        if speed is not None:
            try:
                speed_input = self._normalize_speed_value(speed)
            except (TypeError, ValueError):
                try:
                    speed_values = [float(value) for value in speed]
                except TypeError as exc:
                    raise TypeError("speed 必须是数字或可迭代的速度数组") from exc
                except ValueError as exc:
                    raise ValueError("speed 数组中的元素必须是数字") from exc

                if len(speed_values) != len(joint_names):
                    raise ValueError(
                        f"speed 长度必须等于 {len(joint_names)}，顺序为 {joint_names}"
                    )

                speed_input = dict(zip(joint_names, speed_values))

        return self.set_joint_positions(targets_deg, speed_input)

    def set_hand_positions_broadcast(self, positions: Iterable[float]) -> str:
        targets_deg = self._coerce_hand_positions_to_joint_targets_deg(positions)
        return self.set_joint_positions_broadcast(targets_deg)

    def set_joint_positions_broadcast(self, targets_deg: Dict[str, float]) -> str:
        resolved_targets = self.resolve_joint_targets(targets_deg)
        motor_targets_deg: Dict[int, float] = {}
        for joint_name, angle_deg in resolved_targets.items():
            definition = self._get_joint_definition(joint_name)
            if definition.enabled:
                motor_targets_deg[definition.motor_id] = self._to_motor_angle(definition, angle_deg)
        if not motor_targets_deg:
            return ""
        return self.motor.set_motor_angles_broadcast(motor_targets_deg)

    def _coerce_hand_positions_to_joint_targets_deg(self, positions: Iterable[float]) -> Dict[str, float]:
        joint_names = self.get_hand_joint_order()
        if not joint_names:
            raise RuntimeError("当前配置里没有可用的 joint 映射")

        if isinstance(positions, dict):
            raise TypeError("positions 不能是 dict，请传入位置数组")

        try:
            position_values = [float(value) for value in positions]
        except TypeError as exc:
            raise TypeError("positions 必须是可迭代的位置数组") from exc
        except ValueError as exc:
            raise ValueError("positions 中的元素必须是数字") from exc

        if len(position_values) != len(joint_names):
            raise ValueError(
                f"positions 长度必须等于 {len(joint_names)}，顺序为 {joint_names}"
            )

        return self._raw_hand_positions_to_joint_targets_deg(dict(zip(joint_names, position_values)))

    def _raw_hand_positions_to_joint_targets_deg(self, raw_targets: Dict[str, float]) -> Dict[str, float]:
        targets_deg: Dict[str, float] = {}
        for joint_name, raw_value in raw_targets.items():
            if self.is_pip_dip_joint(joint_name):
                finger_name, _ = self._split_joint_name(joint_name)
                mcp_2_name = f"{finger_name}.mcp_2"
                if mcp_2_name not in raw_targets:
                    raise KeyError(f"未找到 {joint_name} 对应的 {mcp_2_name} 输入")
                targets_deg[joint_name] = self.pip_dip_tendon_length_to_angle_deg(
                    raw_targets[mcp_2_name],
                    raw_value,
                )
            else:
                targets_deg[joint_name] = math.degrees(raw_value)
        return targets_deg

    def _resolve_coupled_mcp1_targets(self, requested_targets_deg: Dict[str, float]) -> Dict[str, float]:
        coupled_targets = {
            joint_name: float(self._get_coupled_joint_baseline(joint_name, requested_targets_deg))
            for joint_name in COUPLED_MCP1_JOINTS
        }

        index_name, middle_name, ring_name, little_name = COUPLED_MCP1_JOINTS
        index_definition = self._get_joint_definition(index_name)
        middle_definition = self._get_joint_definition(middle_name)
        ring_definition = self._get_joint_definition(ring_name)
        little_definition = self._get_joint_definition(little_name)

        index_value = self._clamp_value(coupled_targets[index_name], index_definition.min_deg, index_definition.max_deg)
        little_value = self._clamp_value(coupled_targets[little_name], little_definition.min_deg, little_definition.max_deg)

        middle_lower = max(index_value, middle_definition.min_deg)
        middle_upper = min(little_value, middle_definition.max_deg)
        middle_value = self._clamp_value(coupled_targets[middle_name], middle_lower, middle_upper)

        ring_lower = max(middle_value, ring_definition.min_deg)
        ring_upper = min(little_value, ring_definition.max_deg)
        ring_value = self._clamp_value(coupled_targets[ring_name], ring_lower, ring_upper)

        if middle_value > ring_value:
            middle_value = ring_value
            middle_value = self._clamp_value(middle_value, middle_lower, min(little_value, middle_definition.max_deg))

        return {
            index_name: index_value,
            middle_name: middle_value,
            ring_name: ring_value,
            little_name: little_value,
        }

    def _get_coupled_joint_baseline(self, joint_name: str, requested_targets_deg: Dict[str, float]) -> float:
        if joint_name in requested_targets_deg:
            return float(requested_targets_deg[joint_name])
        if joint_name in self._last_joint_targets_deg:
            return float(self._last_joint_targets_deg[joint_name])
        definition = self._get_joint_definition(joint_name)
        return self._clamp_value(0.0, definition.min_deg, definition.max_deg)

    def _clamp_value(self, value: float, lower: float, upper: float) -> float:
        if lower > upper:
            return float(lower)
        return max(float(lower), min(float(upper), float(value)))

    def pip_dip_compensation_angle_deg(self, mcp_2_angle_rad: float) -> float:
        '''返回 mcp_2 补偿带来的 pip_dip 角度偏移，单位 deg。'''
        compensation_length_mm = (
            math.sqrt(587.75 - 378.98 * math.cos(2 - float(mcp_2_angle_rad)))
            - 27.23
        )
        return -math.degrees(compensation_length_mm / 5.0)

    def pip_dip_tendon_length_to_angle_deg(self, mcp_2_angle_rad: float, tendon_length_m: float) -> float:
        '''将 tendon length(m) 转换为补偿后的 pip_dip 关节角度(deg)。'''
        uncompensated_angle_deg = self._tendon_length_to_uncompensated_pip_dip_angle_deg(tendon_length_m)
        return uncompensated_angle_deg + self.pip_dip_compensation_angle_deg(mcp_2_angle_rad)

    def pip_dip_angle_to_tendon_length_m(self, mcp_2_angle_rad: float, pip_dip_angle_deg: float) -> float:
        '''从补偿后的 pip_dip 关节角度(deg)反算 tendon length(m)。'''
        uncompensated_angle_deg = (
            float(pip_dip_angle_deg)
            - self.pip_dip_compensation_angle_deg(mcp_2_angle_rad)
        )
        return self._uncompensated_pip_dip_angle_to_tendon_length_m(uncompensated_angle_deg)

    def _tendon_length_to_uncompensated_pip_dip_angle_deg(self, tendon_length_m: float) -> float:
        tendon_length_sim = tendon_length_m
        tendon_length_real = (tendon_length_sim - 0.0285) * (0.02233 / (0.0037 - 0.0285))
        pip_dip_angle_deg = math.degrees((1000.0 * float(tendon_length_real)) / 5.0) - 255.0
        return pip_dip_angle_deg

    def _uncompensated_pip_dip_angle_to_tendon_length_m(self, pip_dip_angle_deg: float) -> float:
        tendon_length_real = math.radians(float(pip_dip_angle_deg) + 255.0) * 5.0 / 1000.0
        tendon_length_sim = 0.0285 + tendon_length_real * (0.0037 - 0.0285) / 0.02233
        return tendon_length_sim

    def _conpensated_pip_dip(self, mcp_2_angle_rad: float, pip_dip_m: float) -> float:
        '''根据同一手指的 mcp_2 弧度值和 pip_dip 米制输入计算 pip_dip 目标角度。'''
        return self.pip_dip_tendon_length_to_angle_deg(mcp_2_angle_rad, pip_dip_m)

    def _compensated_pip_dip(self, mcp_2_angle_rad: float, pip_dip_m: float) -> float:
        '''_conpensated_pip_dip 的正确拼写兼容别名。'''
        return self.pip_dip_tendon_length_to_angle_deg(mcp_2_angle_rad, pip_dip_m)

    def set_motor_target(self, motor_id: int, angle_deg: float, speed: Optional[float] = None) -> str:
        '''设置单个电机的目标角度和速度。'''
        self._validate_motor_id(motor_id)
        if not self._is_motor_enabled(motor_id):
            return ""

        if speed is None:
            joint_name = self.motor_to_joint.get(int(motor_id))
            speed_ratio = 0.8 if joint_name and self.is_pip_dip_joint(joint_name) else 0.5
        else:
            speed_ratio = self._normalize_speed_value(speed)

        target_angle = self._to_known_motor_angle(motor_id, angle_deg)
        return self.motor.set_motor_angle(motor_id, math.radians(target_angle), speed=speed_ratio)

    def set_tendon_motor_speeds_broadcast(self, speed: Any = 100.0, default_speed: Any = 50.0) -> str:
        speed_percent = self._normalize_speed_value(speed) * 100.0
        default_speed_percent = self._normalize_speed_value(default_speed) * 100.0
        motor_speeds = {
            definition.motor_id: speed_percent if self.is_pip_dip_joint(joint_name) else default_speed_percent
            for joint_name, definition in self.joint_to_motor.items()
            if definition.enabled
        }
        if not motor_speeds:
            return ""
        return self.motor.set_motor_speeds_broadcast(motor_speeds)

    def jog_joint(self, joint_name: str, direction: int) -> str:
        '''按关节定义执行点动控制。'''
        definition = self._get_joint_definition(joint_name)
        if not definition.enabled:
            return ""
        if direction not in {0, 1, 2}:
            raise ValueError("direction 必须是 0、1 或 2")

        motor_direction = direction
        if definition.reverse and direction in {1, 2}:
            motor_direction = 1 if direction == 2 else 2
        return self.motor.jog_motor(definition.motor_id, motor_direction)

    def jog_motor(self, motor_id: int, direction: int) -> str:
        '''直接对指定电机执行点动控制。'''
        self._validate_motor_id(motor_id)
        if direction not in {0, 1, 2}:
            raise ValueError("direction 必须是 0、1 或 2")
        return self.motor.jog_motor(motor_id, direction)

    def set_zero(self, motor_id: int) -> Dict[str, str]:
        '''通过重使能触发指定电机零位设置。'''
        self._validate_motor_id(motor_id)
        command_ids = {
            "disable_command_id": self.enable_motor(motor_id, False),
        }
        time.sleep(1.0 / self.ctrl_frequency)
        command_ids["enable_command_id"] = self.enable_motor(motor_id, True)
        return command_ids

    def get_joint_positions(self) -> Dict[str, Optional[float]]:
        '''返回所有已启用关节的当前位置。'''
        return {
            joint_name: feedback["position_deg"]
            for joint_name, feedback in self.get_joint_feedback().items()
        }

    def start_tactile(
        self,
        port: Optional[Any] = None,
        baudrate: Optional[Any] = None,
        timeout_s: Optional[Any] = None,
        *,
        mode: Optional[Any] = None,
    ) -> bool:
        '''按配置模式启动触觉传感器；失败时返回 False 并记录异常。'''
        try:
            selected_mode = normalize_tactile_mode(
                self.tactile_config.mode if mode is None else mode
            )
        except Exception as exc:
            self._last_tactile_error = exc
            return False
        if self.tactile_connected and self._tactile_mode == selected_mode:
            return True
        if self._tactile_started:
            self.stop_tactile()

        try:
            if selected_mode == FSR402_MODE:
                config = self.tactile_config.fsr402
                sensor = create_tactile_sensor(
                    selected_mode,
                    port=config.port if port is None else self._parse_tactile_port(port),
                    baudrate=config.baudrate if baudrate is None else self._parse_baudrate(baudrate),
                    timeout_s=(
                        config.timeout_s
                        if timeout_s is None
                        else self._parse_timeout(timeout_s, "tactile.fsr402.timeout_s")
                    ),
                    channel_fingers=config.channel_fingers,
                )
            else:
                config = self.tactile_config.hwk
                sensor = create_tactile_sensor(
                    selected_mode,
                    port=config.port if port is None else self._parse_tactile_port(port),
                    baudrate=config.baudrate if baudrate is None else self._parse_baudrate(baudrate),
                    response_timeout_s=(
                        config.response_timeout_s
                        if timeout_s is None
                        else self._parse_timeout(timeout_s, "tactile.hwk.response_timeout_s")
                    ),
                    write_timeout_s=config.write_timeout_s,
                    poll_interval_s=config.poll_interval_s,
                    device_regions=dict(config.device_regions),
                )
            sensor.start()
        except Exception as exc:
            self._last_tactile_error = exc
            return False

        with self._tactile_lock:
            self._tactile_sensor = sensor
            self._tactile_started = True
            self._tactile_mode = selected_mode
            self._last_tactile_error = None
        return True

    def stop_tactile(self):
        '''停止当前模式的触觉传感器后台读取。'''
        with self._tactile_lock:
            sensor = self._tactile_sensor
            self._tactile_sensor = None
            self._tactile_started = False
            self._tactile_mode = None

        if sensor is not None:
            sensor.stop()

    def get_tactile_snapshot(self) -> Optional[TactileSnapshot]:
        '''返回统一触觉快照；未启动或尚无数据时返回 None。'''
        with self._tactile_lock:
            sensor = self._tactile_sensor
        if sensor is None:
            return None
        return sensor.read()

    def get_tactile_values(self) -> list[float]:
        '''返回五指代表值；两种触觉模式的单位均为 g。'''
        snapshot = self.get_tactile_snapshot()
        if snapshot is None:
            return [-1.0] * len(FINGERTIP_FORCE_ORDER)
        value_map = snapshot.value_map
        return [float(value_map.get(name, -1.0)) for name in FINGERTIP_FORCE_ORDER]

    def get_tactile_forces(self) -> list[float]:
        '''兼容旧 API；返回各指主压力矩阵的最大克重值。'''
        return self.get_tactile_values()

    def get_fingertip_forces(self) -> list[float]:
        '''返回五根手指指尖受力数组，等同于 get_tactile_forces。'''
        return self.get_tactile_forces()

    def get_tactile_force_map(self) -> Dict[str, float]:
        '''按手指名返回代表值；单位由 tactile_unit 指示。'''
        return dict(zip(FINGERTIP_FORCE_ORDER, self.get_tactile_forces()))

    def get_tactile_frame(self) -> Optional[Any]:
        '''兼容原生数据读取：FSR402 返回 SensorFrame，华威科返回设备读数字典。'''
        snapshot = self.get_tactile_snapshot()
        return None if snapshot is None else snapshot.raw

    def get_tactile_matrices(self) -> Dict[str, Any]:
        '''返回区域到矩阵元组的映射；适用于华威科空间压力数据。'''
        snapshot = self.get_tactile_snapshot()
        return {} if snapshot is None else dict(snapshot.matrices)

    def get_tactile_force_summaries(self) -> Dict[str, Dict[str, float]]:
        '''返回华威科各区域的 Fx、Fy、Fnormal_sum 汇总值，单位为 g。'''
        snapshot = self.get_tactile_snapshot()
        if snapshot is None:
            return {}
        return {
            region: {label: float(value) for label, value in summary.items()}
            for region, summary in snapshot.force_summaries.items()
        }

    def calibrate_tactile(self, device_id: int, command: int = 2):
        '''向当前华威科传感器发送归零命令。'''
        with self._tactile_lock:
            sensor = self._tactile_sensor
        if sensor is None or self.tactile_mode != HWK_MODE:
            raise RuntimeError("仅已启动的华威科模式支持归零校准")
        sensor.calibrate(device_id, command)

    def update_joint_definition(self, joint_name: str, config_values: Any):
        '''更新指定关节的映射与运动参数。'''
        if not isinstance(config_values, dict):
            raise TypeError("config_values 必须是 dict")

        new_config = copy.deepcopy(self.config)
        finger_config, local_joint_name = self._get_joint_config_entry(new_config, joint_name)
        current_config = finger_config.get(local_joint_name)
        joint_config = copy.deepcopy(current_config) if isinstance(current_config, dict) else {}

        range_deg = config_values.get("range_deg")
        if not isinstance(range_deg, (list, tuple)) or len(range_deg) != 2:
            raise ValueError("range_deg 必须是长度为2的列表")

        joint_config["id"] = int(config_values.get("motor_id"))
        joint_config["range"] = [float(range_deg[0]), float(range_deg[1])]
        joint_config["reverse"] = bool(config_values.get("reverse"))
        joint_config["enabled"] = bool(config_values.get("enabled"))

        finger_config[local_joint_name] = joint_config
        self._rebuild_from_config(new_config)

    def set_joint_enabled(self, joint_name: str, enabled: bool):
        '''修改指定关节的启用状态。'''
        new_config = copy.deepcopy(self.config)
        finger_config, local_joint_name = self._get_joint_config_entry(new_config, joint_name)
        joint_config = finger_config.get(local_joint_name)
        if not isinstance(joint_config, dict):
            raise KeyError(f"未找到关节映射: {joint_name}")
        joint_config["enabled"] = bool(enabled)
        self._rebuild_from_config(new_config)

    def update_connection_config(
        self,
        ctrl_frequency: Any,
        port: Any,
        baudrate: Any,
        timeout_s: Any,
        write_timeout_s: Any,
    ):
        '''更新控制频率和串口连接参数。'''
        new_config = copy.deepcopy(self.config)
        new_config["ctrl_frequency"] = self._parse_ctrl_frequency(ctrl_frequency)

        serial_config = new_config.get("serial")
        if not isinstance(serial_config, dict):
            serial_config = {}
            new_config["serial"] = serial_config

        serial_config["port"] = self._parse_port(port)
        serial_config["baudrate"] = self._parse_baudrate(baudrate)
        serial_config["timeout_s"] = self._parse_timeout(timeout_s, "timeout_s")
        serial_config["write_timeout_s"] = self._parse_timeout(write_timeout_s, "write_timeout_s")
        self._rebuild_from_config(new_config)

    def update_tactile_config(
        self,
        enabled: Any,
        port: Any,
        baudrate: Any,
        timeout_s: Any,
        channel_fingers: Optional[Any] = None,
        contact_threshold_g: Optional[float] = None,
        mode: Optional[Any] = None,
        device_regions: Optional[Any] = None,
        write_timeout_s: Optional[Any] = None,
        poll_interval_s: Optional[Any] = None,
    ):
        '''更新选中模式的触觉传感器连接参数。'''
        new_config = copy.deepcopy(self.config)
        tactile_config = new_config.get("tactile")
        if not isinstance(tactile_config, dict):
            tactile_config = {}
            new_config["tactile"] = tactile_config

        tactile_config["enabled"] = self._parse_bool(enabled, "tactile", "enabled")
        selected_mode = normalize_tactile_mode(
            tactile_config.get("mode", self.tactile_config.mode) if mode is None else mode
        )
        tactile_config["mode"] = selected_mode
        parsed_port = self._parse_tactile_port(port)

        # 先把解析后的两套配置补齐到新结构，避免迁移旧平铺配置时丢失
        # channel_fingers、阈值或当前未选中模式的参数。
        current_fsr = self.tactile_config.fsr402
        fsr_config = tactile_config.get("fsr402")
        if not isinstance(fsr_config, dict):
            fsr_config = {}
            tactile_config["fsr402"] = fsr_config
        fsr_defaults = {
            "port": "auto" if current_fsr.port is None else current_fsr.port,
            "baudrate": current_fsr.baudrate,
            "timeout_s": current_fsr.timeout_s,
            "channel_fingers": list(current_fsr.channel_fingers),
            "contact_threshold_g": current_fsr.contact_threshold_g,
        }
        for key, value in fsr_defaults.items():
            fsr_config.setdefault(key, value)

        current_hwk = self.tactile_config.hwk
        hwk_config = tactile_config.get("hwk")
        if not isinstance(hwk_config, dict):
            hwk_config = {}
            tactile_config["hwk"] = hwk_config
        hwk_defaults = {
            "port": "auto" if current_hwk.port is None else current_hwk.port,
            "baudrate": current_hwk.baudrate,
            "response_timeout_s": current_hwk.response_timeout_s,
            "write_timeout_s": current_hwk.write_timeout_s,
            "poll_interval_s": current_hwk.poll_interval_s,
            "device_regions": dict(current_hwk.device_regions),
            "contact_threshold_g": current_hwk.contact_threshold_g,
        }
        for key, value in hwk_defaults.items():
            hwk_config.setdefault(key, value)
        hwk_config.pop("contact_threshold_raw", None)

        if selected_mode == FSR402_MODE:
            sensor_config = fsr_config
            sensor_config["port"] = "auto" if parsed_port is None else parsed_port
            sensor_config["baudrate"] = self._parse_baudrate(baudrate)
            sensor_config["timeout_s"] = self._parse_timeout(
                timeout_s, "tactile.fsr402.timeout_s"
            )
            if channel_fingers is not None:
                sensor_config["channel_fingers"] = list(
                    self._parse_tactile_channel_fingers(channel_fingers)
                )
            if contact_threshold_g is not None:
                sensor_config["contact_threshold_g"] = float(contact_threshold_g)
        else:
            sensor_config = hwk_config
            sensor_config["port"] = "auto" if parsed_port is None else parsed_port
            sensor_config["baudrate"] = self._parse_baudrate(baudrate)
            sensor_config["response_timeout_s"] = self._parse_timeout(
                timeout_s, "tactile.hwk.response_timeout_s"
            )
            if write_timeout_s is not None:
                sensor_config["write_timeout_s"] = self._parse_timeout(
                    write_timeout_s, "tactile.hwk.write_timeout_s"
                )
            if poll_interval_s is not None:
                sensor_config["poll_interval_s"] = self._parse_timeout(
                    poll_interval_s, "tactile.hwk.poll_interval_s"
                )
            if device_regions is not None:
                sensor_config["device_regions"] = {
                    device_id: region
                    for device_id, region in self._parse_hwk_device_regions(device_regions)
                }
            if contact_threshold_g is not None:
                sensor_config["contact_threshold_g"] = float(contact_threshold_g)

        # 保存后使用新的分模式结构；读取时仍兼容这些旧字段。
        for legacy_key in (
            "port",
            "baudrate",
            "timeout_s",
            "channel_fingers",
            "contact_threshold_g",
        ):
            tactile_config.pop(legacy_key, None)

        self._rebuild_from_config(new_config)

    def save_config(self):
        '''将当前配置写回配置文件。'''
        try:
            import yaml
        except ImportError as exc:
            raise ImportError("保存 config_hand.yaml 需要先安装 PyYAML") from exc

        config_text = yaml.safe_dump(self.config, allow_unicode=True, sort_keys=False)
        self.config_path.write_text(config_text, encoding="utf-8")

    def reload_config(self):
        '''重新加载并应用配置文件。'''
        self._rebuild_from_config(self._load_config(self.config_path))

    def get_joint_feedback(self) -> Dict[str, Dict[str, Any]]:
        '''返回按关节名整理后的反馈信息。'''
        feedback: Dict[str, Dict[str, Any]] = {}
        with self._feedback_lock:
            for joint_name, definition in self.joint_to_motor.items():
                if not definition.enabled:
                    continue
                motor_feedback = self._get_feedback_entry(definition.motor_id)
                joint_feedback = copy.deepcopy(motor_feedback)
                joint_feedback["motor_id"] = definition.motor_id
                joint_feedback["position_deg"] = self._to_joint_angle(definition, motor_feedback["position_deg"])
                feedback[joint_name] = joint_feedback
        return feedback

    def get_motor_feedback(self, motor_id: int) -> Dict[str, Any]:
        '''返回指定电机的原始反馈信息。'''
        self._validate_motor_id(motor_id)
        with self._feedback_lock:
            feedback = copy.deepcopy(self._get_feedback_entry(motor_id))
        feedback["motor_id"] = motor_id
        return feedback

    def poll_once(self) -> Dict[int, Dict[str, Any]]:
        '''主动轮询一次所有启用电机的状态。'''
        if not self.motor.connected:
            raise RuntimeError("未连接到电机控制系统")

        now = time.monotonic()
        if now < self._poll_bus_pause_until:
            return {}

        ordered_motor_ids = list(self._poll_motor_ids())
        if not ordered_motor_ids:
            return {}

        start_index = self._poll_cursor % len(ordered_motor_ids)
        motor_ids = ordered_motor_ids[start_index:] + ordered_motor_ids[:start_index]
        self._poll_cursor = (start_index + 1) % len(ordered_motor_ids)

        updated_feedback: Dict[int, Dict[str, Any]] = {}
        timeout_s = max(self.motor.timeout_s, 1.0 / self.ctrl_frequency)

        batch_reader = getattr(self.motor, "get_motor_statuses_sync", None)
        eligible_motor_ids = [
            motor_id
            for motor_id in motor_ids
            if now >= self._poll_next_allowed_at.get(motor_id, 0.0)
        ]
        if callable(batch_reader) and eligible_motor_ids:
            statuses = batch_reader(eligible_motor_ids, timeout=timeout_s)
            for motor_id in eligible_motor_ids:
                feedback_update, _ = self._store_motor_status(
                    motor_id,
                    statuses[motor_id],
                )
                updated_feedback[motor_id] = feedback_update
            return copy.deepcopy(updated_feedback)

        for motor_id in motor_ids:
            now = time.monotonic()
            next_allowed = self._poll_next_allowed_at.get(motor_id, 0.0)
            if now < next_allowed:
                continue

            feedback_update, timed_out = self._poll_motor_status(
                motor_id,
                timeout_s=timeout_s,
            )
            updated_feedback[motor_id] = feedback_update
            if timed_out:
                break
        return copy.deepcopy(updated_feedback)

    def poll_next(self) -> Dict[int, Dict[str, Any]]:
        '''轮询一个启用电机，为后台状态读取提供固定总线预算。'''
        if not self.motor.connected:
            raise RuntimeError("未连接到电机控制系统")
        now = time.monotonic()
        if now < self._poll_bus_pause_until:
            return {}
        motor_ids = list(self._poll_motor_ids())
        if not motor_ids:
            return {}
        for offset in range(len(motor_ids)):
            index = (self._poll_cursor + offset) % len(motor_ids)
            motor_id = motor_ids[index]
            if now < self._poll_next_allowed_at.get(motor_id, 0.0):
                continue
            self._poll_cursor = (index + 1) % len(motor_ids)
            timeout_s = max(
                self.motor.timeout_s,
                1.0 / self.ctrl_frequency,
            )
            feedback, _ = self._poll_motor_status(
                motor_id,
                timeout_s=timeout_s,
            )
            return {motor_id: copy.deepcopy(feedback)}
        return {}

    def _poll_motor_status(
        self,
        motor_id: int,
        *,
        timeout_s: float,
    ) -> tuple[Dict[str, Any], bool]:
        status = self.motor.get_motor_status(
            motor_id,
            sync=True,
            timeout=timeout_s,
        )
        return self._store_motor_status(motor_id, status)

    def _store_motor_status(
        self,
        motor_id: int,
        status: Dict[str, Any],
    ) -> tuple[Dict[str, Any], bool]:
        timed_out = bool(status.get("timeout"))
        feedback_update = {
            "position_deg": status.get("angle"),
            "online": status.get("online"),
            "fsm_state": status.get("fsm_state"),
            "error_code": status.get("error_code"),
            "temp": status.get("temp"),
            "bus_voltage": status.get("bus_voltage"),
        }
        if timed_out and not status.get("cached", False):
            feedback_update.pop("position_deg", None)
        if not timed_out and feedback_update.get("position_deg") is not None:
            feedback_update["position_updated_at_monotonic_s"] = (
                status.get("received_at_monotonic_s", time.monotonic())
            )
        period = 1.0 / self.ctrl_frequency
        if timed_out:
            failure_count = self._poll_failure_counts.get(motor_id, 0) + 1
            self._poll_failure_counts[motor_id] = failure_count
            # Back off only this device; keep retries below the deployment
            # freshness budget, without pausing healthy motors on the bus.
            cooldown = min(
                max(0.05, period) * (2 ** min(failure_count - 1, 3)),
                0.25,
            )
            self._poll_next_allowed_at[motor_id] = time.monotonic() + cooldown
        else:
            self._poll_failure_counts.pop(motor_id, None)
            self._poll_next_allowed_at.pop(motor_id, None)
        with self._feedback_lock:
            self._get_feedback_entry(motor_id).update(feedback_update)
        trace = getattr(self.motor, "_trace_event", None)
        if trace is not None:
            trace("feedback_store", motor_id=motor_id, **feedback_update)
        return feedback_update, timed_out

    def start_polling(self):
        '''启动后台线程持续轮询状态。'''
        if self._poll_thread and self._poll_thread.is_alive():
            return
        if not self.motor.connected:
            raise RuntimeError("未连接到电机控制系统")

        self._poll_stop_event.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def stop_polling(self):
        '''停止后台状态轮询线程。'''
        self._poll_stop_event.set()
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=1.0)
        self._poll_thread = None

    def _poll_loop(self):
        '''按控制频率循环执行状态轮询。'''
        # Deployment consumes hand feedback at 20 Hz. A 40 Hz full-hand poll
        # leaves one fresh snapshot per half policy period without saturating
        # the shared serial bus used by outgoing angle broadcasts.
        period = 1.0 / min(self.ctrl_frequency, 40.0)
        while not self._poll_stop_event.is_set():
            started_at = time.monotonic()
            try:
                self.poll_once()
            except Exception as exc:
                self._poll_error_count += 1
                self._last_poll_error = f"{type(exc).__name__}: {exc}"
                now = time.monotonic()
                if now - self._last_poll_warning_at >= 1.0:
                    self._last_poll_warning_at = now
                    logging.getLogger(__name__).warning(
                        "Hand status polling failed (%d errors): %s",
                        self._poll_error_count,
                        self._last_poll_error,
                    )
            elapsed = time.monotonic() - started_at
            pause_until = self._poll_bus_pause_until
            extra_wait = max(0.0, pause_until - time.monotonic()) if pause_until > 0.0 else 0.0
            self._poll_stop_event.wait(max(0.0, period - elapsed, extra_wait))

    def _register_callbacks(self):
        '''注册角度、速度和电流广播回调。'''
        if self._callbacks_registered:
            return

        self.motor.add_response_callback(motor.ResponseType.ANGLE_BROADCAST, self._handle_angle_broadcast)
        self.motor.add_response_callback(motor.ResponseType.SPEED_BROADCAST, self._handle_speed_broadcast)
        self.motor.add_response_callback(motor.ResponseType.CURRENT_BROADCAST, self._handle_current_broadcast)
        self._callbacks_registered = True

    def _unregister_callbacks(self):
        '''注销已注册的电机广播回调。'''
        if not self._callbacks_registered:
            return

        self.motor.remove_response_callback(motor.ResponseType.ANGLE_BROADCAST, self._handle_angle_broadcast)
        self.motor.remove_response_callback(motor.ResponseType.SPEED_BROADCAST, self._handle_speed_broadcast)
        self.motor.remove_response_callback(motor.ResponseType.CURRENT_BROADCAST, self._handle_current_broadcast)
        self._callbacks_registered = False

    def _handle_angle_broadcast(self, response: Any):
        '''处理角度广播并更新缓存。'''
        angles = getattr(response, "data", {}).get("angles", {})
        updated_at = time.monotonic()
        with self._feedback_lock:
            for motor_id, angle_deg in angles.items():
                joint_name = self.motor_to_joint.get(motor_id)
                if joint_name and self.joint_to_motor[joint_name].enabled:
                    feedback = self._get_feedback_entry(motor_id)
                    feedback["position_deg"] = angle_deg
                    feedback["position_updated_at_monotonic_s"] = updated_at

    def _handle_speed_broadcast(self, response: Any):
        '''处理速度广播并更新缓存。'''
        speeds = getattr(response, "data", {}).get("speeds", {})
        with self._feedback_lock:
            for motor_id, speed_pct in speeds.items():
                joint_name = self.motor_to_joint.get(motor_id)
                if joint_name and self.joint_to_motor[joint_name].enabled:
                    self._get_feedback_entry(motor_id)["speed_pct"] = speed_pct

    def _handle_current_broadcast(self, response: Any):
        '''处理电流广播并更新缓存。'''
        currents = getattr(response, "data", {}).get("currents", {})
        with self._feedback_lock:
            for motor_id, current_m_a in currents.items():
                joint_name = self.motor_to_joint.get(motor_id)
                if joint_name and self.joint_to_motor[joint_name].enabled:
                    self._get_feedback_entry(motor_id)["current_mA"] = current_m_a

    def _get_feedback_entry(self, motor_id: int) -> Dict[str, Any]:
        '''获取指定电机的反馈缓存条目。'''
        if motor_id not in self._motor_feedback:
            self._motor_feedback[motor_id] = copy.deepcopy(DEFAULT_FEEDBACK)
        return self._motor_feedback[motor_id]

    def _poll_motor_ids(self) -> Iterable[int]:
        '''返回当前需要轮询的电机 ID 列表。'''
        if self.motor_to_joint:
            return sorted(
                motor_id
                for motor_id, joint_name in self.motor_to_joint.items()
                if self.joint_to_motor[joint_name].enabled
            )
        return ()

    def _build_joint_speed_ratios(self, targets_deg: Dict[str, float], speed: Optional[Any]) -> Dict[str, Optional[float]]:
        '''将速度参数展开为按关节索引的速度比例。'''
        if speed is None:
            return {joint_name: None for joint_name in targets_deg}

        if isinstance(speed, dict):
            speed_ratios: Dict[str, Optional[float]] = {}
            for joint_name in targets_deg:
                if joint_name not in speed:
                    raise ValueError(f"缺少关节 {joint_name} 的速度配置")
                speed_ratios[joint_name] = self._normalize_speed_value(speed[joint_name])
            return speed_ratios

        speed_ratio = self._normalize_speed_value(speed)
        return {
            joint_name: speed_ratio
            for joint_name in targets_deg
        }

    def _resolve_joint_motor_ids(self, joints: Iterable[str], include_disabled: bool = False) -> list[int]:
        '''将关节名集合解析为电机 ID 列表。'''
        joint_names = [joints] if isinstance(joints, str) else list(joints)
        motor_ids = []
        for joint_name in joint_names:
            definition = self._get_joint_definition(joint_name)
            if not include_disabled and not definition.enabled:
                continue
            motor_ids.append(definition.motor_id)
        return motor_ids

    def _get_joint_definition(self, joint_name: str) -> JointDefinition:
        '''获取指定关节的定义信息。'''
        try:
            return self.joint_to_motor[joint_name]
        except KeyError as exc:
            if not self.joint_to_motor:
                raise RuntimeError("当前配置里没有可用的 joint 映射") from exc
            raise KeyError(f"未找到关节映射: {joint_name}") from exc

    def _get_joint_config_entry(self, config: Dict[str, Any], joint_name: str) -> tuple[Dict[str, Any], str]:
        '''定位配置中指定关节的配置节点。'''
        finger_name, local_joint_name = self._split_joint_name(joint_name)
        finger_config = config.get(finger_name)
        if not isinstance(finger_config, dict):
            raise KeyError(f"未找到关节映射: {joint_name}")
        if local_joint_name not in finger_config:
            raise KeyError(f"未找到关节映射: {joint_name}")
        return finger_config, local_joint_name

    def _active_joint_names(self) -> list[str]:
        '''返回所有已启用关节名称。'''
        return [joint_name for joint_name, definition in self.joint_to_motor.items() if definition.enabled]

    def _is_motor_enabled(self, motor_id: int) -> bool:
        '''判断电机是否对应启用中的关节。'''
        joint_name = self.motor_to_joint.get(int(motor_id))
        if not joint_name:
            return True
        return self.joint_to_motor[joint_name].enabled

    def _to_motor_angle(self, definition: JointDefinition, angle_deg: float) -> float:
        '''将关节角度转换为限幅后的电机角度。'''
        clamped_angle = max(definition.min_deg, min(definition.max_deg, float(angle_deg)))
        return -clamped_angle if definition.reverse else clamped_angle

    def _to_known_motor_angle(self, motor_id: int, angle_deg: float) -> float:
        '''按映射关系将关节角度转换为电机角度。'''
        joint_name = self.motor_to_joint.get(motor_id)
        if not joint_name:
            return float(angle_deg)
        return self._to_motor_angle(self.joint_to_motor[joint_name], angle_deg)

    def _to_joint_angle(self, definition: JointDefinition, angle_deg: Optional[float]) -> Optional[float]:
        '''将电机反馈角度转换回关节角度。'''
        if angle_deg is None:
            return None
        return -angle_deg if definition.reverse else angle_deg

    def _normalize_speed_value(self, speed: Any) -> float:
        '''将速度值规范化为 0 到 1 的比例。'''
        value = float(speed)
        if 0.0 <= value <= 1.0:
            return value
        if 0.0 <= value <= 100.0:
            return value / 100.0
        raise ValueError("speed 必须位于 [0, 1] 或 [0, 100]")

    def _validate_motor_id(self, motor_id: int):
        '''校验电机 ID 是否在允许范围内。'''
        upper_bound = 16
        if not 1 <= int(motor_id) <= upper_bound:
            raise ValueError(f"无效的电机ID: {motor_id}")

    def _load_config(self, config_path: Path) -> Dict[str, Any]:
        '''读取并解析配置文件。'''
        if not config_path.exists():
            raise FileNotFoundError(f"配置文件不存在: {config_path}")

        try:
            import yaml
        except ImportError as exc:
            raise ImportError("读取 config_hand.yaml 需要先安装 PyYAML") from exc

        config_text = self._normalize_yaml_text(config_path.read_text(encoding="utf-8"))
        config = yaml.safe_load(config_text) or {}
        if not isinstance(config, dict):
            raise ValueError("配置文件顶层必须是字典")
        return config

    def _normalize_yaml_text(self, config_text: str) -> str:
        '''修正 YAML 中缺失空格的键值写法。'''
        normalized_lines = []
        for line in config_text.splitlines():
            match = re.match(r"^(\s*[^#\s][^:]*):(\S.*)$", line)
            if match and not line.rstrip().endswith(":"):
                normalized_lines.append(f"{match.group(1)}: {match.group(2)}")
            else:
                normalized_lines.append(line)
        return "\n".join(normalized_lines)

    def _rebuild_from_config(self, config: Dict[str, Any]):
        '''根据配置重建映射并同步运行参数。'''
        joint_to_motor = self._build_joint_map(config)
        ctrl_frequency = self._parse_ctrl_frequency(config.get("ctrl_frequency", 50))
        port, baudrate, timeout_s, write_timeout_s = self._extract_connection_config(config)
        tactile_config = self._extract_tactile_config(config)

        self.config = config
        self.ctrl_frequency = ctrl_frequency
        self.tactile_config = tactile_config
        self.motor.port = port
        self.motor.baudrate = baudrate
        self.motor.timeout_s = timeout_s
        self.motor.write_timeout_s = write_timeout_s
        self.joint_to_motor = joint_to_motor
        self.motor_to_joint = {definition.motor_id: name for name, definition in joint_to_motor.items()}
        self._last_joint_targets_deg = {
            joint_name: self._clamp_value(
                self._last_joint_targets_deg.get(joint_name, 0.0),
                definition.min_deg,
                definition.max_deg,
            )
            for joint_name, definition in joint_to_motor.items()
        }

    def _split_joint_name(self, joint_name: str) -> tuple[str, str]:
        '''拆分完整关节名为手指名和局部名。'''
        parts = joint_name.split(".", 1)
        if len(parts) != 2:
            raise KeyError(f"未找到关节映射: {joint_name}")
        return parts[0], parts[1]

    def _build_joint_map(self, config: Dict[str, Any]) -> Dict[str, JointDefinition]:
        '''从配置构建关节到电机的映射表。'''
        joint_map: Dict[str, JointDefinition] = {}
        seen_motor_ids = set()

        for finger_name, finger_config in config.items():
            if finger_name in CONFIG_KEYS or not isinstance(finger_config, dict):
                continue

            for joint_name, joint_config in finger_config.items():
                if not isinstance(joint_config, dict):
                    continue

                full_name = f"{finger_name}.{joint_name}"
                if self._is_empty_joint_config(joint_config):
                    continue

                motor_id = joint_config.get("id")
                angle_range = joint_config.get("range")
                reverse = joint_config.get("reverse")
                if motor_id in (None, "") or angle_range in (None, "") or reverse in (None, ""):
                    raise ValueError(f"关节 {full_name} 的 id/range/reverse 必须同时配置")

                parsed_motor_id = int(motor_id)
                self._validate_motor_id(parsed_motor_id)
                if parsed_motor_id in seen_motor_ids:
                    raise ValueError(f"电机ID重复: {parsed_motor_id}")
                seen_motor_ids.add(parsed_motor_id)

                if "enabled" not in joint_config:
                    joint_config["enabled"] = True

                min_deg, max_deg = self._parse_angle_range(full_name, angle_range)
                joint_map[full_name] = JointDefinition(
                    motor_id=parsed_motor_id,
                    min_deg=min_deg,
                    max_deg=max_deg,
                    reverse=self._parse_bool(reverse, full_name, "reverse"),
                    enabled=self._parse_bool(joint_config.get("enabled"), full_name, "enabled"),
                )

        return joint_map

    def _is_empty_joint_config(self, joint_config: Dict[str, Any]) -> bool:
        '''判断关节配置是否为空占位项。'''
        values = [joint_config.get("id"), joint_config.get("range"), joint_config.get("reverse")]
        return all(value in (None, "", []) for value in values)

    def _parse_angle_range(self, joint_name: str, angle_range: Any) -> tuple[float, float]:
        '''解析并校验关节角度范围。'''
        if not isinstance(angle_range, (list, tuple)) or len(angle_range) != 2:
            raise ValueError(f"关节 {joint_name} 的 range 必须是长度为2的列表")

        min_deg = float(angle_range[0])
        max_deg = float(angle_range[1])
        if min_deg > max_deg:
            raise ValueError(f"关节 {joint_name} 的 range 顺序无效")
        return min_deg, max_deg

    def _parse_bool(self, value: Any, joint_name: str, field_name: str = "reverse") -> bool:
        '''将配置值解析为布尔值。'''
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"true", "1", "yes", "on"}:
                return True
            if normalized in {"false", "0", "no", "off"}:
                return False
        if isinstance(value, int) and value in {0, 1}:
            return bool(value)
        raise ValueError(f"关节 {joint_name} 的 {field_name} 必须是布尔值")

    def _extract_connection_config(self, config: Dict[str, Any]) -> tuple[str, int, float, float]:
        '''提取并校验串口连接配置。'''
        serial_config = config.get("serial", {})
        if serial_config is None:
            serial_config = {}
        if not isinstance(serial_config, dict):
            raise ValueError("serial 配置必须是字典")

        port = self._parse_port(serial_config.get("port", "auto"))
        baudrate = self._parse_baudrate(serial_config.get("baudrate", config.get("baudrate", 230400)))
        timeout_s = self._parse_timeout(serial_config.get("timeout_s", 0.05), "timeout_s")
        write_timeout_s = self._parse_timeout(serial_config.get("write_timeout_s", 0.05), "write_timeout_s")
        return port, baudrate, timeout_s, write_timeout_s

    def _parse_port(self, port: Any) -> str:
        '''解析并校验串口名。'''
        value = str(port).strip()
        if not value:
            raise ValueError("serial.port 不能为空")
        return value

    def _parse_baudrate(self, baudrate: Any) -> int:
        '''解析并校验波特率。'''
        value = int(baudrate)
        if value <= 0:
            raise ValueError("baudrate 必须大于 0")
        return value

    def _parse_timeout(self, value: Any, field_name: str) -> float:
        '''解析并校验超时时间。'''
        timeout_s = float(value)
        if timeout_s < 0:
            raise ValueError(f"{field_name} 不能小于 0")
        return timeout_s

    def _parse_ctrl_frequency(self, ctrl_frequency: Any) -> float:
        '''解析并校验控制频率。'''
        value = float(ctrl_frequency)
        if value <= 0:
            raise ValueError("ctrl_frequency 必须大于 0")
        return value

    def _extract_tactile_config(self, config: Dict[str, Any]) -> TactileDefinition:
        '''提取分模式配置，并兼容旧版平铺的 FSR402 字段。'''
        tactile_config = config.get("tactile", {})
        if tactile_config is None:
            tactile_config = {}
        if not isinstance(tactile_config, dict):
            raise ValueError("tactile 配置必须是字典")

        enabled = self._parse_bool(tactile_config.get("enabled", False), "tactile", "enabled")
        mode = normalize_tactile_mode(tactile_config.get("mode", FSR402_MODE))

        fsr_config = tactile_config.get("fsr402", {})
        if fsr_config is None:
            fsr_config = {}
        if not isinstance(fsr_config, dict):
            raise ValueError("tactile.fsr402 配置必须是字典")
        fsr402 = FSR402TactileDefinition(
            port=self._parse_tactile_port(
                fsr_config.get("port", tactile_config.get("port"))
            ),
            baudrate=self._parse_baudrate(
                fsr_config.get("baudrate", tactile_config.get("baudrate", 115200))
            ),
            timeout_s=self._parse_timeout(
                fsr_config.get("timeout_s", tactile_config.get("timeout_s", 1.0)),
                "tactile.fsr402.timeout_s",
            ),
            channel_fingers=self._parse_tactile_channel_fingers(
                fsr_config.get(
                    "channel_fingers",
                    tactile_config.get("channel_fingers", FINGERTIP_FORCE_ORDER),
                )
            ),
            contact_threshold_g=self._parse_tactile_threshold(
                fsr_config.get(
                    "contact_threshold_g",
                    tactile_config.get("contact_threshold_g", 10.0),
                ),
                "tactile.fsr402.contact_threshold_g",
            ),
        )

        hwk_config = tactile_config.get("hwk", {})
        if hwk_config is None:
            hwk_config = {}
        if not isinstance(hwk_config, dict):
            raise ValueError("tactile.hwk 配置必须是字典")
        legacy_hwk_port = tactile_config.get("port") if mode == HWK_MODE else None
        legacy_hwk_baudrate = tactile_config.get("baudrate", 921600) if mode == HWK_MODE else 921600
        legacy_hwk_timeout = tactile_config.get("timeout_s", 0.2) if mode == HWK_MODE else 0.2
        response_timeout_s = self._parse_timeout(
            hwk_config.get("response_timeout_s", legacy_hwk_timeout),
            "tactile.hwk.response_timeout_s",
        )
        if response_timeout_s <= 0:
            raise ValueError("tactile.hwk.response_timeout_s 必须大于 0")
        hwk = HWKTactileDefinition(
            port=self._parse_tactile_port(hwk_config.get("port", legacy_hwk_port)),
            baudrate=self._parse_baudrate(
                hwk_config.get("baudrate", legacy_hwk_baudrate)
            ),
            response_timeout_s=response_timeout_s,
            write_timeout_s=self._parse_timeout(
                hwk_config.get("write_timeout_s", 0.2),
                "tactile.hwk.write_timeout_s",
            ),
            poll_interval_s=self._parse_timeout(
                hwk_config.get("poll_interval_s", 0.0),
                "tactile.hwk.poll_interval_s",
            ),
            device_regions=self._parse_hwk_device_regions(
                hwk_config.get(
                    "device_regions",
                    {
                        1: "thumb",
                        2: "index",
                        3: "middle",
                        4: "ring",
                        5: "little",
                        6: "palm",
                    },
                )
            ),
            contact_threshold_g=self._parse_tactile_threshold(
                hwk_config.get(
                    "contact_threshold_g",
                    hwk_config.get("contact_threshold_raw", 1.0),
                ),
                "tactile.hwk.contact_threshold_g",
            ),
        )
        return TactileDefinition(
            enabled=enabled,
            mode=mode,
            fsr402=fsr402,
            hwk=hwk,
        )

    def _parse_tactile_port(self, port: Any) -> Optional[str]:
        '''解析触觉传感器串口名；None/auto 表示自动选择。'''
        if port is None:
            return None
        value = str(port).strip()
        if not value or value.lower() == "auto":
            return None
        return value

    def _parse_tactile_channel_fingers(self, channel_fingers: Any) -> tuple[str, ...]:
        '''解析 PA0~PA4 到五指的映射。'''
        try:
            fingers = tuple(str(value).strip() for value in channel_fingers)
        except TypeError as exc:
            raise TypeError("tactile.channel_fingers 必须是长度为 5 的数组") from exc

        if len(fingers) != len(FINGERTIP_FORCE_ORDER):
            raise ValueError("tactile.channel_fingers 长度必须为 5")
        if set(fingers) != set(FINGERTIP_FORCE_ORDER):
            raise ValueError(f"tactile.channel_fingers 必须包含 {list(FINGERTIP_FORCE_ORDER)}")
        return fingers

    def _parse_hwk_device_regions(self, device_regions: Any) -> tuple[tuple[int, str], ...]:
        '''解析华威科设备 ID 到手指/掌心区域的映射。'''
        try:
            items = device_regions.items() if isinstance(device_regions, dict) else device_regions
            parsed = tuple((int(device_id), str(region).strip()) for device_id, region in items)
        except (TypeError, ValueError) as exc:
            raise TypeError("tactile.hwk.device_regions 必须是 ID 到区域名的映射") from exc
        if not parsed:
            raise ValueError("tactile.hwk.device_regions 不能为空")
        device_ids = [device_id for device_id, _ in parsed]
        region_names = [region for _, region in parsed]
        if any(not 1 <= device_id <= 15 for device_id in device_ids):
            raise ValueError("华威科设备 ID 必须在 1..15 之间")
        if len(set(device_ids)) != len(device_ids):
            raise ValueError("华威科设备 ID 不能重复")
        if any(not region for region in region_names):
            raise ValueError("华威科区域名不能为空")
        if len(set(region_names)) != len(region_names):
            raise ValueError("华威科区域名不能重复")
        return parsed

    def _parse_tactile_threshold(self, value: Any, field_name: str) -> float:
        threshold = float(value)
        if threshold < 0:
            raise ValueError(f"{field_name} 不能小于 0")
        return threshold


__all__ = [
    "FSR402TactileDefinition",
    "HWKTactileDefinition",
    "JointDefinition",
    "ParaHand",
    "TactileDefinition",
]
