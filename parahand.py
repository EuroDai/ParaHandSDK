from __future__ import annotations

import copy
import math
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import motor


CONFIG_KEYS = {"ctrl_frequency", "serial", "baudrate"}
COUPLED_MCP1_JOINTS = (
    "index.mcp_1",
    "middle.mcp_1",
    "ring.mcp_1",
    "little.mcp_1",
)
DEFAULT_CONFIG_PATH = Path(__file__).with_name("config_hand.yaml")
DEFAULT_FEEDBACK = {
    "position_deg": None,
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
        self._callbacks_registered = False
        self.ctrl_frequency = self._parse_ctrl_frequency(self.config.get("ctrl_frequency", 50))
        self.joint_to_motor: Dict[str, JointDefinition] = {}
        self.motor_to_joint: Dict[int, str] = {}
        self._last_joint_targets_deg: Dict[str, float] = {}
        self._rebuild_from_config(self.config)

    @property
    def connected(self) -> bool:
        '''返回当前电机连接状态。'''
        return self.motor.connected

    def connect(self) -> bool:
        '''连接电机并注册反馈回调。'''
        if self.motor.connected:
            self._register_callbacks()
            return True

        connected = self.motor.connect()
        if connected:
            self._register_callbacks()
        return connected

    def disconnect(self):
        '''停止轮询并断开电机连接。'''
        self.stop_polling()
        self._unregister_callbacks()
        self.motor.disconnect()

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
                targets_deg[joint_name] = self._conpensated_pip_dip(raw_targets[mcp_2_name], raw_value)
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

    def _conpensated_pip_dip(self, mcp_2_angle_rad: float, pip_dip_m: float) -> float:
        '''根据同一手指的 mcp_2 弧度值和 pip_dip 米制输入计算 pip_dip 目标角度。'''
        return 180* (1000 * pip_dip_m - math.sqrt(587.75 - 378.98 * math.cos(2 - mcp_2_angle_rad)) + 27.23) / (5 * math.pi) - 255

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

        updated_feedback: Dict[int, Dict[str, Any]] = {}
        for motor_id in self._poll_motor_ids():
            status = self.motor.get_motor_status(motor_id, sync=True, timeout=1.0 / self.ctrl_frequency)
            updated_feedback[motor_id] = {
                "position_deg": status.get("angle"),
                "online": status.get("online"),
                "fsm_state": status.get("fsm_state"),
                "error_code": status.get("error_code"),
                "temp": status.get("temp"),
                "bus_voltage": status.get("bus_voltage"),
            }

        with self._feedback_lock:
            for motor_id, values in updated_feedback.items():
                self._get_feedback_entry(motor_id).update(values)
            return copy.deepcopy(updated_feedback)

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
        period = 1.0 / self.ctrl_frequency
        while not self._poll_stop_event.is_set():
            started_at = time.time()
            try:
                self.poll_once()
            except Exception:
                pass
            elapsed = time.time() - started_at
            self._poll_stop_event.wait(max(0.0, period - elapsed))

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
        with self._feedback_lock:
            for motor_id, angle_deg in angles.items():
                joint_name = self.motor_to_joint.get(motor_id)
                if joint_name and self.joint_to_motor[joint_name].enabled:
                    self._get_feedback_entry(motor_id)["position_deg"] = angle_deg

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

        self.config = config
        self.ctrl_frequency = ctrl_frequency
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


__all__ = ["ParaHand", "JointDefinition"]
