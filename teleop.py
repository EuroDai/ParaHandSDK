from __future__ import annotations

import argparse
import importlib.util
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parent
ANYDEX_ROOT = ROOT.parent / "teleop" / "AnyDexRetarget"
ANYDEX_EXAMPLE_ROOT = ANYDEX_ROOT / "example"
if ANYDEX_ROOT.exists() and str(ANYDEX_ROOT) not in sys.path:
    sys.path.insert(0, str(ANYDEX_ROOT))
if ANYDEX_EXAMPLE_ROOT.exists() and str(ANYDEX_EXAMPLE_ROOT) not in sys.path:
    sys.path.insert(0, str(ANYDEX_EXAMPLE_ROOT))

from anydexretarget import Retargeter
from input.visionpro import VisionPro
from parahand import ParaHand


CALIBRATE_SCALING_PATH = ANYDEX_EXAMPLE_ROOT / "test" / "calibrate_scaling.py"
NON_THUMB_FINGERS = ("index", "middle", "ring", "little")
TENDON_TO_PIP_DIP_OFFSET_M = 0.025


def load_calibrate_scaling_helpers():
    spec = importlib.util.spec_from_file_location("anydex_calibrate_scaling", CALIBRATE_SCALING_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load calibration helper: {CALIBRATE_SCALING_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.FINGER_NAMES, module.collect_human_distances, module.get_robot_distances


FINGER_NAMES, collect_human_distances, get_robot_distances = load_calibrate_scaling_helpers()


class LoopMetrics:
    def __init__(self, interval_s: float):
        self.interval_s = interval_s
        self.lock = threading.Lock()
        self.reset(time.perf_counter())

    def reset(self, now: float):
        self.window_start = now
        self.loops = 0
        self.valid_inputs = 0
        self.changed_inputs = 0
        self.applied_controls = 0
        self.last_pose = None
        self.time_sums = {
            "input": 0.0,
            "retarget": 0.0,
            "apply": 0.0,
            "mj_step": 0.0,
            "viewer_sync": 0.0,
            "update_sim": 0.0,
            "real_hand": 0.0,
            "sleep": 0.0,
            "loop": 0.0,
        }
        self.time_counts = {key: 0 for key in self.time_sums}

    def add_time(self, key: str, value_s: float):
        with self.lock:
            self.time_sums[key] += value_s
            self.time_counts[key] += 1

    def add_loop(self):
        with self.lock:
            self.loops += 1

    def add_control(self):
        with self.lock:
            self.applied_controls += 1

    def record_input(self, pose: np.ndarray | None):
        if pose is None:
            return
        with self.lock:
            self.valid_inputs += 1
            if self.last_pose is None or not np.allclose(pose, self.last_pose, atol=1e-6, rtol=0.0):
                self.changed_inputs += 1
                self.last_pose = pose.copy()

    def maybe_print(self, now: float):
        with self.lock:
            elapsed = now - self.window_start
            if elapsed < self.interval_s:
                return

            loops = self.loops
            valid_inputs = self.valid_inputs
            changed_inputs = self.changed_inputs
            applied_controls = self.applied_controls
            time_sums = self.time_sums.copy()
            time_counts = self.time_counts.copy()
            self.reset(now)

        def hz(count: int) -> float:
            return count / elapsed if elapsed > 0.0 else 0.0

        def avg_ms(key: str) -> float:
            return 1000.0 * time_sums[key] / max(1, time_counts[key])

        print(
            "[metrics] "
            f"sim={hz(loops):5.1f}Hz "
            f"valid_input={hz(valid_inputs):5.1f}Hz "
            f"changed_input~={hz(changed_inputs):5.1f}Hz "
            f"control={hz(applied_controls):5.1f}Hz | "
            f"avg_ms input={avg_ms('input'):.2f} "
            f"retarget={avg_ms('retarget'):.2f} "
            f"apply={avg_ms('apply'):.2f} "
            f"mj_step={avg_ms('mj_step'):.2f} "
            f"sync={avg_ms('viewer_sync'):.2f} "
            f"update_sim={avg_ms('update_sim'):.2f} "
            f"real_hand={avg_ms('real_hand'):.2f} "
            f"sleep={avg_ms('sleep'):.2f} "
            f"loop={avg_ms('loop'):.2f}",
            flush=True,
        )


def resolve_path(path_str: str, base_dir: Path) -> Path:
    path = Path(path_str).expanduser()
    if not path.is_absolute():
        path = base_dir / path
    return path.resolve()


def load_retarget_config(config_path: Path, urdf_override: str | None = None) -> dict:
    with config_path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    if not isinstance(config, dict):
        raise ValueError(f"Retarget config must be a YAML mapping: {config_path}")

    robot_config = config.setdefault("robot", {})
    if urdf_override:
        robot_config["urdf_path"] = str(resolve_path(urdf_override, ROOT))
    elif "urdf_path" in robot_config:
        robot_config["urdf_path"] = str(resolve_path(robot_config["urdf_path"], config_path.parent))
    else:
        raise ValueError(f"Missing robot.urdf_path in retarget config: {config_path}")

    return config


def get_latest_mediapipe(input_device: VisionPro, hand_side: str) -> np.ndarray | None:
    try:
        fingers_data = input_device.get_fingers_data()
    except (AttributeError, KeyError, TypeError):
        return None

    fingers_pose = np.asarray(fingers_data.get(f"{hand_side}_fingers"))
    if fingers_pose.shape != (21, 3) or np.allclose(fingers_pose, 0.0):
        return None
    return fingers_pose


def qpos_by_name(retargeter: Retargeter, qpos: np.ndarray) -> dict[str, float]:
    names = retargeter.optimizer.robot.dof_joint_names
    return {name: float(qpos[i]) for i, name in enumerate(names)}


def wait_for_stable_hand(
    input_device: VisionPro,
    hand_side: str,
    stable_duration_s: float,
    threshold_m: float,
    timeout_s: float,
    min_wait_s: float,
) -> bool:
    print(
        f"Hold your {hand_side} hand open and still. "
        f"Waiting for {stable_duration_s:.1f}s stable data..."
    )
    start = time.perf_counter()
    samples: list[tuple[float, np.ndarray]] = []
    last_print = 0.0

    while time.perf_counter() - start < timeout_s:
        now = time.perf_counter()
        pose = get_latest_mediapipe(input_device, hand_side)
        if pose is None:
            time.sleep(0.01)
            continue

        wrist_relative = (pose - pose[0]).reshape(-1)
        samples.append((now, wrist_relative))
        samples = [(t, v) for t, v in samples if now - t <= stable_duration_s]

        if len(samples) >= 5:
            values = np.vstack([v for _, v in samples])
            max_std_m = float(np.max(np.std(values, axis=0)))
            if now - last_print >= 1.0:
                last_print = now
                print(f"  stability max std: {max_std_m * 100:.2f} cm")
            if now - start < min_wait_s:
                continue
            if now - samples[0][0] >= stable_duration_s and max_std_m <= threshold_m:
                print("  hand is stable, start averaging segment lengths")
                return True

        time.sleep(0.01)

    print("  stable-hand wait timed out; continuing with averaging anyway")
    return False


def ratio(robot_d: float | None, human_d: float | None) -> float:
    if robot_d and human_d and human_d > 1e-4:
        return round(robot_d / human_d, 4)
    return 1.0


def auto_segment_scaling(
    input_device: VisionPro,
    retargeter: Retargeter,
    hand_side: str,
    duration_s: float,
    stable_duration_s: float,
    stable_threshold_m: float,
    stable_timeout_s: float,
    stable_min_wait_s: float,
) -> dict[str, list[float]] | None:
    wait_for_stable_hand(
        input_device=input_device,
        hand_side=hand_side,
        stable_duration_s=stable_duration_s,
        threshold_m=stable_threshold_m,
        timeout_s=stable_timeout_s,
        min_wait_s=stable_min_wait_s,
    )

    print(f"Collecting hand samples for {duration_s:.1f}s...")
    (pip_robot, dip_robot, tip_robot), _ = get_robot_distances(retargeter.optimizer)
    frames, cumulative, _ = collect_human_distances(
        input_device=input_device,
        retargeter=retargeter,
        hand=hand_side,
        duration=duration_s,
    )
    if frames == 0 or cumulative is None:
        print("No valid AVP hand frames for auto segment scaling; using YAML defaults")
        return None

    pip_human, dip_human, tip_human = cumulative
    finger_names = [FINGER_NAMES[i] for i in retargeter.optimizer.mp_finger_indices]
    segment_scaling = {}
    for i, finger_name in enumerate(finger_names):
        segment_scaling[finger_name] = [
            ratio(pip_robot[i], pip_human[i]),
            ratio(dip_robot[i], dip_human[i]),
            ratio(tip_robot[i], tip_human[i]),
        ]

    print(f"Auto segment_scaling from {frames} frames:")
    for finger_name, scales in segment_scaling.items():
        print(f"  {finger_name}: {scales}")
    return segment_scaling


def init_mujoco_model(xml_path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_id != -1:
        mujoco.mj_resetDataKeyframe(model, data, home_id)
    mujoco.mj_forward(model, data)
    return model, data


def set_mujoco_ctrl(model: mujoco.MjModel, data: mujoco.MjData, actuator_name: str, value: float):
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
    if actuator_id == -1:
        raise ValueError(f"Actuator not found: {actuator_name}")

    low, high = model.actuator_ctrlrange[actuator_id]
    data.ctrl[actuator_id] = np.clip(value, low, high)


def get_mujoco_joint_qpos(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id == -1:
        raise ValueError(f"Joint not found: {joint_name}")
    return float(data.qpos[model.jnt_qposadr[joint_id]])


def get_mujoco_tendon_length(model: mujoco.MjModel, data: mujoco.MjData, tendon_name: str) -> float:
    tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)
    if tendon_id == -1:
        raise ValueError(f"Tendon not found: {tendon_name}")
    return float(data.ten_length[tendon_id])


def set_mujoco_finger_ctrl(model: mujoco.MjModel, data: mujoco.MjData, finger_name: str, ctrl: np.ndarray):
    actuator_names = [
        f"{finger_name}_swing",
        f"{finger_name}_joint_0",
        f"{finger_name}_tendon",
    ]
    for actuator_name, value in zip(actuator_names, ctrl):
        set_mujoco_ctrl(model, data, actuator_name, float(value))


def set_mujoco_thumb_ctrl(model: mujoco.MjModel, data: mujoco.MjData, ctrl: np.ndarray):
    actuator_names = [
        "thumb_joint_0",
        "thumb_joint_1",
        "thumb_joint_2",
        "thumb_joint_3",
    ]
    for actuator_name, value in zip(actuator_names, ctrl):
        set_mujoco_ctrl(model, data, actuator_name, float(value))


def apply_mujoco_abduction_constraints(model: mujoco.MjModel, data: mujoco.MjData):
    index_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "index_swing")
    middle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "middle_swing")
    ring_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "ring_swing")
    little_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "little_swing")

    middle_lower = max(data.ctrl[index_id], model.actuator_ctrlrange[middle_id, 0])
    middle_upper = min(data.ctrl[little_id], model.actuator_ctrlrange[middle_id, 1])
    data.ctrl[middle_id] = np.clip(data.ctrl[middle_id], middle_lower, middle_upper)

    ring_lower = max(data.ctrl[middle_id], model.actuator_ctrlrange[ring_id, 0])
    ring_upper = min(data.ctrl[little_id], model.actuator_ctrlrange[ring_id, 1])
    data.ctrl[ring_id] = np.clip(data.ctrl[ring_id], ring_lower, ring_upper)


def apply_retarget_qpos_to_mujoco(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    retargeter: Retargeter,
    qpos: np.ndarray,
) -> bool:
    qpos = np.asarray(qpos, dtype=np.float64)
    if not np.all(np.isfinite(qpos)):
        return False

    qpos_map = qpos_by_name(retargeter, qpos)
    thumb_ctrl = np.array([
        qpos_map["thumb_cmc_1"],
        qpos_map["thumb_cmc_2"],
        qpos_map["thumb_mcp"],
        qpos_map["thumb_ip"],
    ])
    set_mujoco_thumb_ctrl(model, data, thumb_ctrl)

    for finger_name in NON_THUMB_FINGERS:
        mcp_1_rad = qpos_map[f"{finger_name}_mcp_1"]
        mcp_2_rad = qpos_map[f"{finger_name}_mcp_2"]
        pip_rad = qpos_map[f"{finger_name}_pip"]
        dip_rad = qpos_map[f"{finger_name}_dip"]
        tendon_m = 0.001 * (
            np.sqrt(170 * (1 - np.cos(1.72 - dip_rad)))
            + np.sqrt(183 * (1 - np.cos(1.72 - pip_rad)))
        )
        set_mujoco_finger_ctrl(model, data, finger_name, np.array([mcp_1_rad, mcp_2_rad, tendon_m]))
    apply_mujoco_abduction_constraints(model, data)
    return True


def mujoco_state_to_parahand_positions(model: mujoco.MjModel, data: mujoco.MjData, hand: ParaHand) -> list[float]:
    values_by_joint = {
        "thumb.cmc_1": get_mujoco_joint_qpos(model, data, "thumb_joint_0"),
        "thumb.cmc_2": get_mujoco_joint_qpos(model, data, "thumb_joint_1"),
        "thumb.mcp": get_mujoco_joint_qpos(model, data, "thumb_joint_2"),
        "thumb.ip": get_mujoco_joint_qpos(model, data, "thumb_joint_3"),
    }

    for finger_name in NON_THUMB_FINGERS:
        tendon_m = get_mujoco_tendon_length(model, data, f"{finger_name}_tendon")
        values_by_joint[f"{finger_name}.mcp_1"] = get_mujoco_joint_qpos(model, data, f"{finger_name}_swing")
        values_by_joint[f"{finger_name}.mcp_2"] = get_mujoco_joint_qpos(model, data, f"{finger_name}_joint_0")
        values_by_joint[f"{finger_name}.pip_dip"] = TENDON_TO_PIP_DIP_OFFSET_M - tendon_m

    return [values_by_joint[joint_name] for joint_name in hand.get_hand_joint_order()]


def init_real_hand(args: argparse.Namespace) -> ParaHand | None:
    if args.no_real_hand:
        print("Real ParaHand output disabled")
        return None

    hand_config_path = resolve_path(args.hand_config, ROOT)
    hand = ParaHand(str(hand_config_path))
    print(f"Using real ParaHand config: {hand.config_path}")
    if not hand.connect():
        raise RuntimeError(f"Failed to connect real ParaHand on {hand.motor.port}")

    hand.enable()
    if args.tendon_speed is not None:
        hand.set_tendon_motor_speeds_broadcast(args.tendon_speed, args.default_motor_speed)
    print("Real ParaHand connected and enabled")
    return hand


def run(args: argparse.Namespace):
    config_path = resolve_path(args.config, ROOT)
    model_path = resolve_path(args.model, ROOT)

    retarget_config = load_retarget_config(config_path, args.urdf)
    retargeter = Retargeter.from_config(retarget_config, args.hand)
    urdf_path = Path(retarget_config["robot"]["urdf_path"])

    model, data = init_mujoco_model(model_path)
    real_hand = None
    control_thread = None
    stop_event = threading.Event()
    try:
        input_device = VisionPro(ip=args.ip)
        streamer = input_device.streamer
        if hasattr(streamer, "configure_mujoco"):
            streamer.configure_mujoco(str(model_path), model, data, relative_to=[0, 0, 0.8, 90], force_reload=True)
        if hasattr(streamer, "start_webrtc"):
            streamer.start_webrtc()
        if hasattr(streamer, "set_origin"):
            streamer.set_origin("sim")

        if args.auto_segment_scaling:
            try:
                segment_scaling = auto_segment_scaling(
                    input_device=input_device,
                    retargeter=retargeter,
                    hand_side=args.hand,
                    duration_s=args.calibrate_duration,
                    stable_duration_s=args.stable_duration,
                    stable_threshold_m=args.stable_threshold,
                    stable_timeout_s=args.stable_timeout,
                    stable_min_wait_s=args.stable_min_wait,
                )
            except Exception as exc:
                print(f"Auto segment_scaling failed ({exc}); using YAML defaults")
                segment_scaling = None
            if segment_scaling is not None:
                retarget_config.setdefault("retarget", {})["segment_scaling"] = segment_scaling
                retargeter = Retargeter.from_config(retarget_config, args.hand)

        control_period_s = 1.0 / args.rate
        sim_period_s = float(model.opt.timestep)
        print(f"Using config: {config_path}")
        print(f"Using retarget URDF: {urdf_path}")
        print(f"Using MuJoCo XML: {model_path}")
        print(f"AVP IP: {args.ip}, hand: {args.hand}")
        print(f"Control dt: {control_period_s:.4f}s ({args.rate:.1f}Hz)")
        print(f"MuJoCo sim dt: {sim_period_s:.4f}s ({1.0 / sim_period_s:.1f}Hz)")
        metrics = LoopMetrics(args.metrics_interval) if args.metrics_interval > 0.0 else None
        real_hand = init_real_hand(args)
        real_hand_period_s = 1.0 / args.real_hand_rate if real_hand is not None else None

        latest_qpos = np.zeros(retargeter.num_joints, dtype=np.float64)
        qpos_lock = threading.Lock()
        qpos_ready = False
        qpos_updated = False

        def control_thread_fn():
            nonlocal qpos_ready, qpos_updated
            while not stop_event.is_set():
                loop_start = time.perf_counter()

                t0 = time.perf_counter()
                mediapipe_pose = get_latest_mediapipe(input_device, args.hand)
                t1 = time.perf_counter()
                if metrics is not None:
                    metrics.add_time("input", t1 - t0)
                    metrics.record_input(mediapipe_pose)

                if mediapipe_pose is not None:
                    t0 = time.perf_counter()
                    qpos = retargeter.retarget(mediapipe_pose)
                    t1 = time.perf_counter()
                    if metrics is not None:
                        metrics.add_time("retarget", t1 - t0)

                    if np.all(np.isfinite(qpos)):
                        with qpos_lock:
                            latest_qpos[:] = qpos
                            qpos_ready = True
                            qpos_updated = True
                        if metrics is not None:
                            metrics.add_control()

                sleep_time = control_period_s - (time.perf_counter() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        control_thread = threading.Thread(target=control_thread_fn, daemon=True)

        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = [-0.15, -0.02, 0.0]
            viewer.cam.distance = 0.45
            viewer.cam.azimuth = 120
            viewer.cam.elevation = -25

            control_thread.start()
            last_real_hand_send = 0.0
            mujoco_target_started = False
            while viewer.is_running():
                step_start = time.perf_counter()
                if metrics is not None:
                    metrics.add_loop()

                qpos_to_apply = None
                with qpos_lock:
                    if qpos_ready and qpos_updated:
                        qpos_to_apply = latest_qpos.copy()
                        qpos_updated = False

                if qpos_to_apply is not None:
                    t0 = time.perf_counter()
                    applied = apply_retarget_qpos_to_mujoco(model, data, retargeter, qpos_to_apply)
                    t1 = time.perf_counter()
                    if metrics is not None:
                        metrics.add_time("apply", t1 - t0)
                    if applied:
                        mujoco_target_started = True

                t0 = time.perf_counter()
                mujoco.mj_step(model, data)
                t1 = time.perf_counter()
                if metrics is not None:
                    metrics.add_time("mj_step", t1 - t0)

                now = time.perf_counter()
                if (
                    real_hand is not None
                    and real_hand_period_s is not None
                    and mujoco_target_started
                    and now - last_real_hand_send >= real_hand_period_s
                ):
                    t0 = time.perf_counter()
                    real_hand_positions = mujoco_state_to_parahand_positions(model, data, real_hand)
                    real_hand.set_hand_positions_broadcast(real_hand_positions)
                    t1 = time.perf_counter()
                    last_real_hand_send = t1
                    if metrics is not None:
                        metrics.add_time("real_hand", t1 - t0)

                t0 = time.perf_counter()
                viewer.sync()
                t1 = time.perf_counter()
                if metrics is not None:
                    metrics.add_time("viewer_sync", t1 - t0)

                if hasattr(streamer, "update_sim"):
                    t0 = time.perf_counter()
                    streamer.update_sim()
                    t1 = time.perf_counter()
                    if metrics is not None:
                        metrics.add_time("update_sim", t1 - t0)

                sleep_time = sim_period_s - (time.perf_counter() - step_start)
                if sleep_time > 0:
                    t0 = time.perf_counter()
                    time.sleep(sleep_time)
                    t1 = time.perf_counter()
                    if metrics is not None:
                        metrics.add_time("sleep", t1 - t0)
                if metrics is not None:
                    now = time.perf_counter()
                    metrics.add_time("loop", now - step_start)
                    metrics.maybe_print(now)
        stop_event.set()
        if control_thread is not None:
            control_thread.join(timeout=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if control_thread is not None:
            control_thread.join(timeout=1.0)
        if real_hand is not None:
            try:
                real_hand.disable()
            except Exception:
                pass
            try:
                real_hand.disconnect()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="AnyDexRetarget AVP retargeting viewer for ParaHand MuJoCo.")
    parser.add_argument("--ip", default="192.168.31.151", help="Vision Pro IP.")
    parser.add_argument("--hand", default="right", choices=("left", "right"), help="AVP hand side to retarget.")
    parser.add_argument("--rate", type=float, default=10.0, help="AVP input + retarget control rate in Hz.")
    parser.add_argument("--config", default="config_teleop.yaml", help="AnyDexRetarget YAML config path.")
    parser.add_argument("--model", default="mujoco/para_fr3.xml", help="MuJoCo XML model path.")
    parser.add_argument("--urdf", default=None, help="Override robot.urdf_path from the YAML config.")
    parser.add_argument("--hand-config", default="config_hand.yaml", help="Real ParaHand motor YAML config path.")
    parser.add_argument(
        "--no-real-hand",
        action="store_true",
        help="Keep MuJoCo visualization only; do not connect or command the real ParaHand.",
    )
    parser.add_argument(
        "--tendon-speed",
        type=float,
        default=100.0,
        help="Broadcast speed percentage for pip_dip tendon motors after connecting the real hand; set <0 to skip.",
    )
    parser.add_argument(
        "--default-motor-speed",
        type=float,
        default=50.0,
        help="Broadcast speed percentage for non-tendon motors when --tendon-speed is enabled.",
    )
    parser.add_argument(
        "--real-hand-rate",
        type=float,
        default=None,
        help="Real ParaHand command rate in Hz; commands are read from the current MuJoCo state.",
    )
    parser.add_argument(
        "--metrics-interval",
        type=float,
        default=1.0,
        help="Print measured loop/control frequencies every N seconds; set <=0 to disable.",
    )
    parser.add_argument(
        "--no-auto-segment-scaling",
        dest="auto_segment_scaling",
        action="store_false",
        help="Use segment_scaling from YAML instead of measuring it from Vision Pro at startup.",
    )
    parser.set_defaults(auto_segment_scaling=True)
    parser.add_argument(
        "--stable-min-wait",
        type=float,
        default=5.0,
        help="Minimum seconds to stay in stable-hand detection before accepting the threshold.",
    )
    parser.add_argument("--calibrate-duration", type=float, default=3.0, help="Seconds to average AVP hand samples.")
    parser.add_argument("--stable-duration", type=float, default=1.0, help="Required stable-hand window before averaging.")
    parser.add_argument(
        "--stable-threshold",
        type=float,
        default=0.06,
        help="Max wrist-relative keypoint std in meters for stable-hand detection.",
    )
    parser.add_argument("--stable-timeout", type=float, default=20.0, help="Max seconds to wait for a stable hand.")
    args = parser.parse_args()
    if args.real_hand_rate is None:
        args.real_hand_rate = args.rate
    if args.real_hand_rate <= 0.0:
        parser.error("--real-hand-rate must be > 0")
    if args.tendon_speed is not None and args.tendon_speed < 0.0:
        args.tendon_speed = None
    run(args)


if __name__ == "__main__":
    main()
