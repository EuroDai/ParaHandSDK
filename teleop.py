import os
import sys
import asyncio
import time
import numpy as np
import mujoco
import mujoco.viewer
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from avp_stream import VisionProStreamer
from parahand import ParaHand

if sys.platform == "win32":
    _start_server = asyncio.start_server

    async def _start_server_without_reuse_port(*args, **kwargs):
        kwargs.pop("reuse_port", None)
        return await _start_server(*args, **kwargs)

    asyncio.start_server = _start_server_without_reuse_port

NODE_IDS = {
    "wrist": 0,
    "thumb":{
        "cmc": 1,
        "mcp": 2,
        "ip": 3,
        "tip": 4,
    },
    "index":{
        "cmc": 5,
        "mcp": 6,
        "pip": 7,
        "dip": 8,
        "tip": 9,
    },
    "middle":{
        "cmc": 10,
        "mcp": 11,
        "pip": 12,
        "dip": 13,
        "tip": 14,
    },
    "ring":{
        "cmc": 15,
        "mcp": 16,
        "pip": 17,
        "dip": 18,
        "tip": 19,
    },
    "little":{
        "cmc": 20,
        "mcp": 21,
        "pip": 22,
        "dip": 23,
        "tip": 24,
    },
    "forearm_wrist": 25,
    "forearm_arm": 26,
}

WRIST_TO_PALM = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=np.float64)
THUMB_MCP_TO_LINK2 = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], dtype=np.float64)

def is_valid_rotation_matrix(rotation):
    if not np.all(np.isfinite(rotation)):
        return False
    if np.linalg.norm(rotation) < 1e-6:
        return False
    return np.linalg.det(rotation) > 0.0

def calculate_flexion_angles(matrix_node_0, matrix_node_1):
    '''计算手指弯曲关节角度'''
    rotation_0 = matrix_node_0[:3, :3]
    rotation_1 = matrix_node_1[:3, :3]
    rotation_diff = rotation_1 @ rotation_0.T
    x_axis = rotation_diff[:3, 0]
    angle = np.arctan2(x_axis[1], x_axis[0])
    return angle

def calculate_abduction_angle(matrix_node_0, matrix_node_1):
    '''计算手指侧摆关节角度'''
    rotation_0 = matrix_node_0[:3, :3]
    rotation_1 = matrix_node_1[:3, :3]
    rotation_diff = rotation_1 @ rotation_0.T

    z_axis = rotation_diff[:3, 2]
    angle = - np.arctan2(z_axis[0], z_axis[2])
    return angle

def calculate_local_z_rotation_angle(matrix_node_0, matrix_node_1):
    rotation_0 = matrix_node_0[:3, :3]
    rotation_1 = matrix_node_1[:3, :3]
    rotation_diff = rotation_0.T @ rotation_1
    return np.arctan2(rotation_diff[1, 0], rotation_diff[0, 0])

def get_finger_ctrl(finger_name, r):
    mcp_1_rad = calculate_abduction_angle(r['right_arm'][NODE_IDS[finger_name]['cmc']], r['right_arm'][NODE_IDS[finger_name]['mcp']])
    mcp_2_rad = calculate_local_z_rotation_angle(r['right_arm'][NODE_IDS[finger_name]['cmc']], r['right_arm'][NODE_IDS[finger_name]['mcp']])
    pip_rad = calculate_local_z_rotation_angle(r['right_arm'][NODE_IDS[finger_name]['mcp']], r['right_arm'][NODE_IDS[finger_name]['pip']])
    dip_rad = calculate_local_z_rotation_angle(r['right_arm'][NODE_IDS[finger_name]['pip']], r['right_arm'][NODE_IDS[finger_name]['dip']])
    tendon_m = 0.001 * (np.sqrt(170 * (1 - np.cos(1.72 - dip_rad))) + np.sqrt(183 * (1 - np.cos(1.72 - pip_rad))))
    pip_dip_m = 0.025 - tendon_m
    return [mcp_1_rad, mcp_2_rad, pip_dip_m]

def make_thumb_ik_context(model):
    thumb_data = mujoco.MjData(model)
    home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    joint_names = ["thumb_joint_0", "thumb_joint_1", "thumb_joint_2"]
    actuator_names = ["thumb_joint_0", "thumb_joint_1", "thumb_joint_2"]

    joint_qpos_adrs = [
        model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        for name in joint_names
    ]
    actuator_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in actuator_names
    ]

    return {
        "data": thumb_data,
        "home_id": home_id,
        "joint_qpos_adrs": joint_qpos_adrs,
        "bounds": model.actuator_ctrlrange[actuator_ids].T,
        "palm_id": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "palm"),
        "link2_id": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "thumb_link_2"),
        "prev_q": np.array([0.0, 0.0, 0.0], dtype=np.float64),
    }

def get_thumb_link2_rotation(model, thumb_ik_context, q):
    data = thumb_ik_context["data"]
    mujoco.mj_resetDataKeyframe(model, data, thumb_ik_context["home_id"])
    for qpos_adr, value in zip(thumb_ik_context["joint_qpos_adrs"], q):
        data.qpos[qpos_adr] = value
    mujoco.mj_forward(model, data)

    palm_rotation = data.xmat[thumb_ik_context["palm_id"]].reshape(3, 3)
    link2_rotation = data.xmat[thumb_ik_context["link2_id"]].reshape(3, 3)
    return palm_rotation.T @ link2_rotation

def solve_thumb_link2_ik(model, thumb_ik_context, target_rotation):
    lower, upper = thumb_ik_context["bounds"]
    initial_q = np.clip(thumb_ik_context["prev_q"], lower, upper)

    def objective(q):
        current_rotation = get_thumb_link2_rotation(model, thumb_ik_context, q)
        rotation_error = current_rotation.T @ target_rotation
        rotvec = Rotation.from_matrix(rotation_error).as_rotvec()
        return float(rotvec @ rotvec)

    result = minimize(
        objective,
        initial_q,
        method="L-BFGS-B",
        bounds=list(zip(lower, upper)),
        options={"maxiter": 25, "ftol": 1e-8},
    )
    thumb_ik_context["prev_q"] = result.x
    return result.x

def get_thumb_ctrl(model, thumb_ik_context, r, previous_ctrl):
    thumb_matrix_wrist = r['right_arm'][NODE_IDS['wrist']][:3, :3]
    thumb_matrix_mcp = r['right_arm'][NODE_IDS['thumb']['mcp']][:3, :3]
    if not is_valid_rotation_matrix(thumb_matrix_wrist) or not is_valid_rotation_matrix(thumb_matrix_mcp):
        return previous_ctrl

    thumb_matrix_diff = WRIST_TO_PALM @ thumb_matrix_wrist.T @ thumb_matrix_mcp @ THUMB_MCP_TO_LINK2
    if not is_valid_rotation_matrix(thumb_matrix_diff):
        return previous_ctrl

    cmc_1_rad, cmc_2_rad, mcp_rad = solve_thumb_link2_ik(model, thumb_ik_context, thumb_matrix_diff)
    ip_rad = calculate_local_z_rotation_angle(r['right_arm'][NODE_IDS['thumb']['mcp']], r['right_arm'][NODE_IDS['thumb']['ip']])
    return [cmc_1_rad, cmc_2_rad, mcp_rad, ip_rad]

def build_hand_positions(model, thumb_ik_context, r, previous_thumb_ctrl):
    thumb_ctrl = get_thumb_ctrl(model, thumb_ik_context, r, previous_thumb_ctrl)
    # thumb_ctrl = [0.0, 0.0, 0.0, 0.0]
    index_ctrl = get_finger_ctrl("index", r)
    middle_ctrl = get_finger_ctrl("middle", r)
    ring_ctrl = get_finger_ctrl("ring", r)
    little_ctrl = get_finger_ctrl("little", r)
    return thumb_ctrl + index_ctrl + middle_ctrl + ring_ctrl + little_ctrl, thumb_ctrl

def set_mujoco_ctrl(model, data, actuator_name, value):
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
    if actuator_id == -1:
        raise ValueError(f"Actuator not found: {actuator_name}")

    low, high = model.actuator_ctrlrange[actuator_id]
    data.ctrl[actuator_id] = np.clip(value, low, high)

def set_mujoco_finger_ctrl(model, data, finger_name, ctrl):
    actuator_names = [
        f"{finger_name}_swing",
        f"{finger_name}_joint_0",
        f"{finger_name}_tendon",
    ]
    for actuator_name, value in zip(actuator_names, ctrl):
        set_mujoco_ctrl(model, data, actuator_name, value)

def set_mujoco_thumb_ctrl(model, data, ctrl):
    actuator_names = [
        "thumb_joint_0",
        "thumb_joint_1",
        "thumb_joint_2",
        "thumb_joint_3",
    ]
    for actuator_name, value in zip(actuator_names, ctrl):
        set_mujoco_ctrl(model, data, actuator_name, value)

def apply_mujoco_abduction_constraints(model, data):
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

def apply_mujoco_visualization_ctrl(model, data, positions):
    set_mujoco_thumb_ctrl(model, data, positions[0:4])
    for finger_name, start_index in (
        ("index", 4),
        ("middle", 7),
        ("ring", 10),
        ("little", 13),
    ):
        mcp_1_rad, mcp_2_rad, pip_dip_m = positions[start_index:start_index + 3]
        tendon_m = 0.025 - pip_dip_m
        set_mujoco_finger_ctrl(model, data, finger_name, [mcp_1_rad, mcp_2_rad, tendon_m])
    apply_mujoco_abduction_constraints(model, data)

def main():
    model = mujoco.MjModel.from_xml_path("para_fr3.xml")
    data = mujoco.MjData(model)

    home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, home_id)
    mujoco.mj_forward(model, data)
    thumb_ik_context = make_thumb_ik_context(model)

    avp_ip = "192.168.31.151"  # Vision Pro IP (shown in the app)
    s = VisionProStreamer(ip=avp_ip)
    s.configure_mujoco("para_fr3.xml", model, data, relative_to=[0, 0, 0.8, 90], force_reload=True,)
    s.start_webrtc()
    s.set_origin("sim")

    hand = ParaHand()
    if not hand.connect():
        raise RuntimeError("Failed to connect ParaHand")

    loop_period_s = 0.02
    thumb_ctrl = [0.0, 0.0, 0.0, 0.0]

    try:
        hand.enable()
        hand.set_tendon_motor_speeds_broadcast(100.0)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = [0.0, 0.0, 0.35]
            viewer.cam.distance = 1.2
            viewer.cam.azimuth = 180
            viewer.cam.elevation = -25

            while viewer.is_running():
                step_start = time.perf_counter()
                r = s.get_latest()

                positions, thumb_ctrl = build_hand_positions(model, thumb_ik_context, r, thumb_ctrl)
                hand.set_hand_positions_broadcast(positions)
                apply_mujoco_visualization_ctrl(model, data, positions)

                mujoco.mj_step(model, data)
                viewer.sync()
                s.update_sim()

                sleep_time = max(0.0, loop_period_s - (time.perf_counter() - step_start))
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            hand.disable()
        finally:
            hand.disconnect()

if __name__ == "__main__":
    main()
