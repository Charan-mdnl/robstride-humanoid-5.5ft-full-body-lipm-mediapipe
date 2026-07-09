#!/usr/bin/env python3
"""
ik_sphere_viz.py
================
MuJoCo kinematic viewer with correct axis conventions, mirroring joint
positions from /joint_states and displaying IK visualization markers.

Uses kinematic mode (no independent physics) for stable visualization
with auto-grounding to keep feet on the ground plane.

Usage:
  ros2 run full_body_mujoco ik_sphere_viz.py
"""

import os
import re
import threading
import numpy as np
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

MESH_DIR   = '/home/me/clean_ws/src/XP_robot_with_hand_mjcf/meshes'
XML_PATH   = '/home/me/clean_ws/install/full_body_mujoco/share/full_body_mujoco/config/angad_full_body.xml'


# ── Shared state ──────────────────────────────────────────────────────────────
_lock     = threading.Lock()
_jpos     = {}          # joint_name → angle (from /joint_states)
_mpos     = np.zeros((10, 3))  # mocap positions (pelvis-frame from markers)
_mpos_valid = [False] * 10


class RobotListener(Node):
    def __init__(self):
        super().__init__("ik_sphere_viz")
        self.create_subscription(JointState,    "/joint_states",       self._js_cb,  10)
        self.create_subscription(MarkerArray, "/nr_ik_test/markers", self._mk_cb, 10)
        self.get_logger().info("ik_sphere_viz ready")

    def _js_cb(self, msg: JointState):
        with _lock:
            for name, pos in zip(msg.name, msg.position):
                _jpos[name] = pos

    def _mk_cb(self, msg: MarkerArray):
        global _mpos, _mpos_valid
        r_step = l_step = 0
        new_pos   = _mpos.copy()
        new_valid = _mpos_valid.copy()
        for m in msg.markers:
            p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            if   m.id == 0: new_pos[0] = p; new_valid[0] = True
            elif m.id == 1: new_pos[1] = p; new_valid[1] = True
            elif m.id == 2: new_pos[2] = p; new_valid[2] = True
            elif m.id == 3: new_pos[3] = p; new_valid[3] = True
            elif m.id >= 10:
                idx = m.id - 10
                if idx % 2 == 0 and r_step < 3:
                    new_pos[4+r_step] = p; new_valid[4+r_step] = True; r_step += 1
                elif idx % 2 == 1 and l_step < 3:
                    new_pos[7+l_step] = p; new_valid[7+l_step] = True; l_step += 1
        with _lock:
            _mpos[:] = new_pos
            _mpos_valid[:] = new_valid


def load_model():
    with open(XML_PATH) as f:
        xml = f.read()
    xml = xml.replace('MESHDIR_PLACEHOLDER', MESH_DIR)
    return mujoco.MjModel.from_xml_string(xml)


def main():
    # ROS2 on background thread
    rclpy.init()
    node = RobotListener()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    model = load_model()
    data  = mujoco.MjData(model)

    # Kick off at the standing keyframe
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'standing')
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)

    # Joint name → qpos address map
    jnt_qposadr = {}
    for i in range(model.njnt):
        name = model.joint(i).name
        jnt_qposadr[name] = model.jnt_qposadr[i]

    # Joint name → actuator index map (for control commands)
    jnt_actuator = {}
    for i in range(model.nu):
        act_name = model.actuator(i).name
        jnt_actuator[act_name] = i
        # Initialize actuators to match keyframe positions
        if act_name in jnt_qposadr:
            data.ctrl[i] = data.qpos[jnt_qposadr[act_name]]

    # Mocap body index map
    marker_names = ["viz_foot_r","viz_foot_l","viz_target","viz_swing",
                    "viz_step_r0","viz_step_r1","viz_step_r2",
                    "viz_step_l0","viz_step_l1","viz_step_l2"]
    mocap_ids = []
    for n in marker_names:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
        mocap_ids.append(model.body_mocapid[bid] if bid >= 0 else -1)

    # Pelvis body index for frame transform
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    foot_r_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "foot_r")
    foot_l_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "foot_l")

    # Initial grounding: adjust pelvis Z so feet are exactly on ground (z=0)
    mujoco.mj_kinematics(model, data)
    foot_r_z = data.xpos[foot_r_id][2]
    foot_l_z = data.xpos[foot_l_id][2]
    lowest_foot = min(foot_r_z, foot_l_z)
    data.qpos[2] -= (lowest_foot - 0.05)  # Set lowest foot at z=0.05
    mujoco.mj_kinematics(model, data)

    print("\n[ik_sphere_viz]  orange=R-foot  cyan=L-foot  red=target  yellow=swing")
    print("[ik_sphere_viz]  Kinematic mode with auto-grounding (correct axes)")
    print("[ik_sphere_viz]  Waiting for joint_states...")
    
    # Wait for initial joint_states before starting viewer
    import time
    timeout = 5.0
    start_time = time.time()
    while len(_jpos) == 0 and (time.time() - start_time) < timeout:
        time.sleep(0.1)
    
    if len(_jpos) == 0:
        print("[ik_sphere_viz]  WARNING: No joint_states received!")
    else:
        print(f"[ik_sphere_viz]  Received {len(_jpos)} joint positions, starting viewer")

    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.distance  = 2.5
        v.cam.elevation = -20
        v.cam.azimuth   = 120
        while v.is_running():
            with _lock:
                jpos_snap  = dict(_jpos)
                mpos_snap  = _mpos.copy()
                valid_snap = list(_mpos_valid)

            # Set joint positions directly (kinematic mode)
            for name, angle in jpos_snap.items():
                if name in jnt_qposadr:
                    data.qpos[jnt_qposadr[name]] = angle

            # Auto-ground: adjust pelvis Z so feet stay on ground
            mujoco.mj_kinematics(model, data)
            foot_r_z = data.xpos[foot_r_id][2]
            foot_l_z = data.xpos[foot_l_id][2]
            lowest_foot = min(foot_r_z, foot_l_z)
            data.qpos[2] -= (lowest_foot - 0.02)

            # Compute forward kinematics (no dynamics)
            mujoco.mj_forward(model, data)

            # Transform marker positions: pelvis-frame → world-frame
            pp = data.xpos[pelvis_id]
            pr = data.xmat[pelvis_id].reshape(3, 3)

            for i, mid in enumerate(mocap_ids):
                if mid < 0 or not valid_snap[i]:
                    continue
                p_local = mpos_snap[i]
                data.mocap_pos[mid] = pp + pr @ p_local

            v.sync()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
