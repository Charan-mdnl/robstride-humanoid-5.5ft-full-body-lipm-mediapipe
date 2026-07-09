#!/usr/bin/env python3
"""
main_walking_node.py — Angad Walking Node (FIXED)
===================================================
Properly integrates LIPM + DCM + IK with:
  - Correct hip axis factor (0.94)
  - Bounded DCM (no exponential blowup)
  - Hard-clamped joint angles
  - Joint state subscriber for debugging
  - Extensive angle logging

Pipeline:  StepPlanner → DCM → FootTrajectory → IK → JointTrajectory

Usage:  python3 main_walking_node.py
"""

import sys
import os
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from step_planner import StepPlanner, FootSide
from lipm_model import LIPMModel
from dcm_controller import DCMController
from foot_trajectory import FootTrajectoryGenerator
from ik_solver import AngadLegIK, compute_standing_angles, HIP_OFFSET_Y, L_FULL
from trajectory_publisher import TrajectoryPublisher


class WalkingNode(Node):

    def __init__(self):
        super().__init__('angad_walking_node')

        # ═══════════════════════════════════════════════════════
        # WALKING PARAMETERS — conservative values
        # ═══════════════════════════════════════════════════════
        self.step_length  = 0.04     # forward per step [m] — SMALL
        self.step_width   = 0.128    # lateral separation [m]
        self.step_height  = 0.025    # swing clearance [m] — LOW
        self.step_period  = 1.2      # per step [s] — SLOW
        self.ds_ratio     = 0.3      # 30% double support — MORE STABLE

        self.hip_height   = 0.74     # hip above ground [m] (~98% extension)
        self.com_height   = 0.74     # LIPM pendulum height [m]

        self.num_steps    = 8        # walking steps
        self.stand_time   = 4.0      # stand before walking [s]
        self.control_hz   = 50.0
        self.dt = 1.0 / self.control_hz

        # ═══════════════════════════════════════════════════════
        # PIPELINE MODULES
        # ═══════════════════════════════════════════════════════
        self.planner = StepPlanner(
            step_length=self.step_length,
            step_width=self.step_width,
            step_height=self.step_height,
            step_period=self.step_period,
            double_support_ratio=self.ds_ratio,
            num_steps=self.num_steps,
        )

        self.lipm = LIPMModel(z_com=self.com_height, dt=self.dt)

        self.dcm = DCMController(
            omega=self.lipm.omega,
            k_dcm=1.0,       # conservative gain
            dt=self.dt,
        )

        self.foot_traj = FootTrajectoryGenerator(step_height=self.step_height)
        self.ik = AngadLegIK()
        self.pub = TrajectoryPublisher(node=self)

        # ═══════════════════════════════════════════════════════
        # JOINT STATE SUBSCRIBER (for debugging)
        # ═══════════════════════════════════════════════════════
        self.joint_states = {}
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
        )

        # ═══════════════════════════════════════════════════════
        # STATE
        # ═══════════════════════════════════════════════════════
        self.state = 'INIT'

        self.com_pos = np.array([0.0, 0.0])
        self.com_vel = np.array([0.0, 0.0])

        self.left_foot  = np.array([0.0,  self.step_width / 2, 0.0])
        self.right_foot = np.array([0.0, -self.step_width / 2, 0.0])

        self.footsteps     = []
        self.step_idx      = 0
        self.step_time     = 0.0
        self.stand_start   = None
        self._done_logged  = False
        self._log_counter  = 0

        # Last published angles for logging
        self._last_angles = {}

        # ═══════════════════════════════════════════════════════
        # START
        # ═══════════════════════════════════════════════════════
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info("=" * 55)
        self.get_logger().info("  Angad Walking — LIPM + DCM + IK (FIXED)")
        self.get_logger().info(f"  Hip height     : {self.hip_height:.3f} m "
                               f"(max {L_FULL:.3f} m)")
        self.get_logger().info(f"  Step           : {self.step_length}m fwd, "
                               f"{self.step_height}m lift, {self.step_period}s")
        self.get_logger().info(f"  Double support : {self.ds_ratio*100:.0f}%")
        self.get_logger().info(f"  Num steps      : {self.num_steps}")
        self.get_logger().info(f"  ω = {self.lipm.omega:.2f} rad/s")
        self.get_logger().info("=" * 55)

    def _joint_state_cb(self, msg: JointState):
        """Store actual joint positions from Gazebo."""
        for name, pos in zip(msg.name, msg.position):
            self.joint_states[name] = pos

    # ──────────────────────────────────────────────────────
    # MAIN TICK
    # ──────────────────────────────────────────────────────
    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.state == 'INIT':
            self.get_logger().info("→ Standing upright (zero ankle, small knee bend)")
            self._publish_stand()
            self.state = 'STANDING'
            self.stand_start = now

        elif self.state == 'STANDING':
            elapsed = now - self.stand_start
            if elapsed >= self.stand_time:
                self._init_walking()
                self.state = 'WALKING'
            elif int(elapsed) % 2 == 0 and int(elapsed * 10) % 20 == 0:
                self._publish_stand()

        elif self.state == 'WALKING':
            self._walk_tick()

        elif self.state == 'DONE' and not self._done_logged:
            self.get_logger().info("✓ Walking COMPLETE — holding stance")
            self._publish_stand()
            self._done_logged = True

    # ──────────────────────────────────────────────────────
    # STANDING
    # ──────────────────────────────────────────────────────
    def _publish_stand(self):
        angles = compute_standing_angles(hip_height=self.hip_height)
        if angles:
            self.pub.publish_standing_pose(angles, duration_sec=2.0)
            self._last_angles = angles
            self._log_angles("STAND")
        else:
            self.get_logger().error("Standing IK FAILED!")

    # ──────────────────────────────────────────────────────
    # INIT WALKING
    # ──────────────────────────────────────────────────────
    def _init_walking(self):
        self.footsteps = self.planner.generate_footsteps()
        self.get_logger().info(f"Plan: {len(self.footsteps)} steps")
        for i, fs in enumerate(self.footsteps):
            self.get_logger().info(
                f"  #{i} {fs.side.name:>5} → "
                f"({fs.position[0]:.3f}, {fs.position[1]:.3f})")

        self.step_idx  = 0
        self.step_time = 0.0
        self.com_pos   = np.array([0.0, 0.0])
        self.com_vel   = np.array([0.0, 0.0])
        self.left_foot  = np.array([0.0,  self.step_width / 2, 0.0])
        self.right_foot = np.array([0.0, -self.step_width / 2, 0.0])

    # ──────────────────────────────────────────────────────
    # ONE WALKING TICK
    # ──────────────────────────────────────────────────────
    def _walk_tick(self):
        if self.step_idx >= len(self.footsteps):
            self.state = 'DONE'
            return

        step = self.footsteps[self.step_idx]
        swing_side = step.side

        T = self.step_period
        T_ds = T * self.ds_ratio
        T_ss = T - T_ds

        # ── Step phase ──
        phase = np.clip(self.step_time / T, 0.0, 1.0)

        # Swing phase: only active during single support
        t_ds_half = T_ds / 2
        if self.step_time < t_ds_half:
            swing_phase = -1   # double support start
        elif self.step_time > T - t_ds_half:
            swing_phase = -1   # double support end
        else:
            swing_phase = np.clip((self.step_time - t_ds_half) / T_ss, 0.0, 1.0)

        # ════════════════════════════════════════════════════
        # 1. DCM — compute CoM for lateral weight shift
        # ════════════════════════════════════════════════════
        support_pos = self.planner.get_support_foot_position(
            self.footsteps, self.step_idx)

        # Next footstep target (for DCM reference)
        if self.step_idx + 1 < len(self.footsteps):
            next_target = self.footsteps[self.step_idx + 1].position
        else:
            next_target = step.position

        dcm_ref = self.dcm.compute_dcm_reference(
            support_pos, next_target, phase)

        # Support polygon center
        support_ctr = (self.left_foot[:2] + self.right_foot[:2]) / 2

        self.com_pos, self.com_vel = self.dcm.step(
            self.com_pos, self.com_vel, dcm_ref, support_ctr)

        # ════════════════════════════════════════════════════
        # 2. FOOT TRAJECTORIES
        # ════════════════════════════════════════════════════
        swing_start = self.planner.get_swing_foot_start(
            self.footsteps, self.step_idx)
        swing_end = step.position.copy()

        if swing_phase >= 0:
            swing_pos = self.foot_traj.compute_foot_position(
                swing_start, swing_end, swing_phase)
        else:
            # Double support — foot stays where it is
            if swing_side == FootSide.LEFT:
                swing_pos = self.left_foot.copy()
            else:
                swing_pos = self.right_foot.copy()

        # Update foot positions
        if swing_side == FootSide.LEFT:
            self.left_foot = swing_pos.copy()
        else:
            self.right_foot = swing_pos.copy()

        # ════════════════════════════════════════════════════
        # 3. IK — foot positions → joint angles
        # ════════════════════════════════════════════════════
        # Convert world frame → base_link frame
        # base_link is at (com_x, com_y, hip_height) in world
        lf_base = np.array([
            self.left_foot[0]  - self.com_pos[0],
            self.left_foot[1]  - self.com_pos[1],
            self.left_foot[2]  - self.hip_height,
        ])
        rf_base = np.array([
            self.right_foot[0] - self.com_pos[0],
            self.right_foot[1] - self.com_pos[1],
            self.right_foot[2] - self.hip_height,
        ])

        angles = self.ik.solve_both_legs(lf_base, rf_base)

        if angles is None:
            self.get_logger().warn(
                f"IK FAIL step={self.step_idx} t={self.step_time:.2f} "
                f"lf={lf_base} rf={rf_base}")
            self.step_time += self.dt
            return

        # ════════════════════════════════════════════════════
        # 4. PUBLISH
        # ════════════════════════════════════════════════════
        self.pub.publish_joint_angles(angles, duration_sec=self.dt * 2)
        self._last_angles = angles

        # Log every 10 ticks
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self._log_angles(f"WALK s{self.step_idx} ph={phase:.2f}")

        # ════════════════════════════════════════════════════
        # 5. ADVANCE
        # ════════════════════════════════════════════════════
        self.step_time += self.dt

        if self.step_time >= T:
            self.get_logger().info(
                f"Step {self.step_idx} done: {step.side.name} → "
                f"({step.position[0]:.3f}, {step.position[1]:.3f}) "
                f"CoM=({self.com_pos[0]:.3f},{self.com_pos[1]:.3f})")

            if swing_side == FootSide.LEFT:
                self.left_foot = step.position.copy()
            else:
                self.right_foot = step.position.copy()

            self.step_idx += 1
            self.step_time = 0.0

            if self.step_idx >= len(self.footsteps):
                self.state = 'DONE'

    # ──────────────────────────────────────────────────────
    # LOGGING
    # ──────────────────────────────────────────────────────
    def _log_angles(self, label: str):
        """Log the joint angles being published."""
        a = self._last_angles
        if not a:
            return

        leg_joints = ['hip_pitch', 'knee_pitch', 'ankle_pitch',
                       'thigh_roll', 'ankle_roll']
        parts = []
        for j in leg_joints:
            lv = a.get(f'{j}_l', 0.0)
            rv = a.get(f'{j}_r', 0.0)
            parts.append(f"{j}: L={np.degrees(lv):+6.1f}° R={np.degrees(rv):+6.1f}°")

        self.get_logger().info(f"[{label}] " + " | ".join(parts))

        # Also log actual vs commanded for hip/knee if we have joint_states
        if 'hip_pitch_l' in self.joint_states:
            for jn in ['hip_pitch_l', 'knee_pitch_l', 'ankle_pitch_l']:
                cmd = np.degrees(a.get(jn, 0))
                act = np.degrees(self.joint_states.get(jn, 0))
                self.get_logger().debug(
                    f"  {jn}: cmd={cmd:+.1f}° act={act:+.1f}° err={cmd-act:+.1f}°")


def main(args=None):
    rclpy.init(args=args)
    node = WalkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
