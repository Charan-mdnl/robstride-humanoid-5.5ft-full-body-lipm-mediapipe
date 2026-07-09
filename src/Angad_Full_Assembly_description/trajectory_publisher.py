#!/usr/bin/env python3
"""
trajectory_publisher.py — JointTrajectory Publisher for Angad Humanoid
======================================================================
Publishes trajectory_msgs/msg/JointTrajectory messages to the
JointTrajectoryController for execution in Gazebo.

All 23 joint names match exactly with the ros2_controllers.yaml config:
  - 12 leg joints:  hip_pitch_l/r, thigh_roll_l/r, thigh_yaw_l/r,
                    knee_pitch_l/r, ankle_pitch_l/r, ankle_roll_l/r
  - 3 torso joints: torso_pitch, torso_roll, torso_yaw
  - 8 arm joints:   shoulder_pitch_l/r, shoulder_roll_l/r,
                    elbow_yaw_l/r, elbow_pitch_l/r
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from typing import Dict, List, Optional
import numpy as np


# ═══════════════════════════════════════════════════════════════════════
# All 23 joint names in the EXACT order from ros2_controllers.yaml
# ═══════════════════════════════════════════════════════════════════════

ALL_JOINT_NAMES = [
    # ── Leg joints (12) ──
    'hip_pitch_l',
    'hip_pitch_r',
    'thigh_roll_l',
    'thigh_roll_r',
    'thigh_yaw_l',
    'thigh_yaw_r',
    'knee_pitch_l',
    'knee_pitch_r',
    'ankle_pitch_l',
    'ankle_pitch_r',
    'ankle_roll_l',
    'ankle_roll_r',
    # ── Torso joints (3) ──
    'torso_pitch',
    'torso_roll',
    'torso_yaw',
    # ── Arm joints (8) ──
    'shoulder_pitch_l',
    'shoulder_roll_l',
    'elbow_yaw_l',
    'elbow_pitch_l',
    'shoulder_pitch_r',
    'shoulder_roll_r',
    'elbow_yaw_r',
    'elbow_pitch_r',
]

# Joints controlled by the walking pipeline (legs only)
LEG_JOINT_NAMES = [
    'hip_pitch_l', 'hip_pitch_r',
    'thigh_roll_l', 'thigh_roll_r',
    'thigh_yaw_l', 'thigh_yaw_r',
    'knee_pitch_l', 'knee_pitch_r',
    'ankle_pitch_l', 'ankle_pitch_r',
    'ankle_roll_l', 'ankle_roll_r',
]

# Default neutral arm positions [rad]
# Arms slightly lowered to the sides for natural standing
DEFAULT_ARM_ANGLES = {
    'shoulder_pitch_l': 0.0,
    'shoulder_roll_l':  0.3,   # slightly abducted
    'elbow_yaw_l':      0.0,
    'elbow_pitch_l':    0.0,
    'shoulder_pitch_r': 0.0,
    'shoulder_roll_r': -0.3,   # slightly abducted (negative for right)
    'elbow_yaw_r':      0.0,
    'elbow_pitch_r':    0.0,
}

# Default torso angles [rad] — upright
DEFAULT_TORSO_ANGLES = {
    'torso_pitch': 0.0,
    'torso_roll':  0.0,
    'torso_yaw':   0.0,
}


class TrajectoryPublisher:
    """
    Publishes JointTrajectory messages for the Angad humanoid.

    Handles the mapping from IK-computed leg angles to full 23-joint
    trajectory messages, filling in neutral positions for arms and torso.

    Parameters:
        node:  ROS 2 node instance (must be already initialized)
        topic: topic name for the JointTrajectoryController
               (default: '/joint_trajectory_controller/joint_trajectory')
    """

    def __init__(
        self,
        node: Node,
        topic: str = '/joint_trajectory_controller/joint_trajectory',
    ):
        self.node = node
        self.publisher = node.create_publisher(JointTrajectory, topic, 10)

        # Store the last published angles for smooth interpolation
        self.last_angles: Dict[str, float] = {}
        for name in ALL_JOINT_NAMES:
            self.last_angles[name] = 0.0

        # Initialize arm and torso defaults
        self.last_angles.update(DEFAULT_ARM_ANGLES)
        self.last_angles.update(DEFAULT_TORSO_ANGLES)

        self.node.get_logger().info(
            f"TrajectoryPublisher initialized on topic: {topic}"
        )
        self.node.get_logger().info(
            f"  Controlling {len(ALL_JOINT_NAMES)} joints"
        )

    def publish_joint_angles(
        self,
        leg_angles: Dict[str, float],
        duration_sec: float = 0.05,
        torso_angles: Optional[Dict[str, float]] = None,
        arm_angles: Optional[Dict[str, float]] = None,
    ):
        """
        Publish a single trajectory point with the given joint angles.

        Args:
            leg_angles:   dict of {joint_name: angle_rad} for leg joints (from IK)
            duration_sec: time_from_start for this trajectory point [s]
            torso_angles: optional dict of torso angles (default: upright)
            arm_angles:   optional dict of arm angles (default: neutral)
        """
        msg = JointTrajectory()
        msg.joint_names = ALL_JOINT_NAMES

        # ── Build the full position vector ──
        positions = []
        for name in ALL_JOINT_NAMES:
            if name in leg_angles:
                angle = leg_angles[name]
            elif torso_angles and name in torso_angles:
                angle = torso_angles[name]
            elif arm_angles and name in arm_angles:
                angle = arm_angles[name]
            elif name in DEFAULT_ARM_ANGLES:
                angle = DEFAULT_ARM_ANGLES[name]
            elif name in DEFAULT_TORSO_ANGLES:
                angle = DEFAULT_TORSO_ANGLES[name]
            else:
                angle = self.last_angles.get(name, 0.0)

            positions.append(float(angle))
            self.last_angles[name] = angle

        # ── Create trajectory point ──
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(ALL_JOINT_NAMES)  # zero velocity targets
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1.0) * 1e9),
        )

        msg.points = [point]

        # ── Publish ──
        self.publisher.publish(msg)

    def publish_multi_point_trajectory(
        self,
        trajectory_points: List[Dict[str, float]],
        time_steps: List[float],
        torso_angles: Optional[Dict[str, float]] = None,
        arm_angles: Optional[Dict[str, float]] = None,
    ):
        """
        Publish a multi-point trajectory for smoother motion.

        Args:
            trajectory_points: list of dicts {joint_name: angle_rad}
            time_steps:        list of time_from_start values [s]
            torso_angles:      optional torso angles (same for all points)
            arm_angles:        optional arm angles (same for all points)
        """
        if len(trajectory_points) != len(time_steps):
            self.node.get_logger().error(
                "Mismatch: trajectory_points and time_steps must have same length!"
            )
            return

        msg = JointTrajectory()
        msg.joint_names = ALL_JOINT_NAMES

        for leg_angles, t in zip(trajectory_points, time_steps):
            positions = []
            for name in ALL_JOINT_NAMES:
                if name in leg_angles:
                    angle = leg_angles[name]
                elif torso_angles and name in torso_angles:
                    angle = torso_angles[name]
                elif arm_angles and name in arm_angles:
                    angle = arm_angles[name]
                elif name in DEFAULT_ARM_ANGLES:
                    angle = DEFAULT_ARM_ANGLES[name]
                elif name in DEFAULT_TORSO_ANGLES:
                    angle = DEFAULT_TORSO_ANGLES[name]
                else:
                    angle = self.last_angles.get(name, 0.0)

                positions.append(float(angle))
                self.last_angles[name] = angle

            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * len(ALL_JOINT_NAMES)
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1.0) * 1e9),
            )
            msg.points.append(point)

        self.publisher.publish(msg)
        self.node.get_logger().debug(
            f"Published trajectory with {len(msg.points)} points"
        )

    def publish_standing_pose(
        self,
        leg_angles: Dict[str, float],
        duration_sec: float = 2.0,
    ):
        """
        Publish a standing pose with a longer duration for smooth transition.

        Args:
            leg_angles:   dict of leg joint angles for the standing pose
            duration_sec: time to reach this pose [s] (default: 2.0)
        """
        self.node.get_logger().info(
            f"Publishing standing pose (transition: {duration_sec:.1f}s)"
        )
        self.publish_joint_angles(leg_angles, duration_sec=duration_sec)
