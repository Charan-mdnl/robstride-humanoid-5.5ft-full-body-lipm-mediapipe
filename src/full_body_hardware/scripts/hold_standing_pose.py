#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

STANDING_POSE = {
    'hip_pitch_r': 0.0,
    'hip_roll_r': 0.0,
    'thigh_yaw_r': 0.0,
    'knee_pitch_r': -0.05,
    'ankle_pitch_r': 0.0,
    'ankle_roll_r': 0.0,
    'hip_pitch_l': 0.0,
    'hip_roll_l': 0.0,
    'thigh_yaw_l': 0.0,
    'knee_pitch_l': -0.05,
    'ankle_pitch_l': 0.0,
    'ankle_roll_l': 0.0,
    'torso_yaw': 0.0,
    'shoulder_pitch_r': 0.0,
    'shoulder_roll_r': 0.3,
    'elbow_yaw_r': 0.0,
    'elbow_pitch_r': 0.0,
    'shoulder_pitch_l': 0.0,
    'shoulder_roll_l': -0.3,
    'elbow_yaw_l': 0.0,
    'elbow_pitch_l': 0.0,
}

CONTROLLERS = {
    'lower_body_controller': [
        'hip_pitch_r','hip_roll_r','thigh_yaw_r',
        'knee_pitch_r','ankle_pitch_r','ankle_roll_r',
        'hip_pitch_l','hip_roll_l','thigh_yaw_l',
        'knee_pitch_l','ankle_pitch_l','ankle_roll_l',
    ],
    'torso_controller': ['torso_yaw'],
    'right_arm_controller': [
        'shoulder_pitch_r','shoulder_roll_r','elbow_yaw_r','elbow_pitch_r',
    ],
    'left_arm_controller': [
        'shoulder_pitch_l','shoulder_roll_l','elbow_yaw_l','elbow_pitch_l',
    ],
}

REACH_TIME_S = 0.5


class StandingPosePublisher(Node):
    def __init__(self):
        super().__init__('hold_standing_pose')

        self._pubs = {}
        for ctrl in CONTROLLERS:
            topic = f'/{ctrl}/joint_trajectory'
            self._pubs[ctrl] = self.create_publisher(
                JointTrajectory, topic, 10
            )

        self.timer = self.create_timer(0.5, self.send_once)
        self.sent = False

    def send_once(self):
        if self.sent:
            return

        for ctrl, joints in CONTROLLERS.items():
            msg = JointTrajectory()
            msg.joint_names = joints

            pt = JointTrajectoryPoint()
            pt.positions = [STANDING_POSE[j] for j in joints]
            pt.time_from_start.sec = 0
            pt.time_from_start.nanosec = int(REACH_TIME_S * 1e9)

            msg.points.append(pt)

            self._pubs[ctrl].publish(msg)
            self.get_logger().info(f'Published to {ctrl}')

        self.sent = True
        self.timer.cancel()


def main():
    rclpy.init()
    node = StandingPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()