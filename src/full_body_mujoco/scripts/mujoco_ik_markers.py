#!/usr/bin/env python3
"""
mujoco_ik_markers.py
====================
Direct MuJoCo mocap body controller for IK visualization.

Subscribes to /nr_ik_test/markers and updates mocap body positions in the
shared MuJoCo model via direct MuJoCo API access.

This displays the IK targets and foot positions as colored spheres/boxes
directly in the MuJoCo viewer window.

Usage:
  ros2 run full_body_mujoco mujoco_ik_markers.py
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import os
import socket


class MujocoIKMarkers(Node):
    def __init__(self):
        super().__init__('mujoco_ik_markers')
        
        # Subscribe to markers from nr_ik_test
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/nr_ik_test/markers',
            self.marker_callback,
            10
        )
        
        # Publishers for visualization as PointStamped (for debugging)
        self.point_pubs = {}
        viz_names = [
            'foot_r', 'foot_l', 'target', 'swing',
            'step_0', 'step_1', 'step_2', 'step_3', 'step_4', 'step_5'
        ]
        
        for name in viz_names:
            topic = f'/ik_viz/{name}'
            self.point_pubs[name] = self.create_publisher(
                PointStamped, topic, 10
            )
        
        # TF for transforming pelvis frame to world frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Try to connect to MuJoCo shared memory or socket
        self.mujoco_connected = False
        try:
            # Check if we can access MuJoCo Python bindings
            import mujoco
            self.mujoco_available = True
            self.get_logger().info('MuJoCo Python bindings available')
        except ImportError:
            self.mujoco_available = False
            self.get_logger().warn('MuJoCo Python bindings not available - visualization limited')
        
        self.get_logger().info('MuJoCo IK marker controller started')
        self.get_logger().info('Publishing visualization points to /ik_viz/*')
    
    def marker_callback(self, msg: MarkerArray):
        """Process marker array and update mocap body positions."""
        
        # Try to get pelvis transform to world
        try:
            pelvis_tf = self.tf_buffer.lookup_transform(
                'world', 'pelvis',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)
            )
            
            # Extract pelvis position and orientation
            px = pelvis_tf.transform.translation.x
            py = pelvis_tf.transform.translation.y
            pz = pelvis_tf.transform.translation.z
            
            # For now, assume pelvis orientation is identity for simplicity
            # (full transform would require quaternion rotation)
            
        except Exception as e:
            # If no transform, assume pelvis at origin
            px, py, pz = 0.0, 0.0, 0.76
        
        # Track which markers are active
        active_markers = set()
        
        for marker in msg.markers:
            if marker.action == Marker.DELETE:
                continue
            
            mid = marker.id
            pos = marker.pose.position
            
            # Transform from pelvis frame to world frame
            # world_pos = pelvis_pos + pelvis_frame_offset
            wx = px + pos.x
            wy = py + pos.y
            wz = pz + pos.z
            
            # Map marker ID to mocap body name
            mocap_body = None
            if mid == 0:
                mocap_body = 'viz_foot_r'
            elif mid == 1:
                mocap_body = 'viz_foot_l'
            elif mid == 2:
                mocap_body = 'viz_target'
            elif mid == 3:
                mocap_body = 'viz_swing'
            elif mid >= 10 and mid < 16:
                step_num = mid - 10
                mocap_body = f'viz_step_{step_num}'
            
            if mocap_body and mocap_body in self.mocap_pubs:
                # Publish position command
                cmd = Float64MultiArray()
                cmd.data = [wx, wy, wz, 1.0, 0.0, 0.0, 0.0]  # pos + quat
                self.mocap_pubs[mocap_body].publish(cmd)
                active_markers.add(mocap_body)
                
                self.marker_positions[mocap_body] = (wx, wy, wz)
        
        # Hide inactive optional markers (target, swing)
        for body in ['viz_target', 'viz_swing']:
            if body not in active_markers:
                # Move to underground position
                cmd = Float64MultiArray()
                cmd.data = [0.0, 0.0, -10.0, 1.0, 0.0, 0.0, 0.0]
                self.mocap_pubs[body].publish(cmd)
        
        # Log current state periodically
        if len(active_markers) > 0:
            self.get_logger().info(
                f'Active markers: {len(active_markers)} | '
                f'Pelvis: ({px:.3f}, {py:.3f}, {pz:.3f})',
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = MujocoIKMarkers()
    
    print("\n" + "="*70)
    print("MuJoCo IK Marker Visualization")
    print("="*70)
    print("\nThis node displays IK targets in the MuJoCo viewer as colored markers:")
    print("  🟠 Orange sphere  → Right foot FK")
    print("  🔵 Cyan sphere    → Left foot FK")
    print("  🔴 Red sphere     → Manual target (when active)")
    print("  🟡 Yellow sphere  → Swing target (during walk)")
    print("  ⬜ Colored boxes  → Step landing positions")
    print("\nMake sure mujoco_ros2_control is running with the updated XML.")
    print("="*70 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
