#!/usr/bin/env python3
"""
mujoco_marker_sync.py
=====================
Subscribe to /nr_ik_test/markers and update MuJoCo mocap bodies to visualize
the IK targets and foot positions in the MuJoCo viewer.

This bridges RViz2 visualization markers to MuJoCo mocap bodies:
  - ID 0: Right foot FK  → viz_foot_r (orange)
  - ID 1: Left foot FK   → viz_foot_l (cyan)
  - ID 2: Manual target  → viz_target (red)
  - ID 3: Swing target   → viz_swing (yellow)
  - ID 10+: Step landing → viz_step_N (alternating colors)

Usage:
  ros2 run full_body_mujoco mujoco_marker_sync.py

Requirements:
  - angad_full_body.xml must have the mocap bodies defined
  - mujoco_ros2_control must be running
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import numpy as np


class MujocoMarkerSync(Node):
    def __init__(self):
        super().__init__('mujoco_marker_sync')
        
        # Subscribe to markers from nr_ik_test
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/nr_ik_test/markers',
            self.marker_callback,
            10
        )
        
        # TF for transforming pelvis frame to world frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Track marker visibility (to hide when not in use)
        self.marker_active = {
            'viz_target': False,
            'viz_swing': False,
        }
        
        self.get_logger().info('MuJoCo marker sync started')
        self.get_logger().info('Subscribing to /nr_ik_test/markers')
        self.get_logger().info('Publishing TF transforms for mocap bodies')
    
    def marker_callback(self, msg: MarkerArray):
        """Process marker array and publish TF transforms for mocap bodies."""
        
        # Try to get pelvis transform to world
        try:
            pelvis_tf = self.tf_buffer.lookup_transform(
                'world', 'pelvis',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            # If no transform yet, assume pelvis is at origin
            pelvis_tf = TransformStamped()
            pelvis_tf.header.frame_id = 'world'
            pelvis_tf.child_frame_id = 'pelvis'
            pelvis_tf.transform.rotation.w = 1.0
        
        # Reset active flags
        marker_seen = {'viz_target': False, 'viz_swing': False}
        
        for marker in msg.markers:
            if marker.action == Marker.DELETE:
                continue
            
            mid = marker.id
            pos = marker.pose.position
            
            # Create TF transform in pelvis frame
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'pelvis'
            
            # Map marker ID to mocap body name
            if mid == 0:
                # Right foot FK - orange
                tf_msg.child_frame_id = 'viz_foot_r'
            elif mid == 1:
                # Left foot FK - cyan
                tf_msg.child_frame_id = 'viz_foot_l'
            elif mid == 2:
                # Manual target - red
                tf_msg.child_frame_id = 'viz_target'
                marker_seen['viz_target'] = True
            elif mid == 3:
                # Swing target - yellow
                tf_msg.child_frame_id = 'viz_swing'
                marker_seen['viz_swing'] = True
            elif mid >= 10 and mid < 16:
                # Step landing targets
                step_num = mid - 10
                tf_msg.child_frame_id = f'viz_step_{step_num}'
            else:
                continue
            
            # Set position from marker
            tf_msg.transform.translation.x = pos.x
            tf_msg.transform.translation.y = pos.y
            tf_msg.transform.translation.z = pos.z
            tf_msg.transform.rotation.w = 1.0
            
            # Broadcast TF
            self.tf_broadcaster.sendTransform(tf_msg)
        
        # Hide inactive markers by moving them far away
        for name, seen in marker_seen.items():
            if not seen and self.marker_active.get(name, False):
                # Hide by moving to z=-10
                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'world'
                tf_msg.child_frame_id = name
                tf_msg.transform.translation.z = -10.0
                tf_msg.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(tf_msg)
            self.marker_active[name] = seen


def main(args=None):
    rclpy.init(args=args)
    
    node = MujocoMarkerSync()
    
    print("\n" + "="*70)
    print("MuJoCo Marker Synchronization")
    print("="*70)
    print("\nThis node updates MuJoCo mocap bodies to visualize IK targets.")
    print("\nMarker mapping:")
    print("  • Orange sphere   → Right foot FK position")
    print("  • Cyan sphere     → Left foot FK position")
    print("  • Red sphere      → Manual IK target (when set)")
    print("  • Yellow sphere   → Swing foot target (during walk)")
    print("  • Colored boxes   → Step landing positions")
    print("\nThe markers are visible in the MuJoCo viewer window.")
    print("Use the viewer controls to rotate/zoom the camera.")
    print("="*70 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
