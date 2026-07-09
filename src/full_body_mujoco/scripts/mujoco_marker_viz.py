#!/usr/bin/env python3
"""
mujoco_marker_viz.py
====================
Subscribe to /nr_ik_test/markers and visualize them in MuJoCo viewer.

Displays:
  - ID 0: Right foot FK (orange sphere)
  - ID 1: Left foot FK (cyan sphere)
  - ID 2: Manual target (red sphere)
  - ID 3: Swing target (yellow sphere)
  - ID 10+: Step landing targets (white cubes)

Usage:
  ros2 run full_body_mujoco mujoco_marker_viz.py

This script listens to the marker array published by nr_ik_test and adds
visual geoms to the MuJoCo scene via the /mujoco/set_geom_pose service.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import numpy as np

class MujocoMarkerViz(Node):
    def __init__(self):
        super().__init__('mujoco_marker_viz')
        
        # Subscribe to markers from nr_ik_test
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/nr_ik_test/markers',
            self.marker_callback,
            10
        )
        
        # TF buffer for transforming from pelvis to world frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('MuJoCo marker visualizer started')
        self.get_logger().info('Subscribing to /nr_ik_test/markers')
        self.get_logger().info('Note: This displays marker info in terminal.')
        self.get_logger().info('For full MuJoCo visualization, see instructions below.')
    
    def marker_callback(self, msg: MarkerArray):
        """Process marker array and log positions."""
        
        marker_names = {
            0: 'Right Foot FK',
            1: 'Left Foot FK',
            2: 'Manual Target',
            3: 'Swing Target'
        }
        
        # Try to get pelvis transform to world for reference
        try:
            transform = self.tf_buffer.lookup_transform(
                'world', 'pelvis',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)
            )
            pelvis_z = transform.transform.translation.z
        except:
            pelvis_z = None
        
        output_lines = []
        output_lines.append('=' * 60)
        if pelvis_z is not None:
            output_lines.append(f'Pelvis height: {pelvis_z:.3f}m')
        
        for marker in msg.markers:
            if marker.action == Marker.DELETE:
                continue
                
            mid = marker.id
            pos = marker.pose.position
            
            # Get marker description
            if mid < 10:
                name = marker_names.get(mid, f'Unknown {mid}')
                color = f'rgba({marker.color.r:.1f},{marker.color.g:.1f},{marker.color.b:.1f},{marker.color.a:.1f})'
                output_lines.append(f'{name:18s}: ({pos.x:6.3f}, {pos.y:6.3f}, {pos.z:6.3f}) {color}')
            elif mid >= 10:
                # Step landing targets
                step_num = mid - 10
                side = 'Right' if step_num % 2 == 0 else 'Left'
                output_lines.append(f'Step {step_num} ({side:5s}): ({pos.x:6.3f}, {pos.y:6.3f}, {pos.z:6.3f})')
        
        output_lines.append('=' * 60)
        
        # Print all at once to avoid interleaving
        self.get_logger().info('\n' + '\n'.join(output_lines), throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    
    node = MujocoMarkerViz()
    
    print("\n" + "="*70)
    print("MuJoCo Marker Visualization Helper")
    print("="*70)
    print("\nThis node displays marker positions in the terminal.")
    print("\nTo visualize markers IN the MuJoCo viewer window, you have two options:")
    print("\n1. SIMPLE: Use MuJoCo's built-in contact/perturbation display:")
    print("   - Press 'C' in MuJoCo viewer to show contact points")
    print("   - Press 'H' to show help with all keyboard shortcuts")
    print("   - Press 'T' to toggle transparent mode")
    print("\n2. ADVANCED: Add mocap bodies to the MuJoCo XML:")
    print("   Add to <worldbody> in angad_full_body.xml:")
    print("   <body name='viz_target' mocap='true' pos='0 0 0'>")
    print("     <geom type='sphere' size='0.05' rgba='1 0 0 0.6' contype='0' conaffinity='0'/>")
    print("   </body>")
    print("   Then use mujoco_ros2_control API to update mocap positions")
    print("="*70 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
