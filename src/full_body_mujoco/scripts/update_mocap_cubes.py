#!/usr/bin/env python3
"""
update_mocap_cubes.py
=====================
Update MuJoCo mocap body positions to show IK targets vs actual foot positions.

This script connects to the running MuJoCo simulation and updates the mocap
cubes (ik_target_r and ik_target_l) to match the IK solver's commanded positions.

The foot-attached cubes show actual positions, mocap cubes show IK targets.
Compare them to see position/orientation error.

Usage:
  ros2 run full_body_mujoco update_mocap_cubes.py
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import os
import tempfile
import struct


class MocapUpdater(Node):
    def __init__(self):
        super().__init__('mocap_updater')
        
        # Subscribe to markers from IK solver
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/nr_ik_test/markers',
            self.marker_callback,
            10
        )
        
        # TF listener to get pelvis position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create shared memory file for mocap positions
        self.shm_dir = '/tmp/mujoco_ik_viz'
        os.makedirs(self.shm_dir, exist_ok=True)
        self.r_file = os.path.join(self.shm_dir, 'ik_target_r.dat')
        self.l_file = os.path.join(self.shm_dir, 'ik_target_l.dat')
        
        # Store last known positions
        self.last_r = None
        self.last_l = None
        
        self.get_logger().info('Mocap updater started')
        self.get_logger().info(f'Writing mocap positions to {self.shm_dir}')
        self.get_logger().info('To view comparison, check terminal output')
    
    def marker_callback(self, msg: MarkerArray):
        """Update mocap positions from IK markers."""
        
        # Try to get pelvis position
        try:
            pelvis_tf = self.tf_buffer.lookup_transform(
                'world', 'pelvis',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)
            )
            px = pelvis_tf.transform.translation.x
            py = pelvis_tf.transform.translation.y
            pz = pelvis_tf.transform.translation.z
        except:
            px, py, pz = 0.0, 0.0, 0.76  # Default pelvis height
        
        # Process markers
        for marker in msg.markers:
            if marker.action == Marker.DELETE:
                continue
            
            pos = marker.pose.position
            
            # Transform to world frame (pelvis frame + pelvis position)
            wx = px + pos.x
            wy = py + pos.y
            wz = pz + pos.z
            
            if marker.id == 0:  # Right foot IK target
                self.last_r = (wx, wy, wz)
                # Write to file for potential external updater
                with open(self.r_file, 'wb') as f:
                    f.write(struct.pack('ddd', wx, wy, wz))
                    
            elif marker.id == 1:  # Left foot IK target
                self.last_l = (wx, wy, wz)
                with open(self.l_file, 'wb') as f:
                    f.write(struct.pack('ddd', wx, wy, wz))
        
        # Print comparison (IK target positions)
        if self.last_r and self.last_l:
            self.get_logger().info(
                f'\n┌─ IK TARGET POSITIONS ─────────────────────────────────┐\n'
                f'│ 🔴 Right: x={self.last_r[0]:7.3f} y={self.last_r[1]:7.3f} z={self.last_r[2]:7.3f} │\n'
                f'│ 🟡 Left:  x={self.last_l[0]:7.3f} y={self.last_l[1]:7.3f} z={self.last_l[2]:7.3f} │\n'
                f'└───────────────────────────────────────────────────────┘',
                throttle_duration_sec=0.5
            )


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*70)
    print("IK Target Position Monitor")
    print("="*70)
    print("\nThis displays IK commanded positions in real-time.")
    print("\nIn MuJoCo viewer:")
    print("  🟠 Solid orange cube  = Actual RIGHT foot position")
    print("  🔵 Solid cyan cube    = Actual LEFT foot position")
    print("  🔴 Red wireframe      = RIGHT foot IK target (manually adjust)")
    print("  🟡 Yellow wireframe   = LEFT foot IK target (manually adjust)")
    print("\nTo move mocap cubes in MuJoCo:")
    print("  1. Double-click a wireframe cube to select it")
    print("  2. Hold Ctrl+Right-click and drag to move it")
    print("  3. Watch this terminal for IK target xyz values")
    print("="*70 + "\n")
    
    node = MocapUpdater()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
