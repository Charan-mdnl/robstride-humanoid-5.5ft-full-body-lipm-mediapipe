#!/usr/bin/env python3
"""
simple_ik_viz.py
================
Simple IK target visualization - displays IK xyz coordinates clearly.

Shows where the IK solver wants the feet to go. Compare with actual foot
positions in MuJoCo to see convergence/error.

Usage:
  ros2 run full_body_mujoco simple_ik_viz.py
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener


class SimpleIKViz(Node):
    def __init__(self):
        super().__init__('simple_ik_viz')
        
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/nr_ik_test/markers',
            self.marker_callback,
            10
        )
        
        # TF to get actual foot positions
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Simple IK Visualizer started')
        print("\n" + "="*80)
        print("IK TARGET vs ACTUAL FOOT POSITIONS")
        print("="*80)
        print("Legend:")
        print("  TARGET = Where IK solver commands feet to go (from nr_ik_test)")
        print("  ACTUAL = Where feet actually are (from TF)")
        print("  ERROR  = Distance between target and actual")
        print("="*80)
    
    def marker_callback(self, msg: MarkerArray):
        """Display IK targets and compare with actual foot positions."""
        
        foot_r_target = None
        foot_l_target = None
        
        for marker in msg.markers:
            if marker.action == Marker.DELETE:
                continue
            
            pos = marker.pose.position
            
            if marker.id == 0:  # Right foot IK target
                foot_r_target = (pos.x, pos.y, pos.z)
            elif marker.id == 1:  # Left foot IK target
                foot_l_target = (pos.x, pos.y, pos.z)
        
        # Try to get actual foot positions from TF
        try:
            tf_r = self.tf_buffer.lookup_transform('pelvis', 'foot_r', rclpy.time.Time())
            foot_r_actual = (
                tf_r.transform.translation.x,
                tf_r.transform.translation.y,
                tf_r.transform.translation.z
            )
        except:
            foot_r_actual = None
        
        try:
            tf_l = self.tf_buffer.lookup_transform('pelvis', 'foot_l', rclpy.time.Time())
            foot_l_actual = (
                tf_l.transform.translation.x,
                tf_l.transform.translation.y,
                tf_l.transform.translation.z
            )
        except:
            foot_l_actual = None
        
        # Calculate errors
        if foot_r_target and foot_r_actual:
            err_r = ((foot_r_target[0] - foot_r_actual[0])**2 +
                     (foot_r_target[1] - foot_r_actual[1])**2 +
                     (foot_r_target[2] - foot_r_actual[2])**2)**0.5
        else:
            err_r = None
        
        if foot_l_target and foot_l_actual:
            err_l = ((foot_l_target[0] - foot_l_actual[0])**2 +
                     (foot_l_target[1] - foot_l_actual[1])**2 +
                     (foot_l_target[2] - foot_l_actual[2])**2)**0.5
        else:
            err_l = None
        
        # Print formatted output
        lines = []
        lines.append("\n" + "─" * 80)
        
        if foot_r_target:
            lines.append(f"RIGHT FOOT:")
            lines.append(f"  🎯 TARGET:  x={foot_r_target[0]:7.3f}  y={foot_r_target[1]:7.3f}  z={foot_r_target[2]:7.3f}")
            if foot_r_actual:
                lines.append(f"  🟠 ACTUAL:  x={foot_r_actual[0]:7.3f}  y={foot_r_actual[1]:7.3f}  z={foot_r_actual[2]:7.3f}")
                if err_r is not None:
                    lines.append(f"  📏 ERROR:   {err_r:.4f} m")
        
        if foot_l_target:
            lines.append(f"LEFT FOOT:")
            lines.append(f"  🎯 TARGET:  x={foot_l_target[0]:7.3f}  y={foot_l_target[1]:7.3f}  z={foot_l_target[2]:7.3f}")
            if foot_l_actual:
                lines.append(f"  🔵 ACTUAL:  x={foot_l_actual[0]:7.3f}  y={foot_l_actual[1]:7.3f}  z={foot_l_actual[2]:7.3f}")
                if err_l is not None:
                    lines.append(f"  📏 ERROR:   {err_l:.4f} m")
        
        lines.append("─" * 80)
        
        for line in lines:
            print(line)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleIKViz()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
