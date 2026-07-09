from full_body_hardware.msg import FootSensor, FootSensorArray
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

SENSOR_POSITIONS = {
    'foot_l_heel_1': (0.0, -0.048, -0.009),
    'foot_l_heel_2': (0.015, -0.048, -0.009),
    'foot_l_toe_1': (0.0, 0.075, -0.009),
    'foot_l_toe_2': (-0.015, 0.075, -0.009),
    'foot_r_heel_1': (0.0, -0.048, -0.009),
    'foot_r_heel_2': (-0.015, -0.048, -0.009),
    'foot_r_toe_1': (0.0, 0.075, -0.009),
    'foot_r_toe_2': (0.015, 0.075, -0.009),
}

class FootForceArrayPublisher(Node):
    def __init__(self):
        super().__init__('foot_force_array_publisher')
        self.sensor_topics = [
            ('foot_l_heel_1', '/foot_l_heel_site_wrench_sensor/wrench'),
            ('foot_l_heel_2', '/foot_l_heel_site_2_wrench_sensor/wrench'),
            ('foot_l_toe_1', '/foot_l_toe_site_wrench_sensor/wrench'),
            ('foot_l_toe_2', '/foot_l_toe_site_2_wrench_sensor/wrench'),
            ('foot_r_heel_1', '/foot_r_heel_site_wrench_sensor/wrench'),
            ('foot_r_heel_2', '/foot_r_heel_site_2_wrench_sensor/wrench'),
            ('foot_r_toe_1', '/foot_r_toe_site_wrench_sensor/wrench'),
            ('foot_r_toe_2', '/foot_r_toe_site_2_wrench_sensor/wrench'),
        ]
        self.forces = {label: 0.0 for label, _ in self.sensor_topics}
        for label, topic in self.sensor_topics:
            self.create_subscription(WrenchStamped, topic, self.make_cb(label), 10)
        self.left_pub = self.create_publisher(FootSensorArray, 'foot_force_left', 10)
        self.right_pub = self.create_publisher(FootSensorArray, 'foot_force_right', 10)
        self.timer = self.create_timer(0.02, self.publish_arrays)

    def make_cb(self, label):
        def cb(msg):
            self.forces[label] = msg.wrench.force.z
        return cb

    def publish_arrays(self):
        left_labels = ['foot_l_heel_1', 'foot_l_heel_2', 'foot_l_toe_1', 'foot_l_toe_2']
        right_labels = ['foot_r_heel_1', 'foot_r_heel_2', 'foot_r_toe_1', 'foot_r_toe_2']

        left_array = FootSensorArray()
        right_array = FootSensorArray()
        for label in left_labels:
            force = self.forces[label]
            x, y, z = SENSOR_POSITIONS[label]
            sensor = FootSensor(label=label, force=force, x=x, y=y, z=z)
            left_array.sensors.append(sensor)
        for label in right_labels:
            force = self.forces[label]
            x, y, z = SENSOR_POSITIONS[label]
            sensor = FootSensor(label=label, force=force, x=x, y=y, z=z)
            right_array.sensors.append(sensor)
        self.left_pub.publish(left_array)
        self.right_pub.publish(right_array)

def main(args=None):
    rclpy.init(args=args)
    node = FootForceArrayPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
