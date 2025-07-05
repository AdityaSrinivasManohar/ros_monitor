#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math
import time

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_msg)  # 10 Hz

    def publish_imu_msg(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Example orientation (quaternion)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(time.time() / 2.0)
        msg.orientation.w = math.cos(time.time() / 2.0)

        # Example angular velocity
        msg.angular_velocity.x = 0.1
        msg.angular_velocity.y = 0.2
        msg.angular_velocity.z = 0.3

        # Example linear acceleration
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 9.8
        msg.linear_acceleration.z = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing IMU data')

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()