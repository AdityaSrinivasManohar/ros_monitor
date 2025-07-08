#!/usr/bin/env python3
"""A ROS2 node that publishes simulated IMU (Inertial Measurement Unit) data at 10 Hz."""

import math
import time
from typing import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


class ImuPublisher(Node):
    """A ROS2 node that publishes simulated IMU (Inertial Measurement Unit) data at 10 Hz."""

    def __init__(self) -> None:
        """Initialize the ImuPublisher node and set up the IMU data publisher and timer."""
        super().__init__("imu_publisher")
        self.publisher_ = self.create_publisher(Imu, "imu/data", 10)
        self.timer = self.create_timer(0.1, self.publish_imu_msg)  # 10 Hz

    def publish_imu_msg(self) -> None:
        """Publish a simulated IMU message with example orientation, angular velocity, and linear acceleration."""
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
        self.get_logger().info("Publishing IMU data")




def main(args: Sequence[str] | None = None) -> None:
    """Initialize the ROS2 Python client library, start the ImuPublisher node, and handle shutdown.

    Args:
        args (Optional[Sequence[str]]): Command-line arguments for ROS2 initialization.

    """
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
