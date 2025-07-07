"""Pretty print a ROS message and its fields."""
from typing import Any

import rclpy
from sensor_msgs.msg import Imu

from ros_monitor.utils.ros_utils import get_single_message


def pretty_print_imu(msg: Any, indent:int = 0) -> str:
    """Recursively prints a ROS message and its fields, removing leading underscores.

    Returns the formatted string.
    """
    prefix = " " * indent
    lines = []
    if hasattr(msg, "__slots__"):
        for field in msg.__slots__:
            value = getattr(msg, field)
            field_name = field.lstrip("_")
            if hasattr(value, "__slots__"):
                lines.append(f"{prefix}{field_name}:")
                lines.append(pretty_print_imu(value, indent + 2))
            else:
                lines.append(f"{prefix}{field_name}: {value}")
    else:
        lines.append(f"{prefix}{msg}")
    return "\n".join(lines)


def main() -> None:
    """Subscribe to the IMU topic and pretty print the messages."""
    rclpy.init()
    node = rclpy.create_node("pretty_print_imu")
    imu_topic = "/imu/data"

    # Get a single message from the IMU topic
    imu_msg = get_single_message(imu_topic, Imu, node=node, timeout_sec=5)

    if imu_msg:
        output = pretty_print_imu(imu_msg)
        node.get_logger().info(output)
    else:
        node.get_logger().error("No IMU message received.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
