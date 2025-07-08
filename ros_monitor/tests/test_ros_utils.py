"""Pytest for ros_utils.py using the imu_publisher node."""

import subprocess
import sys
import time

import pytest
import rclpy

from ros_monitor.utils.ros_utils import Ros2Monitor, get_single_message


@pytest.fixture(scope="module")
def imu_publisher_proc() -> subprocess.Popen:
    """Fixture to start and stop the imu_publisher node as a subprocess.

    Yields:
        subprocess.Popen: The process running the imu_publisher node.

    """
    proc = subprocess.Popen([sys.executable, "-m", "ros_monitor.scripts.imu_publisher"])
    time.sleep(2)
    yield proc
    proc.terminate()
    proc.wait()


@pytest.fixture(scope="module")
def ros2_monitor() -> Ros2Monitor:
    """Fixture to initialize and yield a Ros2Monitor instance, then clean up after use.

    Yields:
        Ros2Monitor: The initialized ROS 2 monitor instance.

    """
    rclpy.init()
    monitor = Ros2Monitor()
    time.sleep(2)
    yield monitor
    monitor.stop()
    monitor.destroy_node()
    rclpy.shutdown()


def test_imu_topic_and_message(imu_publisher_proc: subprocess.Popen, ros2_monitor: Ros2Monitor) -> None:  # noqa: ARG001
    """Test that the /imu/data topic is published and a message can be received.

    Args:
        imu_publisher_proc: The process running the imu_publisher node.
        ros2_monitor: The initialized ROS 2 monitor instance.

    Raises:
        AssertionError: If the topic or message is not found.

    """
    ros2_monitor.print_topics()
    assert "/imu/data" in ros2_monitor.topics_and_types, "imu/data topic not found!"
    msg_type = ros2_monitor.type_maps["/imu/data"]
    assert msg_type is not None, "/imu/data type not resolved!"
    msg = get_single_message("/imu/data", msg_type, node=ros2_monitor, timeout_sec=3)
    assert msg is not None, "No message received from /imu/data!"
    print("Received message:", msg)
