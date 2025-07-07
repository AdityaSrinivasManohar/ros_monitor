"""Pytest for ros_utils.py using the imu_publisher node."""
import subprocess
import sys
import time
import rclpy
import pytest
from utils.ros_utils import Ros2Monitor, get_single_message

@pytest.fixture(scope="module")
def imu_publisher_proc():
    proc = subprocess.Popen([sys.executable, "-m", "ros_monitor.scripts.imu_publisher"])
    time.sleep(2)
    yield proc
    proc.terminate()
    proc.wait()

@pytest.fixture(scope="module")
def ros2_monitor():
    rclpy.init()
    monitor = Ros2Monitor()
    time.sleep(2)
    yield monitor
    monitor.stop()
    monitor.destroy_node()
    rclpy.shutdown()

def test_imu_topic_and_message(imu_publisher_proc, ros2_monitor):
    ros2_monitor.print_topics()
    assert "imu/data" in ros2_monitor.topics_and_types, "imu/data topic not found!"
    msg_type = ros2_monitor.type_maps["imu/data"]
    assert msg_type is not None, "imu/data type not resolved!"
    msg = get_single_message("imu/data", msg_type, node=ros2_monitor, timeout_sec=3)
    assert msg is not None, "No message received from imu/data!"
    print("Received message:", msg)
