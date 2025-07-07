"""ROS 2 TUI."""

import threading
import time
from threading import Event
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from std_msgs.msg import String
from rosidl_runtime_py.utilities import get_message

ROS_MESSAGE = Any


class Ros2Monitor(Node):
    def __init__(self):
        super().__init__("ros2monitor")
        self.topics_and_types = {}
        self.type_maps = {}
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._update_topics_loop, daemon=True)
        self._thread.start()

    def _update_topics_loop(self):
        while not self._stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)  # Process ROS events
            self.get_topics()
            time.sleep(1)  # Update every second

    def stop(self):
        self._stop_event.set()
        self._thread.join()

    def get_topics(self):
        updated_topics = {}
        topics_and_types = self.get_topic_names_and_types()
        for topic, topic_type in topics_and_types:
            if topic not in updated_topics:
                updated_topics[topic] = topic_type
                # self.topics_and_types[topic] = topic_type
                # self.get_logger().info(f'Topic: {topic}, Type: {type}')
                self.type_maps[topic] = get_message(topic_type[0]) if topic_type else None
        self.topics_and_types = updated_topics

    def print_topics(self):
        # self.get_logger().info('Current topics:')
        for topic, topic_type in self.topics_and_types.items():
            # self.get_logger().info(f'Topic: {topic}, Type: {type}')
            print(f"Topic: {topic}, Type: {topic_type}")


def get_single_message(
    topic: str, topic_type: ROS_MESSAGE, node: Optional[Node] = None, timeout_sec: Optional[float] = None
) -> ROS_MESSAGE:
    """Get a single message for this topic.

    Useful to grab camera_info, tf_static information once at startup.

    Args:
        topic: name of the topic
        topic_type: type of the topic, needed to setup the subscriber
        node: node used to create the temporary subscriber, defaults to None and creates a new temporary node
        timeout_sec: number of seconds to wait before timing out, default None waits indefinetly

    Returns:
        msg: first message received on this topic

    Raises:
        TimeoutError: timeout_sec is exceeded without receiving a message

    """
    if node is None:
        node = rclpy.create_node(f"{topic.replace('/', '_')}_single_message_subscriber")

    success, msg = wait_for_message(msg_type=topic_type, node=node, topic=topic, time_to_wait=timeout_sec)
    if not success:
        s = f"Got timeout after {timeout_sec} seconds"
        raise TimeoutError(s)

    return msg


def main(args=None):
    rclpy.init(args=args)
    ros2monitor = Ros2Monitor()
    echo_node = rclpy.create_node(f"message_subscriber")
    try:
        for i in range(20):
            print(i)
            ros2monitor.print_topics()
            time.sleep(1)
            try:
                print(
                    get_single_message("/chatter", ros2monitor.type_maps.get("/chatter"), timeout_sec=1, node=echo_node)
                )
            except TimeoutError as e:
                print(f"Timeout while waiting for message: {e}")

    finally:
        ros2monitor.stop()
        ros2monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    print("ROS 2 TUI Example")
    main()
