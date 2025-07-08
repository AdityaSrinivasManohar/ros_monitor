"""ROS 2 monitor utility functions and classes."""

import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from rosidl_runtime_py.utilities import get_message

ROS_MESSAGE = Any


class Ros2Monitor(Node):
    """A ROS 2 node that monitors available topics and their types."""

    def __init__(self) -> None:
        """Initialize the Ros2Monitor node and start the topic monitoring thread."""
        super().__init__("ros2monitor")
        self.topics_and_types = {}
        self.type_maps = {}
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._update_topics_loop, daemon=True)
        self._thread.start()

    def _update_topics_loop(self) -> None:
        while not self._stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)  # Process ROS events
            self.get_topics()
            time.sleep(1)  # Update every second

    def stop(self) -> None:
        """Stop the topic monitoring thread and wait for it to finish."""
        self._stop_event.set()
        self._thread.join()

    def get_topics(self) -> None:
        """Update the list of available ROS topics and their types.

        This method queries the ROS system for all currently available topics and their types,
        updates the internal mapping of topics to types.
        """
        updated_topics = {}
        topics_and_types = self.get_topic_names_and_types()
        for topic, topic_type in topics_and_types:
            if topic not in updated_topics:
                updated_topics[topic] = topic_type
                self.type_maps[topic] = get_message(topic_type[0]) if topic_type else None
        self.topics_and_types = updated_topics

    def print_topics(self) -> None:
        """Log the list of available ROS topics and their types."""
        for topic, topic_type in self.topics_and_types.items():
            print(f"Topic: {topic}, Type: {topic_type}")


def get_single_message(
    topic: str, topic_type: ROS_MESSAGE, node: Node | None = None, timeout_sec: float | None = None,
) -> ROS_MESSAGE:
    """Get a single message for this topic.

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


def main(args: list[str] | None = None) -> None:
    """Initialize ROS 2, start the Ros2Monitor, and print topics and messages for debugging.

    Args:
        args: Optional list of arguments to pass to rclpy.init().

    This function runs a loop to print available topics and attempts to receive a message from '/chatter'.

    """
    rclpy.init(args=args)
    ros2monitor = Ros2Monitor()
    echo_node = rclpy.create_node("message_subscriber")
    try:
        for i in range(20):
            print(i)
            ros2monitor.print_topics()
            time.sleep(1)
            try:
                print(
                    get_single_message(
                        "/chatter",
                        ros2monitor.type_maps.get("/chatter"),
                        timeout_sec=1,
                        node=echo_node,
                    ),
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
