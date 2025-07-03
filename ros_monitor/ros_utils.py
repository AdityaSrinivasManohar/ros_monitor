"""ROS 2 TUI."""
import rclpy
from rclpy.node import Node
import threading
import time

class Ros2Monitor(Node):
    def __init__(self):
        super().__init__('ros2monitor')
        self.topics_and_types = {}
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
        self.topics_and_types = updated_topics

    def echo_topic(self, topic_name, msg_type):
        # Subscribe to the given topic and print messages
        def callback(msg):
            self.get_logger().info(f'Received: {msg}')
        self.create_subscription(msg_type, topic_name, callback, 10)
        self.get_logger().info(f'Subscribed to {topic_name} (type: {msg_type.__name__})')
        rclpy.spin(self)

    def print_topics(self):
        # self.get_logger().info('Current topics:')
        for topic, topic_type in self.topics_and_types.items():
            # self.get_logger().info(f'Topic: {topic}, Type: {type}')
            print(f'Topic: {topic}, Type: {topic_type}')

def main(args=None):
    rclpy.init(args=args)
    ros2monitor = Ros2Monitor()
    try:
        # Let the background thread run for a few seconds to collect topics
        for i in range(20):
            # rclpy.spin_once(ros2monitor, timeout_sec=1.0)
            # print("Discovered topics:", ros2monitor.topics)
            print(i)
            ros2monitor.print_topics()
            time.sleep(1)

        # Optionally, you could call echo_topic here if desired
    finally:
        ros2monitor.stop()
        ros2monitor.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    print("ROS 2 TUI Example")
    main()