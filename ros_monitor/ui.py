from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Header, Footer
from textual.reactive import reactive
from textual.widgets import DataTable
from ros_utils import Ros2Monitor
import rclpy
import threading
import time
from logging_config import logger
from widgets.topic_table import TopicTable

BINDINGS = [
    Binding("ctrl+c,q", "quit", "Quit", show=True, system=False),
    Binding("d", "toggle_dark", "Toggle dark mode"),
]

class Ros2MonitorApp(App):
    BINDINGS = BINDINGS
    topics = reactive([])  # List of (topic, type) tuples

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.ros_monitor = None
        logger.info("Ros2MonitorApp started.")

    def compose(self) -> ComposeResult:
        yield Header()
        self.table = TopicTable()
        yield self.table
        yield Footer()

    def on_mount(self):
        def start_ros():
            try:
                rclpy.init()
                self.ros_monitor = Ros2Monitor()
                while True:
                    self.topics = list(self.ros_monitor.topics_and_types.items())
                    time.sleep(1)
            except Exception as e:
                logger.exception(f"Exception in start_ros: {e}")
        threading.Thread(target=start_ros, daemon=True).start()

    def on_data_table_row_highlighted(self, event: DataTable.RowHighlighted) -> None:
        self.table.highlighted_row_key = event.row_key

    def watch_topics(self, topics):
        self.table.update_topics(topics)

    def action_quit(self) -> None:
        """Safely quit the application, cleaning up ROS and threads."""
        logger.info("Quitting Ros2MonitorApp...")
        if self.ros_monitor is not None:
            try:
                self.ros_monitor.stop()
                self.ros_monitor.destroy_node()
                rclpy.shutdown()
                logger.info("ROS shutdown complete.")
            except Exception as e:
                logger.exception(f"Error during ROS shutdown: {e}")
        logger.info("Exiting application.")
        self.exit()

if __name__ == "__main__":
    Ros2MonitorApp().run()