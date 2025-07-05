"""main UI for the ROS monitor."""

import threading
import time

import rclpy
from utils.ros_utils import Ros2Monitor
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.reactive import reactive
from textual.widgets import DataTable, Footer, Header
from utils.logging_config import logger
from utils.version import get_version
from widgets.topic_table import TopicTable
from screens.topic_echo_screen import TopicEchoScreen

BINDINGS = [
    Binding("ctrl+c,q", "quit", "Quit", show=True, system=False),
    Binding("d", "toggle_dark", "Toggle dark mode"),
]


class Ros2MonitorApp(App):
    BINDINGS = BINDINGS
    topics = reactive([])

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.ros_monitor = None
        self.version = get_version()
        logger.info("Ros2MonitorApp started.")

    def compose(self) -> ComposeResult:
        yield Header()
        self.table = TopicTable()
        yield self.table
        yield Footer()

    def on_mount(self):
        self.title = "ROS Monitor"
        self.sub_title = f"Version: {self.version}"

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

    async def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        row_key = event.row_key
        topic = self.table.get_row(row_key)[0]
        logger.info(f"Selected topic: {topic}")
        if self.ros_monitor is not None:
            await self.push_screen(TopicEchoScreen(topic, self.ros_monitor))

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
