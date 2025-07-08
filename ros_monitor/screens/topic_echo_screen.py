"""Screen to display live echo of a ROS topic's messages."""
from typing import Any, ClassVar

from textual.binding import Binding
from textual.reactive import reactive
from textual.screen import Screen
from textual.widgets import Footer, Header, Static

from ros_monitor.utils.logging_config import logger
from ros_monitor.utils.ros_utils import get_single_message

ECHO_REFRESH_RATE = 1 / 10  # 10 Hz refresh rate
ROS_MESSAGE = Any



def pretty_print_imu(msg: ROS_MESSAGE, indent: int = 0) -> str:
    """Recursively pretty print a ROS message and its fields, removing leading underscores.

    Args:
        msg (Any): The ROS message to pretty print.
        indent (int): The number of spaces to indent each line for better readability.

    Returns:
        str: A formatted string representation of the ROS message.

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



class TopicEchoScreen(Screen):
    """A Textual screen that displays live echo of a ROS topic's messages.

    Attributes:
        topic_name (str): The name of the ROS topic to echo.
        ros_monitor (Any): The ROS monitor instance used to fetch messages.
        echo_widget (Static): Widget to display the echoed messages.
        _stop (bool): Flag to control stopping the message fetch loop.

    """

    BINDINGS: ClassVar = [Binding("escape", "pop_screen", "Back")]

    msg = reactive("")

    def __init__(self, topic_name: str, ros_monitor: Any, **kwargs: dict[str, Any]) -> None:
        """Initialize the TopicEchoScreen with the topic name and ROS monitor.

        Args:
            topic_name (str): The name of the ROS topic to echo.
            ros_monitor (Any): The ROS monitor instance used to fetch messages.
            **kwargs: Additional keyword arguments for the parent Screen class.

        """
        super().__init__(**kwargs)
        self.topic_name = topic_name
        self.ros_monitor = ros_monitor
        self.echo_widget = Static(f"Echo for {topic_name}:\n", id="echo")
        self._stop = False

    def compose(self) -> None:
        """Compose the screen."""
        yield Header()
        yield self.echo_widget
        yield Footer()

    async def on_mount(self) -> None:
        """Set up periodic message fetching."""
        self.set_interval(ECHO_REFRESH_RATE, self.fetch_msg)

    async def fetch_msg(self) -> None:
        """Fetch the latest message from the ROS topic and update the message."""
        if self._stop:
            return

        try:
            if not self.ros_monitor:
                self.msg = "ROS monitor is not initialized."
                logger.error(self.msg)
                return

            if self.topic_name not in self.ros_monitor.topics_and_types:
                self.msg = "Topic type not found."
                logger.error(self.msg)
                return

            msg_type = self.ros_monitor.type_maps.get(self.topic_name)
            if msg_type is None:
                self.msg = "Could not resolve message type for this topic."
                logger.error(self.msg)
                return

            try:
                msg = get_single_message(self.topic_name, msg_type, node=self.ros_monitor, timeout_sec=5)
                self.msg = pretty_print_imu(msg)
            except Exception as e:
                self.msg = f"Error fetching message: {e}"
                logger.exception(self.msg)

        except Exception as e:
            self.msg = f"Unexpected error: {e}"
            logger.exception(self.msg)

    def watch_msg(self, msg: str) -> None:
        """Update the echo widget with the latest message.

        Args:
            msg (str): The message to display in the echo widget.

        """
        self.echo_widget.update(msg)

    async def action_pop_screen(self) -> None:
        """Handle the action to pop the screen and stop message fetching."""
        self._stop = True
        await self.app.pop_screen()
