from textual.screen import Screen
from textual.widgets import Static, Footer, Header
from textual.binding import Binding
from textual.containers import Vertical
from textual.reactive import reactive
from utils.ros_utils import get_single_message
from utils.logging_config import logger

ECHO_REFRESH_RATE = 1 / 10  # 10 Hz refresh rate


def pretty_print_imu(msg, indent=0):
    """
    Recursively pretty print a ROS message and its fields, removing leading underscores.
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


class TopicEchoScreen(Screen):
    BINDINGS = [Binding("escape", "pop_screen", "Back")]

    msg = reactive("")

    def __init__(self, topic_name, ros_monitor, **kwargs):
        super().__init__(**kwargs)
        self.topic_name = topic_name
        self.ros_monitor = ros_monitor
        self.echo_widget = Static(f"Echo for {topic_name}:\n", id="echo")
        self._stop = False

    def compose(self):
        yield Header()
        yield self.echo_widget
        yield Footer()

    async def on_mount(self):
        self.set_interval(ECHO_REFRESH_RATE, self.fetch_msg)

    async def fetch_msg(self):
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

    def watch_msg(self, msg):
        self.echo_widget.update(msg)

    async def action_pop_screen(self):
        self._stop = True
        await self.app.pop_screen()
