from textual.screen import Screen
from textual.widgets import Static, Button, Footer, Header
from textual.binding import Binding
from textual.containers import Vertical
from textual.reactive import reactive
from ros_utils import get_single_message

ECHO_REFRESH_RATE = 1/10  # 10 Hz refresh rate

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
        await self.fetch_msg()

    async def fetch_msg(self):
        if self._stop:
            return
        try:
            type_list = self.ros_monitor.topics_and_types.get(self.topic_name)
            if type_list:
                msg_type = self.ros_monitor.type_maps.get(self.topic_name)
                if msg_type is not None:
                    node = self.ros_monitor
                    try:
                        msg = get_single_message(self.topic_name, msg_type, node=node, timeout_sec=2)
                        self.msg = f"Echo for {self.topic_name}:\n{msg}"
                    except Exception as e:
                        self.msg = f"Error: {e}"
                else:
                    self.msg = "Could not resolve message type for this topic."
            else:
                self.msg = "Topic type not found."
        except Exception as e:
            self.msg = f"Error: {e}"

    def watch_msg(self, msg):
        self.echo_widget.update(msg)

    async def action_pop_screen(self):
        self._stop = True
        await self.app.pop_screen()
