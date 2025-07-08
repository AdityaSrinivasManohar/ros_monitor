"""Entry point for the ROS 2 monitor application."""

from ui import Ros2MonitorApp


def main() -> None:
    """Start the ROS 2 monitor application."""
    Ros2MonitorApp().run()


if __name__ == "__main__":
    main()
