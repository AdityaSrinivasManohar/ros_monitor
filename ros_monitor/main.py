from ui import Ros2MonitorApp

def main():
    """
    Main entry point for the ROS 2 monitor application.
    Initializes the ROS 2 environment and starts the TUI application.
    """
    Ros2MonitorApp().run()

if __name__ == "__main__":
    main()