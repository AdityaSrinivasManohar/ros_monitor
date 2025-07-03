## To run
docker run -it -v /Users/adityasrinivas.mano/Projects/ros2tui:/ros2tui/ ros2tui

## To build
docker build --platform=linux/amd64 -t ros2tui -f .devcontainer/Dockerfile .

## Best case
Use a devcontainer!

## ROS 2 test commands
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener


