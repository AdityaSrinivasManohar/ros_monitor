# DEV Guide

## To run and mount files similarly to devcontainer
docker run -it -v /Users/adsm/Projects/ros2tui:/workspaces/ros2tui/ ros2tui

## To build
docker build --platform=linux/amd64 -t ros2tui -f .devcontainer/Dockerfile .

## For optimal dev environment
Use the devcontainer!

## ROS 2 test commands
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener

## Make sure to 
```
export PYTHONPATH=/workspaces/ros2tui:/opt/ros/jazzy/lib/python3.12/site-packages
```

## Testing
```
pytest
```