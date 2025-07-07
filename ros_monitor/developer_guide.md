# DEV Guide

## To run and mount files similarly to devcontainer
```
docker run -it -v ~/Projects/ros_monitor:/ros_monitor/ ros_monitor
```

## To build
```
docker build --platform=linux/amd64 -t ros_monitor -f .devcontainer/Dockerfile .
```

## For optimal dev environment
Use the devcontainer!

## ROS 2 test commands
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener

## Make sure to 
```
export PYTHONPATH=/workspaces/ros_monitor:/opt/ros/jazzy/lib/python3.12/site-packages
```

## Testing
```
pytest
```

## Ruff
```
ruff check
ruff format
```

## To install cli
```
pip3 install -e .
```
