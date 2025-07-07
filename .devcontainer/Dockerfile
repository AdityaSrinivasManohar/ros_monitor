FROM ros:jazzy-ros-core

RUN apt-get update && \
    apt-get install -y \
    git \
    nano \
    python3 \
    python3-pip \
    python3-venv \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-tf2-ros && \
    apt-get clean

WORKDIR /ros2tui

COPY .devcontainer/requirements.txt /ros2tui/requirements.txt

RUN cd /ros2tui && \
    python3 -m venv /venv && \
    /venv/bin/pip install -r requirements.txt

COPY .devcontainer/.bashrc /root/.bashrc

ENV PYTHONPATH=/ros2tui:/opt/ros/jazzy/lib/python3.12/site-packages:/venv/lib/python3.12/site-packages
