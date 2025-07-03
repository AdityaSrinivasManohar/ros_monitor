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

RUN cd /ros2tui && \
    python3 -m venv /venv && \
    /venv/bin/pip install textual pyyaml numpy

COPY .devcontainer/.bashrc /root/.bashrc

# Copy the entrypoint script into the container
# COPY entrypoint.sh /entrypoint.sh

# # Make the script executable
# RUN chmod +x /entrypoint.sh

# Set the entrypoint to the entrypoint script
# ENTRYPOINT ["/entrypoint.sh"]
