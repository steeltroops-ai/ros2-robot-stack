FROM osrf/ros:humble-desktop

# Install tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy source
COPY src src

# Build
RUN . /opt/ros/humble/setup.sh && colcon build

CMD ["ros2", "launch", "simulation_bringup", "world.launch.py"]
