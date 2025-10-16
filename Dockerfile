# Use ROS2 Jazzy as base image (supports both x86_64 and arm64)
FROM ros:jazzy-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-rclpy \
    ros-jazzy-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install requests

# Create workspace
WORKDIR /workspace

# Copy the ROS2 workspace
COPY ros2_ws /workspace/ros2_ws

# Build the ROS2 workspace
WORKDIR /workspace/ros2_ws
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Source ROS2 and workspace setup
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the working directory
WORKDIR /workspace/ros2_ws

# Default command
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch scarface scarface.launch.py"]
