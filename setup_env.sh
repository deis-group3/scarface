#!/bin/bash
# Setup script to source ROS2 and the local workspace

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source ROS2 Jazzy (adjust path if needed)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ ROS2 Jazzy sourced"
else
    echo "⚠ ROS2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    echo "  Please adjust the path in setup_env.sh or source ROS2 manually"
fi

# Source the workspace
if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
    echo "✓ Scarface workspace sourced"
else
    echo "⚠ Workspace install not found. Run 'cd ros2_ws && colcon build' first."
fi

echo ""
echo "Environment ready! You can now:"
echo "  - ros2 run scarface scarface_node"
echo "  - ros2 launch scarface scarface.launch.py"
