#!/bin/bash
# Script to rebuild the ROS2 workspace inside the container

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Rebuilding ROS2 workspace inside container...${NC}"

# Check if docker-compose is available
if command -v docker-compose &> /dev/null; then
    DOCKER_COMPOSE="docker-compose"
elif command -v docker &> /dev/null && docker compose version &> /dev/null; then
    DOCKER_COMPOSE="docker compose"
else
    echo "Error: docker-compose or docker compose plugin not found"
    exit 1
fi

# Run colcon build inside the container
$DOCKER_COMPOSE run --rm scarface bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    cd /workspace/ros2_ws && \
    colcon build --symlink-install && \
    echo -e '${GREEN}Build complete!${NC}'"
