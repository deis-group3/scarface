#!/bin/bash
# Script to open a shell in the running Scarface container

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Opening shell in Scarface container...${NC}"

# Check if container is running
if [ "$(docker ps -q -f name=scarface_robot)" ]; then
    docker exec -it scarface_robot bash
else
    echo "Container 'scarface_robot' is not running."
    echo "Starting a new container with bash..."
    docker-compose run --rm scarface bash
fi
