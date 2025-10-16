#!/bin/bash
# Simple script to run the Scarface robot container

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Starting Scarface Robot Docker Container...${NC}"

# Check if docker-compose is available
if command -v docker-compose &> /dev/null; then
    DOCKER_COMPOSE="docker-compose"
elif command -v docker &> /dev/null && docker compose version &> /dev/null; then
    DOCKER_COMPOSE="docker compose"
else
    echo "Error: docker-compose or docker compose plugin not found"
    exit 1
fi

# Build and start the container
$DOCKER_COMPOSE up --build

echo -e "${GREEN}Scarface container stopped.${NC}"
