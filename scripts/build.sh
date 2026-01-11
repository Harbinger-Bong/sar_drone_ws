#!/bin/bash

# SAR Drone Build Script
# Quick build script for the SAR drone simulation workspace

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Get workspace directory
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"

echo -e "${BLUE}🔨 Building SAR Drone Workspace...${NC}"

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}❌ ROS 2 Humble not found. Please run setup.sh first.${NC}"
    exit 1
fi

# Clean build (optional)
if [[ "$1" == "clean" ]]; then
    echo -e "${BLUE}🧹 Cleaning previous build...${NC}"
    rm -rf build install log
fi

# Build with colcon
echo -e "${BLUE}⚙️  Running colcon build...${NC}"
colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Check if build was successful
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Build completed successfully!${NC}"
    echo ""
    echo -e "${BLUE}🔧 To use the workspace:${NC}"
    echo "source install/setup.bash"
    echo ""
    echo -e "${BLUE}🚀 To launch simulation:${NC}"
    echo "ros2 launch sar_drone_description sar_world.launch.py"
else
    echo -e "${RED}❌ Build failed!${NC}"
    exit 1
fi
