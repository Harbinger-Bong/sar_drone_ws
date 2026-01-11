#!/bin/bash

# SAR Drone Complete Launch Script
# Launches Gazebo, SLAM, Nav2, and RViz in sequence

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚁 SAR Drone Complete System Launch${NC}"
echo -e "${YELLOW}=======================================${NC}"

# Check if workspace is built
if [ ! -d "install" ]; then
    echo -e "${RED}❌ Workspace not built! Run colcon build first${NC}"
    exit 1
fi

# Source workspace
echo -e "${BLUE}📦 Sourcing workspace...${NC}"
source /opt/ros/foxy/setup.bash
source install/setup.bash

# Launch complete system
echo -e "${GREEN}🚀 Launching complete SAR drone system...${NC}"
echo -e "${YELLOW}This will start in sequence:${NC}"
echo -e "${YELLOW}  1. Gazebo simulation (immediate)${NC}"
echo -e "${YELLOW}  2. SLAM mapping (after 5s)${NC}" 
echo -e "${YELLOW}  3. Nav2 navigation (after 8s)${NC}"
echo -e "${YELLOW}  4. RViz visualization (after 12s)${NC}"
echo -e "${YELLOW}=======================================${NC}"

ros2 launch sar_drone_description sar_complete.launch.py

echo -e "${GREEN}✅ SAR Drone system ready!${NC}"
echo -e "${BLUE}In RViz:${NC}"
echo -e "${YELLOW}  - Use '2D Pose Estimate' to set initial position${NC}"
echo -e "${YELLOW}  - Use 'Nav2 Goal' to command autonomous navigation${NC}"
echo -e "${YELLOW}  - Watch real-time SLAM mapping build the environment${NC}"