#!/bin/bash

# SAR Drone Unified Launch Script
# Wrapper for simulation or real-hardware bringup
# Architecture: RTAB-Map + EKF + Nav2 (planning only)

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}SAR Drone System Launcher (Humble + Ignition)${NC}"
echo -e "${YELLOW}===========================================${NC}"

# Check workspace
if [ ! -d "install" ]; then
    echo -e "${RED}Workspace not built. Run colcon build first.${NC}"
    exit 1
fi

# Source environments
echo -e "${BLUE}Sourcing environments...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash

# Mode selection
MODE=$1

if [ "$MODE" == "sim" ]; then
    echo -e "${GREEN}Launching SIMULATION stack${NC}"
    echo -e "${YELLOW}- Ignition Fortress${NC}"
    echo -e "${YELLOW}- RTAB-Map SLAM${NC}"
    echo -e "${YELLOW}- EKF localization${NC}"
    echo -e "${YELLOW}- Nav2 planning${NC}"
    echo -e "${YELLOW}- RViz${NC}"

    ros2 launch sar_bringup sim_nav2.launch.py

elif [ "$MODE" == "real" ]; then
    echo -e "${GREEN}Launching REAL DRONE stack${NC}"
    echo -e "${YELLOW}- Real sensors + PX4${NC}"
    echo -e "${YELLOW}- RTAB-Map SLAM${NC}"
    echo -e "${YELLOW}- EKF localization${NC}"
    echo -e "${YELLOW}- Nav2 planning${NC}"
    echo -e "${YELLOW}- Nav2 → PX4 bridge${NC}"

    ros2 launch sar_bringup drone_nav2_full.launch.py

else
    echo -e "${RED}Usage:${NC}"
    echo -e "${YELLOW}  ./launch_all.sh sim   # simulation${NC}"
    echo -e "${YELLOW}  ./launch_all.sh real  # real drone${NC}"
    exit 1
fi

