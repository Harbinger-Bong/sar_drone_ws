#!/bin/bash

# SAR Drone Convenience Launch Script
# Wrapper around canonical bringup launch files
# Architecture: RTAB-Map + EKF + Nav2 (planning only)

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"

echo -e "${BLUE}SAR Drone Launcher (Humble + Ignition)${NC}"

# Check workspace
if [ ! -d "install" ]; then
    echo -e "${RED}Workspace not built. Run colcon build first.${NC}"
    exit 1
fi

# Source environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}ROS 2 Humble not found.${NC}"
    exit 1
fi

source install/setup.bash

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
    echo -e "${YELLOW}  ./scripts/run.sh sim   # simulation${NC}"
    echo -e "${YELLOW}  ./scripts/run.sh real  # real drone${NC}"
    exit 1
fi

