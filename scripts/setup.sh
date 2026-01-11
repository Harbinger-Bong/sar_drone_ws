#!/bin/bash

# SAR Drone Workspace Setup Script
# Target: ROS 2 Humble + Ignition Fortress (ros_gz)

set -e

echo "Setting up SAR Drone Workspace (ROS 2 Humble + Ignition Fortress)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Check OS
if ! command -v apt &> /dev/null; then
    echo -e "${RED}This script requires Ubuntu/Debian with apt${NC}"
    exit 1
fi

# -----------------------------
# ROS 2 Humble installation
# -----------------------------
echo -e "${BLUE}Checking ROS 2 Humble installation...${NC}"

if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${YELLOW}ROS 2 Humble not found. Installing...${NC}"

    sudo apt update
    sudo apt install -y curl gnupg lsb-release

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

    echo -e "${GREEN}ROS 2 Humble installed${NC}"
else
    echo -e "${GREEN}ROS 2 Humble already installed${NC}"
fi

# -----------------------------
# Install required dependencies
# -----------------------------
echo -e "${BLUE}Installing system dependencies...${NC}"

sudo apt update
sudo apt install -y \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rtabmap-ros \
    python3-pip \
    python3-rosdep

# -----------------------------
# rosdep
# -----------------------------
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo -e "${BLUE}Initializing rosdep...${NC}"
    sudo rosdep init
fi

echo -e "${BLUE}Updating rosdep...${NC}"
rosdep update

# -----------------------------
# Source ROS 2
# -----------------------------
echo -e "${BLUE}Sourcing ROS 2 Humble...${NC}"
source /opt/ros/humble/setup.bash

# -----------------------------
# Workspace setup
# -----------------------------
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"

echo -e "${BLUE}Installing workspace dependencies via rosdep...${NC}"
rosdep install --from-paths src --ignore-src -r -y

# -----------------------------
# Build
# -----------------------------
echo -e "${BLUE}Building workspace...${NC}"
colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# -----------------------------
# Source overlay
# -----------------------------
source install/setup.bash

echo -e "${GREEN}Workspace setup completed successfully${NC}"
echo ""
echo -e "${BLUE}Usage:${NC}"
echo "source install/setup.bash"
echo ""
echo "Simulation (Ignition Fortress + SLAM + Nav2):"
echo "ros2 launch sar_bringup sim_nav2.launch.py"
echo ""
echo "Full system (hardware / PX4):"
echo "ros2 launch sar_bringup drone_nav2_full.launch.py"

