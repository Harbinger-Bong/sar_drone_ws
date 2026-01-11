#!/bin/bash

# SAR Drone Workspace Setup Script
# This script sets up the development environment for the SAR drone simulation

set -e

echo "🚁 Setting up SAR Drone Workspace..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if running on Ubuntu
if ! command -v apt &> /dev/null; then
    echo -e "${RED}❌ This script requires Ubuntu/Debian with apt package manager${NC}"
    exit 1
fi

# Check ROS 2 Foxy installation
echo -e "${BLUE}🔍 Checking ROS 2 Foxy installation...${NC}"
if [ ! -f "/opt/ros/foxy/setup.bash" ]; then
    echo -e "${YELLOW}⚠️  ROS 2 Foxy not found. Installing...${NC}"
    
    # Add ROS 2 apt repository
    sudo apt update
    sudo apt install -y curl gnupg2 lsb-release
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Install ROS 2 Foxy
    sudo apt update
    sudo apt install -y ros-foxy-desktop python3-colcon-common-extensions
    
    echo -e "${GREEN}✅ ROS 2 Foxy installed successfully${NC}"
else
    echo -e "${GREEN}✅ ROS 2 Foxy is already installed${NC}"
fi

# Install additional dependencies
echo -e "${BLUE}📦 Installing additional dependencies...${NC}"
sudo apt update
sudo apt install -y \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-slam-toolbox \
    ros-foxy-nav2-bringup \
    ros-foxy-robot-state-publisher \
    ros-foxy-joint-state-publisher \
    ros-foxy-xacro \
    python3-pip \
    python3-rosdep

# Initialize rosdep if not already done
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo -e "${BLUE}🔧 Initializing rosdep...${NC}"
    sudo rosdep init
fi

echo -e "${BLUE}🔄 Updating rosdep...${NC}"
rosdep update

# Source ROS 2 setup
echo -e "${BLUE}🔧 Sourcing ROS 2 environment...${NC}"
source /opt/ros/foxy/setup.bash

# Change to workspace directory
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"

# Install workspace dependencies
echo -e "${BLUE}📋 Installing workspace dependencies...${NC}"
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
echo -e "${BLUE}🔨 Building workspace...${NC}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source the workspace
source install/setup.bash

echo -e "${GREEN}🎉 SAR Drone Workspace setup completed successfully!${NC}"
echo ""
echo -e "${YELLOW}📝 Next steps:${NC}"
echo "1. Open VS Code: code ."
echo "2. Install recommended extensions when prompted"
echo "3. Use Ctrl+Shift+P and run 'Tasks: Run Task' to see available build/run tasks"
echo ""
echo -e "${BLUE}🚀 To launch the simulation:${NC}"
echo "Terminal 1: ros2 launch sar_drone_description sar_world.launch.py"
echo "Terminal 2: ros2 launch sar_drone_description sar_nav2.launch.py"
echo "Terminal 3: ros2 launch nav2_bringup rviz_launch.py"
echo ""
echo -e "${GREEN}Happy coding! 🤖${NC}"