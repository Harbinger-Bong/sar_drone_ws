#!/bin/bash

# ============================================
# SAR Drone Simulation Startup Script
# Validates environment and launches simulation
# ============================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo -e "${BLUE}╔════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     SAR DRONE SIMULATION LAUNCHER (v2.0)       ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════╝${NC}"
echo ""

# ============================================
# 1. Environment Validation
# ============================================
echo -e "${CYAN}[1/6] Validating Environment...${NC}"

# Check ROS 2 installation
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${RED}✗ ROS 2 Humble not found${NC}"
    echo -e "${YELLOW}  Run: ./scripts/setup.sh${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ ROS 2 Humble installed${NC}"

# Check workspace
if [ ! -d "$WORKSPACE_DIR/install" ]; then
    echo -e "${RED}✗ Workspace not built${NC}"
    echo -e "${YELLOW}  Run: colcon build${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ Workspace built${NC}"

# Source environments
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

echo -e "${GREEN}  ✓ Environment sourced${NC}"
echo ""

# ============================================
# 2. Dependency Check
# ============================================
echo -e "${CYAN}[2/6] Checking Dependencies...${NC}"

DEPS_OK=true

# Check Ignition Gazebo
if ! command -v ign &> /dev/null && ! command -v gz &> /dev/null; then
    echo -e "${RED}  ✗ Ignition/Gazebo not found${NC}"
    DEPS_OK=false
else
    echo -e "${GREEN}  ✓ Gazebo simulator available${NC}"
fi

# Check RTAB-Map
if ! ros2 pkg list | grep -q rtabmap_ros; then
    echo -e "${RED}  ✗ RTAB-Map not installed${NC}"
    DEPS_OK=false
else
    echo -e "${GREEN}  ✓ RTAB-Map installed${NC}"
fi

# Check Nav2
if ! ros2 pkg list | grep -q nav2_bringup; then
    echo -e "${RED}  ✗ Nav2 not installed${NC}"
    DEPS_OK=false
else
    echo -e "${GREEN}  ✓ Nav2 installed${NC}"
fi

# Check robot_localization
if ! ros2 pkg list | grep -q robot_localization; then
    echo -e "${RED}  ✗ robot_localization not installed${NC}"
    DEPS_OK=false
else
    echo -e "${GREEN}  ✓ robot_localization installed${NC}"
fi

if [ "$DEPS_OK" = false ]; then
    echo -e "${RED}Missing dependencies. Run setup.sh first.${NC}"
    exit 1
fi

echo ""

# ============================================
# 3. Configuration Validation
# ============================================
echo -e "${CYAN}[3/6] Validating Configuration Files...${NC}"

CONFIG_OK=true

# Check Nav2 params
NAV2_PARAMS="$WORKSPACE_DIR/src/sar_drone_description/config/nav2_params.yaml"
if [ ! -f "$NAV2_PARAMS" ]; then
    echo -e "${RED}  ✗ Nav2 params not found${NC}"
    CONFIG_OK=false
else
    echo -e "${GREEN}  ✓ Nav2 configuration found${NC}"
fi

# Check EKF params
EKF_PARAMS="$WORKSPACE_DIR/src/sar_localization/config/ekf.yaml"
if [ ! -f "$EKF_PARAMS" ]; then
    echo -e "${RED}  ✗ EKF params not found${NC}"
    CONFIG_OK=false
else
    echo -e "${GREEN}  ✓ EKF configuration found${NC}"
fi

# Check RTAB-Map params
RTABMAP_PARAMS="$WORKSPACE_DIR/src/sar_perception/config/rtabmap.yaml"
if [ ! -f "$RTABMAP_PARAMS" ]; then
    echo -e "${RED}  ✗ RTAB-Map params not found${NC}"
    CONFIG_OK=false
else
    echo -e "${GREEN}  ✓ RTAB-Map configuration found${NC}"
fi

# Check world file
WORLD_FILE="$WORKSPACE_DIR/src/sar_drone_description/worlds/sar_world.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo -e "${RED}  ✗ World file not found${NC}"
    CONFIG_OK=false
else
    echo -e "${GREEN}  ✓ World file found${NC}"
fi

if [ "$CONFIG_OK" = false ]; then
    echo -e "${RED}Configuration files missing!${NC}"
    exit 1
fi

echo ""

# ============================================
# 4. Clean Previous Runtime
# ============================================
echo -e "${CYAN}[4/6] Cleaning Previous Runtime...${NC}"

# Kill any existing Gazebo/ROS processes
if pgrep -f "gz sim\|ign gazebo\|rtabmap\|ekf_node" > /dev/null; then
    echo -e "${YELLOW}  ! Killing existing processes${NC}"
    pkill -f "gz sim\|ign gazebo" || true
    pkill -f "rtabmap\|ekf_node" || true
    sleep 2
fi

# Clean RTAB-Map database
RTABMAP_DB="$HOME/.ros/rtabmap.db"
if [ -f "$RTABMAP_DB" ]; then
    echo -e "${YELLOW}  ! Removing old RTAB-Map database${NC}"
    rm -f "$RTABMAP_DB"
fi

echo -e "${GREEN}  ✓ Runtime cleaned${NC}"
echo ""

# ============================================
# 5. Pre-launch Checks
# ============================================
echo -e "${CYAN}[5/6] Pre-launch System Checks...${NC}"

# Check available memory
AVAILABLE_MEM=$(free -m | awk '/^Mem:/{print $7}')
if [ "$AVAILABLE_MEM" -lt 2000 ]; then
    echo -e "${YELLOW}  ⚠ Low memory: ${AVAILABLE_MEM}MB available${NC}"
    echo -e "${YELLOW}    Simulation may run slowly${NC}"
else
    echo -e "${GREEN}  ✓ Sufficient memory: ${AVAILABLE_MEM}MB${NC}"
fi

# Check disk space
DISK_SPACE=$(df -m "$WORKSPACE_DIR" | awk 'NR==2 {print $4}')
if [ "$DISK_SPACE" -lt 1000 ]; then
    echo -e "${YELLOW}  ⚠ Low disk space: ${DISK_SPACE}MB available${NC}"
else
    echo -e "${GREEN}  ✓ Sufficient disk space: ${DISK_SPACE}MB${NC}"
fi

# Check display
if [ -z "$DISPLAY" ]; then
    echo -e "${YELLOW}  ⚠ DISPLAY not set - RViz may not work${NC}"
else
    echo -e "${GREEN}  ✓ Display configured: $DISPLAY${NC}"
fi

echo ""

# ============================================
# 6. Launch Simulation
# ============================================
echo -e "${CYAN}[6/6] Launching Simulation...${NC}"
echo ""
echo -e "${BLUE}╔════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║         STARTING SAR DRONE SIMULATION         ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════╝${NC}"
echo ""

echo -e "${YELLOW}Components launching:${NC}"
echo -e "  • Gazebo Fortress (World + Robot)"
echo -e "  • Sensor Stack (Camera, LiDAR, IMU)"
echo -e "  • RTAB-Map SLAM"
echo -e "  • EKF Localization"
echo -e "  • Nav2 Navigation Stack"
echo -e "  • RViz Visualization"
echo ""

echo -e "${GREEN}This will take ~30 seconds...${NC}"
echo ""

# Launch with error handling
if ros2 launch sar_bringup sim_nav2.launch.py; then
    echo ""
    echo -e "${GREEN}╔════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║         SIMULATION TERMINATED CLEANLY          ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════╝${NC}"
else
    echo ""
    echo -e "${RED}╔════════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║           SIMULATION FAILED TO START           ║${NC}"
    echo -e "${RED}╚════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${YELLOW}Troubleshooting:${NC}"
    echo -e "  1. Check logs in ~/.ros/log/"
    echo -e "  2. Verify all dependencies: ./scripts/setup.sh"
    echo -e "  3. Rebuild workspace: colcon build --symlink-install"
    echo -e "  4. Check Gazebo: gz sim --verbose"
    exit 1
fi
