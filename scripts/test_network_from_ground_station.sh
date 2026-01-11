#!/bin/bash

# ROS 2 Network Communication Test Script
# Run this script on the GROUND STATION (Humble) to test connectivity

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🧪 ROS 2 Network Communication Test${NC}"
echo -e "${YELLOW}=====================================${NC}"
echo ""

# Simulation machine IP (update if different)
SIM_IP="192.168.1.36"

# Check environment variables
echo -e "${BLUE}1️⃣ Checking Environment Variables...${NC}"
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo -e "${RED}❌ ROS_DOMAIN_ID is not set${NC}"
    echo -e "${YELLOW}   Run: export ROS_DOMAIN_ID=7${NC}"
    exit 1
else
    echo -e "${GREEN}✅ ROS_DOMAIN_ID: $ROS_DOMAIN_ID${NC}"
fi

if [ "$ROS_LOCALHOST_ONLY" != "0" ]; then
    echo -e "${RED}❌ ROS_LOCALHOST_ONLY is not 0 (currently: ${ROS_LOCALHOST_ONLY:-not set})${NC}"
    echo -e "${YELLOW}   Run: export ROS_LOCALHOST_ONLY=0${NC}"
    exit 1
else
    echo -e "${GREEN}✅ ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY${NC}"
fi

if [ -z "$RMW_IMPLEMENTATION" ]; then
    echo -e "${YELLOW}⚠️  RMW_IMPLEMENTATION is not set${NC}"
else
    echo -e "${GREEN}✅ RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION${NC}"
fi

echo ""

# Test network connectivity
echo -e "${BLUE}2️⃣ Testing Network Connectivity...${NC}"
if ping -c 1 -W 2 "$SIM_IP" &> /dev/null; then
    echo -e "${GREEN}✅ Can reach simulation machine at $SIM_IP${NC}"
else
    echo -e "${RED}❌ Cannot reach simulation machine at $SIM_IP${NC}"
    echo -e "${YELLOW}   Check network connection and IP address${NC}"
    exit 1
fi

echo ""

# List topics
echo -e "${BLUE}3️⃣ Listing Available Topics...${NC}"
TOPIC_COUNT=$(ros2 topic list 2>/dev/null | wc -l)
if [ "$TOPIC_COUNT" -gt 0 ]; then
    echo -e "${GREEN}✅ Found $TOPIC_COUNT topics${NC}"
    echo ""
    echo -e "${YELLOW}Available topics:${NC}"
    ros2 topic list
else
    echo -e "${RED}❌ No topics found${NC}"
    echo -e "${YELLOW}   Check ROS 2 configuration and network settings${NC}"
    exit 1
fi

echo ""

# Check for specific topics
echo -e "${BLUE}4️⃣ Checking for Key Topics...${NC}"

check_topic() {
    if ros2 topic list | grep -q "$1"; then
        echo -e "${GREEN}✅ Found: $1${NC}"
        return 0
    else
        echo -e "${RED}❌ Missing: $1${NC}"
        return 1
    fi
}

CAMERA_FOUND=0
LIDAR_FOUND=0
IMU_FOUND=0

if check_topic "/sar_drone/camera/image_raw"; then
    CAMERA_FOUND=1
fi

if check_topic "/sar_drone/scan"; then
    LIDAR_FOUND=1
fi

if check_topic "/sar_drone/imu/data"; then
    IMU_FOUND=1
fi

echo ""

# Test topic echo
echo -e "${BLUE}5️⃣ Testing Topic Data Reception...${NC}"

if [ "$CAMERA_FOUND" -eq 1 ]; then
    echo -e "${YELLOW}Testing camera topic...${NC}"
    timeout 3 ros2 topic echo /sar_drone/camera/camera_info --once &> /dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Can receive camera data${NC}"
    else
        echo -e "${YELLOW}⚠️  Camera topic exists but no data received${NC}"
    fi
fi

if [ "$LIDAR_FOUND" -eq 1 ]; then
    echo -e "${YELLOW}Testing LiDAR topic...${NC}"
    timeout 3 ros2 topic echo /sar_drone/scan --once &> /dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Can receive LiDAR data${NC}"
    else
        echo -e "${YELLOW}⚠️  LiDAR topic exists but no data received${NC}"
    fi
fi

if [ "$IMU_FOUND" -eq 1 ]; then
    echo -e "${YELLOW}Testing IMU topic...${NC}"
    timeout 3 ros2 topic echo /sar_drone/imu/data --once &> /dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Can receive IMU data${NC}"
    else
        echo -e "${YELLOW}⚠️  IMU topic exists but no data received${NC}"
    fi
fi

echo ""

# Summary
echo -e "${BLUE}=====================================${NC}"
if [ "$CAMERA_FOUND" -eq 1 ] && [ "$LIDAR_FOUND" -eq 1 ] && [ "$IMU_FOUND" -eq 1 ]; then
    echo -e "${GREEN}🎉 Network communication test PASSED!${NC}"
    echo ""
    echo -e "${GREEN}✅ All key topics are visible and accessible${NC}"
    echo -e "${GREEN}✅ You can now use RViz or other tools to visualize data${NC}"
    exit 0
else
    echo -e "${YELLOW}⚠️  Some topics may be missing${NC}"
    echo -e "${YELLOW}   Check that simulation is running on the Jetson/sim machine${NC}"
    exit 1
fi
