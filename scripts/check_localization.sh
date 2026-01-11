#!/bin/bash
# Script to check if localization is active

echo "=== Checking ROS2 System Status ==="
echo ""

echo "1. Checking active nodes..."
ros2 node list 2>/dev/null || echo "No ROS2 nodes running!"
echo ""

echo "2. Checking scan topic..."
ros2 topic info /sar_drone/scan 2>/dev/null || echo "Scan topic not found!"
echo ""

echo "3. Checking odom topic..."
ros2 topic info /odom 2>/dev/null || echo "Odom topic not found!"
echo ""

echo "4. Checking map topic..."
ros2 topic info /map 2>/dev/null || echo "Map topic not found (SLAM may not be running yet)!"
echo ""

echo "5. Checking TF tree..."
ros2 run tf2_tools view_frames.py 2>/dev/null || echo "Could not generate TF tree!"
echo ""

echo "6. Checking SLAM toolbox status..."
ros2 node info /slam_toolbox 2>/dev/null || echo "SLAM toolbox not running!"
echo ""

echo "=== Localization Check Complete ==="
echo "If all topics and nodes are present, localization should be active."
echo "Check RViz to confirm 'Localization: active' status."
