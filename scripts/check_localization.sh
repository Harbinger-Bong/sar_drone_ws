#!/bin/bash
# Script to check if localization + SLAM are active (RTAB-Map + EKF)

echo "=== Checking ROS2 System Status ==="
echo ""

echo "1. Checking active nodes..."
ros2 node list 2>/dev/null || echo "No ROS2 nodes running!"
echo ""

echo "2. Checking LiDAR scan topic..."
ros2 topic info /sar_drone/scan 2>/dev/null || echo "Scan topic not found!"
echo ""

echo "3. Checking RTAB-Map odometry..."
ros2 topic info /rtabmap/odom 2>/dev/null || echo "RTAB-Map odometry not found!"
echo ""

echo "4. Checking filtered odometry (EKF output)..."
ros2 topic info /odometry/filtered 2>/dev/null || echo "Filtered odometry not found!"
echo ""

echo "5. Checking map topic..."
ros2 topic info /map 2>/dev/null || echo "Map topic not found (RTAB-Map may not be running yet)!"
echo ""

echo "6. Checking TF tree..."
ros2 run tf2_tools view_frames.py 2>/dev/null || echo "Could not generate TF tree!"
echo ""

echo "7. Checking RTAB-Map node..."
ros2 node info /rtabmap 2>/dev/null || echo "RTAB-Map node not running!"
echo ""

echo "8. Checking EKF node..."
ros2 node info /ekf_filter_node 2>/dev/null || echo "EKF node not running!"
echo ""

echo "=== Localization Check Complete ==="
echo "Expected for a healthy system:"
echo "- /rtabmap/odom present"
echo "- /odometry/filtered present"
echo "- /map present"
echo "- TF tree with map → odom → base_link"

