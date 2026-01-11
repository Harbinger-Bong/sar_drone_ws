# Quick Start: ROS 2 Network Communication

## ✅ Current Status

**Simulation Machine (This Machine)**
- ✅ ROS 2 Foxy configured
- ✅ Simulation running (Gazebo + Robot)
- ✅ Topics publishing: Camera, LiDAR, IMU
- ✅ IP Address: 192.168.1.36
- ✅ ROS_DOMAIN_ID: 7

## 🚀 Quick Commands

### On This Machine (Simulation/Jetson)

**Check status:**
```bash
ros2 topic list
ros2 topic info /sar_drone/camera/image_raw
```

**View topics:**
```bash
ros2 topic echo /sar_drone/scan --once
ros2 topic echo /sar_drone/imu/data --once
```

### On Ground Station (Humble)

**1. Configure (one-time setup):**
```bash
# Add to ~/.bashrc:
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash

# Install Fast DDS:
sudo apt install ros-humble-rmw-fastrtps-cpp

# Reload:
source ~/.bashrc
```

**2. Test connection:**
```bash
# List all topics from simulation
ros2 topic list

# View camera data
ros2 topic echo /sar_drone/camera/camera_info --once

# View LiDAR scan
ros2 topic echo /sar_drone/scan --once

# Check topic rates
ros2 topic hz /sar_drone/camera/image_raw
```

**3. Run automated test:**
```bash
# Copy test script to ground station and run:
./test_network_from_ground_station.sh
```

## 📋 Available Topics

- `/sar_drone/camera/image_raw` - Camera images
- `/sar_drone/camera/camera_info` - Camera calibration
- `/sar_drone/scan` - LiDAR scan data
- `/sar_drone/imu/data` - IMU sensor data
- `/joint_states` - Robot joint states
- `/tf` - Transform tree
- `/gazebo/model_states` - Gazebo simulation state

## 🔧 Troubleshooting

**Topics not visible?**
1. Check `ROS_DOMAIN_ID=7` on both machines
2. Check `ROS_LOCALHOST_ONLY=0` on both machines
3. Ping test: `ping 192.168.1.36`
4. Restart ROS daemon: `ros2 daemon stop && ros2 daemon start`

**Firewall issues?**
```bash
sudo ufw allow 7400:7500/udp
```

## 📚 Full Documentation

- `NETWORK_TEST_STATUS.md` - Complete test status and instructions
- `ROS2_NETWORK_SETUP.md` - Detailed setup guide
- `scripts/verify_ros2_config.sh` - Configuration verification
- `scripts/test_network_from_ground_station.sh` - Automated test script
