# ROS 2 Network Communication Test Status

## ✅ Current Status

**Simulation Machine (Foxy)**: Running and publishing topics
- **IP Address**: 192.168.1.36
- **ROS Version**: Foxy
- **ROS_DOMAIN_ID**: 7
- **Status**: ✅ ACTIVE

## 📡 Available Topics

The following topics are currently being published from the simulation:

### Camera Topics
- `/sar_drone/camera/image_raw` (sensor_msgs/msg/Image)
- `/sar_drone/camera/camera_info` (sensor_msgs/msg/CameraInfo)

### LiDAR Topics
- `/sar_drone/scan` (sensor_msgs/msg/LaserScan)

### IMU Topics
- `/sar_drone/imu/data` (sensor_msgs/msg/Imu)

### Robot State Topics
- `/joint_states` (sensor_msgs/msg/JointState)
- `/robot_description` (std_msgs/msg/String)
- `/tf` (tf2_msgs/msg/TFMessage)
- `/tf_static` (tf2_msgs/msg/TFMessage)

### Gazebo Topics
- `/gazebo/link_states` (gazebo_msgs/msg/LinkStates)
- `/gazebo/model_states` (gazebo_msgs/msg/ModelStates)
- `/clock` (builtin_interfaces/msg/Time)

### System Topics
- `/parameter_events` (rcl_interfaces/msg/ParameterEvent)
- `/rosout` (rcl_interfaces/msg/Log)
- `/performance_metrics` (statistics_msgs/msg/MetricsMessage)

## 🧪 Testing from Ground Station (Humble)

### Prerequisites on Ground Station

1. **Configure `~/.bashrc`** with:
   ```bash
   export ROS_DOMAIN_ID=7
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   source /opt/ros/humble/setup.bash
   ```

2. **Install Fast DDS**:
   ```bash
   sudo apt install ros-humble-rmw-fastrtps-cpp
   ```

3. **Reload configuration**:
   ```bash
   source ~/.bashrc
   ```

### Test Commands

#### 1. List All Topics
```bash
ros2 topic list
```

You should see all the topics listed above from the simulation machine.

#### 2. Check Topic Info
```bash
ros2 topic info /sar_drone/camera/image_raw
ros2 topic info /sar_drone/scan
ros2 topic info /sar_drone/imu/data
```

#### 3. Echo Topic Data
```bash
# View camera info
ros2 topic echo /sar_drone/camera/camera_info

# View LiDAR scan (first message)
ros2 topic echo /sar_drone/scan --once

# View IMU data
ros2 topic echo /sar_drone/imu/data --once
```

#### 4. Check Topic Rate
```bash
ros2 topic hz /sar_drone/camera/image_raw
ros2 topic hz /sar_drone/scan
```

#### 5. View Robot State
```bash
ros2 topic echo /joint_states --once
ros2 topic echo /tf --once
```

### Network Verification

1. **Ping Test**:
   ```bash
   ping 192.168.1.36
   ```

2. **Check ROS 2 Discovery**:
   ```bash
   # On ground station
   ros2 daemon stop
   ros2 daemon start
   ros2 topic list
   ```

3. **Verify Environment Variables**:
   ```bash
   echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"      # Should be 7
   echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"  # Should be 0
   echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"  # Should be rmw_fastrtps_cpp
   ```

## 🔧 Troubleshooting

### If topics are not visible:

1. **Check firewall**: Ensure UDP ports 7400-7500 are open
   ```bash
   sudo ufw allow 7400:7500/udp
   ```

2. **Verify network connectivity**:
   ```bash
   ping 192.168.1.36
   ```

3. **Check ROS_DOMAIN_ID matches**:
   - Both machines must use `ROS_DOMAIN_ID=7`

4. **Restart ROS 2 daemon**:
   ```bash
   ros2 daemon stop
   ros2 daemon start
   ```

5. **Check RMW implementation**:
   - Both should use `rmw_fastrtps_cpp`

## 📊 Expected Results

When everything is working correctly, from the ground station you should:
- ✅ See all topics from the simulation machine
- ✅ Be able to echo topic data
- ✅ See topic publishing rates
- ✅ View camera images, LiDAR scans, and IMU data

## 🚀 Next Steps

1. On ground station, verify you can see topics: `ros2 topic list`
2. Test subscribing to camera images
3. Test subscribing to LiDAR scans
4. If using RViz on ground station, add displays for:
   - Camera image
   - LiDAR scan
   - TF tree
   - Robot model

## 📝 Notes

- The simulation is running in the background on the Jetson/sim machine
- Topics are being published continuously
- Network communication uses Fast DDS (FastRTPS) for compatibility between Foxy and Humble
- Both machines must be on the same network segment
