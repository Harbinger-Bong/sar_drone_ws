# ROS 2 Network Configuration Guide

## Overview
This guide documents the ROS 2 network configuration for communication between:
- **Jetson/Sim Machine**: ROS 2 Foxy (this machine)
- **Ground Station**: ROS 2 Humble

## ✅ Configuration Applied to This Machine (Foxy)

The following configuration has been added to `~/.bashrc`:

```bash
# ROS 2 Foxy configuration for network communication with Humble
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
source /opt/ros/foxy/setup.bash

# Use Fast DDS for Foxy ↔ Humble communication
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### What This Does:
- **ROS_DOMAIN_ID=7**: Sets the ROS 2 domain ID to 7 (both machines must use the same ID)
- **ROS_LOCALHOST_ONLY=0**: Allows ROS 2 to communicate over the network (not just localhost)
- **source /opt/ros/foxy/setup.bash**: Initializes ROS 2 Foxy environment
- **RMW_IMPLEMENTATION=rmw_fastrtps_cpp**: Uses Fast DDS as the DDS implementation for compatibility with Humble

## 🔧 Ground Station (Humble) Configuration Required

On your **ground station machine** (running ROS 2 Humble), add the following to `~/.bashrc`:

```bash
# ROS 2 Humble configuration for network communication with Foxy
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash

# Use Fast DDS for Foxy ↔ Humble communication
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Install Fast DDS on Ground Station (Humble):
```bash
sudo apt install ros-humble-rmw-fastrtps-cpp
```

**Note**: Foxy already has Fast DDS by default, so no installation needed on this machine.

## ✅ Verification Steps

### On This Machine (Foxy):
1. Open a new terminal (or run `source ~/.bashrc`)
2. Verify ROS 2 is active:
   ```bash
   ros2 topic list
   ```
   You should see a list of topics (not "command not found")

### On Ground Station (Humble):
1. Open a new terminal (or run `source ~/.bashrc`)
2. Verify ROS 2 is active:
   ```bash
   ros2 topic list
   ```
3. Check if you can see topics from this machine:
   ```bash
   ros2 topic list
   ros2 topic echo /topic_name  # Replace with an actual topic name
   ```

## 🚀 Starting Nodes

Before testing network communication, make sure on this machine (Jetson/sim):
- ✅ Exported `ROS_DOMAIN_ID=7`
- ✅ Exported `ROS_LOCALHOST_ONLY=0`
- ✅ Sourced `/opt/ros/foxy/setup.bash`
- ✅ Set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- ✅ Started your nodes (camera, LiDAR, mavros, Gazebo, etc.)

## 🔍 Troubleshooting

### If topics are not visible across machines:

1. **Check network connectivity:**
   ```bash
   ping <ground_station_ip>
   ```

2. **Verify ROS_DOMAIN_ID matches:**
   ```bash
   echo $ROS_DOMAIN_ID  # Should be 7 on both machines
   ```

3. **Check ROS_LOCALHOST_ONLY:**
   ```bash
   echo $ROS_LOCALHOST_ONLY  # Should be 0 on both machines
   ```

4. **Verify RMW implementation:**
   ```bash
   echo $RMW_IMPLEMENTATION  # Should be rmw_fastrtps_cpp on both machines
   ```

5. **Check firewall settings:**
   - Ensure UDP ports 7400-7500 are open on both machines
   - Fast DDS uses these ports for discovery and communication

6. **Check ROS 2 discovery:**
   ```bash
   ros2 daemon stop
   ros2 daemon start
   ```

## 📝 Notes

- Both machines must be on the same network
- Both machines must use the same `ROS_DOMAIN_ID` (7)
- Both machines must have `ROS_LOCALHOST_ONLY=0`
- Using the same RMW implementation (Fast DDS) ensures better compatibility between Foxy and Humble
