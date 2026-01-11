# Localization Fix - December 5, 2025

## Problem
Localization was showing as "inactive" in Nav2/RViz, preventing autonomous navigation.

## Root Causes Identified

### 1. **SLAM Parameters - Incorrect Scan Topic**
The `slam_params.yaml` was configured to listen to `/scan`, but the LiDAR publishes to `/sar_drone/scan`.

**File**: `config/slam_params.yaml`
**Fix**: Changed `scan_topic` from `/scan` to `/sar_drone/scan`

```yaml
slam_toolbox:
  ros__parameters:
    scan_topic: /sar_drone/scan  # Was: /scan
```

### 2. **Launch Timing Issues**
SLAM toolbox was starting too early (5 seconds), before the TF tree was fully established by Gazebo.

**File**: `launch/sar_complete.launch.py`
**Fix**: Increased startup delays:
- SLAM toolbox: 5s → 8s
- Nav2 stack: 8s → 12s
- RViz: 12s → 16s

### 3. **Unnecessary Remappings**
The SLAM toolbox launch had conflicting remappings that were overriding the configuration file.

**Fix**: Removed remappings from launch file, letting the configuration file handle topics.

## Files Modified

1. `/config/slam_params.yaml` - Fixed scan topic path
2. `/launch/sar_complete.launch.py` - Increased delays and removed remappings
3. `/scripts/check_localization.sh` - Created diagnostic script (NEW)

## Testing Instructions

### Option 1: Using Complete Launch File (Recommended)
```bash
cd ~/sar_drone_ws
source install/setup.bash
ros2 launch sar_drone_description sar_complete.launch.py
```

**Wait times:**
- 8 seconds: SLAM toolbox will start
- 12 seconds: Nav2 stack will activate
- 16 seconds: RViz will open

### Option 2: Step-by-Step Launch (For Debugging)

**Terminal 1 - Gazebo:**
```bash
cd ~/sar_drone_ws
source install/setup.bash
ros2 launch sar_drone_description sar_world.launch.py
```

Wait ~5 seconds for robot to spawn completely.

**Terminal 2 - SLAM + Nav2:**
```bash
cd ~/sar_drone_ws
source install/setup.bash
ros2 launch sar_drone_description sar_nav2.launch.py
```

Wait ~5 seconds for SLAM to initialize.

**Terminal 3 - Check Status:**
```bash
cd ~/sar_drone_ws
source install/setup.bash
./scripts/check_localization.sh
```

**Terminal 4 - RViz:**
```bash
cd ~/sar_drone_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d install/sar_drone_description/share/sar_drone_description/rviz/sar_drone_view.rviz
```

## Verification Checklist

### 1. Check Topics
```bash
ros2 topic list | grep -E "(scan|odom|map)"
```

Expected output:
```
/map
/map_metadata
/odom
/sar_drone/scan
```

### 2. Check Nodes
```bash
ros2 node list
```

Should include:
- `/slam_toolbox`
- `/controller_server`
- `/planner_server`
- `/bt_navigator`

### 3. Check TF Tree
```bash
ros2 run tf2_tools view_frames.py
```

This creates `frames.pdf` showing the transform tree. Should see:
```
map → odom → base_link → lidar_link
                       → camera_link
                       → imu_link
```

### 4. Monitor SLAM in Real-Time
```bash
ros2 topic echo /slam_toolbox/feedback
```

Should show SLAM processing scans without errors.

### 5. Check for TF Errors
```bash
ros2 run tf2_ros tf2_echo map base_link
```

Should show continuous transforms without errors.

## In RViz

Once everything is running, check RViz displays:

1. **TF Display**: Complete tree from `map` to all sensor frames
2. **Map Display**: Building map from SLAM (may take a few seconds to appear)
3. **LaserScan Display**: LiDAR data visualized
4. **RobotModel**: 3D model of the drone
5. **Nav2 Panel**: Should show "Localization: active"

## Common Issues and Solutions

### Issue: "Message Filter dropping message: frame 'lidar_link' for reason 'Unknown'"

**Cause**: SLAM toolbox started before TF tree was established.

**Solution**: 
- Wait longer before starting SLAM (already fixed in launch file)
- If using step-by-step launch, wait at least 5 seconds after Gazebo starts

### Issue: "Failed to compute odom pose"

**Cause**: The `odom → base_link` transform isn't being published.

**Solution**:
- Verify the planar_move plugin is loaded in Gazebo
- Check: `ros2 topic echo /odom` should show odometry messages
- The fix in launch timing resolves this

### Issue: No map appearing

**Cause**: SLAM toolbox isn't receiving scan data.

**Solution**:
- Verify scan topic: `ros2 topic hz /sar_drone/scan` should show ~10 Hz
- Check slam_params.yaml has correct `scan_topic: /sar_drone/scan`

### Issue: Nav2 shows "Localization: inactive"

**Cause**: SLAM isn't publishing to `/map` topic or TF isn't complete.

**Solution**:
- Run diagnostic script: `./scripts/check_localization.sh`
- Verify `/map` topic exists: `ros2 topic info /map`
- Ensure all nodes started in correct order with proper delays

## Architecture Summary

```
Gazebo (Robot + Sensors)
    ↓
    ├─ /sar_drone/scan → SLAM Toolbox
    ├─ /odom (with TF) → SLAM Toolbox
    └─ /cmd_vel ← Nav2
        ↓
    SLAM Toolbox
        ↓
        ├─ /map → Nav2 (Planner)
        └─ TF: map → odom
            ↓
        Nav2 Stack
            ↓
            └─ Autonomous Navigation
```

## Next Steps

1. **Test Navigation**: Set initial pose in RViz with "2D Pose Estimate"
2. **Set Goal**: Use "Nav2 Goal" to command the robot to navigate
3. **Monitor Performance**: Watch costmaps and path planning in RViz
4. **Tune Parameters**: Adjust nav2_params.yaml and slam_params.yaml as needed

## Additional Notes

- All fixes use the existing infrastructure (no new dependencies)
- Changes are minimal and targeted to the root causes
- The symlink install means config changes take effect immediately
- Launch file changes require `colcon build` to take effect
