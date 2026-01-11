# 🚁 SAR Drone Simulation

[![ROS 2 Foxy](https://img.shields.io/badge/ROS%202-Foxy-blue)](https://docs.ros.org/en/foxy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)](http://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-brightgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![VS Code](https://img.shields.io/badge/VS%20Code-Ready-007ACC)](https://code.visualstudio.com/)

A comprehensive **Search and Rescue (SAR) drone simulation** built with ROS 2 Foxy, featuring autonomous navigation, SLAM mapping, and multiple sensors for realistic rescue operations.

## ✨ Features

- 🤖 **Realistic Drone Model**: URDF-based quadcopter with sensors
- 🗺️ **SLAM Mapping**: Real-time environment mapping using slam_toolbox
- 🧭 **Autonomous Navigation**: Nav2 stack for path planning and obstacle avoidance
- 📡 **Sensor Suite**: LiDAR, camera, and IMU integration
- 🌍 **SAR Environment**: Custom Gazebo world with hills, debris, and obstacles
- 🎯 **Mission Planning**: Waypoint-based navigation for search patterns
- 💻 **VS Code Ready**: Fully configured development environment

## 📋 Prerequisites

### System Requirements
- **OS**: Ubuntu 20.04 LTS (recommended)
- **ROS**: ROS 2 Foxy Desktop Full
- **Python**: 3.8+
- **Memory**: 8GB RAM (recommended)
- **Storage**: 10GB free space

### Software Dependencies
```bash
sudo apt update
sudo apt install -y \
    ros-foxy-desktop \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-slam-toolbox \
    ros-foxy-nav2-bringup \
    python3-colcon-common-extensions \
    python3-rosdep
```

## 🚀 Quick Start

### 1. Clone and Setup
```bash
git clone https://github.com/your-username/sar-drone-simulation.git
cd sar-drone-simulation
./scripts/setup.sh
```

### 2. Build the Workspace
```bash
./scripts/build.sh
```

### 3. Launch Simulation
```bash
./scripts/run.sh
```

Or manually launch in separate terminals:
```bash
# Terminal 1 - Gazebo Simulation
ros2 launch sar_drone_description sar_world.launch.py

# Terminal 2 - Navigation Stack  
ros2 launch sar_drone_description sar_nav2.launch.py

# Terminal 3 - RViz Visualization
ros2 launch nav2_bringup rviz_launch.py
```

## 🛠️ VS Code Development

### Automatic Setup
1. Open workspace: `code sar_drone_ws.code-workspace`
2. Install recommended extensions when prompted
3. Use **Ctrl+Shift+P** → "Tasks: Run Task" for build/run commands

### Available Tasks
- `colcon: build` - Build entire workspace
- `ros2: launch gazebo simulation` - Launch Gazebo
- `ros2: launch navigation` - Launch Nav2 stack
- `ros2: launch rviz` - Launch visualization

### DevContainer Support
For isolated development environment:
```bash
code .
# VS Code will prompt to "Reopen in Container"
```

## 🎮 Usage Guide

### Basic Operation
1. **Set Initial Pose**: Use "2D Pose Estimate" in RViz
2. **Set Navigation Goal**: Use "2D Nav Goal" in RViz  
3. **Monitor Progress**: Watch real-time SLAM mapping
4. **View Sensor Data**: 
   - LiDAR: `/sar_drone/scan`
   - Camera: `/sar_drone/camera/image_raw`
   - IMU: `/sar_drone/imu/data`

### Advanced Commands
```bash
# View all topics
ros2 topic list

# Monitor drone pose
ros2 topic echo /odom

# View camera feed
ros2 run rqt_image_view rqt_image_view

# Save generated map
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

## 📁 Project Structure

```
sar_drone_ws/
├── 📂 src/sar_drone_description/     # Main ROS 2 package
│   ├── 🤖 urdf/                     # Robot description files
│   ├── 🌍 worlds/                   # Gazebo world files  
│   ├── ⚙️ config/                   # Navigation parameters
│   ├── 🚀 launch/                   # Launch files
│   └── 👁️ rviz/                     # RViz configurations
├── 💻 .vscode/                      # VS Code configuration
├── 🐳 .devcontainer/               # DevContainer setup
├── 🛠️ scripts/                      # Automation scripts
├── ⚙️ .github/                      # CI/CD workflows
└── 📚 docs/                        # Documentation
```

## 🔧 Configuration

### Drone Parameters
- **Mass**: 5.0 kg
- **Sensors**: LiDAR (720 samples), Camera (640x480), IMU (50Hz)
- **Motion**: Planar movement with odometry
- **Navigation**: DWB local planner, NavFn global planner

### Environment Customization
Edit `worlds/sar_world.world` to modify:
- Terrain features (hills, obstacles)
- Lighting conditions
- Weather effects
- Additional objects

## 🧪 Testing & Validation

### Automated Tests
```bash
# Run package tests
colcon test --packages-select sar_drone_description

# View test results  
colcon test-result --verbose
```

### Manual Validation
- ✅ Gazebo launches without errors
- ✅ Robot spawns at origin
- ✅ Sensor data published correctly
- ✅ Navigation goals reachable
- ✅ Map generation functional

## 🚩 Known Issues & Solutions

| Issue | Solution |
|-------|----------|
| Gazebo crashes on launch | Ensure GPU drivers updated |
| No laser scan data | Check LiDAR plugin configuration |
| Navigation fails | Verify odometry and TF frames |
| Build errors | Run `rosdep install --from-paths src -r -y` |

## 🤝 Contributing

We welcome contributions! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Development Workflow
1. Fork the repository
2. Create feature branch: `git checkout -b feature/amazing-feature`
3. Make changes and test thoroughly
4. Submit pull request with detailed description

### Code Standards
- Follow [ROS 2 style guide](https://docs.ros.org/en/foxy/Contributing/Code-Style-Language-Versions.html)
- Include unit tests for new features
- Update documentation for API changes
- Ensure CI/CD pipeline passes

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **ROS 2 Community** for the robotics framework
- **Gazebo Team** for the simulation environment  
- **Nav2 Project** for navigation capabilities
- **SLAM Toolbox** for mapping functionality

## 📞 Support

- 🐛 **Bug Reports**: [GitHub Issues](https://github.com/your-username/sar-drone-simulation/issues)
- 💬 **Discussions**: [GitHub Discussions](https://github.com/your-username/sar-drone-simulation/discussions)
- 📧 **Contact**: your-email@example.com

## 📊 Project Status

- ✅ **Core Simulation**: Complete
- ✅ **Navigation**: Complete  
- ✅ **SLAM**: Complete
- 🔄 **Advanced AI**: In Progress
- 📋 **Multi-Drone**: Planned
- 📋 **Real Hardware**: Planned

---

<div align="center">

**⭐ Star this repo if it helps with your SAR drone research! ⭐**

[Report Bug](https://github.com/your-username/sar-drone-simulation/issues) • [Request Feature](https://github.com/your-username/sar-drone-simulation/issues) • [Documentation](https://github.com/your-username/sar-drone-simulation/wiki)

</div>