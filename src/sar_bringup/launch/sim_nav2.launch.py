from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    LogInfo,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    sar_drone_pkg = get_package_share_directory('sar_drone_description')
    sar_perception_pkg = get_package_share_directory('sar_perception')
    sar_localization_pkg = get_package_share_directory('sar_localization')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Config files
    nav2_params = os.path.join(sar_drone_pkg, 'config', 'nav2_params.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Startup logging
    startup_log = TimerAction(
        period=1.0,
        actions=[
            LogInfo(msg="="*60),
            LogInfo(msg="SAR DRONE SIMULATION STARTING"),
            LogInfo(msg="="*60),
            LogInfo(msg="Launching Ignition Gazebo..."),
            LogInfo(msg="Waiting for sensors to initialize...")
        ]
    )

    # 1. Ignition Simulation (world + robot + sensors)
    ignition_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sar_drone_pkg, 'launch', 'sar_ignition.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. System Health Monitor
    health_monitor = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="Starting system health monitor..."),
            Node(
                package='sar_missions',
                executable='topic_rate_monitor',
                name='health_monitor',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                # Respawn if crashes
                respawn=True,
                respawn_delay=2.0
            )
        ]
    )

    # 3. RTAB-Map SLAM (authoritative mapping)
    rtabmap = TimerAction(
        period=10.0,  # Give sensors time to publish
        actions=[
            LogInfo(msg="="*60),
            LogInfo(msg="Starting RTAB-Map SLAM..."),
            LogInfo(msg="This may take 10-15 seconds to initialize"),
            LogInfo(msg="="*60),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        sar_perception_pkg,
                        'launch',
                        'rgbd_rtabmap.launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )

    # RTAB-Map watchdog - monitors if RTAB-Map is publishing
    rtabmap_watchdog = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'timeout 5 ros2 topic echo /rtabmap/odom --once > /dev/null 2>&1 && '
                    'echo "[WATCHDOG] ✓ RTAB-Map is publishing odometry" || '
                    'echo "[WATCHDOG] ✗ WARNING: RTAB-Map not publishing yet"'
                ],
                name='rtabmap_watchdog',
                output='screen',
                shell=True
            )
        ]
    )

    # 5. EKF Localization (single source of TF)
    ekf = TimerAction(
        period=18.0,  # Wait for RTAB-Map odometry
        actions=[
            LogInfo(msg="="*60),
            LogInfo(msg="Starting EKF localization filter..."),
            LogInfo(msg="Fusing RTAB-Map odom + IMU data (with covariances)"),
            LogInfo(msg="="*60),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        sar_localization_pkg,
                        'launch',
                        'ekf.launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )

    # EKF watchdog
    ekf_watchdog = TimerAction(
        period=22.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'timeout 5 ros2 topic echo /odometry/filtered --once > /dev/null 2>&1 && '
                    'echo "[WATCHDOG] ✓ EKF is publishing filtered odometry" || '
                    'echo "[WATCHDOG] ✗ WARNING: EKF not publishing yet"'
                ],
                name='ekf_watchdog',
                output='screen',
                shell=True
            )
        ]
    )

    # 6. Nav2 Navigation Stack (DELAYED + TF CHECK)
    nav2_navigation = TimerAction(
        period=35.0,  # ← INCREASED from 25s to 35s
        actions=[
            LogInfo(msg="="*60),
            LogInfo(msg="Starting Nav2 navigation stack..."),
            LogInfo(msg="Waiting for stable TF tree..."),
            LogInfo(msg="="*60),
            # Add TF check before launching Nav2
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'timeout 5 ros2 run tf2_ros tf2_echo map base_link > /dev/null 2>&1 && '
                    'echo "[TF CHECK] ✓ map→base_link transform available" || '
                    'echo "[TF CHECK] ✗ WARNING: map→base_link not available - Nav2 may fail!"'
                ],
                name='nav2_tf_check',
                output='screen',
                shell=True
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        nav2_bringup_dir,
                        'launch',
                        'navigation_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # Nav2 lifecycle state check
    nav2_watchdog = TimerAction(
        period=40.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'ros2 lifecycle get /controller_server 2>/dev/null | grep -q "active" && '
                    'echo "[WATCHDOG] ✓ Nav2 controller is ACTIVE" || '
                    'echo "[WATCHDOG] ✗ WARNING: Nav2 controller not active - check TF tree!"'
                ],
                name='nav2_lifecycle_check',
                output='screen',
                shell=True
            )
        ]
    )

    # 7. RViz with custom config
    rviz_config = os.path.join(
        sar_drone_pkg,
        'rviz',
        'sar_drone_view.rviz'
    )
    
    rviz = TimerAction(
        period=28.0,
        actions=[
            LogInfo(msg="="*60),
            LogInfo(msg="Launching RViz visualization..."),
            LogInfo(msg="="*60),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # 8. TF Monitor (checks TF tree health)
    tf_monitor = TimerAction(
        period=32.0,
        actions=[
            LogInfo(msg="Monitoring TF tree (map → odom → base_link)..."),
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_ros', 'tf2_echo', 'map', 'base_link'],
                name='tf_monitor',
                output='screen'
            )
        ]
    )

    # Final status check
    system_ready = TimerAction(
        period=45.0,  # ← INCREASED from 35s to 45s (after Nav2 watchdog)
        actions=[
            LogInfo(msg="="*60),
            LogInfo(msg="SAR DRONE SIMULATION READY"),
            LogInfo(msg="="*60),
            LogInfo(msg=""),
            LogInfo(msg="🎯 USAGE:"),
            LogInfo(msg="  1. Use RViz '2D Nav Goal' to send navigation commands"),
            LogInfo(msg="  2. Monitor /diagnostics for system health"),
            LogInfo(msg="  3. Check /map topic for SLAM output"),
            LogInfo(msg="  4. View odometry at /odometry/filtered"),
            LogInfo(msg=""),
            LogInfo(msg="🔍 DEBUGGING:"),
            LogInfo(msg="  - Topics: ros2 topic list"),
            LogInfo(msg="  - Nodes: ros2 node list"),
            LogInfo(msg="  - TF: ros2 run tf2_tools view_frames"),
            LogInfo(msg="  - Nav2 State: ros2 lifecycle get /controller_server"),
            LogInfo(msg="  - EKF Rate: ros2 topic hz /odometry/filtered"),
            LogInfo(msg=""),
            LogInfo(msg="="*60),
            # Run comprehensive health check
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'echo "\n[HEALTH CHECK]" && '
                    'echo "Camera: $(ros2 topic hz /camera/image_raw --window 10 2>&1 | grep -q "average rate" && echo "✓" || echo "✗")" && '
                    'echo "LiDAR: $(ros2 topic hz /scan --window 10 2>&1 | grep -q "average rate" && echo "✓" || echo "✗")" && '
                    'echo "IMU: $(ros2 topic hz /imu/data --window 10 2>&1 | grep -q "average rate" && echo "✓" || echo "✗")" && '
                    'echo "IMU Corrected: $(ros2 topic hz /imu/data_corrected --window 10 2>&1 | grep -q "average rate" && echo "✓" || echo "✗")" && '
                    'echo "RTAB-Map: $(ros2 topic list | grep -q "/rtabmap/odom" && echo "✓" || echo "✗")" && '
                    'echo "EKF: $(ros2 topic list | grep -q "/odometry/filtered" && echo "✓" || echo "✗")" && '
                    'echo "Nav2: $(ros2 lifecycle get /controller_server 2>/dev/null | grep -q "active" && echo "✓ ACTIVE" || echo "✗ NOT ACTIVE")" && '
                    'echo "TF map→odom: $(timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "Translation" && echo "✓" || echo "✗")" && '
                    'echo "TF odom→base_link: $(timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation" && echo "✓" || echo "✗")"'
                ],
                output='screen',
                shell=True
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        
        # Startup
        startup_log,
        
        # Core simulation
        ignition_sim,
        
        # Monitoring
        health_monitor,
        
        # SLAM and localization
        rtabmap,
        rtabmap_watchdog,
        
        ekf,
        ekf_watchdog,
        
        # Navigation (DELAYED)
        nav2_navigation,
        nav2_watchdog,
        
        # Visualization
        rviz,
        tf_monitor,
        
        # Final status
        system_ready
    ])

