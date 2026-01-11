from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo,
    EmitEvent
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import lifecycle_msgs.msg


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

    # 1. Ignition Simulation (world + robot + sensors)
    ignition_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sar_drone_pkg, 'launch', 'sar_ignition.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Log when simulation starts
    on_sim_start = RegisterEventHandler(
        OnProcessStart(
            target_action=ignition_sim,
            on_start=[
                LogInfo(msg="Ignition Gazebo simulation started"),
                LogInfo(msg="Waiting for sensors to initialize...")
            ]
        )
    )

    # 2. System Health Monitor
    health_monitor = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='sar_missions',
                executable='topic_rate_monitor.py',
                name='health_monitor',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # 3. RTAB-Map SLAM (authoritative mapping)
    rtabmap = TimerAction(
        period=8.0,  # Give sensors time to publish
        actions=[
            LogInfo(msg="Starting RTAB-Map SLAM..."),
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

    # 4. EKF Localization (single source of TF)
    ekf = TimerAction(
        period=12.0,  # Wait for RTAB-Map odometry
        actions=[
            LogInfo(msg="Starting EKF localization filter..."),
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

    # 5. Nav2 Navigation Stack
    nav2_navigation = TimerAction(
        period=16.0,  # Wait for map and TF
        actions=[
            LogInfo(msg="Starting Nav2 navigation stack..."),
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

    # 6. RViz with custom config
    rviz_config = os.path.join(
        sar_drone_pkg,
        'rviz',
        'sar_drone_view.rviz'
    )
    
    rviz = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg="Launching RViz visualization..."),
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

    # 7. TF Monitor (checks TF tree health)
    tf_monitor = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg="System ready. Monitoring TF tree..."),
            Node(
                package='tf2_ros',
                executable='tf2_echo',
                name='tf_monitor',
                arguments=['map', 'base_link'],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # Shutdown handler for critical failures
    shutdown_on_rtabmap_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rtabmap,
            on_exit=[
                LogInfo(msg="RTAB-Map crashed! Shutting down system..."),
                EmitEvent(event=Shutdown(reason='RTAB-Map failure'))
            ]
        )
    )

    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        
        # Core simulation
        ignition_sim,
        on_sim_start,
        
        # Monitoring
        health_monitor,
        
        # SLAM and localization
        rtabmap,
        ekf,
        
        # Navigation
        nav2_navigation,
        
        # Visualization
        rviz,
        tf_monitor,
        
        # Event handlers
        shutdown_on_rtabmap_exit,
        
        # Startup complete message
        TimerAction(
            period=30.0,
            actions=[
                LogInfo(msg="="*60),
                LogInfo(msg="SAR DRONE SIMULATION READY"),
                LogInfo(msg="="*60),
                LogInfo(msg="Use RViz 2D Nav Goal to send navigation commands"),
                LogInfo(msg="Monitor /diagnostics for system health"),
                LogInfo(msg="="*60),
            ]
        )
    ])
