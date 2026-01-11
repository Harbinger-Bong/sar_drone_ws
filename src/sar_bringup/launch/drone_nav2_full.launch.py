from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    sar_bringup_pkg = get_package_share_directory('sar_bringup')
    sar_perception_pkg = get_package_share_directory('sar_perception')
    sar_localization_pkg = get_package_share_directory('sar_localization')
    sar_nav_bridge_pkg = get_package_share_directory('sar_nav_bridge')
    sar_drone_pkg = get_package_share_directory('sar_drone_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration files
    # Updated configs should support Humble
    nav2_params = os.path.join(sar_drone_pkg, 'config', 'nav2_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    fcu_url = LaunchConfiguration('fcu_url')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_fcu_url = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM0:921600',
        description='FCU connection URL (e.g., /dev/ttyACM0:921600)'
    )

    # 1. Real Hardware Sensors
    drone_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sar_bringup_pkg, 'launch', 'drone_sensors.launch.py')
        ),
        launch_arguments={
            'fcu_url': fcu_url
        }.items()
    )

    # 2. RTAB-Map SLAM (delayed to ensure sensors are running)
    rtabmap_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sar_perception_pkg, 'launch', 'rgbd_rtabmap.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )

    # 3. EKF Localization
    ekf_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sar_localization_pkg, 'launch', 'ekf.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )

    # 4. Nav2 Navigation Stack
    nav2_navigation = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # 5. Nav2 to MAVROS Bridge
    nav2_bridge = TimerAction(
        period=9.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sar_nav_bridge_pkg, 'launch', 'nav2_bridge.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_fcu_url,
        drone_sensors,
        rtabmap_launch,
        ekf_launch,
        nav2_navigation,
        nav2_bridge
    ])