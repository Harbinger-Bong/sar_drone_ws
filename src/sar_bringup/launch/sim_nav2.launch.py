from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    sar_drone_pkg = get_package_share_directory('sar_drone_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Configuration files
    nav2_params = os.path.join(sar_drone_pkg, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(sar_drone_pkg, 'config', 'slam_params.yaml')
    ekf_params = os.path.join(sar_drone_pkg, 'config', 'ekf.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time if true'
    )

    # 1. Start Ignition Simulation (Replaces Gazebo Classic)
    # This launches Fortress, Spawns Robot, and Starts Bridge
    ignition_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sar_drone_pkg, 'launch', 'sar_ignition.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. EKF for odometry fusion 
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 3. SLAM TOOLBOX 
    # Use online_async_launch from the installed slam_toolbox package
    slam_toolbox = TimerAction(
        period=5.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'params_file': slam_params,
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )

    # 4. NAV2 STACK
    nav2_navigation = TimerAction(
        period=10.0, 
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

    # 5. RVIZ
    rviz = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d',
                    os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        ignition_sim,
        ekf_node,
        slam_toolbox,
        nav2_navigation,
        rviz
    ])