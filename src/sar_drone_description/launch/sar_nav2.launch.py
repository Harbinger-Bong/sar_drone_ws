from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('sar_drone_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration files
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # Launch files
    slam_launch_file = os.path.join(pkg_share, 'launch', 'slam.launch.py')
    world_launch_file = os.path.join(pkg_share, 'launch', 'sar_world.launch.py')
    
    # Launch arguments
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time if true'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=nav2_params,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'
    )

    # Variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Include Gazebo world launch (robot and environment)
    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch_file)
    )
    
    # Include SLAM launch file
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Launch Nav2 bringup WITHOUT slam (we're handling it separately)
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=None,
            namespace=namespace),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_respawn': use_respawn,
                'log_level': log_level
            }.items()
        )
    ])



    return LaunchDescription([
        # Launch arguments
        declare_namespace,
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_use_respawn,
        declare_log_level,
        
        # Launch nodes
        world_cmd,
        slam_cmd,
        bringup_cmd_group
    ])