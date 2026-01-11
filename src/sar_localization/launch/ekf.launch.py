from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    sar_localization_pkg = get_package_share_directory('sar_localization')
    ekf_config = os.path.join(sar_localization_pkg, 'config', 'ekf.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # EKF Filter Node
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_filter_node
    ])

