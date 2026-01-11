from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav2_bridge_node = Node(
        package='sar_nav_bridge',
        executable='nav2_to_mavros',
        name='nav2_to_mavros_bridge',
        output='screen'
    )

    return LaunchDescription([
        nav2_bridge_node
    ])

