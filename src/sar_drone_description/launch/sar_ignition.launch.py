import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    Command
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sar_drone = get_package_share_directory('sar_drone_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # -----------------------------
    # Launch arguments
    # -----------------------------
    world_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            pkg_sar_drone,
            'worlds',
            'sar_world.sdf'
        ])
    )

    world_file = LaunchConfiguration('world_file')

    urdf_file = os.path.join(
        pkg_sar_drone,
        'urdf',
        'sar_drone.urdf.xacro'
    )

    bridge_config = os.path.join(
        pkg_sar_drone,
        'config',
        'bridge.yaml'
    )

    # -----------------------------
    # Gazebo Fortress
    # -----------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-r '),
                world_file
            ]
        }.items()
    )

    # -----------------------------
    # Robot State Publisher
    # -----------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command([
                'xacro ',
                urdf_file
            ])
        }],
        output='screen'
    )

    # -----------------------------
    # Spawn Robot
    # -----------------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'sar_drone',
            '-topic', 'robot_description',
            '-z', '0.5'
        ],
        output='screen'
    )

    # -----------------------------
    # ROS ↔ Gazebo Bridge
    # -----------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True
        }],
        output='screen'
    )

    # -----------------------------
    # Image bridge
    # -----------------------------
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        image_bridge
    ])
