from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory('sar_perception')
    params = os.path.join(pkg, 'config', 'rtabmap.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        # =====================================
        # RGB-D Synchronization
        # =====================================
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth_image'),
                ('rgb/camera_info', '/camera/camera_info'),
            ],
            output='screen'
        ),

        # =====================================
        # RGB-D Odometry (CREATES odom -> base_link TF)
        # =====================================
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth_image'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('odom', '/odom'),
            ],
            output='screen'
        ),

        # =====================================
        # RTAB-Map SLAM
        # =====================================
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('rgbd_image', '/rgbd_sync/rgbd_image'),
                ('scan', '/scan'),
                ('imu', '/imu/data'),
                ('odom', '/odom'),
            ],
            arguments=['--delete_db_on_start'],
            output='screen'
        ),
    ])

