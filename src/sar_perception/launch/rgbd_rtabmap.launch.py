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
        # Subscribes to individual topics and outputs synchronized rgbd_image
        # =====================================
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            parameters=[{
                'use_sim_time': use_sim_time,
                'approx_sync': True,
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth_image'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('rgbd_image', '/rgbd_sync/rgbd_image'),
            ],
            output='screen'
        ),

        # =====================================
        # RGB-D Odometry (CREATES odom -> base_link TF)
        # Visual odometry from RGB-D camera
        # =====================================
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': True,
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth_image'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('odom', '/rtabmap/odom'),
            ],
            output='screen'
        ),

        # =====================================
        # RTAB-Map SLAM
        # Uses synchronized rgbd_image + scan + odom
        # =====================================
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[
                params,
                {
                    'use_sim_time': use_sim_time,
                    'frame_id': 'base_link',
                    'odom_frame_id': 'odom',
                    'map_frame_id': 'map',
                    'subscribe_rgbd': True,
                    'subscribe_scan': True,
                    'approx_sync': True,
                }
            ],
            remappings=[
                ('rgbd_image', '/rgbd_sync/rgbd_image'),
                ('scan', '/scan'),
                ('imu', '/imu/data'),
                ('odom', '/rtabmap/odom'),
            ],
            arguments=['--delete_db_on_start'],
            output='screen'
        ),
    ])
