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

        # Static transform bridge for Gazebo camera frame naming
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_frame_fix',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'sar_drone/base_link/rgbd'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Static transform bridge for Gazebo LiDAR frame naming
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_frame_fix',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'sar_drone/base_link/rplidar'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # RGB-D Synchronization
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            parameters=[{
                'use_sim_time': use_sim_time,
                'approx_sync': True,
                'queue_size': 30,
                'approx_sync_max_interval': 0.1,
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth_image'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('rgbd_image', '/rgbd_sync/rgbd_image'),
            ],
            output='screen'
        ),

        # RGB-D Odometry
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,  # ← FALSE: Let EKF publish odom→base_link
                'wait_for_transform': 0.5,
                'approx_sync': True,
                'queue_size': 30,
                'subscribe_rgbd': True,
                'rgbd_cameras': 1,
                'Odom/Strategy': 0,
                'Odom/GuessMotion': True,
                'Odom/ResetCountdown': 15,
                'OdomF2M/MaxSize': 2000,
                'Vis/MinInliers': 15,
                'Vis/MaxDepth': 8.0,
                'Vis/FeatureType': 6,
                'Vis/EstimationType': 1,
            }],
            remappings=[
                ('rgbd_image', '/rgbd_sync/rgbd_image'),
                ('odom', '/rtabmap/odom'),
            ],
            output='screen'
        ),

        # RTAB-Map SLAM
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
                    'map_frame_id': 'map',  # ← EXPLICIT MAP FRAME
                    'publish_tf': True,     # ← TRUE: Publish map→odom transform
                    'wait_for_transform': 0.5,
                    'subscribe_rgbd': True,
                    'subscribe_odom': True,
                    'rgbd_cameras': 1,
                    'subscribe_scan': True,
                    'approx_sync': True,
                    'queue_size': 30,
                    'Rtabmap/DetectionRate': '1.0',
                    'RGBD/NeighborLinkRefining': 'true',
                    'RGBD/ProximityBySpace': 'true',
                    'RGBD/OptimizeFromGraphEnd': 'false',
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

