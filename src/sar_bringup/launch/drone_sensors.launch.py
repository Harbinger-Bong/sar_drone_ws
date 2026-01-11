from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    fcu_url = LaunchConfiguration('fcu_url')
    declare_fcu_url = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM0:921600',
        description='FCU connection URL (e.g., /dev/ttyACM0:921600)'
    )

    # 1. MAVROS PX4
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavros'),
                'launch',
                'px4.launch.py'
            )
        ),
        launch_arguments={
            'fcu_url': fcu_url
        }.items()
    )

    # 2. RealSense D435
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'camera_name': 'camera',
            'enable_color': 'true',
            'enable_depth': 'true',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30',
            'enable_sync': 'true',
            'align_depth.enable': 'true'
        }.items()
    )

    # 3. RPLidar A1
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar.launch.py'
            )
        )
    )

    return LaunchDescription([
        declare_fcu_url,
        mavros_launch,
        realsense_launch,
        rplidar_launch
    ])

