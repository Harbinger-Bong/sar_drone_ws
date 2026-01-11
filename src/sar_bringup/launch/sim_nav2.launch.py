from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Package directories
    sar_drone_pkg = get_package_share_directory('sar_drone_description')
    sar_perception_pkg = get_package_share_directory('sar_perception')
    sar_localization_pkg = get_package_share_directory('sar_localization')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Config files
    nav2_params = os.path.join(sar_drone_pkg, 'config', 'nav2_params.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # 1. Ignition Simulation (world + robot + sensors)
    ignition_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sar_drone_pkg, 'launch', 'sar_ignition.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. RTAB-Map SLAM (authoritative mapping)
    rtabmap = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        sar_perception_pkg,
                        'launch',
                        'rgbd_rtabmap.launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )

    # 3. EKF Localization (sar_localization — single source of TF)
    ekf = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        sar_localization_pkg,
                        'launch',
                        'ekf.launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )

    # 4. Nav2 (PLANNING ONLY)
    nav2_navigation = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        nav2_bringup_dir,
                        'launch',
                        'navigation_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # 5. RViz
    rviz = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d',
                    os.path.join(
                        nav2_bringup_dir,
                        'rviz',
                        'sar_nav2_view.rviz'
                    )
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        ignition_sim,
        rtabmap,
        ekf,
        nav2_navigation,
        rviz
    ])

