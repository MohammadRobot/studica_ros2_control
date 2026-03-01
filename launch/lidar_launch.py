"""
LiDAR-only launch wrapper for ydlidar_ros2_driver.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ydlidar_params_file = LaunchConfiguration('ydlidar_params_file')
    ydlidar_params_arg = DeclareLaunchArgument(
        'ydlidar_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('ydlidar_ros2_driver'), 'params', 'X2.yaml']
        ),
        description='Parameters file for ydlidar_ros2_driver ydlidar_launch.py',
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py']
            )
        ),
        launch_arguments={'params_file': ydlidar_params_file}.items(),
    )

    return LaunchDescription([
        ydlidar_params_arg,
        LogInfo(msg=['Using YDLidar params: ', ydlidar_params_file]),
        lidar,
    ])
