"""Autonomous mapping launch with LiDAR and slam_toolbox."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("studica_ros2_control")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    manual_composition = Node(
        package="studica_ros2_control",
        executable="manual_composition",
        name="control_server",
        output="screen",
        parameters=[params_file],
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
    )

    base_tf = ExecuteProcess(
        cmd=[[
            "ros2 run tf2_ros static_transform_publisher "
            "0 0 0 0 0 0 base_footprint base_link"
        ]],
        shell=True,
    )

    lidar = ExecuteProcess(
        cmd=["ros2", "launch", "ydlidar_ros2_driver", "ydlidar_launch.py"],
        output="screen",
    )

    slam = ExecuteProcess(
        cmd=["ros2", "launch", "slam_toolbox", "online_sync_launch.py"],
        output="screen",
    )

    joy_node = ExecuteProcess(
        cmd=["ros2", "run", "joy", "game_controller_node"],
        output="screen",
    )

    return LaunchDescription([
        foxglove,
        manual_composition,
        base_tf,
        lidar,
        slam,
        joy_node,
    ])
