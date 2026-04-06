"""Autonomous mapping launch with dual LiDAR, camera fusion, and SLAM."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_share = get_package_share_directory("studica_ros2_control")

    resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.04",
        description=(
            "SLAM map resolution in meters per pixel "
            "(e.g., 0.02 for 2 cm cells, 0.05 for 5 cm cells)."
        ),
    )

    lidar_1_file = os.path.join(pkg_share, "config", "ydlidar_f.yaml")
    lidar_2_file = os.path.join(pkg_share, "config", "ydlidar_r.yaml")
    mapper_params_file = os.path.join(
        os.path.dirname(pkg_share),
        "..",
        "..",
        "..",
        "nav2_params",
        "mapper_params_online_sync.yaml",
    )
    merger_params_file = os.path.join(pkg_share, "config", "merger_params.yaml")

    lidar1 = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar1",
        emulate_tty=True,
        output="screen",
        parameters=[lidar_1_file],
        remappings=[("scan", "/scan1")],
        namespace="/",
    )

    tf1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_laser1",
        arguments=["0.144", "0", "0.02", "0", "0", "0", "1", "base_link", "laser1_frame"],
    )

    lidar2 = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar2",
        output="screen",
        emulate_tty=True,
        parameters=[lidar_2_file],
        remappings=[("scan", "/scan2")],
        namespace="",
    )

    tf2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_laser2",
        arguments=["-0.144", "0", "0.02", "0", "0", "1", "0", "base_link", "laser2_frame"],
    )

    camera = ExecuteProcess(
        cmd=["ros2", "launch", "orbbec_camera", "gemini_e.launch.py"],
        output="screen",
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        remappings=[
            ("cloud_in", "/merged_cloud"),
            ("scan", "/merged_scan"),
        ],
        parameters=[mapper_params_file],
    )

    merger = Node(
        package="ros2_laser_scan_merger",
        executable="ros2_laser_scan_merger",
        name="ros2_laser_scan_merger",
        output="screen",
        parameters=[merger_params_file],
    )

    tf3 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_laser_merged",
        arguments=["0", "0", "0.02", "0", "0", "0", "1", "base_link", "laser"],
    )

    slam = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[mapper_params_file, {"resolution": LaunchConfiguration("resolution")}],
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
    )

    return LaunchDescription(
        [
            resolution_arg,
            tf1,
            lidar1,
            tf2,
            lidar2,
            camera,
            pointcloud_to_scan,
            merger,
            tf3,
            slam,
            foxglove,
        ]
    )
