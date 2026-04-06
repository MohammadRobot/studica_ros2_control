"""Launch NAV2 with dual LiDAR and merged scan input."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_share = get_package_share_directory("studica_ros2_control")
    lidar_1_file = LaunchConfiguration("params_file1")
    lidar_2_file = LaunchConfiguration("params_file2")

    params1_arg = DeclareLaunchArgument(
        "params_file1",
        default_value=os.path.join(pkg_share, "config", "ydlidar_f.yaml"),
        description="Parameters file for YDLidar #1",
    )
    params2_arg = DeclareLaunchArgument(
        "params_file2",
        default_value=os.path.join(pkg_share, "config", "ydlidar_r.yaml"),
        description="Parameters file for YDLidar #2",
    )

    nav2_params_file = "/home/vmx/ROS2/nav2_params/nav2_params.yaml"
    office_map_file = "/home/vmx/ROS2/office.yaml"

    base_tf = ExecuteProcess(
        cmd=[[
            "ros2 run tf2_ros static_transform_publisher "
            "0 0 0 0 0 0 base_footprint base_link"
        ]],
        shell=True,
    )

    laser_tf = ExecuteProcess(
        cmd=[[
            "ros2 run tf2_ros static_transform_publisher "
            "--x 0.144 --y 0 --z 0 "
            "--qx 0 --qy 0 --qz 0.7071 --qw -0.7071 "
            "--frame-id base_link --child-frame-id laser_frame"
        ]],
        shell=True,
    )

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

    merge = ExecuteProcess(
        cmd=["ros2", "launch", "ros2_laser_scan_merger", "merge_2_scan.launch.py"],
        output="screen",
    )

    nav_localization = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "nav2_bringup",
            "localization_launch.py",
            f"map:={office_map_file}",
            "use_sim_time:=false",
            f"params_file:={nav2_params_file}",
        ],
        output="screen",
    )

    navigation = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "nav2_bringup",
            "navigation_launch.py",
            "use_sim_time:=false",
            f"params_file:={nav2_params_file}",
        ],
        output="screen",
    )

    tf3 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_laser_merged",
        arguments=["0", "0", "0.02", "0", "0", "0", "1", "base_link", "laser"],
    )

    return LaunchDescription(
        [
            params1_arg,
            params2_arg,
            LogInfo(msg=["Using YDLidar #1 params: ", lidar_1_file]),
            LogInfo(msg=["Using YDLidar #2 params: ", lidar_2_file]),
            base_tf,
            laser_tf,
            tf1,
            lidar1,
            tf2,
            lidar2,
            merge,
            tf3,
            nav_localization,
            navigation,
        ]
    )
