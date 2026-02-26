from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    publish_stamped = LaunchConfiguration("publish_stamped")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy",
        output="screen",
        parameters=[
            {
                "autorepeat_rate": 20.0,
                "deadzone": 0.1,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    gamepad_controller = Node(
        package="studica_ros2_control",
        executable="gamepad_teleop",
        name="gamepad_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "cmd_vel_topic": cmd_vel_topic,
                "publish_stamped": publish_stamped,
                "cmd_vel_frame_id": "base_link",
                "linear_scale": 0.7,
                "angular_scale": 1.0,
                "deadzone": 0.1,
                "turbo_multiplier": 1.5,
                "axis_linear_x": 1,
                "axis_linear_y": 0,
                "axis_angular_z": 3,
                "button_turbo": 5,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time for joystick nodes.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/diffbot_base_controller/cmd_vel",
                description="Velocity command topic published by gamepad teleop.",
            ),
            DeclareLaunchArgument(
                "publish_stamped",
                default_value="true",
                description="Publish geometry_msgs/TwistStamped instead of Twist.",
            ),
            joy_node,
            gamepad_controller,
        ]
    )
