from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    publish_stamped = LaunchConfiguration("publish_stamped")
    linear_scale = LaunchConfiguration("linear_scale")
    angular_scale = LaunchConfiguration("angular_scale")
    deadzone = LaunchConfiguration("deadzone")
    turbo_multiplier = LaunchConfiguration("turbo_multiplier")
    button_turbo = LaunchConfiguration("button_turbo")

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
                "publish_stamped": ParameterValue(publish_stamped, value_type=bool),
                "cmd_vel_frame_id": "base_link",
                "linear_scale": ParameterValue(linear_scale, value_type=float),
                "angular_scale": ParameterValue(angular_scale, value_type=float),
                "deadzone": ParameterValue(deadzone, value_type=float),
                "turbo_multiplier": ParameterValue(turbo_multiplier, value_type=float),
                "axis_linear_x": 1,
                "axis_linear_y": 0,
                "axis_angular_z": 3,
                "button_turbo": ParameterValue(button_turbo, value_type=int),
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
            DeclareLaunchArgument(
                "linear_scale",
                default_value="0.7",
                description="Linear velocity scale in m/s at full stick.",
            ),
            DeclareLaunchArgument(
                "angular_scale",
                default_value="1.0",
                description="Angular velocity scale in rad/s at full stick.",
            ),
            DeclareLaunchArgument(
                "deadzone",
                default_value="0.1",
                description="Joystick deadzone threshold [0.0-1.0].",
            ),
            DeclareLaunchArgument(
                "turbo_multiplier",
                default_value="1.5",
                description="Multiplier when turbo button is pressed.",
            ),
            DeclareLaunchArgument(
                "button_turbo",
                default_value="5",
                description="Turbo button index. Set -1 to disable turbo mode.",
            ),
            joy_node,
            gamepad_controller,
        ]
    )
