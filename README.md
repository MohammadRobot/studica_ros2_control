# studica_ros2_control

ROS 2 control server for Studica robot components (IMU, ultrasonic, servo, Titan, etc.).
This package wraps the Studica VMXPi driver library and publishes ROS topics/services.

## Requirements

- VMXPi library installed on the target machine.
- `studica_drivers` built in the same workspace.
- `joy` package for joystick input (`sudo apt install ros-humble-joy`).

If VMXPi is not available, `studica_drivers` builds as a stub.
In that case, `studica_ros2_control` still builds joystick teleop nodes, but hardware components are skipped.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select studica_drivers studica_ros2_control
source install/setup.bash
```

## Configuration

Hardware components are configured in:

- `config/params.yaml`

Enable or disable components by setting `enabled: true/false` under each section.

## Launch

Default launch (full stack from Studica):

```bash
ros2 launch studica_ros2_control studica_launch.py
```

Sensor-only launch (recommended when using ros2_control for drive):

```bash
ros2 launch studica_ros2_control sensors_only.launch.py
```

Gamepad launch (works in simulation and hardware):

```bash
ros2 launch studica_ros2_control gamepad_launch.py cmd_vel_topic:=/diffbot_base_controller/cmd_vel publish_stamped:=true
```

## Using With studica_vmxpi_ros2

If you use `studica_vmxpi_ros2` for ros2_control drive, keep drive controllers disabled in this package to avoid conflicts.
Use `config/params_sensors.yaml` with `sensors_only.launch.py`.

## Notes

If you modify the low-level drivers, rebuild `studica_drivers` before launching:

```bash
colcon build --packages-select studica_drivers
```

Example of checking data:

```bash
ros2 topic list
ros2 topic echo /imu
```
