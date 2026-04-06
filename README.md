# studica_ros2_control

ROS 2 control server for Studica robot components (IMU, ultrasonic, servo, Titan, etc.).
This package wraps the Studica VMXPi driver library and publishes ROS topics/services.

## Attribution

Portions of this package are derived from
[`Studica-Robotics/ROS2`](https://github.com/Studica-Robotics/ROS2),
licensed under Apache-2.0.

## Maintainer

- Mohammad Alshamsi (`alshamsi.mohammad@gmail.com`)

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

Real-robot conservative speed example:

```bash
ros2 launch studica_ros2_control gamepad_launch.py \
  cmd_vel_topic:=/diffbot_base_controller/cmd_vel \
  publish_stamped:=true \
  linear_scale:=0.20 \
  angular_scale:=0.60 \
  turbo_multiplier:=1.0 \
  button_turbo:=-1
```

Available tuning arguments:

- `linear_scale` (default `0.7`): linear speed at full stick in m/s.
- `angular_scale` (default `1.0`): angular speed at full stick in rad/s.
- `deadzone` (default `0.1`): joystick deadzone.
- `turbo_multiplier` (default `1.5`): multiplier when turbo is pressed.
- `button_turbo` (default `5`): turbo button index; set `-1` to disable turbo mode.

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
