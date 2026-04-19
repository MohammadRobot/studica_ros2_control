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

### Titan Control Modes

Drive components (diff and mecanum) support `titan_control_mode` in the config:

| Mode | Titan API | PID Type | Description |
|------|-----------|----------|-------------|
| `open_loop` | `SetSpeed` | 0 | Duty-cycle control (0.0–1.0). No encoder feedback. |
| `velocity` | `SetTargetVelocity` | 1 | Closed-loop wheel speed (RPM). Titan PID runs on-device. |
| `position` | `SetTargetDistance` | 2 | Closed-loop position (encoder counts). Position hold enabled on-device. |

If `position` is selected but the connected firmware does not support PID type `2`,
the node automatically falls back to `velocity` mode (PID type `1`) and logs a warning.

Default is `velocity`. Change in `config/params.yaml`:

```yaml
diff_drive_component:
  titan_control_mode: "velocity"   # "open_loop" | "velocity" | "position"
```

> **Keep-alive:** The Titan device disables motors if no CAN message is received for 200 ms.
> The `/cmd_vel` subscriber resends commands at ~50 ms, well within this window.
> If `/cmd_vel` stops publishing, motors will time out and brake automatically.

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

## titan_cmd Service

Both `diff_drive` and `mecanum_drive` nodes expose a `titan_cmd` service (`studica_ros2_control/srv/SetData`).
The `params` field selects the command; additional fields come from `name`, `command`, and `initparams`.

| `params` value | Fields used | Action |
|----------------|-------------|--------|
| `enable` | — | Enable Titan (motors armed) |
| `disable` | — | Disable Titan (motors stop immediately) |
| `start` | — | Alias for `enable` |
| `set_mode` | `name` (or `command` as fallback) = `"open_loop"/"velocity"/"position"` | Switch control mode at runtime |
| `set_target_velocity` | `initparams.n_encoder`, `initparams.speed` (RPM) | Directly command one motor's RPM |
| `set_speed` | `initparams.n_encoder`, `initparams.speed` | Mode-aware: duty (open_loop), RPM (velocity), tick offset (position) |
| `stop` | `initparams.n_encoder` | Mode-aware stop for one motor |
| `reset` | `initparams.n_encoder` | Reset encoder count to 0 |
| `setup_encoder` | `initparams.n_encoder` | Run encoder setup sequence |
| `configure_encoder` | `initparams.n_encoder`, `initparams.dist_per_tick` | Set distance-per-tick calibration |
| `get_encoder_distance` | `initparams.n_encoder` | Returns distance (m) as string in `message` |
| `get_rpm` | `initparams.n_encoder` | Returns current RPM as string in `message` |
| `get_encoder_count` | `initparams.n_encoder` | Returns raw encoder count as string in `message` |
| `autotune` | — | Run on-device PID autotune across all motors (**motors will move**) |
| `set_sensitivity` | `initparams.n_encoder`, `initparams.speed` (0–255) | Adjust PID aggressiveness for one motor |

### PID Tuning

The Titan stores PID gains on-device. There are no raw kP/kI/kD registers — tuning is done via autotune or sensitivity adjustment.

**Autotune** (run once per physical robot setup, not needed on every boot):

> **Warning:** Motors will spin during autotune. Lift the robot off the ground first.

```bash
# 1. Switch to velocity mode so the Titan PID is active
ros2 service call /titan_cmd studica_ros2_control/srv/SetData \
  "{params: 'set_mode', name: 'velocity'}"

# 2. Run autotune — gains are saved on the device
ros2 service call /titan_cmd studica_ros2_control/srv/SetData \
  "{params: 'autotune'}"
```

**Sensitivity** (fine-tune aggressiveness per motor, 0–255, higher = more aggressive):

```bash
ros2 service call /titan_cmd studica_ros2_control/srv/SetData \
  "{params: 'set_sensitivity', initparams: {n_encoder: 0, speed: 128.0}}"
```

---

Example — switch to velocity mode at runtime:

```bash
ros2 service call /titan_cmd studica_ros2_control/srv/SetData \
  "{params: 'set_mode', name: 'velocity'}"
```

Example — read encoder count for motor 0:

```bash
ros2 service call /titan_cmd studica_ros2_control/srv/SetData \
  "{params: 'get_encoder_count', initparams: {n_encoder: 0}}"
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
