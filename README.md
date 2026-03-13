# wheelbot_sim

A ROS 2 node that simulates a differential-drive robot. It subscribes to joystick commands and publishes odometry and IMU data using unicycle kinematics.

Designed for use with [Emoco Studio](https://emoco.com)'s teleop panel during development and testing when no physical robot is available.

## Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `cmd_joy` | `sensor_msgs/Joy` | Joystick input — `axes[1]` = forward/back, `axes[3]` = turn |
| Publish | `sim/odom` | `nav_msgs/Odometry` | Pose and twist in the `odom` frame |
| Publish | `sim/imu` | `sensor_msgs/Imu` | Orientation, angular velocity, and simulated accelerometer with noise |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_vel` | `1.0` | Maximum forward speed (m/s) |
| `max_angular_vel` | `2.0` | Maximum turn rate (rad/s) |
| `sim_rate` | `10.0` | Simulation loop frequency (Hz) |

## Safety

A command watchdog zeros the velocity if no `cmd_joy` message is received within 500 ms.

## Build & run

```bash
# Build
cd <colcon_ws>
colcon build --packages-select wheelbot_sim

# Run
ros2 run wheelbot_sim wheelbot_sim

# Or via launch file
ros2 launch wheelbot_sim start.launch.py
```

## Quick test

```bash
# Publish a joystick message (forward + slight turn)
ros2 topic pub /cmd_joy sensor_msgs/msg/Joy \
  '{axes: [0.0, 0.5, 0.0, 0.3]}' --rate 10

# Watch output
ros2 topic echo /sim/odom
```
