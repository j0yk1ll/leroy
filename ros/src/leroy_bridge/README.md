# leroy_bridge

ROS 2 Jazzy hardware bridge for the Leroy MCU serial protocol.

## ROS interfaces

Publishes:

- `/imu/data_raw` (`sensor_msgs/msg/Imu`)
- `/wheel/encoder_counts` (`leroy_msgs/msg/EncoderCounts`)

Subscribes:

- `/cmd_vel` (`geometry_msgs/msg/Twist`)

The MCU is configured for timestamped IMU and encoder streaming at startup. The
bridge also sends keepalives for the MCU watchdog and stops the wheels if
`/cmd_vel` times out.

`/cmd_vel` motor output remains disabled until the mecanum geometry and maximum
wheel angular velocity are configured in `config/leroy_bridge.yaml`.

The encoder message preserves the MCU acquisition timestamp and sequence number.
Once encoder ticks/revolution and wheel geometry are calibrated, the next step is
to publish `sensor_msgs/msg/JointState` and `nav_msgs/msg/Odometry`.


## Temporary degraded odometry

While the front-right and rear-right encoders are unavailable, the bridge can
publish `/odom` using the two working left encoders plus IMU yaw rate. Raw
`/wheel/encoder_counts` is never mirrored or synthesized.

Current calibration:

- wheel radius: 0.020 m
- half wheelbase: 0.180 m
- half track: 0.275 m
- encoder counts/revolution: 2464
- front-left/rear-left encoder sign: -1

The estimator is intentionally temporary and uses conservative covariance.
Replace it with normal four-wheel mecanum forward kinematics once the right
encoders are repaired.
