# leroy_bridge

ROS 2 Jazzy serial bridge for Leroy's MCU. It owns **hardware telemetry and
motor commands**, not the robot's normal odometry or sensor TF.

Publishes `/imu/data_raw` (`sensor_msgs/msg/Imu`, best effort, depth 10,
`imu_link`, nominal 100 Hz) and `/wheel/encoder_counts`
(`leroy_msgs/msg/EncoderCounts`, best effort, depth 10). Subscribes to
`/cmd_vel` (`geometry_msgs/msg/Twist`, reliable, depth 10). The MCU
timestamp/sequence fields are preserved; right-encoder values are never
fabricated. Raw IMU measurements use +X forward, +Y left, +Z up, rad/s and
m/s², with unavailable quaternion orientation
(`orientation_covariance[0] = -1`).

## Odometry ownership

The default configuration sets `degraded_odometry_enabled: false` and
`publish_odom_tf: false`. The node defaults match. When degraded odometry is
disabled, the bridge **does not create a /odom publisher or a dynamic TF
broadcaster**. The separate `leroy_xrslam` adapter owns `/odom` and dynamic
`odom -> base_link`.

The historical FL/RL-encoder plus IMU-yaw estimator remains source-available
only as a deliberately enabled diagnostic fallback. Never run it alongside
XRSLAM; enabling it does **not** happen automatically after tracking loss.

## Motor interlock

`max_wheel_angular_velocity_rad_s: 0.0` keeps motor output disabled. Do not
change this without an independent physical robot safety review. The bridge
still retains `watchdog_ms: 500`, `keepalive_period_s: 0.10`,
`cmd_vel_timeout_s: 0.25`, periodic command timeout checks and a stop on
shutdown. Wheel radius (0.020 m), half-wheelbase (0.180 m), and half-track
(0.275 m) remain necessary for the motor-control kinematics even with odometry
disabled.

## Timestamp validation

The MCU clock mapper unwraps uint32 microseconds and maps them into the ROS
clock domain. The actual live camera/IMU clock offset, timestamp monotonicity,
MCU wrap/reconnect behavior, scheduling latency and `use_sim_time` alignment
**remain to be measured** before XRSLAM pose output is enabled. Keep the
existing sequence-gap diagnostics. A rebooted MCU must not be mistaken for a
continuous VIO session.

See `ros/XRSLAM_INTEGRATION.md` for the staged live checks.
