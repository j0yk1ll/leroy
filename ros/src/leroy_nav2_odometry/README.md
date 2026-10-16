# Milestone 3 — isolated synthetic mecanum odometry and TF

**Not measured wheel odometry, not a real localization stack, not a motion
controller, and not collision-safe.** This milestone demonstrates the ROS 2
odometry interface and forward/lateral/yaw kinematics in an entirely
disconnected fake-wheel fixture. Do not connect the output to Leroy's
`leroy-ros` container, hardware bridge, or motor controls.

The previous M1 controller stays unconfigured; this fixture launches NO
Nav2 controller, planner, navigator, lifecycle manager or costmap. M2's
fake depth fixture is unaffected. Nothing publishes `/cmd_vel`.

## Contract

- **Synthetic only:** Wheel order front-left, front-right, rear-left,
  rear-right; X-roller pattern; guessed wheel radius 0.05 m and fictitious
  half-length/width of 0.25/0.20 m. NOT Leroy measurements or motor signs.
- Deterministic cycling sequence: stopped (0–2 s), forward x (2–5 s),
  lateral y (5–8 s), yaw (8–11 s), stopped (11–14 s). Values are
  conservative *simulation signals*, not validated speed limits.
- Forward mecanum wheel kinematics and midpoint planar odometry,
  bounded `dt` (0 to 0.25 s, strict), finite wheel-speed validation.
  Stalls/time jumps cause a frame to be skipped rather than extrapolated.
- Publish only `/leroy_nav2/diagnostics/synthetic_odom` as
  `nav_msgs/msg/Odometry`; pose is in `odom` frame, body twist in
  `base_link`. Publish matching **synthetic** `odom → base_link`
  on `/tf`, with identical timestamps and pose.
- Nonzero covariances are **illustrative only**. They are not calibrated
  state-estimation uncertainties. No `/odom` topic is created.
- Real encoders, IMU calibration, XRSLAM, frame authority, wheel slip,
  roller efficiency, floor traction, wheel ordering/signs, physical
  stop distance and current-state estimator quality remain unvalidated.
  All are blocked from hardware usage pending separate work.

## Build a separate Podman image (Arch Linux checkout)

```bash
git fetch origin
git switch feature/nav2-m3-offline-mecanum-odom-tf
cd ros
podman build -f Dockerfile.nav2 -t leroy-nav2:jazzy .
```

Build must compile and test all four optional packages. No live
`leroy-ros` container, serial port or device mapping is involved.

## Terminal 1: disconnected synthetic fixture

```bash
podman run --rm -it --name leroy-nav2-m3 \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:jazzy bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_odometry odom_fixture.launch.py
  '
```

## Terminal 2: finite odometry/TF acceptance

```bash
podman exec leroy-nav2-m3 bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  ros2 run leroy_nav2_odometry odom_probe
  echo "=== Nodes ==="
  ros2 node list
  echo "=== Topics ==="
  ros2 topic list
'
```

Expected probe line:
```
PASS: synthetic forward, lateral and yaw odom; matching odom->base_link TF and acquisition stamps; no /cmd_vel, /odom or controller
```

Probe waits for a full cycle (~14 s), tolerates late joins (up to 22 s),
checks all three motion components, stopped samples, same-stamp TF and
odometry pose, valid quaternions, finite state and nonzero synthetic
covariance. It fails if it observes `/cmd_vel`, `/odom`,
`controller_server`, or hardware bridge nodes.

## Safety and acceptance boundary

The only mandatory isolation is separate `podman --network none` without
host PID, host IPC, hardware, or device access. `ROS_DOMAIN_ID=167`,
localhost DDS discovery, and the launch+node environment guards are
defense in depth and cannot verify Podman flags.

**M3 acceptance** requires image build and all tests, a passing ROS
synthetic probe, `/cmd_vel` absent, no live hardware graph and clean
synthetic node shutdown. Do not merge PR or begin hardware/real-sensor
odometry. No activation of controller_server or FollowPath goals.
