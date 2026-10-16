# M7 — disconnected shadow-veto policy replay (NOT a motor gate)

M1–M5 proved isolated synthetic Nav2 interfaces and a diagnostic-only
MPPI proposal. M6 proved a **read-only audit** could distinguish
healthy fake data from missing odometry/depth and detect recovery.

**M7 DOES NOT connect these diagnostics to the robot.** It is the
first *pure policy design/replay* for how an eventual separate
authorization layer might reject proposals. It is NOT a deployable
watchdog, a verified safety interlock, a drive controller, or evidence
for navigation in the real world. The M7 executable imports **no ROS**;
it cannot publish any `Twist`, action goal, topic, bridge or command.

## Source contract

`leroy_nav2_integration.shadow_policy` consumes invented numeric
snapshots with these evidence fields:

- Odometry and depth: both monotonic receive age AND acquisition-stamp
  age must be <=1.5 seconds using the previously tested M6 freshness
  routine; reject future timestamps exceeding 0.1 seconds.
- Costmap: requires a recently received, marked+unknown synthetic map
  flag from a **hypothetical** upstream observer. This does not imply
  safe free-space coverage: unknown space remains unknown.
- MPPI candidate: synthetic, **unstamped** `Twist` values with local
  receive age <=0.4 seconds. Reject missing, stale/future, NaN/Inf,
  non-planar and values outside arbitrary M5 numerical bounds of
  0.15m/s in x/y and 0.3rad/s yaw. No real-world speed certification.
- On a stale source or invalid proposal, the shadow state immediately
  becomes `WOULD_VETO_...`; require 1.0 second of continuous valid
  readings after any fault before returning
  `HEALTHY_DIAGNOSTIC_ONLY`. Reject nonmonotonic local audit clock.
- **A positive diagnostic label is NOT "allow motion".** There is no
  command publication or subscriber which could actuate anything.

All values in the replay are synthetically constructed in Python;
no test obtains physical sensor data or a motor connection.

## Build on Arch, optional image only

```bash
git fetch origin
git switch feature/nav2-m7-offline-shadow-veto-replay
git pull --ff-only origin feature/nav2-m7-offline-shadow-veto-replay
cd ros
podman build -f Dockerfile.nav2 -t leroy-nav2:jazzy .
```

The optional `Dockerfile.nav2` still executes Colcon and pytest across
all five ROS packages before producing an image. It additionally
checks installation of `shadow_replay`. There are no edits to the
hardware Dockerfile, motor bridge, M5 MPPI configuration, M6 launch,
camera calibration or robot sensor drivers.

## Finite acceptance — NO ROS graph, no action or motor command

Run a single disconnected, device-free container:

```bash
podman run --rm \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:jazzy bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 run leroy_nav2_integration shadow_replay --mode depth
    ros2 run leroy_nav2_integration shadow_replay --mode odom
  '
```

Expected for each mode:
```text
M7 REPLAY: t=0.0 WOULD_VETO_REARM
M7 REPLAY: t=1.2 HEALTHY_DIAGNOSTIC_ONLY
M7 REPLAY: t=10.0 WOULD_VETO_BLOCKED_DEPTH  # or BLOCKED_ODOM
M7 REPLAY: t=12.0 WOULD_VETO_BLOCKED_DEPTH  # or BLOCKED_ODOM
M7 REPLAY: t=16.0 WOULD_VETO_REARM
M7 REPLAY: t=17.2 HEALTHY_DIAGNOSTIC_ONLY
PASS: M7 depth synthetic shadow replay vetoed stale input and invalid proposal, then required healthy rearm; NO ROS publisher, motor gate, or hardware permission
```
The odometry replay prints its own corresponding PASS.

## Acceptance and critical limitations

- Successful optional-image Colcon tests, including new deterministic
  pure-Python invalid/missing/future/stale data and threshold tests.
- Both synthetic replay modes print PASS with ordered veto and rearm.
- No `/cmd_vel`, real `/odom`, real robot container, ROS graph,
  network namespace, physical devices, or privileged container.
- `HEALTHY_DIAGNOSTIC_ONLY` is **never** an actual permission to send
  velocity to a motor. No real brake response, watchdog deadline,
  scheduling reliability, controller response to faults, e-stop,
  collision coverage, source authenticity, DDS security, tracking,
  physical measurements or integration with `leroy-ros` is tested.
- ROS Jazzy's diagnostic `Twist` lacks an acquisition timestamp;
  receipt freshness cannot authenticate when/where it was computed.
- M5/M6 synthetic inflation warning and all physical calibration
  limitations remain unchanged.

Do not advance to a live hardware safety interlock or motor commands on
the basis of replay success. This is a reviewable **policy contract**
that can be challenged and refined offline before any separate
implementation. Do not merge without review or start M8 without
operator authorization.
