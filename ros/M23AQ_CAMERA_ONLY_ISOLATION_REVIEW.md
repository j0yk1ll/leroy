# M23AQ — source-only camera-launch isolation review (NO physical test)

## Accepted diagnostic evidence

- M23AJ: verified exact Ubuntu Noble OpenCV 4.6 pthreads/no-TBB
  built in immutable image ID
  `334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025`.
- M23AK + M23AL: 6/6 device-free stock-factory composable
  rectifier clean synthetic shutdowns with all 900 raw, rectified,
  and CameraInfo messages, graph/QoS/stamp/drain and no observed TBB
  executable mapping at M23AF post_factory and on_shutdown phases.
- M23AM: six relevant patched usb_cam/ROS camera-path ELF loader
  closures compatible with pthreads OpenCV, no unresolved relocations
  or detected mixed OpenCV/TBB.
- M23AN: 640x480 RGB8 30.032 delivered FPS, rectified subscriber
  receipt gaps p50 33.325ms, p99 54.156ms, max 56.172ms,
  zero gaps >250ms; p99 raw-to-rect paired receipt skew 15.044ms.
- M23AO: actual diagnostic component child CPU ~1.28 logical
  cores during sampled workload, observed VmHWM 50.83MiB.
- M23AP: real stock `rclcpp_components/component_container`
  (rather than custom M23AF observer manager) clean child exit 0
  and exact 900/900 synthetic messages on pthreads OpenCV.
  Operator-reported available host resources: 16 logical CPUs,
  21,596,412 KiB MemAvailable (~20.6 GiB). Nominal resources
  look adequate for the **isolated** test but do not include
  physical camera, driver, contention or camera-derived latency.

## Critical launch safety finding

`ros/src/leroy_bringup/launch/robot.launch.py` unconditionally
includes the Leroy bridge launch; `ros/exec.sh` always launches
that robot launch. In the repository's own bridge README, opening
the MCU CDC serial port can energize wheels/manipulator even if
software blocks nonzero wheel commands. **Neither launcher is
acceptable for the first physical-camera test**, even when
DepthART and XRSLAM are disabled.

`ros/m23aq_camera_only.launch.py` is a *dormant and uninstalled*
candidate containing exactly:
1. `usb_cam/usb_cam_node_exe` under `/camera`, preserving the
   project's default 640x480, 30Hz, mmap, mjpeg2rgb and optical
   frame configuration and pinned camera YAML path.
2. The unmodified stock `image_proc::RectifyNode` in the
   standard rclcpp_components `component_container`, with
   original image_raw and camera_info remappings.

No bridge, robot description, motor controller, serial, IMU,
DepthART, XRSLAM, Nav2, recorder or other ROS node is included.
The existing driver overlay must still resolve to
`/leroy_ws/install/usb_cam`; the launch itself does not prove that.

A fail-closed source guard refuses to *construct* the launch
unless the operator has separately reviewed and explicitly set
camera-only approval, motor rail isolation, exact video device
path, verified calibration identity/path, no LD_PRELOAD and
pthreads OpenCV first in LD_LIBRARY_PATH. Software environment
flags **cannot verify physical isolation or camera identity**.
Never treat setting them as independent authorization.

## Source-only gate now allowed

From the accepted M23AO worktree:

~~~bash
(
  set -euo pipefail
  test "$(id -u)" -ne 0
  test ! -e /tmp/leroy-m23aq
  cd /tmp/leroy-m23ao
  git fetch origin feature/m23aq-camera-only-launch-source-gate
  git worktree add --detach /tmp/leroy-m23aq     origin/feature/m23aq-camera-only-launch-source-gate
  cd /tmp/leroy-m23aq
  test -z "$(git status --porcelain)"
  PYTHONDONTWRITEBYTECODE=1 python3 -B     ros/m23aq_camera_only_source_test.py
)
~~~

Expected 11 **offline** stub-launch tests: only two allowed ROS
nodes; successful fake construction with mock imports in memory;
launch rejected with any missing manual guard; serial device path,
wrong calibration path or distro/OpenCV-first path rejected.

The source tests do not import actual ROS, start Podman, execute
camera launch, open /dev/video*, or open CDC serial. No V4L2
enumeration, live imagery, recording or real robot launch.

## Operator review required BEFORE any real camera launch

A separate operator-controlled decision must document all of:

1. Exact physical camera identity (vendor, model, serial or
   persistent V4L2 by-id mapping) and the actual `/dev/videoN`
   that belongs to that camera. The historical default
   `/dev/video2` may change between boots and is NOT
   considered an approved device identity.
2. The supplied `/config/camera/front_camera.yaml` with
   nonzero plumb_bob distortion is calibrated to **this**
   camera at exactly 640x480 and correct focus/crop.
   Its existence and numeric nonzero D do not prove that.
3. Physical isolation of motor/manipulator supply, separately
   confirmed by the operator; no serial MCU device mount,
   USB passthrough beyond the approved video node,
   host `--privileged`, home or `/dev` wide mounts,
   bridge or robot launcher. Podman image and package provenance
   remain pinned to the accepted ID.
4. A bounded first trial protocol with timeout, operator-visible
   stop/kill command, logging to a new protected path, fresh
   synthetic-vs-real graph/QoS checks, real-image stamps,
   no gaps >250ms, verified actual component shutdown,
   no unexpected /cmd_vel, /odom, VIO or IMU topics, and
   abort-on-crash/stale-images/wrong-calibration.
5. Resource budget for the **host in the physical test setting**:
   camera + ROS stack CPU/RAM with sufficient reserve for
   any existing workloads, not just the measured isolated
   rectifier's 1.28 cores/50.83MiB. Confirm hardware thermal
   and scheduling constraints if applicable.

These manual checks and their reviewed test protocol are a
separate authorization step; this draft is **NOT** a physical
runtime runbook. Do not launch or attach camera devices based
on this PR alone. It does not merge, modify the robot launcher
or authorize actual hardware access.

## Explicit status

M23AQ_CAMERA_ONLY_SOURCE_GATE=PREPARED_NOT_EXECUTED
M23AQ_PHYSICAL_CAMERA_AUTHORIZED=FALSE
M23AQ_VIO_NAV2_MOTOR_RECORDING_AUTHORIZED=FALSE
M23AQ_PRODUCTION_ACCEPTANCE=BLOCKED
