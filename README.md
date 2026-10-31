# Leroy — build, start, inspect, and develop

This is the **single README and authoritative fresh-project runbook** for
[Leroy](https://github.com/j0yk1ll/leroy), a four-wheel mecanum robot using
ROS 2 Jazzy, **rootless Podman** on Arch Linux, a front USB camera, CPU
DepthART, XRSLAM, and an STM32F401 IMU/motor-controller over USB CDC.

**Current commissioned operation is sensor-only.** The integrated runtime
starts real camera/rectification, depth inference, a diagnostic Nav2
costmap, XRSLAM monocular tracking diagnostics, real STM32 IMU ingestion,
relative yaw estimation, and automatic stationary gyro verification.
It does **not** enable wheel motion, Nav2 driving, calibrated metric SLAM
or autonomous boot movement. Signed motor/yaw code is present for
future commissioning, not an operative deployment path.

Former Stage 3 / Stage 4 milestones are complete as sensor integration
work. **You do not launch them separately.** There is one public
operator interface: `ros/leroy.sh`. Multiple internal containers still
isolate camera ownership from the single STM32 serial writer.

## 1. Fresh Arch Linux host

Install Git, Bash, Python and Podman. Run Podman as your **normal user**
rather than with `sudo`:

```bash
sudo pacman -S --needed git bash python podman
command -v git
command -v podman
podman info --format '{{.Host.Security.Rootless}}'
```

The last command **must print `true`**. If not, configure rootless
Podman, user-namespace/subuid/subgid mappings and device permissions
for your Arch host before continuing. Do not switch to rootful Docker
to work around the check.

You need an internet connection for the first image build, substantial
disk space and RAM for CPU PyTorch, DepthART model weights and native
XRSLAM compilation. Check available disk space with `df -h` and
memory with `free -h`. Image dependencies are pinned in several
places but the ROS base image and package repository snapshots are
not bit-identically frozen; reproducibility means an accepted functional
rebuild, not a byte-for-byte OCI image.

## 2. Clone the complete project

```bash
cd ~/Code
git clone https://github.com/j0yk1ll/leroy.git leroy
cd leroy
git switch main
git status --short
cd ros
```

The project layout is:

| Directory | Contents |
| --- | --- |
| `firmware/` | STM32F401 C firmware, board pin mapping and Makefile |
| `ros/src/` | ROS 2 Jazzy packages, launch files, config and ROS nodes |
| `ros/` | Leroy CLI, Dockerfiles, boot/calibration logic and tests |
| `electronics/` | Experimental tscircuit board design (not fabrication-approved) |
| `hardware/` | Mechanical CAD and printable components |
| `utils/charuco/` | Camera intrinsics / printed ChArUco board tools |
| `utils/imu/` | Camera–IMU temporal calibration diagnostics |

This README is the only README in the active tree. Original
component-level instructions and obsolete milestone runbooks remain
in Git history at `archive/before-single-readme-2026-10-29` and
`archive/full-runtime-history-2026-10-29`. Historical M16–M23
Python scripts, synthetic Nav2 launchers, milestone tests and optional
old Nav2 Docker recipes are additionally preserved at
`archive/pre-ros-python-cleanup`. **The active checkout now contains
132 Python files, 14 shell scripts and six Dockerfiles.** The current
build, runtime and numbered autonomy TODO owners remain in the tree.
Historical experiments and evidence reports have been removed from the
maintained working tree. The current README and runtime code are the
supported source of operating instructions.

## 3. Verify source and build ALL images

From `leroy/ros`:

```bash
bash ./leroy.sh check
bash ./leroy.sh build
```

`check` runs device-free static source, shell, ROS-package and
gyro/autonomous-model regression checks. It never connects to the robot.

`build` runs the entire image build dependency graph with Podman:
`localhost/leroy-ros:jazzy`, `:perception`, `:costmap`,
`:xrslam`, `:stage4b-imu`, the `:stage4-imu` compatibility
alias, and `:stage4c-capture-offline`. The stage-like tag names are
**internal compatibility IDs**, not separate runtime modes. The
canonical build implementation is `ros/build_current_images.sh`.

A successful build prints:

```text
LEROY_CURRENT_IMAGES=BUILT_FROM_CHECKOUT_NO_HARDWARE_LAUNCHED
```

Building images **does not** launch ROS nodes or access the camera,
STM32, or motors. Do not run old `mXX` experiment scripts or build
historic candidate Dockerfiles to start the current robot.

## 4. Verify hardware identity and isolation

The verified physical front camera was host `/dev/video8`, mapped
**inside** the perception container to `/dev/video2`.
The verified STM32 is:

```text
/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00
```

Check the real devices on **this** host:

```bash
ls -l /dev/video8
ls -l /dev/serial/by-id/
readlink -f /dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00
```

The final command should resolve to `/dev/ttyACM0` for this
currently commissioned launcher. If the device changed, stop and
review the mapping rather than choosing a random ACM port.

**Before any live session:** independently disconnect power to
**motor and manipulator driver rails** while allowing the STM32 USB
sensor interface to remain powered. Keep the existing hardwired
power-cut E-stop available. The hardwired whole-robot E-stop by itself
does not prove that motor rails are isolated while the MCU is powered
over USB. Environment flags below are **operator attestations**, not
electrically verified sensors.

A different process must **not** own the STM32 CDC port. Do not run
`leroy_bridge`, the experimental signed motor bridge, an attended
motor test, or a second IMU stream source at the same time.

## 5. Start the full currently working ROS sensor graph

Check for previously retained containers:

```bash
cd ~/Code/leroy/ros
bash ./leroy.sh status
podman ps -a --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
```

`leroy.sh run` deliberately refuses to replace a running **or
stopped** Leroy container. `podman ps` by itself hides stopped
containers. If a *previous Leroy sensor session is fully shut down*
and its containers are stale, remove **only those known instances**:

```bash
podman rm leroy-live-xrslam leroy-live-costmap
```

Do not remove a still-running IMU source, an unknown motor bridge or
other unrelated containers. The default run normally cleans up its
own containers.

With independent motor-driver power isolation physically verified:

```bash
LEROY_MOTOR_RAILS_DISCONNECTED=YES \
LEROY_MCU_BY_ID=/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00 \
bash ./leroy.sh run
```

Type `BOOT_IMU_ONLY` at the supervised prompt. Keep Leroy stationary
through its initial gyro bias and independent **eight-second**
stationary holdout. The command stays in the foreground to publish
live IMU and relative yaw updates until interrupted.

The wrapper starts, in order:

| Container | ROS process / responsibility |
| --- | --- |
| `leroy-live-costmap` | `ros2 launch leroy_perception live_depth_costmap.launch.py`: camera, rectifier, DepthART, provisional static TF, diagnostic costmap |
| `leroy-live-xrslam` | `ros2 run leroy_xrslam leroy_xrslam_node`: camera-only monocular tracking diagnostics |
| `leroy-boot-imu-source` | `live_boot_imu_service.py` / `m18_imu_source`: exclusive serial owner, 50 Hz IMU publish |
| `leroy-live-boot-yaw-persistent` | `live_boot_yaw.py`: relative heading and independent gyro holdout; **no MCU device** |

**Actual implementation entry points:** the top-level CLI is
`ros/leroy.sh`, built by `ros/build_current_images.sh` and checked
through `ros/check_current_checkout.sh`. The camera graph is started by
`ros/src/leroy_bringup/scripts/live_stage3_start.sh` and the installed ROS launch file
`ros/src/leroy_perception/launch/live_depth_costmap.launch.py`.
The MCU/yaw graph is started by `ros/src/leroy_bringup/scripts/live_boot_session.sh` and
`ros/src/leroy_bringup/scripts/live_boot_yaw.sh`, with the serial writer in
`ros/src/leroy_sensors/leroy_sensors/imu_service.py`. This is an implementation map,
**not** an instruction to run these internal scripts simultaneously.
Firmware is independently built using `firmware/Makefile`.

The containers use ROS domain `170`, localhost-only discovery and
Fast DDS within the intentionally isolated parent network/IPC/PID
namespace. Do **not** start a second copy of these nodes via
`ros2 launch leroy_bringup robot.launch.py`: that alternative launch
tree is not the operator-accepted sensor-only graph.

Expected success output from the underlying components:

```text
LIVE_STAGE3_START=PASS_CAMERA_ONLY_CONCURRENT_DDS
BOOT_LIVE_SESSION=IMU_STREAM_READY
BOOT_LIVE_YAW=PASS_RELATIVE_GYRO_HEADING_ONLY
BOOT_AUTO_CALIBRATION=PASS_STATIONARY_GYRO_CONSISTENCY_ONLY
```

The new single-command `leroy.sh run` wrapper has passed
**no-device CI**, but has not yet been operator-verified on the robot
as one combined lifecycle. The component commands listed above were
physically reproduced from a clean checkout on 2026-10-29.

## 6. Inspect the running ROS nodes and topics

Open **another terminal** while `leroy.sh run` is active:

```bash
cd ~/Code/leroy/ros
bash ./leroy.sh status
podman logs --tail 60 leroy-live-costmap
podman logs --tail 60 leroy-live-xrslam
```

The live graph is intentionally isolated from the host. To inspect
its ROS nodes and topic types without introducing a second publisher:

```bash
podman exec leroy-live-costmap bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  ros2 node list
  ros2 topic list -t
'
```

Important live topic interfaces:

| Topic | Meaning |
| --- | --- |
| `/camera/image_rect`, `/camera/camera_info` | Rectified camera images and calibrated-format intrinsics |
| `/depthart/depth` | DepthART float depth |
| `/leroy_nav2/diagnostics/raw_depth_points` | Optical-frame point cloud for diagnostics |
| `/costmap` | **Diagnostic** rolling costmap; not certified obstacle avoidance |
| `/xrslam/tracking_status`, `/xrslam/localization_valid` | Actual XRSLAM status; currently `initializing` / `false` |
| `/imu/data_raw` | Physical STM32 IMU measurements |
| `/leroy_boot/relative_yaw_status` | Integrated **relative** gyro yaw |
| `/leroy_boot/auto_imu_calibration_status` | Stationary consistency result, **not a motion permit** |

For a bounded live IMU topic sanity check:

```bash
podman exec leroy-live-costmap bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  timeout 12s ros2 topic hz /imu/data_raw || test "$?" -eq 124
'
```

You can also inspect read-only gyro and calibration state using the
existing internal diagnostics `live_boot_yaw_snapshot.sh` or
`live_boot_readiness.sh`; these join an **already running**
sensor graph and are not separate startup procedures.

## 7. Stop all ROS nodes and verify MCU shutdown

In the **original foreground terminal**, press **Ctrl+C once**. The
IMU session must attempt the STM32 `set_imu_stream_hz 0` command
and receive an acknowledged stream STOP before the wrapper can
report successful completion. The wrapper then stops XRSLAM **before**
the parent costmap/camera graph.

Look for:

```text
BOOT_IMU_SOURCE_START_ACK=PASS
BOOT_IMU_SOURCE_STOP_ACK=PASS
BOOT_IMU_SOURCE_RESULT=PASS_OPERATOR_STOPPED
BOOT_LIVE_SESSION=STOPPED_NO_MOTION_AUTHORITY
LEROY_RUNTIME=STOPPED_SENSOR_ONLY_NO_MOTION_AUTHORITY
```

If the STOP ACK is absent, treat shutdown as **unverified**, inspect
the logs and independently isolate power before investigating.
An IMU-stream STOP **does not** certify wheel stopping or braking.

After shutdown:

```bash
bash ./leroy.sh status
podman ps -a
```

If abnormal termination leaves stale **stopped** named containers,
remove them deliberately. Do not run `podman rm -a` or reset all
Podman storage; the same host may run unrelated projects.

## 8. Common startup problems

| Message | Cause / action |
| --- | --- |
| `BLOCKED_EXISTING_SESSION_OR_SERIAL_OWNER` | Use `leroy.sh status` and `podman ps -a`; stop/inspect the previous **Leroy** session before removing stopped instances |
| `BLOCKED_PHYSICAL_MOTOR_RAIL_ISOLATION_NOT_ATTESTED` | Confirm actual disconnected driver power; do not fake the confirmation |
| `BLOCKED_STABLE_STM32_DEVICE_REQUIRED` / `BLOCKED_STM32_DEVICE_ID_CHANGED` | Verify the exact by-ID symlink and real tty port |
| `BLOCKED_MISSING_CURRENT_IMAGE` | Run `bash ./leroy.sh build` on this checkout |
| `LIVE_STAGE3_START=BLOCKED_EXISTING_CONTAINER_NAME_IN_USE` | A stopped old camera/XRSLAM container still exists; `podman ps -a` shows it |
| `LIVE_DEPTH_COSTMAP_GRAPH=WAITING` | Can be transient while DepthART starts; requires eventual `READY` and `PASS` |
| `/xrslam/localization_valid=false` | Expected today: metric localization is **not** established |
| `BOOT_IMU_SOURCE_STOP_ACK` missing | No clean MCU IMU shutdown evidence; troubleshoot before another live attempt |

## 9. Build the MCU firmware (development/maintenance ONLY)

The STM32F401 Black Pill source is in `firmware/` with board pin
definitions at `firmware/include/board.h`. The current firmware
uses `arm-none-eabi-gcc`, `libopencm3`, and SWD/ST-Link for flashing.
The electronic `STM32F411` / DRV8876 tscircuit proposal is a
**separate, not fabrication-approved** board design: do not mistake
it for the deployed STM32F401 and BTS7960 wiring.

To build the firmware **without flashing**:

```bash
sudo pacman -S --needed make arm-none-eabi-binutils arm-none-eabi-gcc \
  arm-none-eabi-newlib libopencm3 stlink
cd ~/Code/leroy/firmware
make build
```

Output is `firmware/build/robot_firmware.elf`,
`.bin`, `.hex` and `.map`.

**IMPORTANT: plain `make` or `make all` also flashes the board.**
Never use those as compile-only commands. Flashing is a separate,
explicit physical maintenance operation (`make flash`) requiring
ST-Link wiring, correct power state, motor rails independently
disconnected, all active ROS/serial owners stopped, and a reviewed
firmware/rollback plan. A fresh source checkout does **not** prove
that the STM32 contains this firmware revision.

Pin, sensor, encoder and motor protocols are defined by the source
under `firmware/include/`, `firmware/src/` and the bridge's
`ros/src/leroy_bridge/` package. Do not copy unmeasured
wheel-encoder or old GPIO-feedback assumptions from historic tests.

## 10. Optional calibration development, NOT runtime bringup

For camera intrinsics tools, use
`utils/charuco/generate.py`, `capture.py`, and `calibrate.py`.
Their separate Python environment needs `opencv-contrib-python`,
`numpy`, `pillow`, `reportlab`, and `pyyaml`. ChArUco board
geometry, printing scale and acquisition mode must match the
physical measurements. Typical tools:

```bash
cd ~/Code/leroy/utils/charuco
python -m venv .venv
source .venv/bin/activate
python -m pip install opencv-contrib-python numpy pillow reportlab pyyaml
python generate.py --output-dir calibration_board
python capture.py --output-dir calibration_images --camera 0
python calibrate.py --config calibration_board/board_config.json \
  --images "calibration_images/*.png" --output calibration.yaml
```

Confirm the correct physical camera device and intrinsics before
capturing; `--camera 0` is a **generic example**, not the verified
host `/dev/video8` mapping. These utilities are not necessary
to start the already verified diagnostic ROS graph.

Camera–IMU temporal-offset tools live under `utils/imu/` and
require **independently recorded physical calibration motion**;
correlated arrival timestamps, synthetic poses and the current
monocular graph do not establish metric VIO calibration. Historic
container commands for the old `leroy-ros` instance are archived;
do not apply them to the current unified runtime.

## 11. Fully autonomous operation — numbered implementation TODOs

The `TODO(AUTONOMY) #001` through `#042` code comments form one
**dependency-ordered implementation backlog**. Start with sensors and measured
geometry, then reliable motor control and stopping, then qualified startup
and real navigation. The number identifies a *task*, not a safety approval
or claim that earlier tasks are complete.

| Order | Implementation task | Owner file |
| ---: | --- | --- |
| 001 | IMU acquisition fidelity | [`firmware/src/imu.c`](firmware/src/imu.c) |
| 002 | MCU sensor ownership | [`ros/src/leroy_sensors/leroy_sensors/imu_service.py`](ros/src/leroy_sensors/leroy_sensors/imu_service.py) |
| 003 | Heading quality | [`ros/src/leroy_localization/leroy_localization/yaw_core.py`](ros/src/leroy_localization/leroy_localization/yaw_core.py) |
| 004 | Repeatable stationary check | [`ros/src/leroy_localization/leroy_localization/auto_calibration_core.py`](ros/src/leroy_localization/leroy_localization/auto_calibration_core.py) |
| 005 | Heading observation | [`ros/src/leroy_localization/leroy_localization/heading_node.py`](ros/src/leroy_localization/leroy_localization/heading_node.py) |
| 006 | Heading fusion | [`ros/src/leroy_localization/leroy_localization/yaw_core.py`](ros/src/leroy_localization/leroy_localization/yaw_core.py) |
| 007 | Survey actual sensor mounts | [`ros/src/leroy_description/config/sensor_mounts.yaml`](ros/src/leroy_description/config/sensor_mounts.yaml) |
| 008 | TF publisher ownership | [`ros/src/leroy_description/launch/description.launch.py`](ros/src/leroy_description/launch/description.launch.py) |
| 009 | Metric visual-inertial localization | [`ros/src/leroy_xrslam/src/leroy_xrslam_node.cpp`](ros/src/leroy_xrslam/src/leroy_xrslam_node.cpp) |
| 010 | Pose authority and relocalization | [`ros/src/leroy_xrslam/src/leroy_xrslam_node.cpp`](ros/src/leroy_xrslam/src/leroy_xrslam_node.cpp) |
| 011 | Real depth-to-obstacle calibration | [`ros/src/leroy_perception/leroy_perception/depth_obstacles.py`](ros/src/leroy_perception/leroy_perception/depth_obstacles.py) |
| 012 | All-sided obstacle observability | [`ros/src/leroy_perception/leroy_perception/depth_obstacles.py`](ros/src/leroy_perception/leroy_perception/depth_obstacles.py) |
| 013 | Navigation costmaps | [`ros/src/leroy_perception/launch/live_depth_costmap.launch.py`](ros/src/leroy_perception/launch/live_depth_costmap.launch.py) |
| 014 | Actual wheel calibration | [`firmware/src/platform.c`](firmware/src/platform.c) |
| 015 | Bridge calibration configuration | [`ros/src/leroy_bridge/config/leroy_bridge.yaml`](ros/src/leroy_bridge/config/leroy_bridge.yaml) |
| 016 | Motor motion bounds | [`ros/src/leroy_bridge/leroy_bridge/motor_authority.py`](ros/src/leroy_bridge/leroy_bridge/motor_authority.py) |
| 017 | Measured mecanum odometry | [`ros/src/leroy_localization/leroy_localization/kinematics.py`](ros/src/leroy_localization/leroy_localization/kinematics.py) |
| 018 | Odometry and sensor fusion | [`ros/src/leroy_localization/leroy_localization/kinematics.py`](ros/src/leroy_localization/leroy_localization/kinematics.py) |
| 019 | MCU motor transaction protocol | [`firmware/src/command.c`](firmware/src/command.c) |
| 020 | Durable MCU sequence numbers | [`ros/src/leroy_bridge/leroy_bridge/motor_cdc_protocol.py`](ros/src/leroy_bridge/leroy_bridge/motor_cdc_protocol.py) |
| 021 | Physical MCU fail-stop | [`firmware/src/command.c`](firmware/src/command.c) |
| 022 | Physical stop confirmation | [`ros/src/leroy_bridge/leroy_bridge/motor_cdc_protocol.py`](ros/src/leroy_bridge/leroy_bridge/motor_cdc_protocol.py) |
| 023 | Real stop/driver-power status | [`ros/src/leroy_bridge/leroy_bridge/motor_gpio_feedback.py`](ros/src/leroy_bridge/leroy_bridge/motor_gpio_feedback.py) |
| 024 | Exclusive STM32 bridge ownership | [`ros/src/leroy_bridge/leroy_bridge/node.py`](ros/src/leroy_bridge/leroy_bridge/node.py) |
| 025 | Actual motor interlocks | [`ros/src/leroy_bridge/leroy_bridge/node.py`](ros/src/leroy_bridge/leroy_bridge/node.py) |
| 026 | Motion-mode authority boundaries | [`ros/src/leroy_bridge/leroy_bridge/node.py`](ros/src/leroy_bridge/leroy_bridge/node.py) |
| 027 | Mode-specific motor permits | [`ros/src/leroy_bridge/leroy_bridge/motor_authority.py`](ros/src/leroy_bridge/leroy_bridge/motor_authority.py) |
| 028 | Measured dynamic sensor health | [`ros/src/leroy_bridge/leroy_bridge/motor_sensor_health.py`](ros/src/leroy_bridge/leroy_bridge/motor_sensor_health.py) |
| 029 | Unlocalized boot policy | [`ros/src/leroy_bridge/leroy_bridge/motor_sensor_health.py`](ros/src/leroy_bridge/leroy_bridge/motor_sensor_health.py) |
| 030 | Real boot-readiness evidence | [`ros/src/leroy_bringup/leroy_bringup/readiness.py`](ros/src/leroy_bringup/leroy_bringup/readiness.py) |
| 031 | Actual yaw maneuver dynamics | [`ros/src/leroy_localization/leroy_localization/yaw_tracking_core.py`](ros/src/leroy_localization/leroy_localization/yaw_tracking_core.py) |
| 032 | Commissioned turn-control dynamics | [`ros/src/leroy_bridge/leroy_bridge/motor_authority.py`](ros/src/leroy_bridge/leroy_bridge/motor_authority.py) |
| 033 | Complete turn-swept clearance | [`ros/src/leroy_bridge/leroy_bridge/motor_authority.py`](ros/src/leroy_bridge/leroy_bridge/motor_authority.py) |
| 034 | Attended powered calibration | [`ros/src/leroy_bridge/leroy_bridge/motor_authority.py`](ros/src/leroy_bridge/leroy_bridge/motor_authority.py) |
| 035 | Real yaw and STOP feedback | [`ros/src/leroy_bridge/leroy_bridge/motor_authority.py`](ros/src/leroy_bridge/leroy_bridge/motor_authority.py) |
| 036 | Boot localization state machine | [`ros/src/leroy_localization/leroy_localization/localization_core.py`](ros/src/leroy_localization/leroy_localization/localization_core.py) |
| 037 | Unlocalized recovery/parallax | [`ros/src/leroy_localization/leroy_localization/localization_core.py`](ros/src/leroy_localization/leroy_localization/localization_core.py) |
| 038 | Integrated runtime supervisor | [`ros/leroy.sh`](ros/leroy.sh) |
| 039 | Unified ROS node bringup | [`ros/src/leroy_bringup/launch/robot.launch.py`](ros/src/leroy_bringup/launch/robot.launch.py) |
| 040 | Real Nav2 lifecycle | [`ros/src/leroy_navigation/launch/dry_run.launch.py`](ros/src/leroy_navigation/launch/dry_run.launch.py) |
| 041 | Navigation failure recovery | [`ros/src/leroy_navigation/launch/dry_run.launch.py`](ros/src/leroy_navigation/launch/dry_run.launch.py) |
| 042 | Production enable acceptance | [`ros/src/leroy_navigation/config/hardware.disabled.yaml`](ros/src/leroy_navigation/config/hardware.disabled.yaml) |

The list intentionally **excludes security-specific TODOs** about keys,
cryptographic signatures and credential handling. It retains real-world
motor stopping, fault handling, collision protection and watchdog work;
those are physical and operational requirements, not placeholders for
cybersecurity functionality.

**Do not grant motion by hardcoding healthy flags or disabling existing
interlocks.** The real robot has a hardwired downstream power-cut E-stop,
but the historical GPIO feedback device and separate rotation-clearance
monitor are not commissioned. A software status or fake sensor value is
not a substitute. The current `hardware.disabled.yaml` deployment
manifest must remain inhibited until measured physical acceptance.

Follow the numbered code comments and their specific acceptance conditions.
Keep no-device CI evidence distinct from physical robot testing.

## 12. Preserve external state before replacing GitHub

```bash
cd ~/Code/leroy
git status --short
git fetch --all --tags
git bundle create ../leroy-all-refs.bundle --all
git bundle verify ../leroy-all-refs.bundle
```

Keep the bundle **outside** the directory you plan to delete. Git
does not back up untracked/local calibration and recordings,
credentials, current rootless Podman volumes/images, the firmware
actually flashed, or persistent MCU motor-command sequencing state.
A newly created GitHub repository will not automatically recover
issues, PR reviews, deleted refs or access settings. Verify a second
source checkout and the independent backups before destroying
the original remote.

## 13. Validation status and evidence

On 2026-10-29 the operator rebuilt all current images from a fresh
checkout and independently demonstrated camera/depth/XRSLAM
diagnostic startup plus STM32 IMU START, automatic stationary gyro
holdout and MCU IMU STOP ACK. The gyro holdout used 401 physical
50 Hz samples over 8s with 0.0816° relative drift; XRSLAM
remained `initializing` with `localization_valid=false`.
Those are **component-runner acceptance** results, not proof that the
new single-command `leroy.sh run` has itself completed a full physical
commissioning run.

No powered autonomous rotation, full-environment collision protection,
metric localization, or Nav2 motor authority has been commissioned.
All of those remain explicit tracked engineering TODOs.


## Sensor-only operator stop (uncommissioned motion)

The supported `ros/leroy.sh stop` command stops the existing sensor-only
containers in reverse graph order. It requests shutdown of the exclusive
STM32 IMU source first, checks its emitted `BOOT_IMU_SOURCE_STOP_ACK=PASS`
evidence, and treats missing acknowledgement as a **failure**. A failed IMU
container is retained for investigation. In that case isolate power and
verify hardware state before another session. This is not an emergency-stop
substitute and does not establish autonomous motor authorization.

The current monocular XRSLAM diagnostic launcher still depends on an
**uncalibrated** device template. Metric localization, autonomous navigation,
and motor actuation remain uncommissioned; the template must not be treated
as physical calibration evidence.
