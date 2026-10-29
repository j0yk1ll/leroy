# Leroy

Leroy is a four-wheel mecanum robot with an STM32F401 USB CDC controller,
real monocular camera/DepthART perception, XRSLAM diagnostics, and a ROS 2
Jazzy application packaged for **rootless Podman on an Arch Linux host**.

## Which code runs today?

The last physically observed runtime (2026-10-29) is a **sensor-only**
Stage 3 camera / DepthART / Nav2 diagnostic costmap + XRSLAM **monocular**
graph, optionally augmented by a separate, physically motor-isolated
STM32 50 Hz IMU publisher and persistent yaw/calibration subscriber.
It has demonstrated real relative yaw when the chassis was moved manually.

**It is not an autonomously driving robot.** XRSLAM has remained
`initializing`, metric localization is unvalidated, Nav2 motor output is
disabled, and the signed motor controller is an uncommissioned development
path. Do not treat a successfully rebuilt image or CI pass as motor-release
authorization.

The canonical files to start with are:

- `ros/CURRENT_LIVE_RUNTIME.md` — verified and unverified physical behavior
- `ros/LIVE_STAGE3_REPRODUCIBLE.md` — isolated camera-only Stage 3
- `ros/BOOT_LOCALIZATION_SWEEP_DESIGN.md` — future autonomous boot design
- `ros/Dockerfile`, `ros/Dockerfile.live-perception`,
  `ros/Dockerfile.live-costmap`, `ros/Dockerfile.xrslam`,
  `ros/Dockerfile.live-stage4-imu`,
  `ros/Dockerfile.live-stage4c-capture` — current image recipes

## Reconstruct current images from a clean checkout

Required host dependencies: rootless Podman (with working subordinate
UID/GID mappings), Git, Bash and enough disk/RAM for CPU PyTorch,
DepthART and native XRSLAM builds. The image build downloads ROS packages,
a pinned DepthART source snapshot, the model checkpoint, patched usb_cam
source and a pinned XRSLAM revision. **An internet connection and working
external package/model hosting are required.** OCI base image tags and apt
repositories are not pinned to immutable digests or a package snapshot;
this is a functional rebuild recipe, not a bit-identical supply-chain
reproduction.

From the repository root:

```bash
cd ros
bash ./build_current_images.sh --build
```

This script builds these local tags in dependency order:

| Tag | Purpose |
| --- | --- |
| `localhost/leroy-ros:jazzy` | Base Jazzy, patched usb_cam and DepthART |
| `localhost/leroy-ros:perception` | Live depth adapter |
| `localhost/leroy-ros:costmap` | Stage 3 diagnostic costmap |
| `localhost/leroy-ros:xrslam` | Native XRSLAM monocular adapter |
| `localhost/leroy-ros:stage4b-imu` | Existing standalone M18/Stage 4B IMU image |
| `localhost/leroy-ros:stage4-imu` | Compatibility tag for the Stage 4A runner |
| `localhost/leroy-ros:stage4c-capture-offline` | Passive recorder and boot-yaw runtime |

No hardware is opened and no containers start during the build.

**Clean-clone operator acceptance (2026-10-29):** The owner reported
`LEROY_CURRENT_IMAGES=BUILT_FROM_CHECKOUT_NO_HARDWARE_LAUNCHED` after
rebuilding from `/home/kevin/Code/leroy-clean-main/ros`. The same clean
checkout then produced
`LIVE_STAGE3_START=PASS_CAMERA_ONLY_CONCURRENT_DDS` on the robot,
including the mapped host `/dev/video8` camera, real DepthART optical
point clouds, activated Nav2 diagnostic costmap, and real XRSLAM
monocular process. XRSLAM reported `initializing` and
`/xrslam/localization_valid=false`; no `/odom` message was
observed in the six-second check. This confirms the current Stage 3
nonactuating startup path, **not** Stage 4 IMU session or autonomous
motor/navigation readiness. The owner-reported result has not been
independently replicated by CI or this assistant.

## Run the accepted, non-actuating physical runtime

**Only after independently disconnecting the actual motor power rails**
and inspecting the host camera selection, run on Leroy's Arch host:

```bash
cd ros
LEROY_MOTOR_POWER_DISCONNECTED=1 bash ./live_stage3_start.sh
```

The actual operator-verified camera is **host** `/dev/video8`, mapped to
`/dev/video2` **inside** the container. This is intentional.
Do not automatically select a different camera.

The Stage 3 launcher starts only the camera, depth, costmap and monocular
XRSLAM graph. For continuous, **still motor-isolated**, real STM32 IMU
and startup gyro calibration, the existing launcher is:

```bash
cd ros
LEROY_STAGE4_MCU_BY_ID=/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00 \
LEROY_STAGE4_MOTOR_ISOLATION_ATTESTED=YES \
bash ./live_boot_session.sh
```

That command is exclusively for the separate, physically disconnected
sensor-only session. The hardwired whole-robot E-stop by itself is not
a substitute for keeping the motor drivers isolated while MCU USB and
ROS remain powered. Do **not** run the signed bridge concurrently with
the M18 serial-owner sidecar.

To stop the original sensor-only graph, stop IMU/yaw (Ctrl+C), verify
the MCU IMU STOP ACK, then stop XRSLAM before its parent:

```bash
podman stop leroy-live-xrslam
podman stop leroy-live-costmap
```

The experimental signed motor/yaw code in `ros/live_actuated_yaw_calibration.py`
and the historical motor-commissioning programs are **not** part of
this accepted sensor-only startup.

## Offline checks

```bash
cd ros
bash ./check_current_checkout.sh
```

This verifies the tracked build inputs, static launcher syntax and the
current zero-device Python contracts. It never opens the robot.

## What Git does not restore

A repository is **source code, not a machine image**. Before deleting the
GitHub repository or the local working copy, explicitly preserve:

- Uncommitted or untracked local files (`git status --porcelain`), including
  any host-specific camera/IMU calibration not committed to source.
- Rootless Podman images, volume contents, container configuration and logs;
  a fresh clone does not restore those automatically.
- MCU firmware that is *actually flashed*, any persistent
  `MotorSequenceLedger` state, and protected signing credentials. Never
  upload private keys to GitHub.
- Locally saved ROS bags, capture records, model caches and source calibration
  artifacts not present in the tracked tree.
- Device mapping, permissions, host serial-by-ID identity and the
  hardware isolation/stop configuration.
- The full Git history and review context if needed: deleting the hosting
  repository also deletes its pull requests and issues.

For a portable **Git history** backup, create a bundle locally before
deleting any repository:

```bash
git bundle create ../leroy-all-refs.bundle --all
git bundle verify ../leroy-all-refs.bundle
```

For the **source tree** only, use `git archive` on the intended main
commit. A plain copy of the local folder is not automatically a clean,
rebuildable checkout, because it can contain stale build outputs and may
lack committed feature-branch changes.

The current change-history archive is retained in the
`archive/full-runtime-history-2026-10-29` branch, and the earlier main
in `archive/main-before-runtime-consolidation-2026-10-29`.
These GitHub refs will **not survive deletion of the GitHub repository**
unless you export or push them elsewhere.

## Source layout

- `firmware/` — STM32 code and build system
- `ros/src/` — ROS packages
- `ros/` — canonical image builds, runtime wrappers and offline tests
- `hardware/` — mechanical design source and STL
- `electronics/` — electronics design
- `utils/` — calibration tools and source-controlled capture assets

Older single-milestone Dockerfiles, launch scripts and milestone notes are
historical development artifacts, not extra startup dependencies. Retained
historical snapshots can be read from the archive branch.
