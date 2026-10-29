# Leroy

Leroy is a four-wheel mecanum robot running ROS 2 Jazzy in **rootless
Podman** on an Arch Linux host. Its live runtime includes the actual
front camera, CPU DepthART perception, diagnostic Nav2 costmap, XRSLAM
monocular adapter, and STM32F401 50 Hz IMU with continuous relative
yaw and automatic stationary gyro verification.

**The old Stage 3 / Stage 4 division is complete and retired.** There
is one current **Leroy sensor runtime**, one build command and one
attended startup command. Separate Linux containers remain for
isolating camera and STM32 USB ownership, not as separate project
milestones. Legacy Stage-named implementation files and image tags are
retained only where required for verified backward compatibility.

The currently commissioned system is **sensor-only**. The motion bridge
and closed-loop powered yaw calibration exist as development code but
are not commissioned for motor operation. XRSLAM reports
`initializing`, metric localization is not validated, and Nav2
command output is disabled.

## One Leroy workflow

Host prerequisites: Linux, Bash, Git, rootless Podman, network access
for builds, sufficient disk/RAM for CPU PyTorch and XRSLAM. External
sources and Docker/OCI base tags are not all immutable, so this is a
functionally reproducible build, not a bit-for-bit offline snapshot.

Run commands from a **clean main checkout**, under `ros/`:

```bash
bash ./leroy.sh check       # no hardware
bash ./leroy.sh build       # all images, no hardware
bash ./leroy.sh status      # inspect container states
```

For the live integrated session, the **motor and manipulator power
rails must first be independently, physically disconnected** while
STM32 USB remains powered. Verify the robot's hardwired power-cut
E-stop remains accessible. The environment variable below is an
operator statement, **not a software sensor**.

```bash
cd /home/kevin/Code/leroy-clean-main/ros
LEROY_MOTOR_RAILS_DISCONNECTED=YES \
LEROY_MCU_BY_ID=/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00 \
bash ./leroy.sh run
```

The single invocation, in order:

1. Verifies rootless Podman, isolation acknowledgement, stable STM32
   device, the required images and absence of conflicting owners.
2. Starts the physically verified camera → DepthART → costmap and
   XRSLAM monocular DDS graph in isolated containers.
3. Starts the existing **single serial-owner** STM32 IMU source and
   the gyro-yaw observer in the same isolated DDS graph. The operator
   types `BOOT_IMU_ONLY` for this supervised session.
4. Keeps publishing real relative yaw and stationary gyro verification
   while the supervised foreground session remains active.
5. On Ctrl+C, the IMU source attempts its acknowledged IMU-stream STOP;
   the wrapper then stops XRSLAM and its parent camera container in
   reverse dependency order. A missing IMU STOP ACK fails the session.

This does **not** start an autonomous motor controller, drive the
wheels or enable metric localization. The camera owner uses host
`/dev/video8` mapped to container `/dev/video2`. Only the IMU owner
maps the motor-capable STM32 device `/dev/ttyACM0`; do not run any
other serial-owning bridge concurrently.

The orchestrator intentionally refuses stale or running Leroy
containers instead of replacing them. Inspect the state with
`bash ./leroy.sh status`. Stop only the known old session processes
after verifying no sensor/MCU client is still active; do not remove
unrelated Podman containers.

Legacy `live_stage3_*`, `live_stage4_*` and `live_boot_*` entry
points remain internal compatibility layers and targeted diagnostic
utilities. **Do not use them as a second, parallel startup workflow.**
The operator-facing entry point is `leroy.sh`.

## Physically verified from a clean `main` checkout

On October 29, 2026, the operator built the current images from
`/home/kevin/Code/leroy-clean-main/ros`, reproduced the real camera
and depth/costmap/XRSLAM graph, completed automatic stationary gyro
verification, then shut down the real STM32 IMU stream with START/STOP
acknowledgments.

```text
LEROY_CURRENT_IMAGES=BUILT_FROM_CHECKOUT_NO_HARDWARE_LAUNCHED
LIVE_STAGE3_START=PASS_CAMERA_ONLY_CONCURRENT_DDS
BOOT_LIVE_SESSION=IMU_STREAM_READY
BOOT_LIVE_YAW=PASS_RELATIVE_GYRO_HEADING_ONLY
BOOT_AUTO_CALIBRATION=PASS_STATIONARY_GYRO_CONSISTENCY_ONLY
BOOT_IMU_SOURCE_START_ACK=PASS
BOOT_IMU_SOURCE_STOP_ACK=PASS
BOOT_IMU_SOURCE_RESULT=PASS_OPERATOR_STOPPED
BOOT_LIVE_SESSION=STOPPED_NO_MOTION_AUTHORITY
```

These are **operator-reported results from the verified component
launchers**, not yet a physical validation of the **new unified
`leroy.sh run` wrapper**. The stationary calibration had 401 samples
over eight seconds, zero restarts, and 0.0816° reported relative yaw
drift during that holdout. XRSLAM remained `initializing` with
`/xrslam/localization_valid=false`; no real metric odometry was
approved. The gyro status retains `motor_motion_authorized=false`.
The IMU STOP is **not** proof of motor-driver braking or an E-stop test.

See [`ros/CURRENT_LIVE_RUNTIME.md`](ros/CURRENT_LIVE_RUNTIME.md)
for integration and hardware acceptance details.

## Project directories

- `firmware/` — STM32 source.
- `ros/src/` — maintained ROS packages, including the currently
  disabled/uncommissioned motor and navigation interfaces.
- `ros/` — current build, orchestration, configuration and tests.
- `hardware/` and `electronics/` — mechanical/electrical design.
- `utils/` — calibration utilities and source-controlled data.

Historical one-off `M18`–`M23` launchers, experimental Dockerfiles
and milestone notes are archived at the snapshot branch
`archive/full-runtime-history-2026-10-29`. See
[`ros/ARCHIVE_INDEX.md`](ros/ARCHIVE_INDEX.md).

## Do not delete your only copy of external state

A Git source checkout does not preserve running Podman containers,
locally built images, volumes, untracked files, firmware actually
flashed onto the MCU, protected signing keys, recorded ROS bags or
persistent motor sequence ledgers. Verify these separately before
deleting any local checkout or GitHub repository.

For a complete source **and Git refs** backup (while the remote
still exists):

```bash
git bundle create ../leroy-all-refs.bundle --all
git bundle verify ../leroy-all-refs.bundle
```

A source-only reupload does not restore issues, PR reviews or archived
Git refs automatically. Keep a verified Git bundle independently.
