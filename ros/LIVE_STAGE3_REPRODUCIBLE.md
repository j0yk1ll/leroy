# Leroy ROS 2 Jazzy — reproducible accepted Stage 3 diagnostic startup

This is the **camera-only, non-actuating** live integration demonstrated on
28 October 2026. It is **not** calibrated navigation or a general robot
bringup. Its contract is:

```text
REAL USB camera /dev/video2
  -> usb_cam (pinned timestamp fix), CameraInfo -> image_proc rectifier
    -> real CPU DepthART -> raw optical PointCloud2
      -> Nav2 rolling diagnostic costmap (base_link, provisional static TF)
    -> genuine XRSLAM monocular sidecar (no IMU, camera reused)
       -> /xrslam/tracking_status and /xrslam/localization_valid=false
       -> NO /odom samples, NO map or dynamic odom TF
```

Two rootless Podman containers share only the **parent's isolated** network,
shareable IPC and private PID namespaces (Fast DDS). There is one camera owner
and one set of six provisional static sensor TF publishers. XRSLAM maps no
devices. Neither container maps the motor-capable `/dev/ttyACM0`.
No MCU, bridge, controller, planner, navigation goals or motor interface is
started. **Motor-driver rails must be independently disconnected.**

## Clean main checkout, preserving the current dirty feature tree

Your existing feature checkout contains uncommitted tracked deletions and
untracked logs. Do **not** reset, clean or indiscriminately stash them. Once
the feature-to-main promotion PR is merged, obtain the full main-branch
codebase in a **separate worktree**:

```bash
cd /home/kevin/Code/leroy
git fetch origin main
git worktree add --detach ../leroy-stage3-main origin/main
cd ../leroy-stage3-main/ros
```

This main branch will also contain motor-authority source and MCU firmware
development. **Those files are not invoked by this Stage 3 runner** and their
presence does not certify motor commissioning or permit physical movement.
The feature branch's YAML-based `description.launch.py` is now the
canonical sensor TF launcher; an additional
`live_sensor_mounts.launch.py` offers an equivalent dedicated entry point.
The older URDF file is kept as historical source only, and ordinary
`robot.launch.py` is **not** part of this guarded diagnostic startup.

Do not use the old dirty feature checkout as the source of a main-branch
build. Run the static contracts without any hardware:

```bash
bash -n live_stage3_build.sh live_stage3_start.sh live_stage3_monocular_sidecar.sh
python3 -m unittest -v live_stage3_sidecar_static_test live_stage3_start_static_test
```

## Build the exact image family (no robot launch)

```bash
bash ./live_stage3_build.sh --build
```

This builds the pinned source images in dependency order:

1. `localhost/leroy-ros:jazzy` — patched usb_cam + CPU DepthART.
2. `localhost/leroy-ros:perception` — live point-cloud adapter.
3. `localhost/leroy-ros:costmap` — Nav2 fixture, live static TF and costmap.
4. `localhost/leroy-ros:xrslam` — real native XRSLAM ROS adapter.

The costmap image runs offline contracts and checks that the installed
costmap YAML matches source. A build or source-contract error is a **STOP**.
No arbitrary published image, simulated image source or old local tag should
substitute for an image from this build. The image builds require dependency
access and may be resource intensive; they have **not** been re-executed in
this ChatGPT session.

## Start and accept (motors PHYSICALLY disconnected first)

Stop any older `leroy-live-xrslam` and `leroy-live-costmap` instances
*deliberately* before using this release. The script **refuses** to reuse,
remove or overwrite existing same-named containers.

Confirm the independent motor power cutoff, leave the STM32 and motor driver
disconnected from the runtime, and confirm `/dev/video2` is the verified
front camera. The environment variable below records an **operator
assertion**, not an automatic measurement of motor power:

```bash
LEROY_MOTOR_POWER_DISCONNECTED=1 bash ./live_stage3_start.sh
```

The start script:

- checks rootless Podman, both local images, motor-disconnect assertion and
  the camera character device; refuses pre-existing named containers;
- launches ONLY the isolated real-camera/DepthART/Nav2 parent with
  `--network=none --ipc=shareable --pid=private --device=/dev/video2`;
- waits for `/costmap`, validates *both* obstacle-layer and observation-source
  height bounds are `[-3.0, 3.0]`, explicitly configures/activates the
  costmap, checks its **live ROS parameters**, then requires
  `LIVE_DEPTH_COSTMAP=PASS`;
- starts the original XRSLAM monocular sidecar in detached mode with the
  parent's isolated network/IPC/PID and no mapped devices; requires the
  native `Submitted 1 camera frames to XRSLAM (monocular)` log;
- reads XRSLAM tracking status from Stage 2, requires
  `/xrslam/localization_valid=false`, confirms **no odometry message
  during a six-second observation window**, and reruns the live costmap probe.

A failure stops only the two containers started by that invocation; it
does not modify existing host repository files, firmware, TF calibration,
motor settings or other Podman containers.

Expected successful terminal marker:

```text
LIVE_STAGE3_INSTALLED_COSTMAP_CONFIG=PASS_BOTH_HEIGHT_WINDOWS
LIVE_DEPTH_COSTMAP_GRAPH=READY required_publishers_verified
LIVE_DEPTH_COSTMAP=PASS ... lethal_cells=true unknown_cells=true frame=base_link
LIVE_STAGE3_ODOM=PASS_NO_MESSAGES_IN_6_SECONDS
LIVE_STAGE3_START=PASS_CAMERA_ONLY_CONCURRENT_DDS
```

The observed manual Stage 3 trial submitted **6,000 real images** into native
XRSLAM while the costmap probe passed; both containers received
`/xrslam/tracking_status: initializing`, the localization-valid latch was
`false`, and `/odom` produced no message during a six-second read.
The **one-command wrapper is newly implemented** and needs its own physical
replay before it can be marked operator PASS.

## Inspect or stop

```bash
podman logs --tail 60 leroy-live-costmap
podman logs --tail 60 leroy-live-xrslam

# Stop the sidecar BEFORE its parent.
podman stop leroy-live-xrslam
podman stop leroy-live-costmap
```

## Outstanding gates

The diagnostic costmap marks **unclassified real surfaces**, including
possible ground returns, with broad provisional height windows. Sensor
mounts, floor datum, footprint and DepthART accuracy are uncalibrated.
XRSLAM may remain `initializing`. Monocular scale is ambiguous. DO NOT
enable `pose_convention_validated`, `/odom` or
`odom -> base_link`/map TF; do not start Nav2 planners/controllers or
connect the motor-power rails based on this diagnostic acceptance.

Stage 4 (camera + real MCU IMU, temporal alignment, calibrated camera–IMU
extrinsics, then real VIO tracking) requires **separate supervised approval**
because the STM32 serial interface is motor-capable. It is intentionally not
part of this main-branch startup.
