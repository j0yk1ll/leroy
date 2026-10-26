# Stage 3: XRSLAM Camera–IMU Calibration Provenance

## Sensor geometry is configured in YAML

Hardware-specific fixed sensor transforms now have a single editable
configuration file:

`ros/src/leroy_description/config/sensor_mounts.yaml`

The canonical `leroy.urdf.xacro` **consumes** that YAML. Existing
IMU/camera positions and pitches were migrated without changing
their numeric values. The package installs the YAML file, and the
standard ROS optical-axis conventions remain in Xacro.
No hardware measurements are needed simply to edit the configuration.

The values still have `status: provisional`; editing them does not
establish validated physical sensor centers or temporal alignment.
The next source-level acceptance check is
`bash ./m23bv_v3_stage3_sensor_mounts_yaml_no_device.sh`, which uses
a pinned network-isolated rootless container with **no devices** to
expand Xacro and compare resulting transforms to YAML.

The earlier camera/IMU provenance runner is revision-pinned to the
*prior* literal-Xacro source blob. Its successful operator output is
retained as historical evidence, not a validation of the new
YAML-driven source. Do not weaken old source pins to claim an old
physical trial covered the new description.

## Existing Leroy frame definitions and calibration history — reuse, do not re-ask

The operator has **already specified geometry and sensor axes before**
this Stage 3 review. The previous request to define all four quantities
again was redundant. This file now inventories exactly what can be
recovered from tracked sources without inventing measurements.

| Quantity | Already established source | Scope and current authority |
| --- | --- | --- |
| Robot coordinate directions | `ros/src/leroy_description/urdf/leroy.urdf.xacro`: +X forward, +Y left, +Z up; sensor mounts loaded from YAML | Axis convention **DEFINED**; the physical chassis landmark chosen as the `base_link` origin is **not explicitly identified in the current checked-in URDF/README** |
| IMU axes on assembled robot | `firmware/include/imu.h` and `firmware/src/imu.c`: forward=−ICM X, left=−ICM Y, up=+ICM Z, CCW yaw=+ICM gyro Z; gyro roll/pitch signs inverted, yaw unchanged | **ALREADY DEFINED AND IMPLEMENTED** in `imu_read_robot()`; `/imu/data_raw` contains robot-frame acceleration/gyro values from the MCU. **Do not apply another 180° sensor-axis correction in the ROS TF** |
| IMU center translation in current TF | `ros/src/leroy_description/config/sensor_mounts.yaml`: +95 mm X, −95 mm Y, 0 mm Z (expanded by Xacro) | Existing parameter; its own source explicitly marks this geometry **PROVISIONAL**, including unknown physical height. Not independently certified as the IMU die/sensing center |
| Front camera orientation and frame axes | Mount pitch +30° in `sensor_mounts.yaml`; conventional optical frame +X right, +Y down, +Z forward and `camera_optical_frame` alias in Xacro | Existing defined TF *model*, but physical 30° mount angle still explicitly **PROVISIONAL** |
| Camera optical center translation | `sensor_mounts.yaml`: (0, 0, 0) m | Existing value is an explicitly documented **zero placeholder**, not an assertion that the optical center is at the base origin |
| Workspace camera | `sensor_mounts.yaml`: +60° pitch, zero translation | **PROVISIONAL**, not the front camera used by Stage 3 VIO |
| Wheels/nominal chassis geometry | `ros/src/leroy_bridge/config/leroy_bridge.yaml`: radius=20 mm, half-wheelbase=180 mm, half-track=275 mm | Existing kinematic configuration; **does not independently identify the physical `base_link` reference point** or validate camera/IMU mounting |
| Past camera–IMU calibration work | `utils/imu/README.md` and `utils/imu/*`: ChArUco image/gyro timing analyses and historical supervised captures | **ALREADY ATTEMPTED**, not a validated current camera/IMU phase; historical timing methods compare headers from a different runtime context |
| Previously tested camera epoch conversion | `ros/CAMERA_TIMESTAMP_FIX.md`, pinned effective `/leroy_ws/install/usb_cam` overlay | Active arithmetic fix already implemented and operator verified; true physical exposure event remains unmeasured |

The historical ChArUco documentation reports unstable, estimator-dependent
timing estimates from three earlier captures (approximately
−42/−58/−66 ms by adjacent-frame yaw versus −52/−90/−114 ms by integrated
gyro). It explicitly warns **none are validated sensor offsets**.
No prior offset may be copied into XRSLAM just because an analysis
script produced it: sign convention, source timestamp semantics, runtime
mapping and repeatability are distinct verification gates.

**Configured, not calibrated:** update YAML directly when the physical
mounting configuration changes; the Xacro itself should not repeat
hardware-specific numbers. The present positions and angles remain
provisional. Keep XRSLAM pose and Nav2 motion disabled until calibration
is actually verified. No motion, recording, camera or MCU serial
access is authorized.


## Effective patched camera overlay — operator PASS

The hardware-free overlay test subsequently passed:
`STAGE3_USB_CAM_EFFECTIVE_PREFIX=/leroy_ws/install/usb_cam`,
`STAGE3_USB_CAM_EFFECTIVE_SOURCE=CORRECT_MICROSECOND_EPOCH_FORMULA`,
`STAGE3_USB_CAM_EFFECTIVE_LINKAGE=OVERLAY_COMPONENT_AND_LIBRARY`,
`STAGE3_USB_CAM_EFFECTIVE=PASS_PATCHED_OVERLAY_SELECTED_NO_DEVICE`.
The camera executable SHA-256 is
`6f19b25e2ce97bdaac6106e93de1f2a8fe0842cb6a5f3fba1f8a084c75a87edf`;
the linked camera library SHA-256 is
`8841f868515337e0b637520d5da9e99ea4776f8eae39653ef8f50498d8381aeb`.

**The earlier concern that the Debian-installed header's epoch
arithmetic bug is the live camera code is resolved.** The active
camera uses the previously patched overlay. The true exposure
timestamp convention, camera–IMU phase and rigid mounting geometry
remain unmeasured; neither source nor build provenance fixes those.

The existing physical worksheet below is the next non-runtime task.
Confirm `base_link` origin and camera/IMU physical frame axes,
sensor center offsets and their uncertainty **without moving the rig
or opening any device**. A later excitation/calibration session
would require a distinct reviewed plan and new operator authorization.


> **Correction after reviewing the existing Leroy build recipe.**
> The image's Debian-installed usb_cam 0.8.1 header carries the
> upstream divide-by-1000 bug, but the accepted
> `ros/Dockerfile` already builds a separate fixed usb_cam
> overlay. Do not assume the header examined by the underlay-only
> inventory is the header or implementation selected by the live
> `ros2 launch` process. The next no-device test is
> `bash ./m23bv_v3_stage3_usb_cam_effective_overlay.sh`, which
> explicitly reconstructs the accepted Stage 3 overlay sourcing
> order and resolves the executable, component, linked library
> and patched source. The physical exposure event/phase and rig
> extrinsics remain unmeasured even if this overlay check passes.


**Status:** Pinned source inspection only; physical calibration has **not**
been performed. This document must never be interpreted as approval to
record, move the robot, operate the MCU or enable navigation.

## What the accepted source actually establishes

The accepted `ros/m23bv_reliable_rectifier_v3_candidate.launch.py` starts
`usb_cam_node_exe` with the visually identified front camera mapped
to `/dev/video2`, at configured `640×480`, `30.0 FPS`,
`io_method=mmap`, `pixel_format=mjpeg2rgb`, frame
`camera_optical_frame`. Intrinsics come from pinned
`ros/config/camera/front_camera.yaml`, SHA-256
`f1c99cb8d4e4f3c77373dcabbe683dee9ffbc050614396fcd2964d61e965a138`.
This YAML contains **intrinsic lens calibration**, NOT measured camera
mounting extrinsics or a camera–IMU timestamp offset.

The pinned reliable rectifier preserves the original Image header:
`cv_bridge::CvImage(image_msg->header, ...)`. Its no-distortion fast path
also republishes the incoming image unmodified. Previously accepted
Stage 3 passive tests observed matching raw, rectified, CameraInfo and
DepthART stamps, but **that only establishes header preservation and
cross-topic matching**, not when actual photons hit the sensor.

The camera's initial ROS Image timestamp originates in the `usb_cam`
driver. The upstream ROS 2 `usb_cam` node sources stamps from its
camera's `get_image_timestamp()`; upstream timestamp utilities can map
V4L2 buffer time to an epoch timestamp. **Do not assume the exact
source behavior of the compiled Leroy binary from a GitHub upstream
main-branch page.** The installed package version and immutable binary
SHA-256 must first be inventoried, and the corresponding released
source reviewed if a precise time-domain assertion is required.

The accepted `McuClockSync` mapper anchors its **first** MCU sample
timestamp approximately 5 ms before that sample's ROS host receipt
time and thereafter advances according to corrected MCU tick deltas.
It estimates **relative rate**, not true one-way USB transmission
latency, camera exposure timestamp convention or constant camera–IMU
phase. The latest real VIO report's −28.97 ppm, 1.3 ms last mapped
age and 11.7 ms maximum mapped age therefore cannot be substituted for
`camera_imu_offset_sec` calibration.

The deployed diagnostic XRSLAM launch uses a clearly **provisional**
`camera_imu_offset_sec=0.0`,
`pose_convention_validated=false`,
`enable_map_tf=false`. Native VIO frame submission and a 50 Hz
IMU stream were actually observed, but XRSLAM remained
`initializing`.

## Current geometric values (YAML input, Xacro output)

`ros/src/leroy_description/config/sensor_mounts.yaml` supplies the mounting transforms; Xacro supplies the conventional optical-frame and legacy-alias joints:

| Relationship | xyz metres | rpy radians | Confidence |
| --- | --- | --- | --- |
| `base_link -> imu_link` | `0.095 -0.095 0.0` | `0 0 0` | **PROVISIONAL; not measured** |
| `base_link -> front_camera_link` | `0 0 0` | `0 +0.5235987755982988 0` | **PROVISIONAL; camera origin zero placeholder** |
| `front_camera_link -> front_camera_optical_frame` | `0 0 0` | `-1.5707963267948966 0 -1.5707963267948966` | Formal ROS optical-axis convention, not a mounting survey |
| `front_camera_optical_frame -> camera_optical_frame` | `0 0 0` | `0 0 0` | Legacy frame ID alias |

**Edit the YAML for mounting changes, not duplicate values in the Xacro.**
The camera and IMU are distinct packages, and the front camera's
zero translation is still a placeholder, not a validated optical-center
measurement.

## Current no-device configuration check

From `ros/`, after updating the Stage 3 branch:

```bash
bash ./m23bv_v3_stage3_sensor_mounts_yaml_no_device.sh
```

This reads the checked-in YAML and Xacro, then expands them inside
the existing immutable V3 sensor image. It verifies the configured
fixed joints and proves that changing a **temporary** YAML value
changes the generated URDF. It never starts ROS nodes, opens video or
MCU devices, records data or moves the robot.

The earlier `m23bv_v3_stage3_calibration_provenance_host.sh` is
kept for the **original, now superseded source revision**. Its
historical result was PASS; its exact old URDF Git blob check will
correctly BLOCK after this intentional YAML refactor. Do not use
its expected success markers as a claim about the new geometry.

## Configured values and unresolved calibration

For normal geometry updates, edit the `mounts.imu`,
`mounts.front_camera` and `mounts.workspace_camera` entries in
`sensor_mounts.yaml`: each has `xyz` metres and `rpy` radians
in the defined robot base frame. No duplicate numeric editing of
Xacro is necessary.

For current Stage 3 validation, retain `status: provisional`.
A YAML value is a **configuration**, not proof that the base-link
physical landmark, camera optical centre or IMU sensing die has
been physically surveyed. The firmware publishes robot-aligned
IMU axes; ROS must not apply the chip-axis rotation again. The
camera exposure timestamp event and camera/IMU absolute phase
remain unmeasured, and the camera–IMU phase 0.0 s is a provisional
diagnostic setting. Do not promote these to VIO/odometry or
motion authority simply because a YAML-to-TF check succeeds.

## Gate status

- ACCEPTED: pinned real sensor transport; camera/header/depth contract;
  native XRSLAM camera/IMU diagnostic ingress; actual isolated
  controller/planner/BT navigator Configure/Cleanup and graceful exit.
- UNMEASURED: true exposure event time, absolute camera–IMU phase,
  actual camera/IMU mounting transforms, VIO tracking performance.
- BLOCKED: metric pose output, map/odom TF, live Nav2 activation,
  costmap collision-safety claim, motor commands, motion and recording.

## Newly identified upstream usb_cam 0.8.1 release timestamp defect

The operator completed the accepted source + pinned image inventory:

```text
STAGE3_CAL_PROVENANCE=PASS_SOURCE_ONLY_UNCALIBRATED
STAGE3_CAL_USB_CAM_INSTALLED_PACKAGE=ros-jazzy-usb-cam
STAGE3_CAL_USB_CAM_INSTALLED_VERSION=0.8.1-1noble.20260912.190806
STAGE3_CAL_USB_CAM_NODE_BINARY_SHA256=b760f640ecb19fdded3c4028c2f4cbd44b64ed93912246eaaad86155c5d96249
STAGE3_CAL_PROVENANCE=PASS_PINNED_SOURCE_AND_INSTALLED_DRIVER_INVENTORY
STAGE3_CAL_CAMERA_EXPOSURE_PHASE=NOT_MEASURED
STAGE3_CAL_EXTRINSICS=NOT_MEASURED
```

Review of **tagged upstream usb_cam 0.8.1** uncovered the
`epoch_time.tv_usec / 1000.0` conversion defect in the epoch shift
helper. Upstream issue #327 was addressed by PR #338 **after** the
0.8.1 tag. The affected 0.8.1 MMAP capture path converts V4L2
buffer timestamps through that helper. Depending on the process's
start-time microsecond fraction, the tagged source can apply up to
almost one second of systematic timestamp bias, without breaking
raw/rectified/DepthART stamp matching. The tagged source flaw is
real, but it remains necessary to verify the shipped binary was
built from that implementation; a package version alone cannot
prove it. This is a **new software-timing risk**, separate from the
unmeasured physical camera–IMU offset.

The next strictly no-device source/install inspection is:

```bash
git pull --ff-only
bash -n ./m23bv_v3_stage3_usb_cam_epoch_inspect.sh
bash ./m23bv_v3_stage3_usb_cam_epoch_inspect.sh
```

See `ros/M23BV_V3_STAGE3_USB_CAM_TIMESTAMP_REGRESSION.md` for
the exact upstream source/issue links, formula, prediction examples,
operator commands, and confidence limits. Physical calibration,
video recording, camera restarts, MCU tests and robot/sensor movement
remain unauthorized. Do **not** enable XRSLAM pose or Nav2.
