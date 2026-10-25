# Stage 3: XRSLAM Camera–IMU Calibration Provenance

## Source-only CAD reconciliation is next

The old worksheet below was written **before** the operator reminded
us that these quantities had already been defined. It is an inventory
of provenance gaps in the current Stage 3 ROS Xacro, **not an
instruction to physically measure everything again**.

`hardware/parts.FCStd` is present alongside the canonical Xacro.
Use `ros/m23bv_v3_stage3_existing_geometry_cad_audit.py`
to read the existing FreeCAD object's metadata and proposed
mount-related placements without FreeCAD, ROS or any hardware.
See `M23BV_V3_STAGE3_EXISTING_GEOMETRY_RECONCILIATION.md`
for the evidence, operator command and limitations: CAD part
placements must not be promoted automatically to calibrated
`base_link -> camera/imu` extrinsics.

## Existing Leroy frame definitions and calibration history — reuse, do not re-ask

The operator has **already specified geometry and sensor axes before**
this Stage 3 review. The previous request to define all four quantities
again was redundant. This file now inventories exactly what can be
recovered from tracked sources without inventing measurements.

| Quantity | Already established source | Scope and current authority |
| --- | --- | --- |
| Robot coordinate directions | `ros/src/leroy_description/urdf/leroy.urdf.xacro`: +X forward, +Y left, +Z up | Axis convention **DEFINED**; the physical chassis landmark chosen as the `base_link` origin is **not explicitly identified in the current checked-in URDF/README** |
| IMU axes on assembled robot | `firmware/include/imu.h` and `firmware/src/imu.c`: forward=−ICM X, left=−ICM Y, up=+ICM Z, CCW yaw=+ICM gyro Z; gyro roll/pitch signs inverted, yaw unchanged | **ALREADY DEFINED AND IMPLEMENTED** in `imu_read_robot()`; `/imu/data_raw` contains robot-frame acceleration/gyro values from the MCU. **Do not apply another 180° sensor-axis correction in the ROS TF** |
| IMU center translation in current TF | `ros/src/leroy_description/urdf/leroy.urdf.xacro`: +95 mm X, −95 mm Y, 0 mm Z | Existing parameter; its own source explicitly marks this geometry **PROVISIONAL**, including unknown physical height. Not independently certified as the IMU die/sensing center |
| Front camera orientation and frame axes | Same Xacro: +30° camera pitch; optical frame +X right, +Y down, +Z forward; `camera_optical_frame` alias | Existing defined TF *model*, but physical 30° mount angle still explicitly **PROVISIONAL** |
| Camera optical center translation | Same Xacro: (0, 0, 0) m | Existing value is an explicitly documented **zero placeholder**, not an assertion that the optical center is at the base origin |
| Workspace camera | Same Xacro: +60° pitch, zero translation | **PROVISIONAL**, not the front camera used by Stage 3 VIO |
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

**Unresolved documentation reconciliation, not a request to redefine
the robot:** The checked-in canonical Xacro describes IMU center,
camera center and mount pitch as *provisional*. The user reports they
already supplied their actual geometry elsewhere. Retrieve those
original measurement records and their `base_link` origin/reference
before making a new Xacro edit. Do not replace the source's explicit
uncertainty with guessed measurements, and do not request another
physical session while existing evidence has not been reconciled.
No motion, recording, camera or MCU serial access is authorized.


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

## Current geometric placeholders (canonical URDF)

`ros/src/leroy_description/urdf/leroy.urdf.xacro`:

| Relationship | xyz metres | rpy radians | Confidence |
| --- | --- | --- | --- |
| `base_link -> imu_link` | `0.095 -0.095 0.0` | `0 0 0` | **PROVISIONAL; not measured** |
| `base_link -> front_camera_link` | `0 0 0` | `0 +0.5235987755982988 0` | **PROVISIONAL; camera origin zero placeholder** |
| `front_camera_link -> front_camera_optical_frame` | `0 0 0` | `-1.5707963267948966 0 -1.5707963267948966` | Formal ROS optical-axis convention, not a mounting survey |
| `front_camera_optical_frame -> camera_optical_frame` | `0 0 0` | `0 0 0` | Legacy frame ID alias |

**Do not update the URDF just because these values exist or because
the ROS TF tree is connected.** Camera and IMU are distinct physical
packages, and the front camera has no measured translational offset
in this URDF.

## Next independent read-only operator check

From `ros/` on the updated Stage 3 branch:

```bash
git pull --ff-only
bash -n ./m23bv_v3_stage3_calibration_provenance_host.sh
python3 -B - <<'PY'
import ast
from pathlib import Path
p = Path("m23bv_v3_stage3_calibration_provenance.py")
ast.parse(p.read_text(), filename=str(p))
print("STAGE3_CAL_SOURCE_AST=PASS")
PY
bash ./m23bv_v3_stage3_calibration_provenance_host.sh
```

This executes a read-only pinned-source audit and inspects the
installed `ros-jazzy-usb-cam` Debian package version and
`usb_cam_node_exe` binary hash in the **existing immutable pinned
sensor image** via rootless Podman `--network none`, with no
device bindings and no ROS nodes started. It does **not** open
`/dev/video*`, `/dev/ttyACM*`, or change calibration.

Expected terminal marker (pending operator execution):

```text
STAGE3_CAL_PROVENANCE=PASS_PINNED_SOURCE_AND_INSTALLED_DRIVER_INVENTORY
STAGE3_CAL_CAMERA_EXPOSURE_PHASE=NOT_MEASURED
STAGE3_CAL_EXTRINSICS=NOT_MEASURED
STAGE3_CAL_MOTION_RECORDING_MCU_ACCESS=NOT_AUTHORIZED
```

A PASS **does not close** the exposure-phase/extrinsic gaps; it
documents precisely which source/binary version requires further
inspection.

## Operator measurement worksheet — no rig/chassis motion authorized

With both motor power rails physically isolated and robot stationary,
a competent operator can prepare the following measurement references.
No video recording, MCU serial opening or moving calibration manoeuvre
is part of the current scope.

| Datum | Required observation | Currently documented |
| --- | --- | --- |
| Physical `base_link` origin | Physical landmark and +x/+y/+z directions on chassis | Defined axis convention only; physical origin **unknown** |
| Front camera optical center | x/y/z in metres relative to `base_link`; uncertainties | **Unknown**, current xyz zero is placeholder |
| Front camera axis orientation | Measured orientation of forward/right/down optical axes relative to base, with uncertainty | **Unknown**; 30° pitch is assumed |
| IMU sensing-package origin | x/y/z relative to `base_link`; uncertainties | **Unknown**; current 95 mm offsets are provisional |
| IMU axes | STM32 ICM-20948 package-to-robot polarity and frame convention | **ALREADY DEFINED AND IMPLEMENTED:** −ICM X forward, −ICM Y left, +ICM Z up; robot-aligned `/imu/data_raw` needs no second ROS TF rotation. Physical IMU die center remains unverified |
| Frame hardware model | Exact camera sensor, video driver + firmware timestamp conventions | USB model known; exposure event timing **not verified** |
| Frame `header.stamp` event | Exposure start, midpoint, end of readout, V4L2 completion or delivery — identify and measure | **Unknown** |
| Absolute camera–IMU time offset | Measured phase, sign convention, uncertainty and drift | **Unknown**; 0.0 s is diagnostic placeholder |

An independent spatial measurement can be done without making the
robot move. **Do not physically excite the sensor rig, move the
chassis, reconnect either motor rail, open the MCU or record camera/IMU
streams until there is a separately reviewed test and one-time
operator authorization.** Tracking initialization from a stationary
trial is inconclusive; do not relax estimator or odometry gates to
manufacture a localization PASS.

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
