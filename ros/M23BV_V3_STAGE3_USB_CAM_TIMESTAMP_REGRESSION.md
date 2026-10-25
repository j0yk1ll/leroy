# Stage 3: usb_cam 0.8.1 epoch timestamp regression investigation

> **Update — overlay selection is the decisive question.** The accepted
> `ros/Dockerfile` already clones upstream usb_cam 0.8.1 at
> `cb2ae6bc0a312de629f84fef933fb3e07dac1ef8`, substitutes the
> corrected `epoch_time.tv_usec` expression and installs a separate
> `/leroy_ws/install/usb_cam` overlay. `ros/CAMERA_TIMESTAMP_FIX.md`
> records previous physical measurements with that fix (about 50–60 ms
> apparent raw-image delay versus 110–971 ms for the Debian binary).
> The pinned V3 image derives from the M16 sensors layer, which declares
> it inherits that patched base. **The operator's just-reported
> `LEGACY_DIVIDE_1000_PRESENT` was found in the Debian underlay's
> installed headers; it does not demonstrate the active camera node
> executes that code.** My earlier interpretation of those installed
> headers as evidence of an unresolved runtime bug was too strong.
>
> **Next source-resolving test, NOT YET EXECUTED**:
> `bash ./m23bv_v3_stage3_usb_cam_effective_overlay.sh` from `ros/`,
> after `git pull --ff-only`. It checks `ros2 pkg prefix usb_cam`,
> patched source bytes, overlaid executable/component and dynamic
> `libusb_cam.so` linkage in the exact accepted image, using
> `--network none` with no devices. If PASS, the known tagged upstream
> conversion defect is **already mitigated in the running ROS overlay**;
> the true exposure timestamp event and absolute camera–IMU phase
> would still be unmeasured.


## Status

## Operator-reported pinned underlay artifact hashes

The independent no-device inventory completed successfully:

```text
STAGE3_USB_CAM_EPOCH_SOURCE_REGRESSION=PASS_UPSTREAM_0_8_1
STAGE3_USB_CAM_EPOCH_INSTALLED_HEADER=UNRECOGNIZED
STAGE3_USB_CAM_EPOCH_HEADER_SHA256=b4b967e469a67a92367da3c2ed7c55621e507624bea630fa81810424d93ebff5
STAGE3_USB_CAM_EPOCH_INSTALLED_HEADER=LEGACY_DIVIDE_1000_PRESENT
STAGE3_USB_CAM_EPOCH_HEADER_SHA256=aa758d0dbe8703bb12545dc3c6d114e00d17e436aaa3e8dc37c8a278560b1ae4
STAGE3_USB_CAM_EPOCH_LIBRARY_SHA256=cdc776abd3c21f1c2b6c8928241bad59eca83b9c7a100d2624bbc637d332b705
STAGE3_USB_CAM_EPOCH_LIBRARY_SHA256=5feab7508ca523e31f71c58a30686ff8151fbf80ea420b95619c57e7787eb8fe
STAGE3_USB_CAM_EPOCH_INSPECT=PASS_SOURCE_AND_INSTALLED_ARTIFACT_INVENTORY
STAGE3_USB_CAM_EPOCH_COMPILED_IMPLEMENTATION=NOT_INDEPENDENTLY_VERIFIED
```

These are **Debian package artifact hashes, not proof of which
ROS overlay executable/component was active** in the accepted
physical Stage 3 V3 runs. Source-selection verification is still
pending via `ros/m23bv_v3_stage3_usb_cam_effective_overlay.sh`.


The accepted no-device provenance inventory identified:

- Pinned immutable sensor image:
  `f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c`.
- Installed package:
  `ros-jazzy-usb-cam=0.8.1-1noble.20260912.190806`.
- `usb_cam_node_exe` SHA256:
  `b760f640ecb19fdded3c4028c2f4cbd44b64ed93912246eaaad86155c5d96249`.

**New source-derived risk:** the upstream `ros-drivers/usb_cam` **tag
`0.8.1` (released May 1, 2024)** has a unit error in
`include/usb_cam/utils.hpp`,
`get_epoch_time_shift_us()`:

```cpp
const int64_t epoch_us =
  epoch_time.tv_sec * 1000000 + epoch_time.tv_usec / 1000.0;
```

`epoch_time.tv_usec` is **already microseconds**. Dividing it by
1,000 effectively discards the fractional second of the epoch time
sample taken when the camera object is constructed. The correct
formula uses `+ epoch_time.tv_usec`. Upstream bug
[ros-drivers/usb_cam#327](https://github.com/ros-drivers/usb_cam/issues/327)
was closed by [PR #338](https://github.com/ros-drivers/usb_cam/pull/338)
on June 22, **after the released 0.8.1 tag**.
The current upstream main-branch code has the corrected formula.

Release source:
- https://raw.githubusercontent.com/ros-drivers/usb_cam/0.8.1/include/usb_cam/utils.hpp
- https://raw.githubusercontent.com/ros-drivers/usb_cam/0.8.1/src/usb_cam.cpp
- https://raw.githubusercontent.com/ros-drivers/usb_cam/0.8.1/src/ros2/usb_cam_node.cpp

The tagged 0.8.1 `src/usb_cam.cpp` MMAP path uses `VIDIOC_DQBUF`
`buf.timestamp` and `calc_img_timestamp(...,m_epoch_time_shift_us)`.
The ROS 2 node copies `get_image_timestamp()` into
`/camera/image_raw` and `CameraInfo`. The accepted V3 rectifier
preserves this header. Therefore, if the affected conversion is in
the **installed library**, a substantial bias can propagate
unchanged through DepthART and XRSLAM even while all earlier topic
stamp-equality checks pass.

### Mathematical impact — source-code prediction, NOT measured hardware

For an epoch `tv_usec` sampled when `UsbCam` constructs its epoch
shift, the approximate bias is:

`rounding_or_truncation(tv_usec / 1000) - tv_usec`
microseconds.

| Epoch fractional microsecond sample | Approximate stamp bias |
| --- | --- |
| 0 | 0 ms |
| 250,000 | −249.75 ms |
| 500,000 | −499.50 ms |
| 999,999 | about −999 ms |

This is mainly a **process-startup-dependent constant offset** in the
camera's epoch conversion, not necessarily variable jitter on each frame.
It **may** explain the lack of XRSLAM tracking, but the accepted
`initializing` outcome alone is not proof of that causal relationship.
No source-only test can infer the real camera exposure timestamp event
from V4L2 buffering; the Linux kernel allows driver-specific timestamps
and source flags:
https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/buffer.html

### Confidence limits

The operator has **proved the pinned binary package version and the
executable checksum**, but has not supplied a hash or behavior check
for its linked driver library, source build recipe or installed header.
A package version bearing upstream 0.8.1 does **not by itself prove**
the packaged binary contains the bug: vendor patches and backports
are possible, and inline implementations may be optimized by the compiler.

The separately pinned M18 MCU clock mapper remains a different
uncalibrated transport-anchoring mechanism (first stamp at host
receipt minus 5 ms, relative rate correction afterwards). Even a
correctly patched camera driver will NOT validate the absolute
camera–IMU exposure phase or physical sensor extrinsics.

### Next operator action — NO DEVICES

On the updated Stage 3 branch, from `ros/`:

```bash
git pull --ff-only
bash -n ./m23bv_v3_stage3_usb_cam_epoch_inspect.sh
bash ./m23bv_v3_stage3_usb_cam_epoch_inspect.sh
```

The source-only fixture demonstrates the exact **tagged upstream
arithmetic** using deterministic synthetic fractional seconds. The
pinned, rootless, read-only network-none sensor-image inventory checks
the exact installed package and executable hash; if available, it
also classifies the installed `usb_cam/utils.hpp` source text and
fingerprints the installed shared library. It never starts ROS,
opens a camera or MCU, modifies image bytes, records video or moves
anything.

It reports `COMPILED_IMPLEMENTATION=NOT_INDEPENDENTLY_VERIFIED`
even if the installed header matches the defect, and
`EXPOSURE_PHASE=NOT_MEASURED`. A possible installed-header
`LEGACY_DIVIDE_1000_PRESENT` result is evidence of the source-level
risk in that package; it is not proof of the exact compiled code.

**Do not reuse the accepted live VIO test to compensate for or tune
away this possible epoch bias.** If confirmed against the packaged
library, stage a separately pinned and compiled fixed-camera
*diagnostic image* that changes only the upstream unit bug, with
source-level and no-device regression checks BEFORE any new physical
camera authorization. Never silently patch or retag the immutable
accepted V3 sensor image.

### Safety and deployment limits

No validated camera/IMU phase, metric pose, extrinsics or motor
authorization exists. Keep `pose_convention_validated=false`,
`enable_map_tf=false`; real Nav2 stays inactive, recording and
physical sensor-rig movement remain unauthorized. The previous
operator's camera/MCU tests were stationary approvals and do not
extend to new experiments.
