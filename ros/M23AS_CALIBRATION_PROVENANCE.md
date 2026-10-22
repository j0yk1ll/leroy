# M23AS — offline calibration provenance gate before a camera-only trial

## Evidence already established

The operator has confirmed both the motor and manipulator power supplies
are **physically disconnected**. This is an operator safety attestation,
not a sensor-verified power measurement. The existing source-only M23AQ
launch accepts only patched usb_cam + unmodified stock composable
RectifyNode, and refuses to construct a launch without explicit
operator review; all 11 tests passed. M23AP stock
rclcpp_components/component_container synthetic pthreads OpenCV
900/900 test passed with actual child exit 0.

Read-only udev preflight identifies /dev/video8 as a capture-capable
Wed Camera (vendor/product 32e6:9221), with stable /dev/v4l/by-id/
and by-path/ symlinks. Device indices are volatile: do not
automatically assume /dev/video8 persists across replug/reboot.
Do not republish a full USB serial string.

## Newly discovered original calibration source

The repository contains both:
- `utils/charuco/calibration.yaml`: the generated ChArUco result
  using 92 source calibration PNG images (none rejected), 640x480,
  `plumb_bob`, and RMS reprojection error approximately **0.3184px**,
  with the intrinsic K, five distortion terms, R and rectified P.
- `ros/config/camera/front_camera.yaml`: the deployment copy
  of the same intrinsics, distortion and projection matrices rounded
  to roughly 6 decimal places (camera_name changed to front_camera).

The original 92 PNGs are also retained in
`utils/charuco/calibration_images/`.
The ChArUco README explicitly requires constant focal position,
crop, stabilization and capture mode, but the calibration YAML
**does not record the USB vendor/product or camera serial**.

The pure-stdlib M23AS audit compares source vs deployment
image resolution and K/D/R/P arrays (tolerance 5e-6), validates
stored source view count and RMS metadata, and examines the
PNG signature and IHDR size of each of the 92 referenced
calibration images without decoding their pixel content.

This demonstrates numerical/source-image provenance and nominal
640x480 agreement. It does **NOT** establish that the current
Wed Camera captured those files or that its focus/zoom/crop and
sensor mode are unchanged. Explicit physical operator
confirmation is still required before any actual camera launch.

## Safe one-time operator source check — no video/ROS/Podman access

From the accepted M23AQ clean worktree:

~~~bash
(
  set -euo pipefail
  test "$(id -u)" -ne 0
  test ! -e /tmp/leroy-m23as
  cd /tmp/leroy-m23aq
  git fetch origin feature/m23as-calibration-provenance-offline
  git worktree add --detach /tmp/leroy-m23as     origin/feature/m23as-calibration-provenance-offline
  cd /tmp/leroy-m23as
  test -z "$(git status --porcelain)"
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23as_source_test.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23as_calibration_provenance.py
  PYTHONDONTWRITEBYTECODE=1     python3 -B ros/m23as_calibration_provenance.py --check
)
~~~

Expected 12 offline tests, inert default marker, then
`M23AS_VERDICT=STORED_CALIBRATION_NUMERICAL_PROVENANCE_MATCH`.
If blocked, stop, preserve checkout and output and ask to review.
No video device is opened, no serial ports accessed, no network
inside runtime, no ROS launched, no images recorded.

## Remaining before a separately reviewed real camera-only trial

- Confirm **the currently identified Wed Camera** was the
  physical camera whose images comprise the 92-image calibration,
  and that its original lens/focus, FOV/crop, and 640x480 capture
  mode are unchanged. If identity is unknown, require a separately
  reviewed limited real-camera validation/recalibration procedure;
  do not label mathematical source match as physical accuracy.
- Prior to an authorized run, re-resolve chosen /dev/v4l/by-id
  link (redact serial from published logs); confirm attached
  camera and only that capture node passes through rootless
  Podman; verify original motor-capable container/bridges are
  not running and motors/manipulator power is still disconnected.
- Use ONLY a separately reviewed camera-only launch, never
  `robot.launch.py` or `ros/exec.sh` (the existing bridge
  launch is unconditional and its MCU serial open is hazardous).
- Review time-limited capture start/stop, calibrated CameraInfo,
  message rate/gap and shutdown, unexpected graph publishers,
  CPU and memory headroom on host, logs and abort conditions.
  No other /dev, IMU, STM32, XRSLAM, DepthART, recording, motor,
  motion, Nav2 or production authorization.

M23AS_PHYSICAL_CAMERA_AUTHORIZED=FALSE
M23AS_PRODUCTION_ACCEPTANCE=BLOCKED
