# M21D — Bounded passive metadata recorder prototype (NOT physical capture)

## Context

M20 accepted 120 seconds of concurrent physical camera/IMU ROS receipt, with
119.886 seconds overlapping windows and a reported IMU rate of 50 Hz.
M21A accepted offline log readiness and exposed a 1.351-second maximum
rectified-image receipt gap (2082 images/120s, mean 17.36 Hz). M21B tested
synthetic feature alignment (+47ms injected, +46.5ms recovered). M21C
validated the strictly bounded metadata JSONL schema. No physical
camera exposure-to-MCU acquisition offset has been measured.

## New components

- m21d_metadata_capture.py: pure-Python bounded, lock-serialized in-memory
  metadata capture. Accepts only exactly typed camera, CameraInfo, and MCU
  fields; writes NEW JSONL files without overwriting existing captures.
  Maximum 20,000 events and 8 MiB serialized output. Only writes after a
  complete validation using M21C; excludes all image pixel payloads.
- m21d_passive_ros_adapter.py: ROS-only, import-only observer for existing
  /camera/image_rect and /camera/camera_info topics. Reads their header
  timestamps and host monotonic receive times, not image pixel content.
  It checks expected publishers, forbidden graph nodes, environment guards.
  It also defines record_from_m18_source, which must be invoked by a future
  reviewed integration with ACTUAL parsed MCU fields after successful IMU
  publication. No motor, MCU/serial, camera-device, or ROS publisher access
  is established by simply importing the adapter.
- m21d_metadata_capture_test.py: 12 pure-Python regression tests for
  schema conformity, safe output, overflow, duplicate timestamps/sequences,
  source mismatch, uint32 MCU wrap, and absence of any auto-start entrypoint.

## Important non-goals and known integration work

THIS PR DOES NOT START LIVE CAPTURE and does not include a Podman image or
hardware command. The adapter has NO rclpy.init(), main() or serial.Serial().
Existing sensor_msgs/msg/Imu does NOT contain raw MCU seq/t_us, so a future
source-side integration must emit the metadata as the M18 source publishes.
Future review must test that both producers share the same bounded buffer
thread-safely, sample host monotonic receipt before any slow callback work,
validate ROS graph provenance, and preserve MCU START/STOP safety and source
fault behavior. The /camera/image_rect and CameraInfo subscriber must be
exercised inside an isolated Jazzy test image before any physical experiment.

Publisher names found from the ROS graph are observational, not verified
device identities or cryptographically authenticated source claims. A
REAL_UNCALIBRATED manifest never claims calibrated absolute phase or spatial
extrinsics. M21D does not estimate optical rotation from image pixels.

## Hardware-free Arch host validation

Run from the Leroy repository checkout (NO Podman build or ROS required):

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m21d-bounded-passive-metadata-recorder
  git pull --ff-only origin feature/m21d-bounded-passive-metadata-recorder
  cd ros
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s . -p m21d_metadata_capture_test.py -v
  TMP_M21D="$(mktemp)"
  rm -f "$TMP_M21D"
  trap 'rm -f "$TMP_M21D"' EXIT
  PYTHONDONTWRITEBYTECODE=1 python3 - "$TMP_M21D" <<'PY'
import json, sys
from m21d_metadata_capture import MetadataCapture
from m21d_metadata_capture_test import example
capture = MetadataCapture('SYNTHETIC')
example(capture)
report = capture.finish(sys.argv[1])
print('M21D_DEMO_REPORT_JSON=' + json.dumps(report, sort_keys=True))
PY
  PYTHONDONTWRITEBYTECODE=1 python3 m21_record_contract.py "$TMP_M21D"
) 2>&1 | tee m21d_offline_recorder.log
M21D_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M21D_OFFLINE_EXIT_CODE=$M21D_OFFLINE_EXIT"
```

Expected: 12 unit tests PASS, M21D_DEMO_REPORT_JSON reports 33 synthetic
events and device_access false, M21C_FORMAT_REPORT_JSON verdict
M21C_OFFLINE_METADATA_FORMAT_VALID, and exit 0.

## Mandatory future physical safety gate — not authorized here

Before ANY further physical MCU serial access: remove/inhibit BOTH wheel
AND manipulator power, verify physical E-stop engaged, immobilize Leroy,
verify current STM32 by-id and camera identity (historical front camera
host /dev/video8), ensure exclusive ownership, keep leroy-ros exited,
use a rootless disposable sensor-only Podman container with --network none,
private pid/ipc, cap drop, read-only root, minimal /dev/video and /dev/tty
devices. Never publish /cmd_vel or enable normal bridge, odom, TF, XRSLAM,
Nav2 or DepthART. Only existing approved M18 START/STOP commands may be
used; missing STOP ACK leaves state unknown and motor power inhibited.
No physical capture commands are provided for M21D. CPU DepthART
optimization remains deferred.
