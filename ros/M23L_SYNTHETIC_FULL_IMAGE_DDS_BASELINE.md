# M23L — Synthetic ROS 2 full-RGB Image DDS transport baseline

**Draft implementation. Zero camera, STM32, IMU, robot or VIO access.**
This PR is stacked on M23K PR #105; do not merge before parent PR
or retargeting and validation.

## Reason for the diagnostic

First genuine M23H closed camera+IMU bag:
729 rect Image, 3906 CameraInfo, 6001 IMU; 158 image intervals
above the M23A 250ms gate, max image gap 2951.988ms; image
window starts 10570.173ms before IMU, ends 335.923ms after.
That bag is correctly BLOCKED for VIO, remains immutable,
and will NOT be modified or recaptured by M23L.

USB camera's native V4L2 MJPG 640x480 30fps stream completed 900
frames at ~29.81fps, exit 0; thus native MJPEG transfer succeeds.
Operator M23K camera-only C++ header counter runs without rectifier:
- raw_only_mjpeg: raw 178 over ~46s (~3.855Hz), CameraInfo
  1414 (~29.814Hz), 46 raw gaps >250ms, max 3288ms.
- raw_only_yuyv: raw 234 over ~47s (~5.001Hz), CameraInfo
  1413 (~29.812Hz), 42 raw gaps >250ms, max 3656ms.
- Both raw-only modes had clean shutdown and no MCU device.
- Full-image DDS subscribers (even C++) observe far fewer images
  than CameraInfo. Subscriber reception and publisher count differ,
  so these results do not identify usb_cam publish rate.

Upstream ros-drivers/usb_cam source has *separate* update and
publish timer callbacks for regular image transport, but exact
locally installed Jazzy usb_cam build has NOT been inspected.
These callbacks are a plausible scheduling issue, not a conclusion.

## Controlled test

M23L generates **900 SYNTHETIC 640x480 RGB8 Image messages,
921600 bytes each, nominal 30Hz**, with 900 CameraInfo messages.
C++ publisher in one ROS process, existing M23K C++ image-header
counter in a **second ROS process**; distinct processes require
actual DDS serialization/deserialization. Publisher uses reliable
depth-100 QoS; subscriber uses best-effort SensorDataQoS. Same
basic payload and topic names as the M23K raw-only test, but
NO USB camera, JPEG decoding, rectification, or sensors.

The runner checks:
- exact C++ publisher image and info counts of 900
- actual elapsed time between first and last publisher timer
  callback starts, not merely nominal requested timer period
- maximum publisher callback duration
- independently observed image + info count in subscriber
- missing or malformed publisher or subscriber exit/final marker
- no physical device mappings, no ROS graph bridge
- 90% min observed image and CameraInfo counts for a purely
  diagnostic synthetic DDS-good classification (not M23A VIO
  acceptance). A bad result reports exact loss, no blanket PASS.

Interpret:
- Synthetic publisher completes 900 in ~30s and subscriber
  receives >=90% full Images: DDS can carry that workload
  in an isolated container. Investigate usb_cam acquisition,
  conversion, shared message updates/publish scheduling, plus
  real-world ROS graph differences. Does NOT prove ROS image
  DDS cannot drop in concurrent full camera+IMU workload.
- Synthetic publisher completes 900 near 30Hz but subscriber
  receives far fewer full Images while receiving CameraInfo:
  investigate DDS fragmentation, shared memory, reliable pub /
  best-effort sub interaction, executor/receive CPU. Does NOT
  prove usb_cam produces correct frames.
- Publisher itself cannot sustain 900 in ~30–37s: publisher
  scheduling/serialization is too slow under this full-size
  synthetic test; cannot attribute to subscriber alone.
- Any missing counters/errors/timed out processes BLOCK.

Only the **two synthetic ROS processes** run; no ttyACM or video
devices are mapped. No change to M23A's 250ms header-gap gate,
camera settings, production rosbag or XRSLAM.

## Phase A — 16 pure-Python offline source/logic tests

~~~bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23l-image-dds-baseline
git pull --ff-only origin feature/m23l-image-dds-baseline

cd ros
PYTHONDONTWRITEBYTECODE=1 \
  python3 -u m23l_synthetic_dds_baseline_test.py
~~~

Require M23L_OFFLINE_SYNTHETIC_DDS_CONTRACT=PASS and
M23L_DEVICE_ACCESS=NONE. The tests use fake counter log strings
and in-memory dictionaries; they do not invoke ROS.

## Phase B — build local offline M23L image

Use the locally built M23K image, previously created for the
actual camera-only experiment. This is a disconnected build; it
compiles the synthetic publisher and M23K header counter again
into a separate, local ament install directory.

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  podman image exists localhost/leroy-sensors:m23k-camera-path-probe
  podman build --network none --no-cache \
    -f Dockerfile.m23l-dds \
    -t leroy-sensors:m23l-synthetic-image-dds .
) 2>&1 | tee m23l_no_device_build.log
M23L_BUILD_EXIT=${PIPESTATUS[0]}
echo "M23L_BUILD_EXIT_CODE=$M23L_BUILD_EXIT"
~~~

Require M23L_JAZZY_SYNTHETIC_CPP_BUILD=PASS and build exit 0.
If build dependencies are unavailable, fail and report exact error;
do not download or silently change the image.

## Phase C — run the actual synthetic DDS comparison, NO DEVICE

~~~bash
set +e
set +o pipefail
podman run --rm --name leroy-m23l-synthetic-dds \
  --network none --ipc private --pid private \
  --shm-size 512m \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  localhost/leroy-sensors:m23l-synthetic-image-dds \
  2>&1 | tee m23l_synthetic_dds_run.log
M23L_SYNTH_EXIT=${PIPESTATUS[0]}
echo "M23L_SYNTH_EXIT_CODE=$M23L_SYNTH_EXIT"
~~~

No camera or serial \`--device\` flags and no recording folder
mount. If possible, make no concurrent real camera/robot runs
while evaluating isolated reproducible transport.

Collect \`M23L_SYNTHETIC_DDS_JSON\`, \`M23L_VERDICT\`,
\`M23L_DEVICE_ACCESS=NONE\`, and exit code. Successful classification
is \`SYNTHETIC_FULL_IMAGE_DDS_PROFILE_ONLY\`, NOT a real-camera,
calibration, tracking or motion acceptance. Failure classification
identifies whether publisher scheduling or subscriber delivery
lagged in this synthetic context.

Do NOT re-run real STM32 or remap any device to troubleshoot
synthetic DDS. Preserve the original M23H bag and exact M23A
input gate.
