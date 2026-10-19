# M23K — Isolated RAW vs RECT camera path diagnosis

**Draft: operator runtime and image build not yet tested.**
This branch is independently based on merged main
c05b126226249657635c050d48ceeba1b3415ee6.
No dependency on the unmerged M23I/M23J PRs #103/#104.

## Evidence motivating this focused check

The *first actual complete but unready recording* is immutable:
m23g_live_capture001, 729 rectified Image, 3906 CameraInfo,
6001 IMU, SHA stable; M23J shows 158 rectified-image gaps >250ms,
longest 2951.988ms. During that largest interval, 87 CameraInfo
and 148 IMU messages arrived. Image/recorder receipt gap shapes
closely agree; no proof whether source, rectifier, DDS or recorder
drops a particular frame. M20 observer recorded 636 rectified
images while rosbag captured 729: recorder alone is not proven
to be the bottleneck.

M20 currently launches usb_cam at 640x480, requested 30Hz,
mmap, mjpeg2rgb, and image_proc::RectifyNode, with the
camera stream at /camera/image_raw and rectified stream at
/camera/image_rect. CameraInfo is /camera/camera_info.

**Separately, M23J showed the IMU begins 10570.173ms after the
first camera frame and ends 335.923ms before the last image.**
This matches M20's intentional camera-first + 9s wait; it
requires a future M23H recorder-window alignment change, but
cannot account for the 158 mid-run camera stalls.

## M23K purpose

M23K compares in one isolated ROS 2 Jazzy graph:
- /camera/image_raw (from usb_cam)
- /camera/image_rect (from image_proc rectifier)
- /camera/camera_info (from usb_cam)

It reports per-topic count/rate; header and monotonic receipt
interval max/mean and gaps >250ms; raw/rect ratio and matching
exact header stamps across separately subscribed topics.
This is a **diagnostic observation only**. Python subscribing
to full Image payloads adds CPU/memory/DDS load, and two
independent best-effort subscribers may lose samples at different
rates. A raw-vs-rect difference is a strong lead, NOT conclusive
proof of stage-local drops. This run does not produce a rosbag.

Most importantly **M23K maps only the verified camera** to
/dev/video2. The new runner fails if any /dev/ttyACM0/1 is present,
and does NOT launch the M20 combined camera+MCU sensor runner.
No STM32 USB CDC port is opened, no IMU, bridge, Nav2, XRSLAM,
TF, odometry, or motor command topics are published. This is
a different camera-only operation from the earlier M23H bag.

## Phase A — pure offline tests

~~~bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23k-camera-raw-rect-diagnostic
git pull --ff-only origin feature/m23k-camera-raw-rect-diagnostic

cd ros
PYTHONDONTWRITEBYTECODE=1 \
  python3 -u m23k_camera_path_probe_test.py
~~~

Require 13/13 tests PASS,
M23K_RAW_RECT_OBSERVER_OFFLINE=PASS, M23K_DEVICE_ACCESS=NONE.

## Phase B — disconnected camera-only image build

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m20-camera-imu
  podman build --network none --no-cache \
    -f Dockerfile.m23k-camera-path \
    -t leroy-sensors:m23k-camera-path-probe .
) 2>&1 | tee m23k_camera_only_build.log
M23K_BUILD_EXIT=${PIPESTATUS[0]}
echo "M23K_BUILD_EXIT_CODE=$M23K_BUILD_EXIT"
~~~

Require M23K_JAZZY_CAMERA_PROBE_IMPORT=PASS,
M23K_RAW_RECT_OBSERVER_OFFLINE=PASS and build exit 0.

## Phase C — no-device container self-test

~~~bash
set +e
set +o pipefail
podman run --rm --name leroy-m23k-no-device \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  localhost/leroy-sensors:m23k-camera-path-probe \
  2>&1 | tee m23k_no_device.log
M23K_SIM_EXIT=${PIPESTATUS[0]}
echo "M23K_SIM_EXIT_CODE=$M23K_SIM_EXIT"
~~~

Require M23K_NO_DEVICE_SELFTEST=PASS,
M23K_DEVICE_ACCESS=NONE and exit 0.

## Phase D — operator camera-only physical diagnosis

Run only after phases A-C. Ensure production leroy-ros is
stopped, only the selected front USB camera is owned by this
container and the current /dev/v4l/by-id identity resolves to
the expected front USB camera. The earlier operator saw
usb-Wed_Camera_Wed_Camera_20240420112206-video-index0
-> /dev/video8; it must be rechecked after re-enumeration.
The previous laptop-integrated Quanta camera is NOT the
external robot front camera.

Set the current by-id symlink and opt in interactively:

~~~bash
cd "$(git rev-parse --show-toplevel)/ros"
export M23K_HOST_CAMERA_BY_ID="/dev/v4l/by-id/usb-Wed_Camera_Wed_Camera_20240420112206-video-index0"
readlink -f "$M23K_HOST_CAMERA_BY_ID"
export M23K_OPERATOR_CAMERA_ONLY_APPROVED=YES
bash ./m23k_operator_camera_probe.sh \
  2>&1 | tee m23k_first_camera_raw_rect_probe.log
~~~

The script checks the by-id symlink, resolved character device,
a stopped production container, exclusive camera ownership,
explicit operator approval and an interactive AUTHORIZE_M23K.
The only mapping is camera->/dev/video2, with no network
or STM32 CDC mapping.

Expected camera-only output:
M23K_CAMERA_PROBE_JSON={...}, M23K_VERDICT=CAMERA_PATH_DIAGNOSTIC_ONLY,
M23K_MCU_DEVICE_ACCESS=NONE.
If missing graph streams or another fault, fail with exit 2;
do not reinterpret it as a throughput diagnosis.

## How to interpret

1. **Raw and rect both low**, with similar >250ms gaps:
   investigate USB MJPEG decode, usb_cam camera-mode negotiation,
   camera bandwidth/host scheduling, and the cost of the diagnostic
   subscriber itself. This does NOT isolate which substage first
   lost the frames.
2. **Raw near nominal requested 30Hz but rect slow/gappy**:
   investigate image_proc rectification and its subscription/
   callback executor, DDS QoS/delivery and CPU load.
3. **Raw and rect both near nominal** on camera-only run:
   suspect effects specific to the concurrent M23H workload
   (image vs IMU observer/rosbag/DDS/SQLite), not proof of
   a permanent camera throughput fix.
4. **Missing raw/rect topics or no subscriber matches**:
   inspect camera launch/ROS graph and publisher offer QoS;
   do not treat as an image throughput measurement.

The next decision after an operator result is whether to fix
camera source/rectification or bounded concurrent sensor
capture. Only after continuity improves should we separately
re-align recorder time boundaries so IMU covers the image window,
then collect and READ-ONLY audit a new complete real bag. Never
change M23A's 250ms acceptance limit to manufacture a PASS.
