# M23K — Isolated RAW vs RECT camera path diagnosis

**Revised after the first operator camera-only run. New C++ observer and
container rebuild NOT YET TESTED.**
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

## Evidence from the initial 45-second M23K Python observation

The operator's first M23K live camera-only run **completed**, but its
single Python executor reported raw 121 (2.606Hz), rect 450 (9.538Hz),
CameraInfo 1411 (29.811Hz). Only 83 raw/rect image headers matched;
**367 rectified headers were never observed by the raw subscriber**.
Because rectification normally retains source header stamps, this
strongly indicates that raw subscriber *observations* were lost or
unfairly scheduled; it is NOT proof the USB camera only published
2.606Hz, nor proof image_proc generated extra frames.

Raw max header gap 4260.056ms, rect 1140.052ms, CameraInfo 36.158ms.
Both raw and rect probes reported 41 >250ms gaps, versus zero
CameraInfo gaps. The exact stage dropping frames remains unresolved.

At shutdown the \`image_proc\` component container died with
**SIGSEGV (exit -11)**, despite M23K reporting an empty fault list.
A graceful camera stop must not hide this. The original run's
CAMERA_PATH_DIAGNOSTIC_ONLY verdict was therefore incomplete.

### Corrective instrumentation on this same draft PR

The revised observer is a **dedicated C++ rclcpp SingleThreadedExecutor**
rather than the Python \`spin_once\` loop. Its callbacks access only
\`sensor_msgs::msg::Image::header\` and \`CameraInfo::header\`
(no copying pixels into Python, no image processing and no publication).
Full DDS image message deserialization still occurs in rclcpp, so it
is LOWER-OVERHEAD, not a guaranteed lossless publisher counter.

M23K's Python parent supervises the unmodified camera launch and
the standalone compiled C++ subscriber as separate process groups,
captures sample lines to bounded /tmp logs and refuses incomplete
or mismatched C++ counter totals. It also scans the camera launch log
for child crash messages such as \`process has died ... exit code -11\`.
A SIGSEGV during shutdown now causes
\`INCOMPLETE_CAMERA_PATH_OBSERVATION\`, even if the image counts are
sufficient; ROS launch's normal SIGINT exit code 130 is distinguished
from a child crash.

The new C++ source lives under \`ros/m23k_cpp/\` and is built ONLY
inside the disconnected M23K image as the standalone ament package
\`leroy_m23k_camera_counter\`, separate from the production workspace.
Its executable must exist at
\`/opt/m23k-cpp-install/lib/leroy_m23k_camera_counter/m23k_camera_header_counter\`.
The observer and launch logs are written under /tmp in the ephemeral
container and are not modified on the host.

## M23K purpose

M23K compares in one isolated ROS 2 Jazzy graph:
- /camera/image_raw (from usb_cam)
- /camera/image_rect (from image_proc rectifier)
- /camera/camera_info (from usb_cam)

It reports per-topic count/rate using compiled header-only callbacks; header and monotonic receipt
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

Require 21/21 tests PASS,
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
M23K_RAW_RECT_OBSERVER_OFFLINE=PASS, compiled C++ counter present, and build exit 0.

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


## Follow-up evidence — second 45s C++ camera-only probe

Operator selected verified front USB camera by-id
`usb-Wed_Camera_Wed_Camera_20240420112206-video-index0`, currently
resolving to /dev/video8. The revised C++ header-only subscriber
showed raw **90 at 1.883 Hz**, rect **467 at 9.944 Hz**,
CameraInfo **1411 at 29.811 Hz**. Counts of >250ms header gaps:
raw **29** (max 4659.976ms), rect **50** (max 971.981ms),
CameraInfo **0** (max 36.12ms). Raw/rect identical stamp
intersection **59**, raw-only 31, rect-only **408**.

Since image_proc uses camera raw frames, independent DDS raw-image
subscription misses many frames relative to the rectifier's output.
The C++ callback only inspects headers but still receives/deserializes
full sensor_msgs/Image payloads. Do not infer that raw camera is
literally publishing at 1.883Hz or CameraInfo's 29.811Hz proves a
30Hz image publication rate.

**Repeatable SIGSEGV during camera launch SIGINT:** component_container
exit code **-11**, caught as CAMERA_CHILD_PROCESS_CRASHED,
shutdown_clean=false and INCOMPLETE_CAMERA_PATH_OBSERVATION. This
fault must not be marked clean or hidden; keep it separate from
the unknown frame-rate bottleneck. No STM32 accessed.

## Next discriminator — native host V4L2 MJPEG streaming benchmark

This is a **separate, direct camera-only experiment**: no ROS, no
image_proc, no DDS/rectified subscriber, no STM32. Use the same
currently verified external USB camera and ensure no process owns it.
Requires `v4l2-ctl` from Arch `v4l-utils`. Streaming data is
discarded, and neither original bag nor a new bag is written.

~~~bash
CAMERA_BY_ID="/dev/v4l/by-id/usb-Wed_Camera_Wed_Camera_20240420112206-video-index0"
CAMERA="$(readlink -f -- "$CAMERA_BY_ID")"
test -c "$CAMERA"
command -v v4l2-ctl
command -v timeout
command -v fuser
fuser -v "$CAMERA" || true
# Ensure the camera has NO competing owner before continuing.

v4l2-ctl -d "$CAMERA" --set-fmt-video=width=640,height=480,pixelformat=MJPG \
  --set-parm=30 --get-fmt-video --get-parm

set +e
set +o pipefail
timeout --signal=INT --kill-after=3s 45s \
  v4l2-ctl -d "$CAMERA" --stream-mmap=4 \
  --stream-count=900 --stream-poll \
  2>&1 | tee m23k_native_v4l2_640x480_mjpg.log
M23K_NATIVE_EXIT=${PIPESTATUS[0]}
echo "M23K_NATIVE_V4L2_EXIT_CODE=$M23K_NATIVE_EXIT"
~~~

Nominal target: 900 frames in ~30 seconds. Exit 0 if the
requested frame count finishes, 124 if the 45s timeout is reached
(which is evidence of <20fps average or stall/other streaming
issue, NOT by itself an exact frame count). Inspect the displayed
frame-rate and V4L2 errors; the stream is intentionally discarded.
If `v4l2-ctl` or `timeout` is missing, install/use the appropriate
host package rather than running within a motor-capable container.

- Native V4L2 roughly 30fps but ROS raw/rect remain gappy:
  prioritize usb_cam JPEG decode to RGB, image_proc processing,
  ROS 2 DDS delivery, executor CPU scheduling, and independent
  shutdown crash.
- Native V4L2 also stalls / low FPS: prioritize USB/UVC camera
  bandwidth, negotiated format, host USB topology, and driver logs.
- Always preserve the original m23g_live_capture001 forensics bytes
  and M23A 250ms header-gap blocker. No physical camera-IMU
  calibration follows from this benchmark.
