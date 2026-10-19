# M23N — Real front-camera-only DDS receiver reliability comparison

**Draft / not run on real hardware.** This branch is stacked on
M23M PR #107, M23L #106, and M23K #105, all currently draft.
This milestone is camera-only, operator-triggered; NO STM32 CDC
mapping, IMU collection, ROS bag, XRSLAM, odometry or motor commands.

## Grounded evidence

Native Wed USB camera V4L2 MJPG 640×480 30fps: 900-frame
direct capture at ~29.81fps, exit0. Isolated M23K C++
raw-only MJPEG ROS observer: **178 Images, ~3.855Hz**,
CameraInfo 1414 at ~29.814Hz; 46 image intervals >250ms,
longest 3288.079ms. Isolated raw-only YUYV similarly 234
Images at ~5.001Hz. These measure subscriber receipts,
not authenticated usb_cam publisher output.

M23M synthetic full-image DDS matrix held the 900×921600B
RGB8 publisher at ~29.967s plus 2s subscriber drain:

| Receiver | Image receipts | >250ms gaps | largest gap |
|---|---:|---:|---:|
| best_effort_5 | 127/900, 14.11% | 27 | 2333.411ms |
| best_effort_100 | 127/900, 14.11% | 18 | 3733.493ms |
| reliable_100 | 886/900, 98.44% | 0 | 34.133ms |

All profiles received 887/900 CameraInfo. Publisher offered
reliable depth100; the specific active RMW was **not** identified
(reported `unspecified_default`). These results demonstrate
QoS sensitivity under this one synthetic setup, not a
camera-side delivery fix or proven DDS fragmentation mechanism.

## Experiment

With current verified Wed external camera /dev/v4l/by-id
only, run the original M23K **raw_only_mjpeg** camera launch:
usb_cam 640×480 30fps requested, mmap, `mjpeg2rgb` and
existing calibration. **Do not start image_proc rectifier**.

Compare the SAME C++ raw/Image + CameraInfo header observer
using a different subscription on separate runs:

| M23N_SUB_QOS | reliability | history |
|---|---|---:|
| best_effort_100 | Best effort | 100 |
| reliable_100 | Reliable | 100 |

Parent M23N Python process selects the *prebuilt*
`/opt/m23m-cpp-install/.../m23k_camera_header_counter`,
sets `M23M_SUB_QOS` for its child only, and requires exactly one
matching `M23M_COUNTER_QOS_PROFILE=` line from the C++ process.
An unknown mode, missing C++ marker, camera stream or CameraInfo
must result in INCOMPLETE_CAMERA_PATH_OBSERVATION, not a PASS.

As in synthetic M23M, allow 2 seconds for receipt draining
after stopping the unchanged camera launch and before stopping
the subscriber. The whole run still observes only ~45 seconds
of the physical camera and uses a new container per variant.

Reports `M23N_CAMERA_QOS_JSON`,
`M23N_CAMERA_QOS_VERDICT`, receiver QoS verified status, raw
and CameraInfo counts/receipt rates, image stamp gap counts,
`shutdown_clean`, and `camera_publisher_rate_verified=false`.
Neither reliable subscription nor 30Hz CameraInfo authenticates
camera image publisher rate, source timestamps or camera-IMU phase.

**Important:** a reliable subscription is only compatible with
a publisher offering compatible QoS. M23M's synthetic publisher
offered reliable100, while the installed `usb_cam` binary's
offered QoS has not been verified. If no messages arrive, do
not label this test a 'camera failure' without checking QoS
compatibility; record INCOMPLETE and the camera launch output.

## Phase A — offline-only unit acceptance

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23n-real-camera-qos-probe
git pull --ff-only origin feature/m23n-real-camera-qos-probe
cd ros
PYTHONDONTWRITEBYTECODE=1 python3 -u m23n_camera_qos_probe_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23m_dds_qos_matrix_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23l_synthetic_dds_baseline_test.py
```

Require new **10/10**, previous M23M **9/9**, M23L **16/16**.
The M23N tests do not open `/dev/video*`, `/dev/ttyACM*`
or start ROS nodes.

## Phase B — disconnected local build

```bash
set +e
set +o pipefail
podman image exists localhost/leroy-sensors:m23m-synthetic-qos
podman build --network none --no-cache \
  -f Dockerfile.m23n-camera-qos \
  -t leroy-sensors:m23n-real-camera-qos . \
  2>&1 | tee m23n_no_device_build.log
echo "M23N_BUILD_EXIT_CODE=${PIPESTATUS[0]}"
```

Require `M23N_CAMERA_QOS_SOURCE_CONTRACT=PASS`,
`M23N_JAZZY_CAMERA_QOS_IMPORT=PASS`, exit0. Dockerfile
reuses already-built local M23M C++ binary; no `apt-get`,
remote fetch, source build or physical camera during build.

## Phase C — no-device smoke test

```bash
set +e
set +o pipefail
podman run --rm --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  localhost/leroy-sensors:m23n-real-camera-qos \
  2>&1 | tee m23n_no_device_selftest.log
echo "M23N_SELFTEST_EXIT_CODE=${PIPESTATUS[0]}"
```

This defaults to inherited M23K synthetic-only selftest.
Require `M23K_NO_DEVICE_SELFTEST=PASS` and exit0.

## Phase D — two separate OPERATOR-APPROVED camera-only runs

**Only after A–C pass**: verify the robot's motors remain
physically inhibited, production `leroy-ros` stopped,
camera is exclusive and current by-id resolves to the same
Wed USB camera as M23K. The host wrapper checks these and
prompts for `AUTHORIZE_M23N` on EACH physical camera run.
Do NOT map or open the STM32 serial device.

```bash
cd "$(git rev-parse --show-toplevel)/ros"
export M23K_HOST_CAMERA_BY_ID="/dev/v4l/by-id/usb-Wed_Camera_Wed_Camera_20240420112206-video-index0"
export M23N_OPERATOR_CAMERA_ONLY_APPROVED=YES
readlink -f "$M23K_HOST_CAMERA_BY_ID"

set +e
set +o pipefail
export M23N_SUB_QOS=best_effort_100
bash ./m23n_operator_camera_qos_probe.sh \
  2>&1 | tee m23n_camera_best_effort_100.log
echo "M23N_BEST_EFFORT_EXIT_CODE=${PIPESTATUS[0]}"

# Only after the previous container exited and camera is free:
export M23N_SUB_QOS=reliable_100
bash ./m23n_operator_camera_qos_probe.sh \
  2>&1 | tee m23n_camera_reliable_100.log
echo "M23N_RELIABLE_EXIT_CODE=${PIPESTATUS[0]}"
unset M23N_SUB_QOS
```

Collect full `M23N_CAMERA_QOS_JSON` plus each exit code.
Do not infer a reliable camera stream from a successful
synthetic DDS run. If reliable100 drastically improves raw
subscriber receipt continuity at fixed camera settings,
QoS should be investigated in a separate, reviewed production
change only after rechecking matching publisher offers and
recorder/image_proc path. If both remain low, inspect usb_cam
frame update/publish scheduling and CPU/DDS fragmentation.

Original first physical rosbag SHA-256 evidence remains
unchanged. `image_proc` shutdown SIGSEGV in full mode remains
unfixed. M23A's **250ms image header gap limit**, IMU coverage,
physical timing/extrinsic calibration and VIO/motion gates
remain BLOCKED; no merge or live robot motion from M23N.
