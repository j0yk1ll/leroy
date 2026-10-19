# M23P — no-device image_proc shutdown isolation

**Draft / no M23P runtime results yet.** This milestone is stacked
on M23O draft PR #109. It deliberately uses the existing ROS 2 Jazzy
`image_proc::RectifyNode` plugin without modifying ROS libraries,
production launch or live devices. No robot motion is authorized.

## Why

Operator camera-only M23O runs have twice observed approximately
30Hz raw/rectified subscriber delivery with **zero image header gaps
above 250ms**, but `component_container` consistently died with
**exit -11/SIGSEGV**, after the ROS launch shutdown SIGINT.

Most recent run: raw=1412 at 29.814Hz; rect=1412 at 29.822Hz;
all 1412 header stamps matched; real RMW `rmw_fastrtps_cpp`,
all observed camera/rectifier/receiver endpoints RELIABLE/VOLATILE.
The launch context showed:
- `[launch] user interrupted with ctrl-c (SIGINT)`
- `sending signal 'SIGINT' to component_container`
- `usb_cam process has finished cleanly`
- `component_container process has died ... exit code -11`

The faulting call stack is unknown. These lines identify a shutdown
association, not a proven destructor, callback race or DDS bug.
Original bag and M23A strict 250ms gap gate are unchanged.

## Design — two mutually exclusive isolated profiles

- `idle`: launch **only** the existing `image_proc::RectifyNode`
  component in `component_container`, wait 8 seconds, send SIGINT.
  No image publishers, no camera, no IMU.
- `synthetic`: launch that same component, the existing M23M C++
  `reliable_100` raw+rect header observer, and the unmodified M23L
  producer of **900 fabricated 640x480 RGB8 images and CameraInfo**
  messages at nominal 30Hz. Stop the rectifier after the synthetic
  publisher terminates and a 2-second drain. A missing synthetic
  rect stream yields BLOCKED/INCOMPLETE, not a false PASS.

Both variants: separate ephemeral rootless Podman container,
`--network none`, private IPC/PID, no privilege elevation, isolated
ROS_DOMAIN_ID=170, read-only root with tmpfs, **no mapped hardware**.
Script refuses visible video and STM32 device nodes and unexpected
profile/environment. No ROS bag, Nav2, XRSLAM, IMU or motor bridge.

Success is **ONLY** `NO_DEVICE_IDLE_RECTIFIER_SHUTDOWN_ONLY` or
`NO_DEVICE_SYNTHETIC_RECTIFIER_SHUTDOWN_ONLY`. It means the corresponding
synthetic setup shut down cleanly once. The runner independently
examines `component_container` child failure/clean-exit log markers,
rather than trusting the `ros2 launch` parent return code. Any
process death, incomplete producer/observer logs, missing stream,
timeout or missing explicit clean child exit yields exit code 2.

This does not test physical camera intrinsics, authentic camera
publication rates, or metric tracking. CameraInfo from the M23L
synthetic publisher is fabricated and not verified real calibration.

## Phase A — offline source contracts

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23p-no-device-rectifier-shutdown
git pull --ff-only origin feature/m23p-no-device-rectifier-shutdown
cd ros

PYTHONDONTWRITEBYTECODE=1 python3 -u m23p_no_device_rectifier_shutdown_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23o_rect_camera_qos_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23n_camera_qos_probe_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23m_dds_qos_matrix_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23l_synthetic_dds_baseline_test.py
```

Require M23P 10/10 + M23O 10/10 + M23N 10/10 +
M23M 9/9 + M23L 16/16 = **55/55 PASS**.
New marker: `M23P_NO_DEVICE_SHUTDOWN_SOURCE_CONTRACT=PASS`.

## Phase B — disconnected build

```bash
set +e
set +o pipefail
podman image exists localhost/leroy-sensors:m23o-rect-camera-qos || exit 2

podman build --network none --no-cache \
  -f Dockerfile.m23p-rectifier-shutdown \
  -t leroy-sensors:m23p-rectifier-shutdown . \
  2>&1 | tee m23p_no_device_build.log
echo "M23P_BUILD_EXIT_CODE=${PIPESTATUS[0]}"
```

Require source contract marker, `M23P_JAZZY_NO_DEVICE_IMPORT=PASS`,
and `M23P_BUILD_EXIT_CODE=0`. No live test until these pass.

## Phase C — isolated idle component shutdown, no device

```bash
set +e
set +o pipefail
podman run --rm \
  --network none --ipc private --pid private \
  --shm-size 512m \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  -e M23P_PROFILE=idle \
  localhost/leroy-sensors:m23p-rectifier-shutdown \
  2>&1 | tee m23p_idle_shutdown.log
echo "M23P_IDLE_EXIT_CODE=${PIPESTATUS[0]}"
```

Collect `M23P_LIFECYCLE_JSON`, `M23P_LIFECYCLE_VERDICT`,
`M23P_DEVICE_ACCESS=NONE` and exit code. A crash remains BLOCKED.

## Phase D — isolated synthetic full-image shutdown, no device

Only after Phase C has been characterized; a crash in C does not
justify opening any hardware. This second test still uses no devices.

```bash
set +e
set +o pipefail
podman run --rm \
  --network none --ipc private --pid private \
  --shm-size 512m \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  -e M23P_PROFILE=synthetic \
  localhost/leroy-sensors:m23p-rectifier-shutdown \
  2>&1 | tee m23p_synthetic_shutdown.log
echo "M23P_SYNTHETIC_EXIT_CODE=${PIPESTATUS[0]}"
```

Collect the same JSON, child exit classification, synthetic
publisher/observer count markers, and bounded crash context.

Interpretation: if idle crashes, a reproducer exists without sensor
traffic; if only synthetic crashes, the failure likely depends on
active subscriptions or teardown while processing traffic. If neither
crashes, the physical-configuration crash is not reproduced by these
simplified conditions; no conclusion about precise root cause follows.

No physical-device experiment or separate production fix is
authorized by this PR. A **faulting backtrace** will be necessary
before attributing the crash to a specific package or destructor.
Do not lower M23A's 250ms image-header gap gate or touch the
STM32/motors.
