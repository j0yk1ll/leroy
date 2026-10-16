# M16 continuation — actual camera + DepthART in isolated, sensor-only Podman

**Purpose:** obtain real USB camera and DepthART ROS messages without starting
the historical `leroy-ros` full robot launch, `leroy_bridge`, serial,
XRSLAM, odometry, Nav2 or any motor command publisher. This follows the
2026-10-15 systemd user-session teardown that stopped `leroy-ros`. Do not
restart the old motor-capable container as part of these procedures.

The new `m16_sensor_only.launch.py` starts exactly THREE application
processes:
1. `/camera/usb_cam` — existing patched V4L2 USB camera driver;
2. `/m16_image_proc_container` — one `image_proc::RectifyNode` component,
   generating `/camera/image_rect` with the existing `CameraInfo`;
3. `/depthart` — existing CPU DepthART runtime, producing
   `/depthart/depth` as `32FC1` with source acquisition timestamp.

No `leroy_bridge`, serial `/dev/tty*`, motor `/cmd_vel`, real `/odom`,
XRSLAM, fake odometry, controller, costmap, or obstacle publisher starts.
M16 observer runs separately **inside the SAME disposable container**
after sources start; no ROS DDS host-network connection is needed. The
only mapped host device is `/dev/video2`. Domain 168 is confined inside
`--network none` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
No host ROS services or production container are changed.

### Important image boundary

`Dockerfile.m16-sensors` deliberately **inherits the already-existing,
operator-tested** `localhost/leroy-ros:jazzy-lowpwm` image to reuse patched
`usb_cam`, CPU DepthART and its pinned checkpoint. That image still
**contains** unused bridge software in its filesystem. The M16 custom
`CMD` replaces the old full-robot startup, launches no bridge process,
is off the host network, and maps no serial hardware. The observer and
new launch are copied from the M16 branch; the original image/tag and
`leroy-ros` container are not changed.

The checked-in `/config/camera/front_camera.yaml` provides the prior
calibration *asset* only. Its accuracy on the currently mounted camera,
optical-frame extrinsics, and physical depth correctness are NOT certified
by this run. If `/dev/video2` is not the actual video capture device,
STOP and report the preflight; do not map random video or serial devices.

The rectifier uses the same component architecture as the existing robot
launch. The previous full-robot `component_container` terminated with
SIGSEGV **during KDE session shutdown**; a clean startup/shutdown in M16
must be demonstrated, not assumed. The captured log will preserve
shutdown errors for follow-up if they recur.

## Step 1 — Checkout and test/build the offline monitor

Run from an existing Leroy Git checkout (this block finds repository root):

```bash
set -e
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/nav2-m16-sensor-only-capture
git pull --ff-only origin feature/nav2-m16-sensor-only-capture
cd ros
podman build --no-cache -f Dockerfile.nav2 -t leroy-nav2:m16-passive .
```

This disconnected optional image runs Colcon/pytest, including static
tests of the three-process sensor-only launch, capture runner,
packaging, prohibition of command nodes and offline isolation.

## Step 2 — Check the existing camera-only prerequisites

Still in `ros/`:

```bash
set -e
podman image exists localhost/leroy-ros:jazzy-lowpwm
test -c /dev/video2
ls -l /dev/video2
podman inspect leroy-ros --format \
  'M16 existing robot container: status={{.State.Status}} image={{.Config.Image}}'
```

**Expected:** base image exists, `/dev/video2` is a character device,
and the preexisting `leroy-ros` is still stopped. Do NOT use
`podman start leroy-ros`; the normal robot launch starts a motor-capable
bridge and is not required for this sensor experiment.

If `test -c /dev/video2` fails, stop. Verify which physical camera
corresponds to the checked-in calibration file *outside this test*;
do not guess a different V4L2 node. The current follow-up is fixed to
the known, previously configured `/dev/video2` interface.

## Step 3 — Build separate M16 camera-only image

```bash
set -e
podman build --no-cache \
  -f Dockerfile.m16-sensors \
  -t leroy-ros:m16-sensor-only .
```

This is a lightweight derived image; it does not redownload the model,
recompile the camera or rebuild the motor bridge.

## Step 4 — Complete 45-second real-camera observation (one terminal)

```bash
set -o pipefail
podman run --rm --name leroy-m16-camera-only \
  --network none --ipc private --pid private --shm-size 512m \
  --cap-drop all --security-opt no-new-privileges \
  --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --device /dev/video2:/dev/video2 \
  -e ROS_DOMAIN_ID=168 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e LEROY_M16_SENSOR_ONLY=1 \
  -e LEROY_NAV2_DRY_RUN=1 \
  -e LEROY_M16_DURATION_S=45 \
  -e M16_VIDEO_DEVICE=/dev/video2 \
  leroy-ros:m16-sensor-only \
  2>&1 | tee m16_real_camera_depth.log
```

This one terminal automatically performs all operations:

1. Starts exactly three camera/rectifier/DepthART source processes;
2. Checks for forbidden `/cmd_vel`, `/odom`, bridge and controller graph nodes;
3. Runs the original **M16 read-only observer** for a bounded 45 seconds;
4. Emits the full `M16_PASSIVE_REPORT_JSON=`, `M16_VERDICT=` and
   **new** `M16_CAMERA_DEPTH_VERDICT=`;
5. Signals the launch for shutdown; a 12-second fallback terminates it
   if graceful cleanup hangs; Podman then removes only the dedicated
   `leroy-m16-camera-only` container.

No second terminal is necessary for normal execution. If it hangs,
press Ctrl+C in that terminal. In a second host terminal, stop ONLY
the disposable M16 sensor-only container if needed:

```bash
podman stop --time 15 leroy-m16-camera-only
```

Do **not** run the stop command after the container has auto-exited;
`podman stop` would then report that it no longer exists.

## Step 5 — Extract the report and verify cleanup

Still in `ros/` after the observation ends:

```bash
echo '=== M16 REPORT/VERDICTS ==='
grep -E '^M16_(PASSIVE_REPORT_JSON|VERDICT|CAMERA_DEPTH_VERDICT)=' \
  m16_real_camera_depth.log || true
echo '=== SENSOR RUNTIME AND SHUTDOWN ==='
grep -E 'M16 SENSOR-ONLY:|process has (finished cleanly|died)|FAIL:|ERROR|Traceback' \
  m16_real_camera_depth.log | tail -n 80 || true
echo '=== ORIGINAL ROBOT CONTAINER (UNCHANGED) ==='
podman inspect leroy-ros --format \
  'status={{.State.Status}} image={{.Config.Image}}'
echo '=== M16 DISPOSABLE CONTAINER ==='
podman ps -a --filter name=leroy-m16-camera-only \
  --format '{{.Names}} {{.Status}}'
```

Share both `M16_PASSIVE_REPORT_JSON=` and the M16 runtime/shutdown
log. The log contains ROS node names, metrics and timing, but no raw
frames or camera images.

### How to interpret M16 outcomes

- `M16_CAMERA_DEPTH_VERDICT=OBSERVED_CAMERA_DEPTH_MESSAGE_CONTRACT`:
  at least two valid/fresh **real** camera images, CameraInfo messages
  and DepthART images were observed, at least one DepthART stamp/frame
  matched cached CameraInfo, and exactly one expected `/depthart`
  publisher was seen. **Not proof of depth accuracy or calibration.**
- `M16_CAMERA_DEPTH_VERDICT=INCOMPLETE_OR_UNVERIFIED_CAMERA_DEPTH`:
  inspect stream counts, invalid reasons, encoding, acquisition ages
  and producer graph from the JSON. If DepthART cannot initialize or
  the camera device is unavailable, an incomplete result is expected.
- `M16_VERDICT=INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS` is **expected**
  even if camera/depth passes, because this **deliberately isolated**
  source does not start IMU, XRSLAM, odom/TF or calibrated obstacle
  conversion. Those are separate M16 validation steps.
- A missing report line, nonzero process exit, or unclean three-process
  shutdown is a test-infrastructure failure and needs log review.

**This is partial real-source software validation only.** It is a
step toward validating Leroy's actual camera/DepthART performance
and timestamps, not completion of full M16 sensor integration,
motor-command ownership, or physical safe-stop validation.
