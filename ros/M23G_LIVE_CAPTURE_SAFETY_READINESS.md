# M23G — Preparation for the first genuine camera/IMU ROS 2 recording

**Status: OFFLINE CAPTURE PREPARATION ONLY.**
This milestone does **not** open a camera or STM32, start a recorder,
start M20, map any host device or authorize physical operation.

## Verified context

PR #100 (M23F) merged at b032112e7d94eaa90afb87224436ba757dcca501.
Its isolated synthetic tests and 75/75 offline regressions PASS.
M23A–M23F provide structurally validated ROS 2 Jazzy CDR tooling and
synthetic XRSLAM ingestion, but there is NO complete existing real bag.

The historical M20 run did receive actual concurrent camera/IMU data:
about 17.36 Hz rectified image, 29.81 Hz CameraInfo and 50 Hz IMU over
a 120-second stationary experiment. The 1.351-second maximum camera
receive gap needs investigation. The existing M20 sensor-only program
starts usb_cam + image_proc and a separate M18 IMU source; it does
NOT write full rosbag2 Image+CameraInfo+Imu CDR files.

The actual source topics are:
- /camera/image_rect: sensor_msgs/msg/Image (M20 rectified RGB)
- /camera/camera_info: sensor_msgs/msg/CameraInfo
- /imu/data_raw: sensor_msgs/msg/Imu (M18 MCU-derived mapped stamps)

M18 uses best-effort QoS for IMU. A recorder MUST use compatible
sensor-data QoS; m23g_sensor_qos.yaml proposes best_effort/volatile
for those three topics, not any wheel, TF, odom, command or bridge
topic. The CLI plan explicitly selects sqlite3 because the accepted
M23A–M23F readers currently accept only sqlite3 for read-only
recording audits.

## What is implemented NOW

m23g_live_recording_preflight.py:
- Reads only ordinary existing local output-directory metadata and
  statvfs free-space counters, no recording, USB/serial, ROS or video.
- Validates an explicit session id and refuses output-path collision,
  symlink output-parent and system/device filesystem roots.
- Estimates worst-case space at 640x480, 3 bytes/pixel, 30 frames/s,
  duration 30–180s, twice the raw payload plus 2 GiB for storage
  overhead and margin. At 120s: raw camera ~3.32 GB, required
  ~8.78 GB. This is a budget, NOT proof against device/disk failure.
- Reports exact recorder command *arguments* as
  planned_argv_NOT_EXECUTED, with physical access status FALSE.
  It does not execute, shell out, create files or launch ROS.
- Never changes camera–IMU synchronization or promotes any
  unmeasured exposure-to-MCU sampling offset to a calibration.

m23g_live_recording_preflight_test.py:
19 new pure-Python regressions for exact topics, bounded storage,
path denial, collision, no output writes, no serial/ROS/process
invocation, QoS and all physical permission flags remaining FALSE.

There is intentionally NO M23G "start live capture" executable.
That requires a separately reviewed physical safety gate.

## Phase A: Pure offline host tests — safe immediately

Run these on the Arch repository checkout, with no active robot
containers or physical device touches:

~~~bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m23g-live-recording-preflight
  git pull --ff-only origin feature/m23g-live-recording-preflight
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  python3 -u m23g_live_recording_preflight_test.py
  python3 -u m23f_existing_recording_inventory_test.py
  python3 -u m23e_offline_evidence_snapshot_test.py
  python3 -u m23d_recorded_bag_readiness_test.py
  python3 -u m23a_xrslam_rosbag_preflight_test.py
) 2>&1 | tee m23g_offline_contract.log
M23G_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M23G_OFFLINE_EXIT_CODE=$M23G_OFFLINE_EXIT"
~~~

Expected 19 M23G + 18 M23F + 19 M23E + 17 M23D + 21 M23A
= **94/94 tests PASS**. No physical access or ROS runtime.

## Phase B: Offline storage and command PLAN (not a recording)

Choose a pre-existing writable ordinary directory on a filesystem
with adequate space. The Python program does NOT create it.
For example, set a path after creating it by your ordinary file
management procedures (not as part of this program):

~~~bash
cd "$(git rev-parse --show-toplevel)/ros"
DATA_ROOT="/absolute/path/to/existing-recordings-directory"
test -d "$DATA_ROOT"
PYTHONDONTWRITEBYTECODE=1 python3 -u \
  m23g_live_recording_preflight.py \
  --output-root "$DATA_ROOT" \
  --session-id survey01 \
  --duration-seconds 120
~~~

Expected only if space is sufficient:
- M23G_OFFLINE_VERDICT=DISK_BUDGET_PASS_HARDWARE_NOT_AUTHORIZED
- M23G_PHYSICAL_CAPTURE_AUTHORIZED=FALSE
- M23G_HARDWARE_ACCESS=NONE

A disk-space failure is BLOCKED_INSUFFICIENT_CAPTURE_SPACE.
A disk PASS confers NO right to open the camera or MCU.

## Physical safety gate before any later implementation or operation

The STM32 USB CDC serial device can reset the controller and energize
WHEELS AND MANIPULATOR merely upon opening; the stream start command
is NOT the only hazard. M20's STOP-ACK flow is software only and
cannot guarantee safe state after MCU reset or abrupt process death.

Before a later capture implementation is even considered for live
operation, require all of the following confirmed by the operator:

1. BOTH wheel and manipulator **motor power physically inhibited or
   disconnected** independently of software, and verified unable to
   energize even on STM32 USB reset or arbitrary firmware output.
2. Robot completely immobilized; E-stop engaged and independently
   verified. No hands or fragile items in wheel/manipulator paths.
3. Original motor-capable leroy-ros container exited; Nav2, XRSLAM,
   bridge, depth and controllers prohibited. No /cmd_vel or odom/map TF.
4. Exact current front-camera identity and exact STM32 USB by-id
   identity re-verified without opening their data interfaces. The
   M20 host video device number varied in prior sessions; never
   blindly reuse /dev/video6 or /dev/video8, nor trust ttyACM0
   without its current by-id link. Ensure exclusive device ownership.
5. Dedicated bounded capture session, enough free disk, one isolated
   no-network/private PID/IPC container, no privileged flag, only
   exact verified devices, and recorder in same ROS 2 domain as the
   M20 camera+IMU publishers. Do not run a production robot launch.
6. Only source-output topics above, capture complete serialized Image,
   CameraInfo and Imu in sqlite3 CDR; startup readiness, recorder
   queue/QoS and STOP order must be tested offline first.
7. On any timeout, SIGKILL, container crash or missing verified MCU
   stream STOP ACK, keep motor power physically disconnected and
   E-stop engaged. No cleanup script alone can guarantee shutdown.
8. Renewed, explicit operator authorization for physical camera and
   MCU access **after** the above conditions are verified. This
   milestone does NOT infer permission from a generic request to
   record live data, env var, software dry-run or disk-space PASS.

## Separate future capture implementation (NOT in this PR)

The next change should integrate rosbag2 recorder into the existing
M20 *sensor-only* lifecycle within ONE isolated ROS graph, starting
the recorder before the two sensor publishers, and stopping it with
graceful SQLite metadata finalization after sources finish.
Test that ordering and kill/fault behavior offline WITHOUT devices
before an actual physically authorized run.

ROS 2 Jazzy rosbag CLI supports explicit topics and QoS overrides;
the proposed command arguments are in this PR's JSON PLAN, never
executed by its Python code. Use a no-device Jazzy self-test to
validate the exact CLI/version against local image before attempting
any real capture. Do NOT invoke the planned command from M23G yet.

Once a true physically isolated, authorized 120s bag exists,
first run the merged M23F/M23E/M23D/M23A tools READ-ONLY on
its closed files. A completed real bag is not an authenticated
recording automatically, and even good image/IMU headers do not
establish exposure–IMU phase, camera/IMU extrinsics, noise covariance,
XRSLAM tracking accuracy or motor safety.

**M23G verdict cannot be "LIVE_RECORDING_VALIDATED".**
