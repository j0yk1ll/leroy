# Stage 4C — camera/IMU calibration evidence and safe capture design

**Status (2026-10-28): OFFLINE ALGORITHM TESTS PASSED (13/13); PHYSICAL
CALIBRATION BLOCKED.** This is a no-hardware, design-only milestone.
Do not infer a camera/IMU phase from the Stage 4B ROS receipts or from
the MCU's host-relative rate estimate. No new live capture is included
or authorized by this branch.

## Accepted prior gates

Stage 4A accepted the 75-second supervised physical MCU IMU transport
session; Stage 4B subsequently diagnosed subscriber callback starvation
and passed a separately authorized corrected run: 3,750 physical IMU
source publications at 50 Hz, **3,750** identical-stamp passive receipt
callbacks, zero unmatched stamps, 0.690 ms p50 and 3.528 ms p95
*post-publication→callback* delay, 74.957 s concurrent camera/IMU ROS
receipt and confirmed START/STOP ACKs. These are transport/scheduling
metrics, **not exposure-to-IMU-acquisition latency**.

The operator's Stage 4C software checks used a **no-device, no-network,
read-only, private IPC/PID** local `localhost/leroy-ros:stage4b-imu`
Podman container, whose Python exposes OpenCV 4.6.0 and NumPy 1.26.4.
Both legacy synthetic tests passed:

- `utils/imu/test_orientation_timing.py`: **5/5**
- `utils/imu/test_corner_bearing_timing.py`: **8/8**
- **13/13 overall**. Positive/negative synthetic lag, frame/axis sign,
  strong vs insufficient excitation and disjoint baseline cases passed.

The native Arch Python 3.14 could not import `cv2`, so the host test
failure was an **environment dependency**, not failed calibration math.
Do not modify the Arch system Python to work around it.

Existing-data inventory was **bounded**, not an exhaustive forensic
filesystem audit. Neither the repository nor the inspected
`$HOME/Code`, `$HOME/Documents`, `$HOME/Downloads` and `/tmp`
directories contained a matching `metadata.yaml`, `.db3`,
`.mcap` or `*summary.json` file. The old `leroy-ros` runtime
container no longer exists. The remaining `n8n_*` containers belong
to an unrelated application; do not operate on them.

## Existing source evidence and its limits

The historical `utils/imu/README.md` describes ChArUco recordings,
including `/tmp/leroy_charuco_auto_yaw_02` and
`/tmp/leroy_charuco_auto_yaw_75s_03`. Documentation is **not**
proof those bags still exist.

The recorded-run methods were estimator-dependent: early adjacent-frame
yaw estimates approximately -42 / -58 / -66 ms, while orientation
vs integrated gyro estimates approximately -52 / -90 / -114 ms.
All are **uncertified**. Camera/extrinsics configuration in
`ros/src/leroy_description/config/sensor_mounts.yaml` remains
**provisional**: camera optical-center zero position is a placeholder,
model camera pitch is not measured, and MCU gyro axes already map to
robot frame. Do not add a second IMU frame flip.

The old `utils/imu/calibrate_time_offset.py` is an integrated
bridge-era ROS recorder and has an optional `--arm` control path
which creates a `/cmd_vel` publisher. In its default non-armed
path, `self.pub=None` and its `send_stop()` branch doesn't
publish. Nevertheless **do not use either mode for current Stage 4C
physical recording**: its ROS graph, bridge assumptions, compressed
camera topic and output isolation differ from the accepted Stage
3/4B architecture. Do not start the production `leroy-ros` image,
read/use motor parameters, or invoke any `--arm` command.

## Physical offset is not observable in stationary no-motion data

A stationary camera+IMU relative to a stationary ChArUco target has
no common time-varying physical event. Stationary IMU sequences are
appropriate for bias, variance and stability *diagnostics*, not for
estimating camera/IMU phase. Moving the ChArUco **board alone** while
the camera+IMU remain stationary also provides no correlated inertial
event and cannot estimate temporal offset.

A future dedicated, separately reviewed experiment must introduce
measured visual and inertial excitation of the **rigidly connected
camera+IMU assembly** (without robot wheel actuation), or an independently
traceable external physical event observable in both sensors. Rigid
sensor movement, tether stress, E-stop/mechanical restraint and the
MCU's motor-capable CDC interface require their **own physical safety
review**. Do not move the rover or its sensor assembly based on this
design-only document.

## New Stage 4C recorder candidate — offline-only, not commissioned

This draft branch now includes the following offline-only candidates
(no sensor sessions or calibration are authorized):

- `live_stage4c_capture_contract.py`: **pure-Python** validated receipt
  policy for exactly `/camera/image_rect` (`sensor_msgs/Image`),
  `/camera/camera_info` (`CameraInfo`),
  `/imu/data_raw` (`Imu`) and
  `/stage4c/imu_source_meta` (`std_msgs/String`).
  Checks expected frames, ordered per-stream ROS stamps, bounded
  serialized payload and total record count, publication/receipt
  monotonic times, MCU sequence continuity (including uint32 wrap),
  `t_us` and exact image/IMU-to-source stamp matching.
- `live_stage4c_passive_recorder.py`: candidate subscriber-only
  ROS 2 `rosbag2_py` CDR/SQLite writer with per-topic publisher
  graph checks, denied motor/nav/odom/dynamic TF graph, prohibited
  visible devices, finite 75-second session, a single new
  `/capture/stage4c-*` output directory, no overwrite, file hashes
  and an explicitly **uncalibrated** manifest.
- `Dockerfile.live-stage4c-capture`: separate image FROM the accepted
  local Stage 4B image, no source or bridge modifications,
  `CMD ["/bin/false"]`. It installs the rosbag2 writer dependencies.
- `live_stage4c_capture_test.py` and
  `live_stage4c_source_meta_test.py`, executed by the Stage 4C
  GitHub `no-device-contract` workflow: **24/24 tests** on the
  candidate branch. They exercise synthetic capture acceptance,
  missing/invalid source metadata, MCU sequence and tick wrap,
  source callback failure latching, timestamp regressions,
  quota, malformed JSON, unsafe topics, disk-write exceptions,
  the Python 3.14 `return`-in-`finally` regression,
  and the no-device/no-actuation hard gate.

**Hard live gate:** `LIVE_CAPTURE_COMMISSIONED = False` and
`STAGE4C=BLOCKED_LIVE_CAPTURE_NOT_COMMISSIONED` by default. The
module's only approved executable path for now is
`--contract-smoke`, which imports no ROS runtime, creates no
output files, and connects to no hardware.

**Critical remaining integration dependency:** Stage 4B's M18
post-publish callback sees original MCU `seq` and `t_us`,
but its accepted runtime does **not** publish
`/stage4c/imu_source_meta`. Stage 4C requires that real source
feed before even describing a complete recorded input set.
`live_stage4c_source_meta.py` is now a **pure-Python, source-only,
fail-closed, unconnected** adapter for the accepted M18
`on_imu_published(fields, stamp_ns, steady_ns)` hook. It validates
original `seq`/`t_us`, uint32 wrap and source stamp order,
takes a **new post-publication monotonic timestamp** (not the
earlier clock-mapper observation), and passes canonical JSON to an
injected sender. Synthetic tests prove the sequence, tick, timestamp
and sender-failure latches. The module imports no ROS runtime,
creates no publisher and has **not** been wired into M18 or the
STM32 sidecar. A separate reviewed source-side ROS
`std_msgs/String` publisher still needs to use it with
the exact `leroy_stage4c_mcu_v1` schema and the accepted STOP
contract. Faking ticks from ROS headers or treating callback
receipts as MCU acquisition ticks is forbidden.

The contract's `mcu_metadata_stamp_match_complete` refers to
syntactic, per-sample agreement only; all reports retain
`physical_source_authenticated=false` and
`physical_camera_imu_offset_validated=false`.

### Simulated source-side ROS metadata publisher — still disconnected

The draft now also contains `live_stage4c_source_meta_ros.py`:
a **non-executable, source-only** wrapper that can turn a validated
M18 post-publication metadata JSON into `std_msgs/msg/String`,
using the already-existing `/m18_imu_source` Node rather than
instantiating another source. It demands the expected source
name/namespace and rejects topic remapping. These checks only
verify the supplied node's identifiers; they are not physical
provenance authentication.

The wrapper is **NOT** copied into the candidate passive-recorder
image, NOT imported by M18 or the accepted Stage 4B sidecar, and
has no executable launch path. `live_stage4c_source_meta_ros_test.py`
uses a **fake M18 node/publisher** to check exact metadata payloads,
publisher failures, missing MCU ticks, source identity guards and
unchanged Stage 4B wiring. No ROS graph or device is started.

### Disconnected DDS/ROS bag integration candidate — synthetic only

`live_stage4c_synthetic_dds_bag.py` uses a real `rclpy` graph
inside a **disposable, network-disconnected, device-free** container.
It creates a **FAKE M18-named ROS node** (not the actual M18
`ImuStreamSource`), a fake camera publisher, and a synthetic
capture-contract subscriber. The fake M18 node publishes 12 synthetic
IMU samples; the actual `SourceMetadataRosAdapter` publishes
matching synthetic `seq/t_us` metadata after each IMU publish.
The fake camera publishes matching 2x2 synthetic Image and CameraInfo.
A bounded callback loop requires all 48 ROS receipts before a
temporary SQLite rosbag2 write/read comparison. The script refuses
to run unless `ROS_DOMAIN_ID=171`, local-only discovery, the
accepted Fast DDS RMW and `LEROY_STAGE4C_SYNTHETIC_ONLY=1` are set,
and the known camera/MCU/GPIO device paths are absent.

**Limitations:** This is actual ROS 2 DDS integration but still
**synthetic provenance**. It does NOT construct the real M18 source,
does NOT connect to or exercise the hard-disabled Stage 4C passive
recorder entrypoint, does NOT stress delivery at full real sensor
rates, and does NOT exercise the MCU START/STOP ACK path. Its
12 synthetic samples intentionally yield
`BLOCKED_INCOMPLETE_CAPTURE`; the test only checks internal
per-message matching, replay and fail-closed reporting. A pass is
**not** authority for a real capture.

From the existing Stage 4C `ros` worktree, first fetch the new
source-only test:

```bash
git status --short
git fetch origin feature/stage4c-calibration-evidence-plan
git merge --ff-only FETCH_HEAD
PYTHONDONTWRITEBYTECODE=1 python3 -Werror::SyntaxWarning -m unittest -v \
  live_stage4c_capture_test live_stage4c_source_meta_test \
  live_stage4c_source_meta_ros_test live_stage4c_synthetic_dds_test
```

The above commands need only host Python; **no ROS**, devices or
containers. The disposable ROS test below uses the already-built
`localhost/leroy-ros:stage4c-capture-offline` image; **no rebuild
required**. Bind the new source tree read-only:

```bash
podman run --rm --pull=never \
  --network=none --ipc=private --pid=private \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=256m \
  -v "$PWD:/offline:ro" \
  -e PYTHONPATH=/offline:/opt/stage4c \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -e ROS_DOMAIN_ID=171 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LEROY_STAGE4C_SYNTHETIC_ONLY=1 \
  -e ROS_LOG_DIR=/tmp/stage4c-ros \
  -e XDG_CACHE_HOME=/tmp/stage4c-cache \
  --entrypoint bash \
  localhost/leroy-ros:stage4c-capture-offline -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    exec python3 -u /offline/live_stage4c_synthetic_dds_bag.py
  '
```

Expected, if the operator's real disconnected ROS graph and bag
storage pass:

```text
STAGE4C_SYNTHETIC_DDS_BAG_INTEGRATION=PASS_NO_DEVICE
STAGE4C_SYNTHETIC_IMU_META_MATCHED=12
STAGE4C_LIVE_CAPTURE_AUTHORIZED=FALSE
STAGE4C_PHYSICAL_CALIBRATION_VALIDATED=FALSE
```

Only after an independent operator run should this stage be
described as passed. The next gate after that remains **separate
live-sidecar integration review, callback load testing and physical
safety authorization**. Do not alter the accepted M18 runner,
MCU STOP-ACK contract or live-capture hard block.



The policy limits payload to 900 MB of *serialized messages*,
20,000 total records, at most 4 MB per message and 75 seconds.
Serialized payload is **not** an exact on-disk quota; SQLite
overhead, journal files and metadata may use additional space.
Real-world resource and sensor callback-load validation is pending,
and the eventual launcher must bound the actual writable mount.

### Reproduce source-only no-device tests

This requires **no camera, ROS graph, STM32 or motors**.
In a new detached worktree so the accepted Stage 4B one is untouched:

```bash
cd /home/kevin/Code/leroy
git fetch origin feature/stage4c-calibration-evidence-plan
git worktree add --detach ../leroy-stage4c \
  origin/feature/stage4c-calibration-evidence-plan
cd ../leroy-stage4c/ros
PYTHONDONTWRITEBYTECODE=1 python3 -Werror::SyntaxWarning -m unittest -v \
  live_stage4c_capture_test live_stage4c_source_meta_test \
  live_stage4c_source_meta_ros_test
PYTHONDONTWRITEBYTECODE=1 python3 -Werror::SyntaxWarning \
  live_stage4c_passive_recorder.py --contract-smoke
```

Both commands require no third-party Python dependencies. The
`--contract-smoke` output is
`STAGE4C_OFFLINE_CAPTURE_CONTRACT=PASS_NO_DEVICE` and
`STAGE4C_LIVE_CAPTURE_AUTHORIZED=FALSE`.

To validate within the previously accepted no-device image instead
(without rebuilding it), run from the **new worktree's** `ros`
directory:

```bash
podman run --rm --pull=never \
  --network=none --ipc=private --pid=private \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$PWD:/offline:ro" \
  --entrypoint python3 localhost/leroy-ros:stage4b-imu \
  /offline/live_stage4c_passive_recorder.py --contract-smoke
```

The recorder initially generated a Python 3.14 warning because its
cleanup `finally` block contained a `return` and could suppress
an exception. The branch now places the return **after** cleanup;
the AST regression test rejects any recurrence.

### Offline synthetic ROS 2 bag round-trip — no physical devices

The proposed image includes
`live_stage4c_rosbag_smoke.py`. It writes and reads
**synthetic** Image, CameraInfo, Imu and original-tick-shaped
metadata with Jazzy `rosbag2_py` in an automatically deleted
SQLite bag inside an ephemeral `/tmp`. This tests preserved
serialized CDR bytes, per-topic type names, header stamps,
pixel content, MCU sequence and acquisition-tick fields. It
**never** creates a ROS node, opens a camera/MCU or infers a
physical offset.

From the Stage 4C `ros` worktree, and only after source-only
tests pass, you may build an offline candidate image (apt dependency
resolution may require network during **build only**, not for the
subsequent test):

```bash
podman build -f Dockerfile.live-stage4c-capture \
  -t localhost/leroy-ros:stage4c-capture-offline .
```

The first candidate image build failed at STEP 8 because its
`rosbag2_py` import check ran in a bare Python process without sourcing
ROS Jazzy. The apt install step had completed; ROS-managed Python
paths were not on `sys.path`. The Dockerfile now sources
`/opt/ros/jazzy/setup.bash` and
`/leroy_ws/install/setup.bash` before its ROS import check,
and a source-only regression asserts the setup order.
**The corrected image still needs an operator rebuild** and has not
yet passed a native Jazzy bag read/write test on the host.

Then run this deliberately disconnected, no-device smoke:

```bash
podman run --rm --pull=never \
  --network=none --ipc=private --pid=private \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=128m \
  -e PYTHONDONTWRITEBYTECODE=1 \
  --entrypoint bash localhost/leroy-ros:stage4c-capture-offline -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    python3 /opt/stage4c/live_stage4c_rosbag_smoke.py
  '
```

Require
`STAGE4C_SYNTHETIC_ROSBAG_ROUNDTRIP=PASS_NO_DEVICE` and
`STAGE4C_PHYSICAL_CALIBRATION_VALIDATED=FALSE`.

The ROS bag writer adapter and this new image remain **source-only
candidates** until the actual disconnected smoke is independently
run on the operator's local Podman installation. Do not launch
the candidate in the real sensor graph or remove the hard live gate.

## Future capture contract — requires separate review/authorization

Any eventual implementation must first pass no-device, simulated-ROS
and fault-injection tests, plus an explicit operator hardware review:

1. **Physical safety boundary**: motor-driver rails and manipulator
   power independently physically disconnected; immobile or
   independently secured chassis; verified E-stop and person at cutoff;
   no `/cmd_vel`, `/odom`, dynamic `/tf`, Nav2 controllers or
   `sensor.mode=monocular_imu`. Opening `/dev/ttyACM0` can reset
   motor-capable STM32 firmware. Keep START/STOP allowlist, bounded
   serial session, required STOP ACK and fail-closed MCU fault latch.
2. **Isolation**: Stage 3 real camera and monocular status parents
   retain only their accepted video permissions; only disposable M18
   source sidecar can map the verified STM32 serial by-id endpoint.
   A **different passive recorder**, without `--device`, uses
   the isolated ROS graph, bounded time/size quota, read-only root
   filesystem and explicitly limited new host output directory.
   No other container may access the serial port. Secure any
   recording files against accidental overwrites.
3. **Replayable physical observations**: record actual rectified
   camera image *content* and full Image header, CameraInfo with
   image geometry/K/D and header semantics, IMU gyro and acceleration
   content with mapped ROS header, as well as actual **source-side**
   MCU `seq` and `t_us` via the existing M18 post-publication
   callback. `sensor_msgs/Imu` does not carry raw MCU acquisition ticks:
   they cannot be reverse-engineered from subscriber messages.
   Record host monotonic receipt times only as transport context,
   not as physical exposure times.
4. **Camera time provenance**: record actual selected patched
   `usb_cam` executable/library, image header epoch conversion,
   V4L2 buffer timestamp semantics, exposure timing convention,
   frame period and uncertainty. Correcting the upstream
   microsecond arithmetic bug alone did not identify exposure start,
   midpoint or readout timing. The camera/IMU absolute phase is
   **UNKNOWN** until independently measured.
5. **Rigid geometry provenance**: preserve existing TF axis
   conventions, camera optical frame and provisional transforms.
   Independently check sensor centers and uncertainty. Do not
   alter configuration from an algorithm's apparent timing lag.
6. **Calibration-quality checks**: multiple distinct motion/event
   trials with diverse nonperiodic excitation, reproducible event
   correspondence, equal-subset candidate-lag scans, wide-enough
   search interval, clearly separated peak, split-window stability,
   independent raw-corner vs orientation-method agreement and
   uncertainty from both sampling and acquisition timestamps.
   If weak, aliased, broad, contradictory or time-varying,
   **withhold an offset**.
7. **Independent replay and metadata audit**: preserve immutable
   original files and per-session manifest/hash, hardware/software
   identity, MCU sequence fault counts, source/subscriber matching,
   exposure and ROS time domains, static TF snapshot, motor cutoff
   attestation and selected board config. Run established M23F/M23D/
   M23E offline evidence snapshot checks before claiming an
   actual data set. A rosbag content PASS does not authenticate
   physical provenance or validate calibration.

**Stage 4C output acceptance (not yet achieved):** a physical
camera/IMU phase estimate with explicit **sign convention**, real
capture identity, repeatable correlated events, residuals/uncertainty
and corroboration across independent trials, without changing
the provisional sensor TF. Only then may an independently reviewed
XRSLAM-specific phase conversion be considered. It is not yet
permission to start VIO, publish metric pose or energize motors.

## Reproduce the accepted offline software checks — no devices

From the `/home/kevin/Code/leroy` repository root, and without
restarting stopped Stage 3 parents:

```bash
podman run --rm --pull=never \
  --network=none --ipc=private --pid=private \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=64m \
  -v "$PWD/utils/imu:/offline:ro" \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -e OPENBLAS_NUM_THREADS=1 \
  --entrypoint bash localhost/leroy-ros:stage4b-imu -lc '
    set -e
    python3 -m unittest discover -s /offline -p test_orientation_timing.py -v
    python3 -m unittest discover -s /offline -p test_corner_bearing_timing.py -v
  '
```

Expect **5 tests OK** and **8 tests OK**. These synthetic successes
are software validations, not physical offset measurements.

**Current verdict: STAGE4C_OFFLINE_ALGORITHMS=PASS_13_OF_13;
STAGE4C_CALIBRATION=BLOCKED_NO_PHYSICAL_CORRELATED_DATA.**
