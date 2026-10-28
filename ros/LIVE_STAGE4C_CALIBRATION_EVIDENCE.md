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
