# M23BV — additive, hardware-free capture correction draft

**Status:** offline Python/shell contracts only. NOT a physical-run authorization,
ROS acceptance verdict, or proof of camera–IMU synchronization. The existing
M23BU recording and all accepted M23BT images/logs MUST remain unchanged.

## Source basis (read-only)

Inspected the GitHub default-branch versions of:
- `ros/m20_sensor_only_run.sh` (`88c300b174f6e53bd536467f2e445d22e5ed1f8c`)
- `ros/m20_camera_only.launch.py` (`a72000f97b72ec3b773879fb1066405b505f764a`)
- `ros/m20_combined_observer.py` (`5cf2f88cb2f79be80821888401b57a6393ee5e3b`)
- `ros/m23h_bounded_capture.py` (`eb40aa6e636e7ba602554136a7f8272e9b407025`)
- `ros/m23h_operator_host_capture.sh` (`bdcb4d1e0274a0add223fb15756e383e47358d07`)
- `ros/m23g_sensor_qos.yaml` (`13e482087a1fc3dc729addb5046a18a6cdcfeb3d`)
- `ros/m23g_live_recording_preflight.py` (`23f3ea629d0c18b07ce77db364b53094c920e3b9`)

Also inspected M23I PR #103 and M23N/M23O PRs #108/#109. **The GitHub
main branch version of M23A still has the old unbounded CameraInfo bug**;
use the M23I-corrected variant for eventual acceptance, without silently
replacing the user's locally dirty checkout.

The local M23BT-06 script and existing image contents were NOT accessible.
The pinned baseline acceptance log from the engineering handoff is not
replaced or reinterpreted here.

## Design

| New additive file | Purpose |
| --- | --- |
| `ros/m23bv_sensor_qos.yaml` | `RELIABLE/VOLATILE`, depth 100, rectified-image recorder subscription only. CameraInfo and IMU remain compatible best effort. |
| `ros/m23bv_camera_only.launch.py` | M20 camera parameters; separate `image_proc rectify_node` executable with per-process alternative pthreads OpenCV. No global library injection. |
| `ros/m23bv_qos_guard.py` | Read-only graph inspection, fail closed unless actual reliable raw publisher, rectifier input/output, and rosbag recorder input endpoints agree. Runs **before opening MCU serial**. |
| `ros/m23bv_sensor_only_run.sh` | Additive M20 shell lifecycle variant. Reuses original `m20_combined_observer.py` and its fixed IMU START/STOP/ACK protocol, preserving IMU-then-camera shutdown ordering. |
| `ros/m23bv_bounded_capture.py` | Candidate 120-second capture supervisor based on M23H. Separate operator approval variable, exclusive recording/log target, SIGTERM sensor-parent, SIGINT recorder group, strict close/count checks. **Not invocable through a provided host physical launcher**. |
| `ros/m23bv_window.py` | Pure evidence-based header timestamp overlap selection. Rejects missing pairs, backward stamps, internal gaps >250 ms, IMU gaps, bad frames or insufficient counts. |
| `ros/m23bv_derive_closed_bag.py` | **Offline-only** two-pass derivation from a new, finalized source bag: hashes source files before/after, checks whole input, writes selected original CDR payloads and original bag receipt timestamps to fresh sqlite3 ROS bag, closes writer, hashes metadata/DB; never rewrites source. |
| `ros/m23bv_offline_tests.py` | 26 offline source/behavior tests, including fake-bag write/finalization. |
| `ros/Dockerfile.m23bv-capture-prep` | Proposed local derived image. Starts from **M23H sensor image**, copies **only pthreads OpenCV libraries** from the M23BT-06 image and tests dependencies. Default entrypoint runs offline tests. |

**Recording window:** the still-uncorrected broad M23H-like capture retains its
original timestamps/data as evidence. The derived bag only keeps messages
whose ORIGINAL header stamps are in `[first_imu_stamp, last_imu_stamp]`.
The continuous retained image window must be entirely covered by the IMU.
Image/CameraInfo pairs are verified on exact header stamp + frame id + size.
**No interpolation, padding, synthesized IMU, or retimestamping.**
The derived bag remains `PENDING_M23A_M23I_M23F_M23J` even after success.

## Hardware-free local commands (no Podman required)

From the extracted `ros/` directory:

```bash
bash ./m23bv_offline_gate.sh
```

Expected `M23BV_OFFLINE_UNIT_TESTS=PASS` (26 tests),
`M23BV_HOST_OFFLINE_CONTRACT=PASS`. These checks use *synthetic message
metadata and fake rosbag writer*, not live DDS or the physical robot.

## Offline image build — operator's host only; no device mapping

First verify the image IDs in the operator's local Podman store. The accepted
OpenCV image must be the **exact** local ID below. The M23H base ID was not
supplied in the handoff; RECORD its full ID and fail if it differs from the
operator-approved locally installed M23H sensor image.

```bash
podman image inspect localhost/leroy-ros:xrslam-pthreads-opencv-v2 \
  --format '{{.Id}}'
# REQUIRE 40351f56ff4fa54da881178050b91e74db0450b45b07a51ca7bc67b7590112d1
podman image inspect localhost/leroy-sensors:m23h-bounded-recording \
  --format '{{.Id}}'

# From the package's ros/ directory. No network, sensors or production process.
# Caller is responsible for uniquely named, owner-private log path.
# Use the prechecked, additive offline-only script. Require the approved
# full M23H image ID (not available in this handoff).
export M23BV_M23H_EXPECTED_IMAGE_ID=<approved_64_hex_M23H_image_id>
export M23BV_LOG_DIR=<existing_private_absolute_log_directory>
bash ./m23bv_podman_build_offline.sh
```

Do NOT rebuild established release/diagnostic images or use `--device` here. The script verifies source image IDs and records SHA-256 hashes of unique private logs and the new derived image ID.
Do NOT start physical capture from this package. An operator-owned physical
host wrapper would require separate review, independently verified actuator
power inhibition, immobilization, E-stop, device exclusivity, exact stable
USB mappings, image ID pinning, and NEW explicit authorization.

## Still required before any physical recording

1. Verify the copied `/opt/m23aj-opencv/lib` is complete in derived image.
   `ros2 pkg executables image_proc` must list `rectify_node`. Check direct
   `ldd -r` and transitive ELF dependencies of both the executable and the
   image_proc rectifier library under the **actual rectifier-only** loader
   environment. Require no `not found` / undefined symbols. Independently
   verify system OpenCV remains available for cv_bridge; do not set a global
   alternative `LD_LIBRARY_PATH`.
2. Run **real no-device ROS 2 Jazzy synthetic DDS** publishers/rectifier/
   reliable recorder tests with the same RMW and start order; verify actual
   graph QoS endpoints and rate/continuity. Unit mocks do NOT prove this.
3. Run a synthetic **actual rosbag2 sqlite3** end-to-end derive/finalization
   test with matching CameraInfo and IMU: require closed metadata, exact
   preserved serialized CDR bytes and timestamps, source-file hash stability.
   Run negative >250ms gap, out-of-IMU-window, mismatched CameraInfo,
   unknown topic, and interrupted-writer cases in the real Jazzy image.
4. Confirm clean camera/rectifier/sensor process shutdown in no-device graph
   fixtures (and later in separately authorized physical capture); mock tests
   are not an adequate substitute for exit status evidence.
5. Integrate the existing **M23I corrected M23A auditor** from PR #103 in a
   separate derived offline audit image. Do not use default-branch unpatched
   M23A and do not relax its original image 250ms gap rule.
6. Capture unique immutable acceptance logs and image IDs; SHA-256 each log.
   Refuse any sensor access until all these no-device gates pass.

## Strict read-only derivation plan for a future closed bag

In a ROS 2 Jazzy offline image **with no hardware devices mounted**, after
passing the full synthetic rosbag acceptance above:

```bash
# EXAMPLE ONLY: choose new, nonexisting output path, source mounted READ-ONLY.
python3 /opt/m23bv/m23bv_derive_closed_bag.py \
  --source /input/existing_closed_bag \
  --output /derived/brand_new_overlap_bag
```

Mount source `:ro`, derived root separately `:rw`. **Do not run this against
the immutable M23BU bag expecting a PASS**; that bag has 64 known image gaps
above 250ms and the gate correctly blocks *before creating output*.
Output only asserts header-domain structural overlap, never physical
camera/IMU acquisition-time phase, extrinsics, VIO, navigation or motion.

## Acceptance status

- Static Python AST and Bash syntax: PASS locally.
- Pure-Python unit tests: 26/26 PASS locally, hardware access NONE.
- Source inspection: PASS for linked default-branch source + historical PRs.
- Real ROS Jazzy, actual image loader, synthetic DDS, real rosbag2 write,
  actual endpoint sampling, process shutdown: **NOT YET VERIFIED**.
- Physical test authorized: **NO**.
- Metric pose and physical motion authorized: **NO**.