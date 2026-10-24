# M23BV — Source parity and negative-gate review (hardware-free)

Status: **review draft; no device execution, no physical authorization**. This
document records source-level findings and expected verification gates. A
test script committed to this branch is not a PASS until the operator runs it
and retains exit-code and immutable-log evidence.

## Scope and pinned source baselines

The additive patch does not replace M20 or M23H files. Host-only regressions
now assert exact Git blob SHAs for:

| Unmodified ancestor source | Expected Git blob SHA |
| --- | --- |
| `ros/m20_camera_only.launch.py` | `a72000f97b72ec3b773879fb1066405b505f764a` |
| `ros/m20_sensor_only_run.sh` | `88c300b174f6e53bd536467f2e445d22e5ed1f8c` |
| `ros/m20_combined_observer.py` | `5cf2f88cb2f79be80821888401b57a6393ee5e3b` |
| `ros/m23h_bounded_capture.py` | `eb40aa6e636e7ba602554136a7f8272e9b407025` |

These pin checks are intentionally strict. Any upstream changes to these
sources require review of the new hashes and safety contract before changing
the expectations. Do NOT disable the test to accommodate unreviewed changes.

## 1. Camera launch contract

`m23bv_camera_only.launch.py` carries forward M20's exact `usb_cam`
node parameters: `/dev/video2`, 640×480, 30 fps, mmap, `mjpeg2rgb`,
`front_camera`, optical frame ID and file URL for the existing camera
calibration. The new source check compares the complete AST of the camera
node arguments against M20, not merely a few strings.

The M20 composable `image_proc::RectifyNode` inside
`rclcpp_components/component_container` is intentionally replaced with a
separate `image_proc/rectify_node` process. The alternate pthreads OpenCV
library directory is specified in that one node's `additional_env` only,
not in the global Python `cv_bridge` environment.

**Not yet proven on actual equipment:** the launched physical `usb_cam`
publisher's offered QoS, the `image_proc` process tree's teardown inside
`ros2 launch`, or physical MJPEG decoding under sustained recording. The
successful synthetic DDS test supplied its own reliable `usb_cam`-named
publisher; this cannot by itself establish the real USB camera publisher's
QoS. The graph guard correctly blocks any actual BEST_EFFORT raw-image
publisher before MCU serial acquisition.

## 2. M20 serial / process lifecycle

The M23BV sensor runner inherits M20's isolation variables, 9-second
camera-first launch delay, forbidden ROS graph checks, and ordered
`imu`-then-`camera` TERM cleanup. It adds the fail-closed actual ROS
camera/rectifier/recorder endpoint QoS guard **before** starting
`/opt/m20/m20_combined_observer.py`. The original M20 observer still owns
the exclusive STM32 serial file descriptor, allowlisted IMU START/STOP
commands, ACK handling, and protected `STOP` attempt on exit.

The M23BV capture supervisor (not connected to a host physical-launcher)
inherits the M23H process-order contract: recorder starts before sensors;
TERM signals only the sensor *parent* so its cleanup can send STM32 STOP;
SIGINT then signals the recorder *process group* to flush rosbag metadata.
Timeouts and failed shutdowns must return blocked verdicts. The additional
mocked tests cover both sensor TERM timeout and recorder SIGINT flush
timeout without starting any subprocesses, devices, ROS graph or Podman.

**Residual lifecycle risk:** the M20-derived shell may TERM or ultimately
KILL a `ros2 launch` *parent* while individual camera children have not
been independently observed to exit. The successful direct executable
synthetic SIGINT exit does not verify actual launch-managed USB camera
descendant shutdown. This requires separate contained, non-actuating
acceptance evidence before any real-device use is proposed. Do not add
global process-group KILL handling that could skip MCU STOP.

## 3. Expanded negative tests

`ros/m23bv_negative_contract_tests.py` is a host-only supplement to the
original 26 tests, covering:
- 3,906 CameraInfo messages with every image matched, bounded cache at
  2,048 and expected eviction count.
- 3,906 CameraInfo with images every five frames (normal sparse pairing).
- More than 2,048 CameraInfo followed by one genuinely unpaired image;
  it must be BLOCKED even after cache eviction.
- A deliberate greater-than-250-ms internal image gap, despite a dense
  CameraInfo stream; no trimming, smoothing or gate relaxation.
- Invalid CameraInfo rectified projection baseline.
- Fail-closed independent auditor provenance, VIO and motion flags.
- Original production threshold and M20/M23H source pin checks.
- Real raw-image CameraInfo publisher missing, BEST_EFFORT rectifier image
  endpoint and nonvolatile raw endpoint; all must block.
- Simulated sensor-parent TERM timeout and recorder SIGINT finalization
  timeout; both must be rejected with original signal ordering preserved.

These are source/model negative regressions. They do not open or decode real
camera frames, touch STM32, run XRSLAM, or test physical calibration.

## 4. Offline operator commands

In the separately checked out clean `leroy-m23bv` worktree only:

~~~bash
cd /home/kevin/Code/leroy-m23bv
git status --short
git fetch origin feature/m23bv-hardware-free-capture-correction
git merge --ff-only FETCH_HEAD

cd ros
bash ./m23bv_offline_gate.sh
~~~

Require both `M23BV_OFFLINE_UNIT_TESTS=PASS` and
`M23BV_NEGATIVE_SOURCE_REGRESSIONS=PASS` with overall exit 0. This host-only
step has **not yet been confirmed** by user evidence for the new tests.

## 5. Acceptance limits

The prior derived image
`57d886bf0b903e08ff1dc9f766d3daea28f44a20bcf9fd34733e7d3eab3029ec`
remains pinned. The new regression script runs on the Arch host and is NOT
part of that previously accepted image's source build; no rebuild is
needed for the proposed source-level gate.

The last accepted synthetic DDS/derivation/M23I audit log SHA-256 is
`8ddc9f26b43e45d99e9c638fb3883b5de606623c990523c2fdb22ddf0ceb1872`.
It provides synthetic evidence only; its 265 image/CameraInfo pairs never
reached the 2,048-entry CameraInfo cache ceiling.

Physical capture, real clock alignment, spatial extrinsics, XRSLAM and
metric motion remain unverified and unauthorized. The original M23BU
recording retains its 64 internal gaps greater than 250 ms; no derived
trimming or synthetic PASS repairs that source evidence.
