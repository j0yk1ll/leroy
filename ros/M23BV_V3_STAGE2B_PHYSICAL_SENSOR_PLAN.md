# M23BV V3 Stage 2B — operator-supervised physical camera + MCU IMU

**Status: SOURCE CANDIDATE / NOT PHYSICALLY EXECUTED.** Dedicated draft
branch stacked on the six-case synthetic-accepted PR #152 commit
`2feee11f0043454a9b618521c737ac0468f0712e`.
Keep #150, #151, #152 and this review PR draft and unmerged.

## What this test actually attempts

**Priority is real hardware integration.** The existing Stage 1 physical camera
gate passed: V3 compiled rectifier, actual RELIABLE/VOLATILE DDS and calibrated
640×480 `rgb8`, 109 matched camera/rectified stamps and clean shutdown.
The Stage 2A six-case synthetic DDS suite passed on one pinned review commit.
No further synthetic replay is a prerequisite for this proposed Stage 2B attempt.

Stage 2B begins with the exact accepted V3 camera launch and a **read-only
no-recorder graph + 90 matched camera stamp + calibration check** *before*
starting any MCU program. It then starts `/opt/m20/m20_combined_observer.py`
**unchanged** in the same isolated ROS 2 Jazzy domain. This invokes the
original M18 serial permit list only: `set_imu_stream_hz 50\r\n` (START) and
`set_imu_stream_hz 0\r\n` (STOP), requiring both ACKs and original
`M20_COMBINED_VERDICT=OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT`. No
recording, selected M23BV/M20 parent change, bridge, motor command, Nav2
or XRSLAM. The original M20 acceptance remains 120 seconds, >=5700
published IMU, >=400 image and camera info and >=90 seconds of
receipt overlap. Even success does **NOT** establish absolute
camera–IMU offset, extrinsics, metric VIO accuracy or safety for motion.

**MCU USB serial is motor-capable.** CDC open or DTR can reset MCU firmware
and can alter actuator state. Software allowlists are NOT adequate physical
isolation. The operator must personally disconnect **both motor-power
rails**, immobilize and secure the chassis, verify tested E-stop,
supervise immediate power disconnection, and confirm power state just
before each physical attempt. Never map MCU while motor power is connected.
If any condition fails, **DO NOT RUN**.

## Host identity prerequisites — read-only inventory first

**Unknown until operator reports:** stable STM32 MCU
`/dev/serial/by-id/...` symlink and its `udevadm ID_SERIAL` value.
Do not guess a device path or assume host `/dev/ttyACM0` is MCU.

On the owner-controlled host:
```bash
for link in /dev/serial/by-id/*; do
  [[ -L "$link" ]] || continue
  resolved="$(readlink -f -- "$link")"
  case "$resolved" in /dev/ttyACM[0-9]*) ;; *) continue ;; esac
  printf '%s -> %s\n' "$link" "$resolved"
  udevadm info --query=property --name="$resolved" |
    grep -E '^(ID_SERIAL|ID_VENDOR_ID|ID_MODEL_ID)=' || true
done
```
This reads udev inventory without opening any tty. Require a
single independently identified STM32 device by stable link and matching
`ID_SERIAL`. If no unique by-id link exists, BLOCK and design a
different pinned mapping; do not substitute an unverified `/dev/ttyACM0`.

Camera remains the pinned
`/dev/v4l/by-id/usb-Wed_Camera_Wed_Camera_20240420112206-video-index0`
resolved to host `/dev/video8`; container `/dev/video2`.
Calibration `ros/config/camera/front_camera.yaml` SHA-256
`f1c99cb8d4e4f3c77373dcabbe683dee9ffbc050614396fcd2964d61e965a138`.
V3 image ID must remain
`f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c`.

## Only essential no-device checks

New `ros/m23bv_v3_stage2b_static_gate.py` checks protected source pins,
ordering and command allowlist (source-only). New
`ros/m23bv_v3_stage2b_preflight.sh` runs it plus Python AST/Bash syntax,
then imports the original M20/M18 stack in **the pinned immutable V3
image with no devices, mounts or network**; verifies exact M20,
M18 and readiness checker source pins and command bytes. This is only
to prevent a doomed live MCU session from missing Python dependencies.
It is **not another broad synthetic test suite**.

No physical Stage 2B attempt should begin unless this small preflight
passes and the MCU USB identity is established. All tests are operator
invoked; the assistant has no hardware access.

## Controlled physical launch, after preflight and MCU identification

`ros/m23bv_v3_stage2b_host_run.sh` is a **different**, opt-in
host program, not the accepted Stage 1 runner. It pins the source
branch/clean tree/protected blobs, immutable image, camera ID and
calibration, and an operator-supplied exact MCU stable USB by-id
symlink plus its udev `ID_SERIAL`. It requires an interactive physical
safety attestation, separate camera-calibration attestation and the
exact phrase `AUTHORIZE V3 CAMERA AND M20 IMU NO RECORDING`; writes
a **fresh dated approval record mode 0600** and private SHA-256 log.

The isolated container maps **exactly two** devices:
camera `host /dev/video8 -> /dev/video2` and operator-verified MCU
`host /dev/ttyACM* -> /dev/ttyACM0`. It mounts only camera
calibration and two read-only proposed source scripts, uses `--network none`,
private PID/IPC, `--read-only`, dropped capabilities and no-new-privileges.
There is **no rosbag process, recorder override or motor bridge**.
Pre-serial graph validation rejects any unexpected preexisting topic or
node and requires the compiled V3 camera chain to have >=90 matching
raw/CameraInfo/rectified stamps. If blocked, the original M20
serial observer is never started.

When M20 starts, its original source independently checks prohibited
nodes/publishers, starts and stops IMU streaming with ACKs and
asserts 120-second camera+IMU overlap/count/clock/source contracts.
The new probe must await M20 STOP/ACK before shutting down
the V3 camera. It rejects missing acknowledgements, nonzero M20 exit,
camera loss, forced camera shutdown, unreaped children or unexpected
launch process deaths. There is a bounded outer timeout. **If STOP
ACK is absent or unverified, the MCU state is unknown; power must remain
physically isolated, operator intervenes, and no acceptance is recorded.**
Never retry automatically. Source-only checks cannot prove safe
physical USB CDC behavior.

The host will only be offered the physical invocation after the first
MCU USB inventory and necessary source/image preflight have been
reviewed. No need for new hardware-free synthetic robustness
tests first.

## Evidence acceptance

Demand all:
```text
M23BV_V3_STAGE2B_RUNTIME_PINS=PASS
M23BV_V3_STAGE2B_PRE_SERIAL_GRAPH_QOS=PASS
M20_IMU_STREAM_START_ACK=PASS
M20_IMU_STREAM_STOP_ACK=PASS
M20_COMBINED_VERDICT=OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT
M23BV_V3_STAGE2B_CAMERA_CHILD_REAP=PASS
M23BV_V3_STAGE2B_CAMERA_LAUNCH_REAPED=PASS
M23BV_V3_STAGE2B_RESULT=PASS
M23BV_V3_STAGE2B_PROCESS_EXIT=0
M23BV_V3_STAGE2B=PHYSICAL_CAMERA_IMU_PASS
```
Additionally inspect JSON report counts, topic provenance, RMW, receipt
overlap and every child exit; hash the private log/approval and inspect
Podman for remaining container processes. An isolated sensor contract
PASS is not a physical motion permit, a bag recording, camera–IMU
time calibration or production promotion. Keep draft PRs unmerged.
