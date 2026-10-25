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

**Now operator-inventoried (must be rechecked at each attempt):**
the identified motor-capable MCU is
`/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00`
resolving to host `/dev/ttyACM0` with udev
`ID_SERIAL=Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT`,
`ID_VENDOR_ID=1209`, `ID_MODEL_ID=f401`.
The host wrapper MUST re-resolve and match the stable by-id link
and serial immediately before the physical attempt.

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

The camera's nominal `udevadm ID_SERIAL` remains
`Wed_Camera_Wed_Camera_20240420112206`, and the immutable container
still expects `/dev/video2`. **The real physical stream's stable V4L
selector is not currently established.** During a Stage 2B attempt,
`/dev/v4l/by-id/usb-Wed_Camera_Wed_Camera_20240420112206-video-index0`
resolved to host `/dev/video6`, with matching serial and
`ID_V4L_CAPABILITIES=:capture:`; the operator then manually viewed
`/dev/video8` using `ffplay` and reported that `/dev/video8`, not
`/dev/video6`, is the correct camera. Identical USB serial/capture
metadata alone are therefore **not** a sufficient stream identity proof.
Host `/dev/videoN` indices remain ephemeral. Do not hard-code
`/dev/video8` or automatically use the `video-index0` link.
Calibration `ros/config/camera/front_camera.yaml` SHA-256
`f1c99cb8d4e4f3c77373dcabbe683dee9ffbc050614396fcd2964d61e965a138`.
V3 image ID must remain
`f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c`.

## Only essential no-device checks

New `ros/m23bv_v3_stage2b_static_gate.py` checks protected source pins,
ordering and command allowlist (source-only). New
`ros/m23bv_v3_stage2b_preflight.sh` runs it plus Python AST/Bash syntax,
then imports M20/M18 in **the pinned immutable V3 image** with
no **device mappings** or network. It deliberately includes **two exact
read-only source-file bind mounts**, matching the physical runner:
`ros/m20_combined_observer.py -> /opt/m20/m20_combined_observer.py`
and `ros/m18_imu_stream_source.py -> /opt/m18/m18_imu_stream_source.py`.
It verifies the effective mounted source pins, resolved Python import
locations, readiness checker, M18 command whitelist, and missing
physical device nodes. This is only to prevent a doomed live MCU
session from missing Python dependencies.
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
camera `explicit operator-specified stable /dev/v4l/by-id or
/dev/v4l/by-path link -> visually confirmed host /dev/videoN ->
container /dev/video2` and operator-verified MCU
`pinned by-id -> current host /dev/ttyACM* -> /dev/ttyACM0`.
The physical runner requires **both**
`M23BV_STAGE2B_CAMERA_STABLE_LINK` and
`M23BV_STAGE2B_CAMERA_VISUALLY_VERIFIED_NODE` to be explicitly supplied.
Their resolved paths must agree, the selected node must have
`ID_SERIAL=Wed_Camera_Wed_Camera_20240420112206`, report
`ID_V4L_CAPABILITIES=:capture:`, and pass a new interactive
`CAMERA STREAM VERIFIED /dev/videoN` attestation.
The selected stable link, both USB IDs, current resolved character
devices and major:minor numbers are revalidated **after** the physical
safety signoff and immediately before Podman. A mismatch blocks
*before* MCU serial access. It mounts the calibration and
**four read-only reviewed scripts** (Stage 1 camera observer, Stage 2B
orchestrator, updated M20 observer, updated M18 IMU source);
uses `--network none`,
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

## Inherited M20/M18 source drift — resolved in review, runtime pending

Read-only container inventory found **exactly two** mismatched files:
- Installed `/opt/m20/m20_combined_observer.py` Git blob
  `0aada34d8cf1d5608db29b8e832335876e6f50b3`
  (original accepted M20 commit `5976d209c5e7992b92fea0d2992834d97063ca82`);
  current review Git blob
  `5cf2f88cb2f79be80821888401b57a6393ee5e3b`.
  Newer M20 source moves serial-open CDC settle/input-reset into a
  `try/finally` that attempts STOP even when settling/reset fails.
- Installed `/opt/m18/m18_imu_stream_source.py` blob
  `5f15cbe64a26bf2d666251f224af0de49921a35e`
  (same original M20 commit); current blob
  `3c5fb37ca291533e2702a9159f3e8db11b076d22`.
  Newer M18 adds terminal fault latching and an optional metadata
  callback; unchanged M20 callers do not provide that callback.

The other five inspected image sources (M20 contract, M18 contract,
camera-readiness gate, V3 launch and compiled rectifier source) matched
the reviewed pins. **Do not retag, alter or rebuild the immutable V3
camera image; do not waive expected source pins**. Instead two
`readonly` Podman bind mounts substitute only the newer reviewed
M20/M18 Python modules at their original installed paths, with
Git-blob checks on the host and again in the container.

The repository diff has been reviewed; however the **corrected
mount-and-import preflight has NOT yet run on the operator's host**.
No physical camera/MCU test is cleared until this minimal check passes.
The original read-only diagnostic and all six source-level safety tests
passed; the prior import preflight appropriately BLOCKED the older
unmounted image contents. No new broad synthetic suite is necessary.

## First physical attempt: BLOCKED at host video-number check

The first opt-in physical Stage 2B invocation stopped at
`M23BV_V3_STAGE2B=BLOCKED_CAMERA_DEVICE_CHANGED`, **before** opening
the camera or MCU, and before operator attestation. The old wrapper
required the symlink to resolve specifically to `/dev/video8`, a
historical Stage 1 host enumeration. That is overly strict:
`/dev/videoN` is allocated dynamically by the kernel. The diagnostic
alone does not reveal the current node, so do NOT infer that a specific
new number or image-capture function was observed.

The reviewed correction removes ONLY the fixed host video-index
assumption. It still requires a character device at `/dev/video[0-9]+`,
the exact preexisting `video-index0` USB by-id symlink, the same
udev `ID_SERIAL`, and the same pinned calibration SHA. It records
the resolved device and major:minor in the private log, and
rechecks the camera and MCU symlinks, serial identities and device
numbers after the human authorization and before Podman. The
container-side video mapping remains `/dev/video2`, with no change
to V3 launch, binaries, M20, M18 or motor safety controls.

**Correction status:** SOURCE COMMITTED, new focused source gate and
operator retry NOT YET EXECUTED. First run the narrow
`ros/m23bv_v3_stage2b_static_gate.py` plus `bash -n` once on a
clean pinned review commit; inspect the resolved by-id camera link
read-only. Only after those checks and fresh physical safety attestation
may the revised operator-controlled runner be used for a single
bounded physical attempt. No additional broad synthetic suite.
Do not rerun the old commit.

## Second physical attempt halted: matching USB identity, WRONG feed

Operator-host read-only inventory during revised Stage 2B setup showed
```text
M23BV_V3_STAGE2B_CAMERA_RESOLVED=/dev/video6
ID_V4L_CAPABILITIES=:capture:
ID_SERIAL=Wed_Camera_Wed_Camera_20240420112206
```
The operator explicitly stated: **`/dev/video6` is not the correct
camera, `/dev/video8` is correct by manual `ffplay` inspection.**
Only the operator's safety-prompt text was submitted after that;
there was **no evidence of affirmative physical safety authorization
or a started Podman/M20 process**. The current selected stream remains
unverified and Stage 2B remains **BLOCKED**.

The next step is **read-only V4L interface inventory**:
resolve all symlinks under `/dev/v4l/by-id` and `/dev/v4l/by-path`
that target video6 or video8; capture `udevadm ID_SERIAL`,
`ID_PATH`, `ID_V4L_CAPABILITIES`, and sysfs video-node names
for both. This opens neither video stream nor motor-capable MCU.
If no uniquely stable symlink pointing to the visually confirmed
`/dev/video8` also has the calibrated camera's serial, the Stage 2B
runner MUST remain blocked: do not simply pass a raw `/dev/video8`
without stable identity or loosen the calibration pin.

**Stable interface resolved by operator read-only inventory:** both
`/dev/video6` and `/dev/video8` report
`V4L_NAME=Wed Camera: Wed Camera`, USB serial
`Wed_Camera_Wed_Camera_20240420112206`, and USB vendor/product
`32e6:9221`. The **critical distinction** is physical USB path:
`/dev/video6` has `ID_PATH=pci-0000:00:14.0-usb-0:3.4.1:1.0`,
while the visually verified `/dev/video8` has
`ID_PATH=pci-0000:00:14.0-usb-0:3.4.2:1.0`.
Its stable link is exactly
`/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3.4.2:1.0-video-index0`.
No other path or the by-id `video-index0` entry is permitted for
this run. The operator must still independently attest that the
physically identified stream was used for the calibration file.

**Source repair committed, physical runtime UNTESTED:** require explicit
stable link and its `ffplay`-confirmed host node to match **AND**
the exact `3.4.2:1.0` USB ID_PATH, serial, vendor/product
`32e6:9221`, capture capability, fresh interactive stream
attestation and post-signoff identity recheck. Re-run only the
focused source/syntax checks after updating the review branch.
No broad synthetic tests, image rebuild, physical recording,
motion or auto-retry.

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
