# M23BV V3 Stage 1: CAMERA ONLY (review candidate; **NOT AUTHORIZED**)

**Stacked draft PR base:** `feature/m23bv-hardware-free-capture-correction` (PR #150).
**Review branch:** `feature/m23bv-v3-stage1-camera-only-optin`.
**Base:** draft PR #150, `feature/m23bv-hardware-free-capture-correction`, verified handoff head
`ba50cfdb63d247d71458cc495ab48ef8ae28a060` (2026-10-24).
This is an additive design; the selected `m23bv_sensor_only_run.sh` still invokes the
legacy `m23bv_camera_only.launch.py` and **does not test V3**.

## Scope and exact selection

- New opt-in host entrypoint: `ros/m23bv_v3_stage1_camera_run.sh`.
- New in-container bounded supervisor/observer:
  `ros/m23bv_v3_stage1_camera_observer.py`.
- New no-device source regression gate: `ros/m23bv_v3_stage1_static_gate.py`.
- The runner pins **existing immutable** V3 image ID
  `f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c`
  (`localhost/leroy-sensors:m23bv-reliable-rectifier-v3`) and sources the
  separate `/opt/m23bv_reliable_rectifier_proto/ws/install/setup.bash` overlay.
- It invokes only `/opt/m23bv/m23bv_reliable_rectifier_v3_candidate.launch.py`,
  with the existing SIGINT-v2 readiness gate and fail-closed camera-exit handler.
- No new image is built or tagged. Only camera `/dev/video2` can be mapped (camera `rw` for V4L2 streaming; no `mknod`);
  calibration and the observer are read-only mounts. No MCU serial device,
  M20 observer, recorder, XRSLAM, Nav2 or motor graph is started.
- The existing legacy camera launch, sensor runner, V3 candidate launch, M20
  START/STOP/ACK, bounded recorder, prior tests, and all image tags remain untouched.

## Stage 0 gate: mandatory independent human approval BEFORE any physical access

Do not invoke the physical runner on the basis of this document, a green
synthetic test, or a software flag. Record dated operator name, equipment,
verified *physical disconnect of both motor-power rails*, secured chassis,
tested E-stop and a person at the power disconnect. Obtain distinct,
written **camera-only Stage 1** authorization and agree an abort plan.
Serial authorization is **not** implied; `/dev/ttyACM0` must stay unmapped.
The code additionally requires an interactive exact-phrase confirmation,
four Stage 1 signoff variables, and an operator/dated approval-record file; these only
record human assertions and cannot measure motor isolation or E-stop state.

Before authorized operation, independently establish the host's camera
`/dev/v4l/by-id/...` symlink resolves to `/dev/video2` (without opening the
device); real calibration file provenance and expected SHA-256 (correct
physical camera, resolution 640x480 and plumb_bob intrinsics); pinned
container image ID; owner-private log directory and available disk; ROS 2
Jazzy and Podman identity; intended USB mapping and operator emergency
abort authority. Missing information is **NO-GO**.

## No-device review and source checks

Source-only validation is allowed before signoff (these do not run Podman
or launch ROS). Review the diff, run `bash -n` on the new `.sh`,
`python3 -m py_compile` on the new Python files and
`python3 ros/m23bv_v3_stage1_static_gate.py` on the operator checkout.
Check protected `git rev-parse HEAD:<path>` and working-tree blob SHA.
These are **SOURCE CHECKS**, not observed physical outcomes.

## Bounded behavior (NOT yet executed on camera)

After authorization, the host script requires clean target branch,
protected Git blob pins, stable camera by-id identity, exact calibration
SHA-256, no other running Podman container, and exact immutable V3 image
ID. Failing any check blocks before `podman run`. The launch is bounded:
48-second in-container ROS observer, 75-second outer timeout with SIGINT,
then escalation and shutdown. Both scripts attempt cleanup; forced cleanup
is **not accepted**.

The observer subscribes to image_rect to activate Jazzy's lazy rectifier,
and independently inventories DDS endpoints:

- `/camera/image_raw`: exactly one `usb_cam` RELIABLE/VOLATILE publisher
  and one `rectify_node` RELIABLE/VOLATILE raw reader;
- `/camera/image_rect`: exactly one `rectify_node` RELIABLE/VOLATILE publisher;
- `/camera/camera_info`: RELIABLE/VOLATILE `usb_cam` publisher, compatible
  VOLATILE rectifier subscriber;
- read-only image, CameraInfo and rectified message checks: positive stamps,
  `camera_optical_frame`, 640x480 `rgb8`, acceptable buffer layout,
  `plumb_bob` calibrated intrinsics exactly cross-checked against the mounted camera YAML
  (K, D, R, P), at least 90 matching unique
  raw/CameraInfo/rectified stamps with plausible timing;
- V3 original readiness gate log must show its 10 matched frames and PASS;
- unknown nodes/topics (including MCU, motion, TF, Nav2, XRSLAM or recorder)
  block; no bag is recorded and no MCU serial port opened.

**Conservative assumption:** Stage 1 expects `rgb8`; a legitimate `bgr8`
or other physical encoding must produce a NO-GO pending inspection and
review, not an automatic fallback. Source pinning and calibration hash do
not prove optical calibration accuracy. Expected output rate and pixel
quality must be analyzed from actual evidence, not assumed from the source.
A camera disconnect **after** a successful short sample is not proven
fail-closed; a separately approved unplug/exit negative test is required.

## Acceptance / evidence

After an authorized, observed run, require all of:
`M23BV_V3_STAGE1_QOS=PASS`,
`M23BV_V3_STAGE1_FRAMES=PASS`,
`M23BV_V3_STAGE1_LAUNCH_REAPED=PASS`,
`M23BV_V3_STAGE1_RESULT=PASS` plus wrapper exit 0,
`M23BV_V3_STAGE1=CAMERA_ONLY_PASS`, private complete log SHA-256, exact
image/calibration/source fingerprints and operator/date/safety record.
The wrappers print full private log path and SHA even if runtime blocks.
Check physical graph and frame metadata independently from log outputs;
a source-level QoS declaration is **not** an observed endpoint.

Stop on missing camera identity/calibration, wrong QoS, unknown graph,
early camera exit, timeout, forced cleanup or physical isolation doubt.
Retain failure logs plus SHA; mark Stage 1 **BLOCKED**; do not weaken
guards or promote the production selector.

**Stage 2 is NOT authorized or included here.** Only after Stage 1 evidence
review may a separate V3+M20 sensor/recording integration be designed.
That requires fresh motor-rail and E-stop verification and explicit MCU
serial approval, with original M20 START/STOP/ACK semantics untouched.
PR #150 must remain **DRAFT / UNMERGED**; Stages 3/4 remain outside
this candidate.

## Review-only operator host commands (DO NOT START HARDWARE)

From a clean worktree, fetch the new branch then run the no-device source
checks listed in the chat. `m23bv_v3_stage1_camera_run.sh` **is not** one
of those commands and is prohibited pending separately confirmed Stage 0.

## Known acceptance limitations

The source-only checks cannot certify USB camera enumeration, Podman V4L2
permissions, device calibration provenance, real DDS QoS, launch cleanup,
image quality or real frame rates. Unknown ROS graph publishers are rejected
at observation time, not before USB camera startup. Camera disconnect after
the successful short sample is NOT validated by this run. Require distinct
approved disconnect test before claiming camera-loss physical acceptance.