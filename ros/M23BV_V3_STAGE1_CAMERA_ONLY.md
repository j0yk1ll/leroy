# M23BV V3 Stage 1: CAMERA ONLY (operator-authorized; **inventory/physical acceptance PENDING**)

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
- No new image is built or tagged. Operator-identified host camera `/dev/video8` must be confirmed via a stable `/dev/v4l/by-id/...` symlink, and is mapped **only** to `/dev/video2` **inside** the container (camera `rw` for V4L2 streaming; no `mknod`);
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

Before camera execution, independently establish the host's camera
`/dev/v4l/by-id/...` symlink resolves **exactly** to `/dev/video8` (without opening the
device); real calibration file provenance and expected SHA-256 (correct
physical camera, resolution 640x480 and plumb_bob intrinsics); pinned
container image ID; owner-private log directory and available disk; ROS 2
Jazzy and Podman identity; intended USB mapping and operator emergency
abort authority. Missing information is **NO-GO**.

## Operator signoff received; remaining NO-GO conditions

On **2026-10-24**, the operator explicitly attested in conversation that both
motor-power rails are physically disconnected, the chassis is secured,
the E-stop has been verified, and a supervisor can immediately remove power.
The operator explicitly authorized **Stage 1 camera-only**, with no MCU serial,
motors, recording, Nav2 or XRSLAM, and identified the host camera as
`/dev/video8`. This is **operator testimony**, not remotely verified physical
isolation or hardware runtime evidence. The invocation still requires an
owner-private dated approval record and a second interactive confirmation.

The operator's later udev output reports camera `ID_V4L_CAPABILITIES=:capture:`,
model `Wed_Camera`, serial `Wed_Camera_Wed_Camera_20240420112206`,
and stable path `/dev/v4l/by-id/usb-Wed_Camera_Wed_Camera_20240420112206-video-index0`
resolving to host `/dev/video8`. These are **operator-supplied read-only
observations**, not assistant access to the USB endpoint. The runner now
pins that exact path and checks the udev serial again prior to device mapping.

The operator reported calibration file
`ros/config/camera/front_camera.yaml` with SHA-256
`f1c99cb8d4e4f3c77373dcabbe683dee9ffbc050614396fcd2964d61e965a138`.
Its content matches GitHub blob `a2536ccdc02979087bc18f61789686bd52e23371`,
and the YAML has 640×480 `plumb_bob` with valid K/D/R/P. The YAML
**does not encode the physical USB serial or prove optical provenance**.

**Execution remains BLOCKED** until the operator explicitly verifies that
this calibration was obtained for the enumerated `Wed_Camera` USB unit,
pins the host calibration file and SHA, checks the immutable container,
completes the private dated safety record and reviews abort arrangements.
The runner requires an additional exact-phrase calibration provenance
attestation at the console; an unsupported guess must not be entered. The approved
scope does not include unplug/disconnect tests, recording, MCU serial access,
or robot motion.

## No-device review and source checks

Source-only validation is allowed before signoff (these do not run Podman
or launch ROS). Review the diff, run `bash -n` on the new `.sh`,
`python3 -m py_compile` on the new Python files and
`python3 ros/m23bv_v3_stage1_static_gate.py` on the operator checkout.
Check protected `git rev-parse HEAD:<path>` and working-tree blob SHA.
These are **SOURCE CHECKS**, not observed physical outcomes.

## Bounded behavior (NOT yet executed on camera)

After the authorized operator completes device/calibration inventory, the host script requires clean target branch,
protected Git blob pins, stable camera by-id identity, exact calibration
SHA-256, no other running Podman container, and exact immutable V3 image
ID. Failing any check blocks before `podman run`. The launch is bounded:
48-second in-container ROS observer, 75-second outer timeout with SIGINT,
then escalation and shutdown. Both scripts attempt cleanup; forced cleanup,
nonzero ROS launch exit, or surviving output reader is **not accepted**.

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

## Operator host commands (do not start hardware before full Stage 0 inventory)

From a clean worktree, fetch the new branch then run the no-device source
checks listed in the chat. `m23bv_v3_stage1_camera_run.sh` **is not** a
source-only check; its invocation opens the authorized USB camera and must
follow completed identity, calibration, image, safety and private-log review.

## Known acceptance limitations

The source-only checks cannot certify USB camera enumeration, Podman V4L2
permissions, device calibration provenance, real DDS QoS, launch cleanup,
image quality or real frame rates. Unknown ROS graph publishers are rejected
at observation time, not before USB camera startup. Camera disconnect after
the successful short sample is NOT validated by this run. Require distinct
approved disconnect test before claiming camera-loss physical acceptance.