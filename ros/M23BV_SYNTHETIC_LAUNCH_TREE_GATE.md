# M23BV — No-device ROS 2 launch-tree shutdown gate

**Status: unexecuted source in draft PR #150.** This supplements, but does not
replace, the already accepted no-device Jazzy, synthetic DDS, derived-bag,
independent M23I/M23A audit and host negative-regression gates.

## The gap being tested

The previous direct executable test sent SIGINT to a standalone
`image_proc/rectify_node` subprocess. It did **not** prove that a
`ros2 launch` parent can cleanly signal and reap all the child processes
it created. The M20/M23BV sensor runner relies on signaling its camera
`ros2 launch` parent **after** the protected M20 STM32 observer has completed
or been terminated to allow the allowlisted STOP/ACK sequence.

This test narrows that gap WITHOUT running the real physical camera launcher.

## What the test actually executes

The wrapper `ros/m23bv_synthetic_launch_tree_preflight.sh`:
- Pins the already accepted derived image ID to
  `57d886bf0b903e08ff1dc9f766d3daea28f44a20bcf9fd34733e7d3eab3029ec`.
- Refuses to run when the production `leroy-ros` container is running.
- Uses rootless Podman with `--network none`, separate ROS domain 179,
  `--ipc private --pid private`, dropped capabilities, read-only root
  and a disposable in-container `/tmp`; mounts **no devices or volumes**.
- Pipes the offline Python probe as standard input, leaving the pinned
  image unchanged.
- Records the complete probe output in an owner-private host log and
  emits its SHA-256.

The Python probe extracts the **exact rectifier Node construction AST**
from `/opt/m23bv/m23bv_camera_only.launch.py`, including its remappings,
QoS overrides and rectifier-local alternative OpenCV environment. It
constructs a **separate disposable synthetic ROS 2 launch** in tmpfs with
two launch-managed children:

1. An in-memory synthetic `/camera/usb_cam` publisher producing
   `/camera/image_raw` and `/camera/camera_info` at approximately 30 Hz
   with explicitly reliable endpoints and synthetic nonzero distortion.
   **This is NOT the usb_cam USB device executable.**
2. The actual `image_proc/rectify_node` process created with that
   exact extracted production rectifier Node expression.

A separate, test-only observer verifies raw-image/CameraInfo/rectified
DDS endpoints, receives at least 90 rectified images, and then sends
**SIGINT only to the disposable ROS launch parent**. It requires:
- The launch parent to terminate with exit code 0.
- The synthetic camera child to log both startup and clean STOP.
- The ROS launch log to enumerate both child PIDs.
- Neither launch-reported child PID to remain in the container's
  `/proc` after shutdown; otherwise fail closed.

This is a synthetic process-tree lifetime smoke test, not a guarantee
that descendants of a *real USB camera* launch exit in every failure
mode. No physical M20 sensor script or STM32 process is started.

## Operator commands (clean separate worktree)

~~~bash
cd /home/kevin/Code/leroy-m23bv
git status --short
git fetch origin feature/m23bv-hardware-free-capture-correction
git merge --ff-only FETCH_HEAD

cd ros
bash ./m23bv_offline_gate.sh

if M23BV_LOG_DIR="$HOME/.local/state/leroy-m23bv" \
   bash ./m23bv_synthetic_launch_tree_preflight.sh; then
  echo "M23BV_LAUNCH_TREE_TEST_EXIT=0"
else
  echo "M23BV_LAUNCH_TREE_TEST_EXIT=$?"
fi
~~~

The source host gate now covers 26 original and 18 negative/safety tests.
**This new 44-test suite and launch-tree runtime probe have not yet been
observed passing.** Previous 42 tests remain accepted independently.

Expected success markers (unverified until operator runs them):

~~~text
M23BV_RECTIFIER_LAUNCH_AST_FROM_PINNED_SOURCE=PASS
M23BV_LAUNCH_TREE_GRAPH_QOS=PASS
M23BV_LAUNCH_TREE_PARENT_EXIT=0
M23BV_LAUNCH_TREE_CHILDREN_REAPED=PASS
M23BV_LAUNCH_TREE_CLEAN_SHUTDOWN=PASS
M23BV_SYNTHETIC_LAUNCH_LIFECYCLE=PASS
M23BV_LAUNCH_TREE_PREFLIGHT=PASS
M23BV_PHYSICAL_ACCESS=NONE
~~~

## 2026-10-24: first as-launched trial BLOCKED (preserved)

User-observed `m23bv_launch_tree_6JqvcSnu.log`:
- `M23BV_LAUNCH_TREE_PREFLIGHT_EXIT=2` and fail-closed verdict.
- The exact rectifier AST was extracted from the accepted image; synthetic
  camera STARTED and STOPPED cleanly.
- `BLOCKED_LAUNCH_GRAPH:BLOCKED_LAUNCH_QOS:/camera/image_rawsubscriptionsrectify_node`.
  This is not accepted launch-tree QoS, and process-tree shutdown evidence
  has not yet passed its entire acceptance contract.
- SHA-256: `630a43286fadcc3f960dc84f5ea22cef3b59f4a1f20a7c8b1f70345c7455cd72`.

ROS Jazzy image_proc has a reported startup-order-dependent raw subscriber
QoS issue: https://github.com/ros-perception/image_pipeline/issues/1121 .
The earlier *passing* direct rectifier DDS smoke started the synthetic raw
publisher before the rectifier. The failing full ROS launch starts both
children together. A startup-order race is a **hypothesis**, not yet proof.

Two no-device modes are now available without changing the pinned derived
image or physical M23BV camera launch:

- `as_is` (default) reproduces simultaneous launch-managed startup and
  must remain blocked unless actual QoS/stream/children all pass.
- `camera_first_3s` is an explicitly labeled **DIAGNOSTIC ONLY**. It delays
  starting the exact same rectifier Node for three seconds, while the
  synthetic camera starts immediately. It prints
  `M23BV_PRODUCTION_LAUNCH_TIMING_REPRODUCED=FALSE`. A green outcome in
  this variant cannot promote the unchanged as-launched production path.

Run only the diagnostic variant (no hardware):

~~~bash
cd /home/kevin/Code/leroy-m23bv/ros
M23BV_LAUNCH_STARTUP_MODE=camera_first_3s \
M23BV_LOG_DIR="$HOME/.local/state/leroy-m23bv" \
bash ./m23bv_synthetic_launch_tree_preflight.sh
~~~

The test additionally prints `M23BV_LAUNCH_GRAPH_JSON` on endpoint
failure so the actual offer/request policies can be inspected, rather
than guessed from an error string. It does not loosen the reliable gate.

**NEXT DECISION:** If camera-first reliably passes where `as_is` fails,
design a separate deterministic camera-ready-before-rectifier sequence in
the additive camera launch. That is a *production source change requiring a
new derived image and renewed full lifecycle tests*. Do not silently alter
the existing pinned image or treat test-only delay as an accepted physical
launch.

## 2026-10-24: camera-first diagnostic accepted; graph/data-ready prototype next

Operator-observed isolated `camera_first_3s` diagnostic **PASS**:
- Exact pinned rectifier launch AST, DDS offered/requested RELIABLE, 90 rectified frames.
- `ros2 launch` parent exit 0; two child PIDs seen and reaped, synthetic
  camera START and STOP both PASS.
- Explicit `M23BV_PRODUCTION_LAUNCH_TIMING_REPRODUCED=FALSE`, and
  `M23BV_AS_LAUNCHED_SOURCE_QOS=UNRESOLVED`.
- Log: `m23bv_launch_tree_WOkjmftF.log`, SHA-256
  `9f444d08bce3eecee0a9ff22d9d1ef5e345bf16af6f0c946d8e84d86f683e993`.

**Important:** a time-based delay worked once but does not guarantee
readiness under different startup times, load or real MJPEG behavior.
Do not add an arbitrary 3s production sleep and declare reliability fixed.

New **third synthetic-only mode** `camera_ready` (not yet executed):
- Starts the same synthetic camera publisher under disposable `ros2 launch`.
- Starts an *additional* ROS readiness process that observes only the DDS
  graph and synthetic Image/CameraInfo timestamps. It requires both named
  `/camera/usb_cam` publishers to offer RELIABLE + VOLATILE and receives
  at least **10 exact timestamp-matched pairs** before returning zero.
- A 15-second monotonic deadline returns nonzero when readiness is absent.
- `OnProcessExit` returns the exact pinned rectifier `Node` action only
  when the readiness process exits **0**; every other exit emits launch
  `Shutdown(reason='BLOCKED_CAMERA_READINESS')`.
- After the rectifier starts, the existing strict graph test and 90-frame
  delivery test still apply, followed by launch parent SIGINT, complete
  launch PID inventory and child reaping. This run expects **3 children**
  (camera, readiness guard, rectifier).
- The wrapper adds `M23BV_CAMERA_READY_GATED_DIAGNOSTIC=PASS_ONLY`
  if all tests pass; `M23BV_AS_LAUNCHED_SOURCE_QOS=UNRESOLVED`
  remains unchanged. No physical launcher, Podman image, motor power,
  camera device, MCU or existing real bag is modified.

Run only this additional hardware-free diagnostic in the clean worktree:

~~~bash
cd /home/kevin/Code/leroy-m23bv/ros
bash ./m23bv_offline_gate.sh
if M23BV_LAUNCH_STARTUP_MODE=camera_ready \
   M23BV_LOG_DIR="$HOME/.local/state/leroy-m23bv" \
   bash ./m23bv_synthetic_launch_tree_preflight.sh; then
  echo "M23BV_GRAPH_READY_TEST_EXIT=0"
else
  echo "M23BV_GRAPH_READY_TEST_EXIT=$?"
fi
~~~

The expanded host-only suite now expects **26 original + 19 negative =
45 tests**. The new 45-test gate and `camera_ready` runtime probe are
**NOT YET VERIFIED**. Preserve preceding 42-test and camera-first PASS
evidence separately; a future failure must not silently rewrite history.

If this graph/data-readiness variant passes, the *next design task* is
to build a distinct additive camera launch candidate with an explicitly
reviewed readiness subprocess and return-code-checked launch handler.
That candidate needs a new image ID, new no-device runtime and lifecycle
evidence, and an independent physical-safety authorization process.
The currently accepted image is unchanged.

## Safety boundary

This test does not substitute for physical video-node QoS discovery, MJPEG
continuity, verified motor-power isolation before MCU CDC opening,
M20 STOP/ACK handling, approved real-sensor capture, camera/IMU offset
or extrinsic calibration, XRSLAM tracking or navigation. It also does not
repair the original M23BU raw bag's 64 excessive image-header gaps.

Leave PR #150 **DRAFT**. Do not start real camera, STM32, motor or Nav2
processes as a result of a synthetic PASS.
