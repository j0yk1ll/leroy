# M23BB — Fail-closed host-persistent TEXT evidence staging (source only)

**Status: DRAFT, unbuilt, uninstalled, no ROS and no physical test authorization.**
Stacked on M23BA PR #143, whose 28 synthetic offline correlator tests
were reported PASS. That correlator can classify repeated camera
M23AX pre-send timestamps versus M23K raw subscriber observations,
but M23AU's original full per-message logs were lost from an
ephemeral container's /tmp; its publisher root cause remains UNKNOWN.

M23AY's diagnostic binary image ID
9ff2faf53109f88333e79c998b6946caa49f09fc807d2da422c1adcd47bda571
and M23AZ's device-free ctypes dlopen proof remain immutable.
M23BB does NOT install into, tag, start or rebuild either image.

## Existing-source foundation

Actual M23K Python supervisor (ros/m23k_camera_path_probe.py):
- COUNTER_LOG = Path("/tmp/m23k-cpp-counter.log")
- CAMERA_LAUNCH_LOG = Path("/tmp/m23k-camera-launch.log")
- Opens BOTH file objects in mode "xb" (exclusive create).
- Passes these descriptors directly to the C++ observer and ROS launch
  subprocess stdout/stderr with start_new_session=True, and reads them
  after shutdown to preserve its existing >250ms/QoS fault gates.
- In a disposable --rm guest, /tmp logs vanish after exit or crash.

Actual C++ observer
ros/m23k_cpp/src/m23k_camera_header_counter.cpp:
- Emits "M23K_SAMPLE raw|rect|info STAMP RECEIPT_STEADY_NS\n"
  with each callback.
- Flushes every 2 seconds in a timer; on abrupt crash, final
  buffered events could be lost.
- Emits the completed "M23K_COUNTER_COUNTS=RAW,RECT,INFO" summary.

M23AX camera instrumentation emits "M23AX_PUBLISH" to stderr
with std::endl before a ROS publish call; this is a PRE-SEND
record, not acknowledgment that DDS delivered an Image.

## Proposed isolated copy and evidence path contract

This PR contains only:
- ros/m23bb_host_evidence_adapter.py: INERT by default. Its
  staged_paths() method is a narrow, fail-closed source function
  that verifies an **exact** future /evidence mount prior to any
  camera/observer process startup. It never starts a process.
- ros/m23bb_evidence_source_contract.py: INERT/no writes by
  default. --check verifies exact M23K Python and C++ anchors.
  Its in-memory transform() replaces BOTH /tmp paths with
  camera and observer paths returned by staged_paths(), preserving
  exclusive-create "xb" and all existing M23K subprocess/ROS
  monitoring logic. It also changes only the observer callback's
  terminal newline to std::endl, flushing each sampled header.
- ros/m23bb_source_test.py: synthetic negative controls, no
  imports of ROS, camera drivers, or robot binaries.

For a **future separately reviewed diagnostic image** only, the
transformed source must be COPIED into a fresh path alongside the
adapter, must override the old M23K Python import deterministically,
and the changed C++ observer must be compiled separately. A future
read-only build/ABI/source audit is mandatory. The existing M23AY
camera image and any original M23K binaries MUST remain untouched.
M23BB itself provides NO --write flag, no builder, no live launcher,
no host device mapping and no automatic retry.

Future guest path selection is hardcoded:
- /evidence/m23bb-camera-launch.log (ROS launch camera/pre-send output)
- /evidence/m23bb-counter.log (C++ raw/rect/info subscriber output)

The exact env markers must be present:
- M23BB_HOST_TEXT_EVIDENCE_REVIEWED=YES_NO_LIVE_AUTH
- M23BB_EVIDENCE_MOUNT=/evidence

In addition, staged_paths() requires:
- exactly one separately mounted RW /evidence path in
  /proc/self/mountinfo (not an inherited rootfs directory);
- no tmpfs, overlay, ramfs, proc or other known ephemeral FS;
- a 0700 real directory owned by the guest effective UID;
- a 0600 real .m23bb-host-bind-ready marker, owned by the same
  UID, with literal contents M23BB_HOST_BIND_READY_V1 followed
  by a newline; symlinked directories or markers are rejected;
- both target log files MUST NOT exist (including symlinks);
  the existing M23K "xb" paths then defend against creation races.

**These software flags do not prove a bind is a safe host filesystem
or that physical motors are disconnected.** A future operator
must separately attest the host mount/source identity, host-side
destination ownership, exact Podman guest mount and power isolation,
while a separate live runner must enforce size/time/resource limits
and prevent unrestricted guest writes. Do not mount all of HOME,
a production dir, entire /dev, USB buses or any MCU/serial device.
The mount token is a coarse integrity marker, not authentication.

Because subprocess file descriptors point DIRECTLY at the mounted
host paths, stdout/stderr bytes already written should survive
guest deletion. The C++ observer's each-line flush narrows buffering
loss; **a sudden fault, write failure, full filesystem or power loss
can still leave partial evidence**. No guarantee of completeness
without the final C++ count summary, ROS child exit inspection and
host file digests. A future review needs explicit active maximum-log
size limits, per-file host SHA256, the source/image manifest, launch
exit and the original M23O graph/QoS verdicts. Both logs must be
retained even for failed or inconclusive trials. Never silently
accept a partial log.

The M23BA correlator separately requires complete M23K counts,
one FULL reliable_100 profile and valid contiguous pre-send
metadata within its bounded input size. It reports patterns only;
a duplicate per-message header is not uniquely linkable to a
publisher index. Receipt steady-clock time and V4L2 timeval are
NOT directly comparable.

## Safe source-only operator gate (NOW)

This is the ONLY current operator procedure. Use a **fresh**
sparse checkout pinned to the exact PR SHA supplied with the
command (no Podman, no ROS execution, no device map). Include the
original M23K supervisor and C++ observer from the same immutable
commit for exact source checking:

~~~bash
(
  set -euo pipefail
  umask 077
  DIR="$HOME/.local/state/leroy-m23bb-source"
  test "$(id -u)" -ne 0
  test ! -e "$DIR"
  git clone --filter=blob:none --depth=1 --single-branch --no-checkout \
    --branch feature/m23bb-host-text-evidence-staging \
    https://github.com/j0yk1ll/leroy.git "$DIR"
  cd "$DIR"
  git sparse-checkout init --no-cone
  git sparse-checkout set --no-cone \
    '/ros/m23bb_host_evidence_adapter.py' \
    '/ros/m23bb_evidence_source_contract.py' \
    '/ros/m23bb_source_test.py' \
    '/ros/m23k_camera_path_probe.py' \
    '/ros/m23k_cpp/src/m23k_camera_header_counter.cpp'
  git checkout --detach
  test -z "$(git status --porcelain)"

  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bb_source_test.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bb_host_evidence_adapter.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bb_evidence_source_contract.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B \
    ros/m23bb_evidence_source_contract.py --check \
    --supervisor ros/m23k_camera_path_probe.py \
    --observer ros/m23k_cpp/src/m23k_camera_header_counter.cpp
)
~~~

Expected: 32 offline unit tests PASS, adapter inert marker
M23BB_STATUS=INERT_NO_DEVICE_NO_ROS, source transformer inert marker
M23BB_PATCH_STATUS=INERT_NO_WRITES_NO_DEVICE, and exact source
M23BB_PATCH_VERDICT=EXACT_SOURCE_ANCHORS_MATCH_NO_WRITES.
These are expected contracts, NOT operator-observed evidence yet.

Do not call the adapter with --check on the host: that option
intentionally requires a future real /evidence guest bind and
will fail on a normal host. The source check above does not
enter the native loader or create any log.

## Hold points

- M23BB source gate PASS would mean **only** that copy-transform
  anchors and private host-mount preflights have synthetic coverage.
- No M23BB variant binary has been built or dynamically loaded.
- No host side evidence directory has been prepared or mounted.
- No new live trial is approved; original M23AU trial is CLOSED,
  timestamp-integrity verdict FAILED/BLOCKED, DDS root cause UNKNOWN.
- Future physically isolated camera-only trial requires its own
  explicit review and authorization AFTER source, build, evidence
  size bounds and clean shutdown guarantees are independently vetted.
- Never use ros/exec.sh or ros/robot.launch.py: these always start
  the robot bridge and can access STM32 CDC serial and actuators.
- No ROS graph launch, IMU, recording, DepthART, XRSLAM, Nav2,
  wheel/manipulator actuation or production acceptance here.

M23BB_CAMERA_RETEST_AUTHORIZED=FALSE
M23BB_PRODUCTION_ACCEPTANCE=BLOCKED
