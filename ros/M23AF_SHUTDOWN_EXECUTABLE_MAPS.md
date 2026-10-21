# M23AF — exact-run executable-library map snapshots before teardown

**Diagnostic only; new image is unbuilt/untested until operator Gates A/B.**
All M23AE, M23AD, M23AC evidence is preserved. Do not replace or
retag the accepted M23AC v1 image ID.

## Preregistered question

Can the **same component-container process** expose executable
file-backed mappings (including librectify.so) after the stock factory
and again when the ROS shutdown callback fires, while independently
reproducing the full-delivery child SIGSEGV -11 in the existing
nonzero-D synthetic camera fixture? A later and separately gated
postmortem can correlate this run's retained core with the exact
snapshot, but only with a verified same-run identity, not just ASLR
page-offset resemblance.

The one active arm is M23AF_ARM=control. It retains the M23AC control
mechanism: stock Jazzy factory first, RTLD_NOW|RTLD_NOLOAD promotion
and matching diagnostic dlclose, **without RTLD_NODELETE**. The new
C++ executable additionally reads **only /proc/self/maps metadata**,
not raw mapped memory, once at post-factory creation and once via
rclcpp::on_shutdown. It filters executable file-backed path mappings,
emits begin/entry/end markers with a random 32-hex process run nonce
and PID in the container namespace. The nonce is also added to the
container process's argv as ROS parameter m23af_run_id to permit later
host coredump identity checking.

The inherited M23AA 900/900 frame delivery, CameraInfo, 900 raw/rect
stamp equality, zero >250ms header gaps, graph/QoS verified, synthetic
calibration and publisher drain, actual component child exit -11
must ALL pass. The new parent prints full map arrays into its
protected Podman stdout log only if both mapping phases and the
inherited crashing control validate. A clean shutdown, incomplete
maps, missing callback, unexpected child exit, or any source/build
failure is **BLOCKED**, not a partial success. A wrapper exit0
indicates this expected-crash *diagnostic protocol* passed; it never
indicates the component exited cleanly.

## Stage A: source-only

From the already accepted /tmp/leroy-m23ae repository checkout:

```bash
cd /tmp/leroy-m23ae
git fetch origin feature/m23af-shutdown-executable-maps
git worktree add --detach /tmp/leroy-m23af origin/feature/m23af-shutdown-executable-maps
cd /tmp/leroy-m23af
git diff --name-status aaf7a5fe921f22aaf4575002d7a809e9d2adb471 HEAD
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23af_source_test.py
```

Expect **eight added files only**, nine offline tests and OK.
No ROS, Podman, GDB, core reads, hardware or kernel modifications.

## Stage B: rootless disconnected build only

First verify that accepted parent image is still exactly:
`a3ca7cef00be950a52da0685407f54e12bc2d14b7a0a327e745c3e5adc9392f6`.
Build source from the pinned reviewed checkout using a unique new tag;
capture protected log, refuse preexisting evidence:

```bash
(
set -euo pipefail
set -C
umask 077
test "$(id -u)" -ne 0
D="$HOME/.local/state/leroy-m23af"
mkdir -p "$D"
test ! -e "$D/build-01.log"
test ! -e "$D/image-id-01.txt"
PARENT="localhost/leroy-sensors:m23ac-loaded-rectify-v1"
EXPECTED="a3ca7cef00be950a52da0685407f54e12bc2d14b7a0a327e745c3e5adc9392f6"
test "$(podman image inspect "$PARENT" --format '{{.Id}}')" = "$EXPECTED"
cd /tmp/leroy-m23af/ros
podman build --network=none --pull=never -f Dockerfile.m23af-shutdown-maps \
  -t localhost/leroy-sensors:m23af-shutdown-maps-v1 . >"$D/build-01.log" 2>&1
grep -Fq 'M23AF_BUILD_CONTRACT=PASS' "$D/build-01.log"
CHILD="$(podman image inspect localhost/leroy-sensors:m23af-shutdown-maps-v1 --format '{{.Id}}')"
test -n "$CHILD" && test "$CHILD" != "$EXPECTED"
printf '%s\n' "$CHILD" >"$D/image-id-01.txt"
echo "M23AF_BUILD_GATE=PASS"
echo "M23AF_NEW_IMAGE_ID=$CHILD"
)
```

STOP on any nonzero command/failed source/build gate. M23AC tag
and older images must remain unchanged. The build has no network.

## Stage C: ONE synthetic runtime, no devices

Only after Stage A/B and read-only image provenance checks.
No LD_PRELOAD, camera/serial/USB mapping, host network, privileged
container, robotics services, motion or production configuration.
Start from exact saved image **ID**, not mutable tag. One fresh log,
120 second timeout with fail-closed status:

```bash
(
set -euo pipefail
set -C
umask 077
test "$(id -u)" -ne 0
D="$HOME/.local/state/leroy-m23af"
LOG="$D/control-01.log"
test ! -e "$LOG"
IMAGE_ID="$(cat "$D/image-id-01.txt")"
test "$IMAGE_ID" = "$(podman image inspect localhost/leroy-sensors:m23af-shutdown-maps-v1 --format '{{.Id}}')"
test "$(podman image inspect localhost/leroy-sensors:m23ac-loaded-rectify-v1 --format '{{.Id}}')" = \
  "a3ca7cef00be950a52da0685407f54e12bc2d14b7a0a327e745c3e5adc9392f6"
ENVS="$(podman image inspect localhost/leroy-sensors:m23af-shutdown-maps-v1 \
  --format '{{range .Config.Env}}{{println .}}{{end}}')"
for VALUE in 'M23AF_ARM=' 'M23AC_ARM=' 'M23AB_ARM=' 'M23AA_ARM=' 'M23R_TRACE_ONLY=0'; do
  grep -Fxq "$VALUE" <<<"$ENVS"
done
! grep -q '^LD_PRELOAD=' <<<"$ENVS"
if timeout --signal=TERM --kill-after=5s 120s \
  podman run --rm --pull=never --network=none --ipc=private --pid=private \
  --cap-drop=all --security-opt=no-new-privileges --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,size=128m --pids-limit=128 --memory=2g \
  --env M23AF_ARM=control "$IMAGE_ID" >"$LOG" 2>&1; then RC=0; else RC=$?; fi
echo "M23AF_CONTROL_EXIT=$RC"
grep -E '^(M23AF_RUN_ID=|M23AA_VERDICT=|M23AF_RESULT_JSON=|M23AF_VERDICT=|M23AF_BLOCKED=)' "$LOG" || true
)
```

All executable mapping metadata is preserved in the private log as
`M23AF_MAP_SNAPSHOT_JSON` lines. **Do not paste those giant arrays
unless asked.** Provide only M23AF_CONTROL_EXIT, M23AF_RUN_ID,
M23AA_VERDICT, M23AF_RESULT_JSON, M23AF_VERDICT or M23AF_BLOCKED.
Expected M23AF_CONTROL_EXIT=0 with M23AA_VERDICT=SIGSEGV... means
**controlled child SIGSEGV -11 reproduced**, not clean rectifier.

No second control or NODELETE run is authorized by this protocol.

## Coredump correlation remains a separate next gate

The snapshot addresses are valid **only in that exact process**, as
identified by its cryptographic run nonce in argv, time window and
host-PID journal identity. Container PID is **not** host PID.
Current M23AF does not retrieve any core, inspect saved RIP, or identify
a prior owner for an unmapped PC. Historic M23Q 46312/48349 absolute
addresses must NOT be compared with new M23AF maps. If a new core is
unavailable or nonce not visible in its command line, do not claim
same-process correlation. No automatic rerun to generate a core.

Maps taken in the ROS shutdown callback reflect that callback's point
in the lifecycle; they do not themselves record a DSO unmap, prove a
callback UAF, or establish which unload or finalizer caused SIGSEGV.
Mapping capture can perturb timing. Successful Gate C is a diagnostic
prerequisite for a separately reviewed M23AG existing-core join, not
production release.

All physical M23A >250ms rectified image continuity, cameras,
DepthART/XRSLAM VIO, STM32/IMU, recording, Nav2/motion and production
remain BLOCKED. PRs are diagnostic drafts; M23Z #120 paused.
