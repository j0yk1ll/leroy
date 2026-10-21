# M23AL — five independent synthetic pthreads composable shutdown runs

## Purpose and existing evidence

M23AK PR #130 succeeded **1/1** with stock composable
`image_proc::RectifyNode`, 900/900 synthetic raw/rectified/CameraInfo
messages, matching headers, QoS/graph and drain; actual child exit 0
with OpenCV 4.6 pthreads built from the pinned Ubuntu source.
M23AK same-process post-factory and on-shutdown ELF maps showed all
six alternate OpenCV DSOs under `/opt/m23aj-opencv/lib/` and no
`libtbb*` executable mappings. It does NOT demonstrate all-times
absence of TBB, proven ABI compatibility, long-term reliability or
permission to use physical devices.

M23AL asks only: *Will five fresh, isolated containers reproduce that
same device-free clean shutdown, with non-reused run nonces and
independent child-exit verification?*

This is an operator-run diagnostic; original successful image, past
tests, cores and synthetic evidence are immutable. Five is an
engineering screening threshold, not a statistical guarantee.

## Sources and contract

Three additive files: `m23al_repeatability.py`, offline tests, this
document. The runner is **inert by default**; it requires an explicit
`--run` flag. For each trial it creates a separate fresh
`podman run --rm` with `--pull=never --network=none --read-only`,
private IPC/PID, no added devices, no privileges, and a private
128MiB /tmp tmpfs with `notmpcopyup`.

It invokes the *same* M23AK comparison binary + Python from the
accepted M23AJ immutable image ID
`334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025`.
The two read-only Python mounts and LD_LIBRARY_PATH point to the same
reviewed M23AK + alternate OpenCV scripts. No host/OS library change.

Each of five runs requires:
- inherited M23AA stock composable child exit 0, no NODELETE,
  graph and 900/900 synthetic camera/calibration/QoS/stamp/drain checks;
- M23AK exact successful verdict, factory-control verification;
- both same-process maps with stock rectifier and all six alternate
  OpenCV 4.6 modules from the target prefix, and no TBB executable
  mapping at either observed point;
- distinct 32-hex run nonce, wrapper 0, device access NONE and
  production acceptance BLOCKED.

Failure creates a protected log for that trial, emits
`M23AL_FAILED_TRIAL=N` and `M23AL_STOP=NO_MORE_TRIALS`, and will NOT
start any remaining trial. No deleting/log overwrite or automated
retries. The `M23AL_VERDICT=FIVE_OF_FIVE_SYNTHETIC_COMPOSABLE_CLEAN`
marker appears only after **all five** pass.

## Stage A — source-only operator check

From the previously accepted M23AK worktree:

~~~bash
(
  set -euo pipefail
  test "$(id -u)" -ne 0
  cd /tmp/leroy-m23ak
  git fetch origin feature/m23al-pthreads-five-run-repeatability
  git worktree add --detach /tmp/leroy-m23al     origin/feature/m23al-pthreads-five-run-repeatability
  cd /tmp/leroy-m23al
  test -z "$(git status --porcelain)"
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23al_source_test.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23al_repeatability.py
)
~~~

Expect 12 source-only tests and
`M23AL_STATUS=NO_PODMAN_OR_ROS_BY_DEFAULT`. No container or ROS
execution in this stage.

## Stage B — explicit single five-run test batch

Only after Gate A. Stop if
`~/.local/state/leroy-m23al` already exists, the image identities
have changed, the source checkout is unclean or any trial fails.
Five protected logs are created one by one; no prior log is touched.

~~~bash
(
  set -euo pipefail
  set -C
  umask 077
  test "$(id -u)" -ne 0
  cd /tmp/leroy-m23al
  test -z "$(git status --porcelain)"
  test "$(git rev-parse HEAD)" =     "$(git rev-parse origin/feature/m23al-pthreads-five-run-repeatability)"
  test ! -e "$HOME/.local/state/leroy-m23al"
  PYTHONDONTWRITEBYTECODE=1 python3 -B     ros/m23al_repeatability.py --run
)
~~~

Expected five `M23AL_TRIAL=PASS` lines, followed by
`M23AL_RESULT_JSON` and
`M23AL_VERDICT=FIVE_OF_FIVE_SYNTHETIC_COMPOSABLE_CLEAN`.

No full logs/ASLR mappings need to be pasted into GitHub.
If blocked, report the earliest M23AL failure marker and only a
small relevant tail from its existing log. Do not rerun, prune images,
delete historical evidence, capture cores or proceed automatically.

## Gate after M23AL

Passing repeatability alone **does not unblock physical devices**.
The remaining pre-camera items are:
1. ABI/ELF/runtime dependency inspection of the camera-driver,
   cv_bridge, image_geometry and other required ROS camera-path
   libraries with the replacement OpenCV; no mixed libraries or TBB.
2. Synthetic performance at the actual camera resolution/frame rate,
   with image latency, CPU/memory budget and >250ms-gap analysis.
3. Reviewed, bounded camera-only launch with explicit device allowlist,
   STM32/IMU/XRSLAM/DepthART/Nav2, recording and motor/motion disabled
   and stop criteria; calibrated real CameraInfo.

Until separately approved, no physical /dev/video, USB camera,
serial/STM32/IMU, DepthART, XRSLAM, Nav2, recording, motor or
physical motion. Production is BLOCKED.
