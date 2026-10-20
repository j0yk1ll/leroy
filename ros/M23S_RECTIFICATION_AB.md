# M23S — isolate zero versus nonzero camera distortion at shutdown

**Draft PR; stacked directly on M23R #112. No operator M23S build,
ROS run or physical device access has occurred.**

## Cross-PR basis and falsifiable hypothesis

M23N #108's last clean camera diagnostic used `raw_only_mjpeg` and
had **no** `image_proc::RectifyNode` / `component_container`.
M23K #105 had already observed `component_container` SIGSEGV
during full rectification, before M23O #109's QoS work. M23O
reused the unchanged M20 full-rectifier launch; M23P #110 first
reported a clean device-free synthetic run with uncalibrated
`K[0]==0`, **zero rectified images**, then reproducible component
SIGSEGV with valid fabricated K and **nonzero D**. M23R #112
reproduced SIGSEGV (-11) in a fully delivering publisher-first
synthetic graph (899 matching raw/rectified stamps; maximum rectified
header gap 33.524 ms) and observed successful loader closes.
None of those runs establishes a faulting module.

The upstream Jazzy `image_proc/src/rectify.cpp` has distinct
branches when there is a downstream rectified subscriber:
- `K[0]==0`: reject uncalibrated CameraInfo, **no rectified output**.
- Calibrated `K[0]>0` and all `D[i]==0`: publish original image
  directly on `image_rect`, **no `model_.rectifyImage` call**.
- Calibrated `K[0]>0` and nonzero D: call
  `model_.fromCameraInfo`, `cv_bridge::toCvShare`,
  `model_.rectifyImage`, allocate/serialize rectified image,
  then publish. This changes more than OpenCV alone: memory allocation
  and image lifetime can also differ.

Source: https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/src/rectify.cpp

**M23S question:** Does activating the nonzero-D rectification branch
correlate with the previously reproduced component teardown SIGSEGV
when the image stream, DDS QoS and lifecycle order are otherwise held
constant? This is an A/B diagnostic, not a fix or proof of causation.

## Experiment invariants

Same image, exact single standalone `image_proc::RectifyNode` in the
**unchanged** M23R traced `component_container`; unchanged M23P
publisher-first graph order, 900 synthetic **640x480 RGB8** frames,
CameraInfo K/R/P, frame stamps, 30Hz nominal timer, two-second drain,
RELIABLE/VOLATILE history100 endpoints, C++ raw/rect/image-info
observer, SIGINT group shutdown, and original strict M23P child-exit
classifier. Neither profile runs a camera, MCU, recorder, Nav2,
XRSLAM, DepthART or motor bridge.

One newly compiled C++ binary (`m23s_ab_publisher`) is a direct
copy of M23P's accepted calibrated publisher, with only env-gated
`info_.d` selection and explicit profile/D markers added.
It deliberately retains the same ROS node name, endpoints, QoS,
payload and timer so M23P's real endpoint audit still applies.
Only `M23S_DISTORTION_PROFILE` changes between **two separate**
disposable no-device runs:

| Profile | Intrinsics | `CameraInfo.D` | Expected rectifier path |
|---|---|---|---|
| `passthrough` | K[0]=380, existing K/R/P | 0,0,0,0,0 | calibrated passthrough; nonzero rectified output |
| `rectify` | exactly same | -0.08,0.01,0.001,-0.001,0 | actual OpenCV/image-geometry processing |

`m23s_rectification_ab.py` overrides **only** the M23P synthetic
publisher executable and its calibration-marker validation, then
calls M23R's unchanged `run()`, which in turn calls M23P's unchanged
`run()`. The validation adapter checks exact requested
`M23S_PUBLISHER_DISTORTION_PROFILE`, D-vector and truthful
`M23P_SYNTHETIC_CALIBRATION` marker **before** adapting a
*private validation string* for M23P's original strict
900-image/timing/payload/graph-ready checks. The real publisher
log is never rewritten, and M23P's result reports the actual
fabricated calibration label. This accommodation never changes a
ROS message or a launch process.

**A/B validity requires real rectified deliveries in BOTH profiles.**
An uncalibrated `K==0` run with 0 rectified messages is NOT
an acceptable passthrough control.

All new files are additive in `ros/m23s_*`,
`ros/Dockerfile.m23s-rectification-ab` and this runbook.
No existing M23P, M23R, ROS production source, M23A or firmware
is changed.

## Gate A — offline tests only, no ROS/container execution

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23s-rectify-distortion-ab
git pull --ff-only origin feature/m23s-rectify-distortion-ab
cd ros

PYTHONDONTWRITEBYTECODE=1 python3 -u m23s_rectification_ab_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23r_shutdown_trace_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23p_no_device_rectifier_shutdown_test.py
```

Require all tests PASS with
`M23S_NO_DEVICE_AB_SOURCE_CONTRACT=PASS` and
`M23S_DEVICE_ACCESS=NONE`; retain the previous M23Q offline
regression gates. Never advance on missing markers.

## First operator Gate A result and false-positive source-test correction

The first operator offline run completed **21/22** tests successfully.
Only `test_no_new_ros_launch_or_custom_shutdown` failed because its
literal `"ros2 launch"` substring check matched an explanatory comment
inside `m23s_rectification_ab.py`. The actual runner contains no new
process launch or shutdown action; it delegates to M23R/M23P.

The M23S test now uses Python's `ast` to inspect imports, actual
call expressions and string literals, ignoring comment text. It
refuses direct `subprocess`, `rclpy`, `serial`, `os.killpg`
or `os.system` invocations and embedded `ros2 launch` string
literals, while still requiring `m23r.run()` and the original
no-device preconditions. Neither ROS process creation nor the
existing M23P/M23R lifecycle logic was modified.

**Corrected test has NOT yet been rerun by the operator.**
Repeat Gate A and require **22/22 PASS** before building.

## Gate B — disconnected image build, separately approved

Confirm the already-validated local parent image:
`localhost/leroy-sensors:m23r-shutdown-trace`, from M23R #112
Gate B and its device-free idle/synthetic operator runs. Inspect and
record the actual immutable image ID locally **before** building;
no particular M23R image digest is asserted in this PR.
Refuse unexpected retag/rebuild of parent. Do not use network pulls.

```bash
umask 077
DIR="$HOME/.local/state/leroy-m23s"
mkdir -p "$DIR"
chmod 700 "$DIR"
PARENT=localhost/leroy-sensors:m23r-shutdown-trace
podman image exists "$PARENT" || exit 2
PARENT_ID="$(podman image inspect "$PARENT" --format '{{.Id}}')" || exit 2
test -n "$PARENT_ID" || exit 2
echo "M23S_PARENT_IMAGE_ID=$PARENT_ID"

LOG="$DIR/build-01.log"
test ! -e "$LOG" || { echo M23S_BLOCKED=EXISTING_LOG; exit 2; }
BUILD_RC=0
podman build --network none --pull=never \
  -f Dockerfile.m23s-rectification-ab \
  -t localhost/leroy-sensors:m23s-rectification-ab . \
  >"$LOG" 2>&1 || BUILD_RC=$?
echo "M23S_BUILD_EXIT_CODE=$BUILD_RC"
grep -E 'M23S_NO_DEVICE_AB_IMAGE_CONTRACT=|error:|Error:|FAILED' \
  "$LOG" | tail -20 || true
test "$BUILD_RC" -eq 0 || exit 2
test "$(podman image inspect "$PARENT" --format '{{.Id}}')" = "$PARENT_ID" || {
  echo M23S_BLOCKED=PARENT_IMAGE_CHANGED; exit 2;
}
```

Require `M23S_NO_DEVICE_AB_IMAGE_CONTRACT=PASS`,
`M23S_BUILD_EXIT_CODE=0`, M23S source tests and exact unchanged
parent image ID. **No ROS or device execution during this build.**

## Gate C — device-free A/B, NOT authorized by merely opening this PR

Both profiles intentionally start NEW ROS `component_container`
processes. Run neither until a human reviews and accepts Gate A/B
and explicitly authorizes a diagnostic synthetic-only scope.
This does **not** authorize any live camera, any `/dev/ttyACM*`,
recorders, IMU, XRSLAM, Nav2, firmware, real robot motion, or
a production change. Stop immediately if any isolation gate fails.

Execute **passthrough first**, assess the preserved M23P/ M23R/M23S
verdicts, then independently decide whether to run `rectify`.
Use a fresh container and distinct, protected host log per profile.

```bash
umask 077
DIR="$HOME/.local/state/leroy-m23s"
test -d "$DIR" || exit 2
LOG="$DIR/passthrough-01.log"
test ! -e "$LOG" || { echo M23S_BLOCKED=EXISTING_LOG; exit 2; }

podman image exists localhost/leroy-sensors:m23s-rectification-ab || exit 2
set +e
podman run --rm --pull=never \
  --network none --ipc private --pid private --shm-size 512m \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  -e M23P_PROFILE=synthetic \
  -e M23S_DISTORTION_PROFILE=passthrough \
  localhost/leroy-sensors:m23s-rectification-ab \
  >"$LOG" 2>&1
RC=$?
echo "M23S_PASSTHROUGH_EXIT_CODE=$RC"
grep -E '^(M23P_LIFECYCLE_VERDICT=|M23R_LIFECYCLE_VERDICT=|M23S_AB_VERDICT=|M23S_AB_RESULT_JSON=|M23S_DEVICE_ACCESS=|M23S_PRODUCTION_ACCEPTANCE=)' "$LOG" || true
```

Once reviewed, the independent `rectify` run uses the **identical
command/options/image**, replacing *only* `M23S_DISTORTION_PROFILE`
with `rectify`, the host log path with
`"$DIR/rectify-01.log"`, and exit marker with
`M23S_RECTIFY_EXIT_CODE`. Do NOT concatenate the logs or
run both profiles in a single container. The parent image
ID and final M23S image ID must remain constant.

The inherited M23P observer requires each profile to publish exactly
900 fabricated RGB8 Image and 900 CameraInfo messages, >=810
observed raw and rectified Images, >=90% matching rectified header
stamps, zero unmatched rectified stamps, and zero rectified
header-stamp gaps **>250ms**. A verified RELIABLE/VOLATILE
actual graph and clean publisher/observer exit are mandatory.
Crashing component exit must **remain blocked** even if the stream
itself is healthy.

## Gate D — offline paired evidence, no ROS

Only after two **separately authorized and completed** runs:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -u m23s_compare_ab_logs.py \
  --passthrough-log "$DIR/passthrough-01.log" \
  --rectify-log "$DIR/rectify-01.log"
```

The parser requires real image/rect/info counts, matching header
stamps, true profile/D labels, full M23P lifecycle and M23R
tracer verdicts, a single source PID, and no physical device access.
It recognizes only independently supported **clean** or confirmed
**child SIGSEGV -11** outcomes; a signal-ambiguous, missing,
incomplete or contradictory run yields `M23S_PAIR_BLOCKED`.
A successful *offline comparison* can report one of four
`M23S_PAIR_VERDICT` categories: passthrough clean/rectify crash
association; both crash; both clean under trace; or reverse pattern.
**Every** paired outcome retains
`M23S_PRODUCTION_ACCEPTANCE=BLOCKED` and does not establish
the originating function or library.

This **one-pair** A/B experiment prioritizes subsequent source-level
review; confirm any association with repeated *separate authorized*
no-device pairs and retain the possibility of shutdown race/timing
confounding before claiming root cause. M23R's LD_PRELOAD tracer
can perturb scheduling in both arms. A clean zero-D result is not
proof that TBB or OpenCV alone caused the nonzero-D crash.

## Operator M23S A/B results — one paired experiment completed

**Source/build gates:** the operator reported the corrected M23S source
suite **22/22 PASS**, `M23S_NO_DEVICE_AB_SOURCE_CONTRACT=PASS`,
`M23S_DEVICE_ACCESS=NONE`; the disconnected image build
`M23S_BUILD_EXIT_CODE=0` and
`M23S_NO_DEVICE_AB_IMAGE_CONTRACT=PASS`.
Previously reported `M23R_NO_DEVICE_TRACE_SOURCE_CONTRACT=PASS`;
the M23S parent image unchanged marker was also confirmed.
The parent M23P offline test suite was requested separately but its
result was **not included in these latest markers**. Do not invent it.

**Same-image, two separate device-free experiments:**
`M23S_AB_IMAGE_ID=01a0334afd19fea39cc4b8a1dc457e65452d6b0d73d1b959ce654904e29e78c2`
was reported for the zero-D arm; the nonzero-D arm's operator
command required this exact image ID to match before starting.
Each arm ran in a separate disposable container with its own
private saved log.

| | Calibrated zero-D (`passthrough`) | Calibrated nonzero-D (`rectify`) |
|---|---:|---:|
| Rectified images received | 900 | 900 |
| Matching raw/rect header stamps | 900 | 900 |
| Maximum rectified header gap | 33.562ms | 33.521ms |
| M23P/M23R lifecycle | Clean under trace | Blocked component lifecycle |
| Component child exit | Clean | **-11 SIGSEGV** |
| M23S operator exit | 0 | 2 |
| Devices | NONE | NONE |
| Production acceptance | BLOCKED | BLOCKED |

The operator then ran M23S's read-only two-log comparison, which
returned:

```text
M23S_PAIR_VERDICT=DISTORTION_BRANCH_ASSOCIATION_SUPPORTED_NOT_CAUSAL
M23S_PRODUCTION_ACCEPTANCE=BLOCKED
M23S_DEVICE_ACCESS=NONE
```

The reported `M23S_PAIR_JSON` additionally confirmed both real
rectified streams, matching header stamps, valid source-D profiles
and separately classified child outcomes. This **one paired A/B**
strongly associates activating the nonzero-distortion path with the
component teardown SIGSEGV under the tested conditions.
The experiment does **not** isolate an individual OpenCV call,
prove `libtbbmalloc` unload causality, or validate a fix. It also
does not eliminate stochastic shutdown race/timing effects.
No robot/physical sensor, IMU/MCU or production tests are authorized.

**Disposition:** M23S controlled differential diagnosis **PASS**
(as an evidence-collection objective); zero-D lifecycle **CLEAN
OBSERVED UNDER TRACE ONLY**; nonzero-D component shutdown
**BLOCKED — confirmed SIGSEGV -11**. Root cause still **UNKNOWN**.
Next engineering work should inspect and, if separately scoped,
instrument the **nonzero-D execution branch** and its resource
ownership/teardown at source level; keep both private logs and
the existing cores. The original strict M23A image-header
`>250ms` and all physical/production acceptance gates remain BLOCKED.

## Absolute safety and retention

The existing image tags, two historical private M23Q cores,
and private synthetic logs remain untouched. Retain only
bounded, sanitized summaries in GitHub; never upload any core
or raw process memory. M23A's physical image-header `>250ms`
BLOCKED gate remains unchanged. No VIO, recorder, camera, STM32
serial or robot motion is permitted by an M23S PASS.
**Keep M23S and all stacked parents draft/unmerged.**
