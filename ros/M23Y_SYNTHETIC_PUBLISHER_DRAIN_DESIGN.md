# M23Y — synthetic publisher final-frame drain diagnostic (design/source only)

**Status: SOURCE PREPARED, NOT BUILT, NOT RUN.** This experimental
branch is stacked on M23X draft #118, which is stacked on M23W
draft #117. Do not merge or treat this as a release fix.
All ROS/Podman executions, physical camera access, IMU/STM32,
VIO/Nav2, recorder, motor/motion and production are **excluded**
from source preparation. The original shutdown SIGSEGV root
cause remains **UNRESOLVED** and M23A real-camera image-gap
acceptance (>250ms) and production are **BLOCKED**.

## Existing evidence: one frame short, shutdown crash still reproducible

Operator's exact **M23X v2** A1 read-only evidence audit:

```text
M23X_V2_A1_EVIDENCE_AUDIT=PASS
M23X_V2_A1_PUBLISHED=900
M23X_V2_A1_RAW_RECEIVED=899
M23X_V2_A1_RECT_RECEIVED=899
M23X_V2_A1_INFO_RECEIVED=900
M23X_V2_A1_MATCHED=899
M23X_V2_A1_UNMATCHED_RECT=0
M23X_V2_A1_RECT_MAX_GAP_MS=33.599
M23X_V2_A1_GRAPH_QOS_VERIFIED=TRUE
M23X_V2_A1_PARAMETER_VERIFIED=TRUE
M23X_V2_A1_NONZERO_D_VERIFIED=TRUE
M23X_V2_A1_PUBLISHER_EXIT=0
M23X_V2_A1_OBSERVER_EXIT=0
M23X_V2_A1_CHILD_EXIT=-11
M23X_V2_A1_FAULT_CATEGORIES=BLOCKED_RECTIFIER_CHILD_CRASH
M23X_V2_A1_TRACER_FAULT_COUNT=0
M23X_V2_A1_ACCEPTANCE=BLOCKED_PENDING_REVIEW
M23X_NEXT_ARMS=STOPPED
M23X_ROOT_CAUSE=UNRESOLVED
M23X_PRODUCTION_ACCEPTANCE=BLOCKED
```

Thus default output publisher parameter was observed, the
stock nonzero-D rectifier processed all 899 observed raw
headers into 899 matching rect headers with no >250ms
image header gaps, and the child died **SIGSEGV -11**.
The strict 900/900 experimental delivery acceptance is
**NOT** satisfied, and no M23X v2 A2 or B1/B2 experiment
is permitted based on this A1. Do not recast the 899
result as PASS merely to accelerate plugin investigations.

## Falsifiable hypothesis, not established root cause

The unchanged synthetic publisher `ros/m23s_cpp/src/m23s_ab_publisher.cpp`,
pinned blob `bab0ee8adfa046614c43c7ad4b019df968f7958e`,
at frame 900 calls `image_pub_->publish(image_)`,
then `info_pub_->publish(info_)`, immediately emits
its 900/900 counter and invokes `rclcpp::shutdown()`.
The parent harness waits another 2s for rectification **after
publisher exit**, not while the final publisher entity still
exists. A large final RGB8 Image could conceivably be lost
on publisher context teardown while small CameraInfo arrives;
the aggregate 899/899/900 counts do not prove that the lost
frame was the last one, or identify its cause. A timed
drain is **not** evidence of delivery ACK, and may fail.

## M23Y fixture — additive and isolated

M23Y builds a **separate new C++ executable**,
`leroy_m23y_drain_publisher/m23y_drained_publisher`,
using an exact copy of the historical M23S C++ source
with **only two controlled source substitutions**:

1. Within the `if (++count_ == kFrames)` branch,
   replace immediate `rclcpp::shutdown()` with:
   cancel the existing 30Hz timer, mark drain start, create
   a 2-second wall timer which marks elapsed monotonic time
   and drains to a final `rclcpp::shutdown()`.
2. Add the two private members `drain_timer_` and
   `drain_started_ns_` to keep timer and start stamp alive.

All other C++ lines must be **byte-for-byte identical**
to the original M23S publisher: `kFrames=900`, RGB8
640×480 with constant payload, publisher node identity,
RELIABLE/VOLATILE KeepLast(100), K/R/P and nonzero
D, image then CameraInfo publish order, stamps, 8s
graph-ready warmup and 30Hz timer. No physical input
or recorder. No 901st message, changed raw publisher
QoS or new thread. A canceled timer does not authorize
unbounded runtime; inherited M23P waits and host total
run timeout still bound the process.

The private publisher log emits, in exactly this order:

- `M23P_PUBLISHER_COUNTS=900,900`
- `M23Y_PUBLISHER_DRAIN_STARTED=YES`
- `M23Y_PUBLISHER_DRAIN_ELAPSED_NS=<positive numeric>`
- `M23Y_PUBLISHER_DRAIN_COMPLETED=YES`

`m23y_drained_ab.py` requires exactly one of each marker,
the original 900/900 counter **before** drain markers,
and a measured elapsed monotonic interval between 1.5
and 10 seconds. These bounds reject missing/aborted
or runaway drain; they are **not proof of DDS delivery**.
The original M23S source/payload validators still run
and the original M23X 900/900 strict classifier remains
unchanged. The new wrapper temporarily substitutes the
single new synthetic publisher path and a drain-aware
publisher-log validator inside inherited M23S→M23R→M23P
execution, **identically for both `M23X_ARM` values**.
Its state is restored in `finally`.

The M23Y Dockerfile inherits the operator's **existing
immutable, tagged M23X v2 diagnostic image**, checks
offline Python source contracts and compiles the new
isolated publisher using a disconnected image build.
The original M23S and original M23X v2 images and
their existing private logs remain untouched. The
local v2 image ID must match the already saved
`build-v2-01-image-id.txt` **before** any build.
Do not pull, rebuild or retag the v2 parent. New
M23Y child image gets a unique tag and private ID/log.

## Follow-up gate design — no runtime authorization implied

**Gate A**: operator independently performs host-only
read-only source inspection and offline tests, including
byte-for-byte reference C++ comparison after the
two exact substitutions and checks that all changes
are **additive only**. No ROS, colcon, containers.

**Gate B**: separately reviewed, disconnected image
build from exact pinned M23X v2 child image. Rootless
Podman, `--network=none --pull=never`, no
`--device` or global `LD_PRELOAD`, unique private
build log; offline source contract and isolated compile.
No ROS experiment during this build.

**Gate C**: read-only local image provenance validation
of exact commit, preserved original M23S and v1/v2
images, pinned v2 parent, new immutable v3 image and
diagnostic-only defaults; no container start.

**Future operator-run isolated pilot**: first new
M23Y-image `baseline` A1 with nonzero D, original
default output plugin fan-out, child-only M23R tracer
and same M23X preframe effective string-array parameter
audit. Require **900 published / 900 raw / 900 rect /
900 CameraInfo observed / 900 matching stamps**, zero
unmatched, zero rect header gap >250ms, parameter and
graph/QoS provenance, verified 2s publisher drain and
**child SIGSEGV -11** to accept a reproducing control.
If any condition fails, **STOP** and inspect existing
private evidence; no claim of a clean rectifier merely
because the child survived an aborted run.

If and only if control A1 is accepted, a separate A2
in the **same immutable v3 image** must also satisfy
the crashing-control gate before B1/B2 are considered.
The only intended within-v3 A/B variable must be
`image_rect.enable_pub_plugins` in the component
(`baseline` default versus raw-only
`["image_transport/raw"]`); fixture drain applies
**equally** to both arms. Maximum two baseline and
two intervention runs, strictly sequential. No
M23T OpenCV helper or zero-distortion substitute.
A cleaner shutdown with the raw-only option would
only implicate plugin fan-out/lifetime as an
investigation lead; it would not identify Theora,
Zstd, TBB, `dlclose`, destructor or UAF causality.
Four runs would not be a statistically reliable fix.

**Runtime, builds or diagnostic code deployment are NOT
authorized by source review alone.** Only the human
operator can execute runs on the local ROS/Podman host.
No device or production acceptance follows from any
M23Y synthetic result.

## Source change-scope contract

Allow ONLY these seven **new** paths relative to the
M23X parent branch (no edits to any prior files):

- `ros/m23y_cpp/src/m23y_drained_publisher.cpp`
- `ros/m23y_cpp/CMakeLists.txt`
- `ros/m23y_cpp/package.xml`
- `ros/m23y_drained_ab.py`
- `ros/m23y_source_contract_test.py`
- `ros/Dockerfile.m23y-synthetic-drain`
- `ros/M23Y_SYNTHETIC_PUBLISHER_DRAIN_DESIGN.md`

All branches and PRs remain draft/unmerged.
Release/M23A/physical-device acceptance remains BLOCKED.

## Status

```text
M23Y_SOURCE_IMPLEMENTATION=PREPARED_FOR_REVIEW
M23Y_OFFLINE_TESTS=NOT_OPERATOR_RUN
M23Y_IMAGE_BUILD=NOT_RUN
M23Y_ROS_EXECUTION=NOT_RUN
M23Y_DEVICE_ACCESS=NONE_BY_SOURCE_PREPARATION
M23Y_ROOT_CAUSE=UNRESOLVED
M23Y_PRODUCTION_ACCEPTANCE=BLOCKED
```
