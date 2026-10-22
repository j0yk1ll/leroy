# M23BF — Camera publisher completion epilogue (source-only patch proposal)

**Draft / NOT compiled, NOT installed, NOT run on camera, ROS or robot.**
Stacked on M23BE [draft PR #147](https://github.com/j0yk1ll/leroy/pull/147).
Operator reran corrected M23BE at
`819e56c2d60583caa20f8250a83048a06c36ce75`;
all **24** offline tests PASS. The initial M23BE run of 24 tests with
two assertion failures remains preserved and explicitly NOT retroactively
passed.

The original one approved M23AU camera-only trial is CLOSED/BLOCKED:
raw subscriber observed one repeated header stamp, 1404 callback
observations versus 1403 distinct header stamps. No evidence yet
attributes that repetition to camera acquisition, duplicate publication,
DDS delivery, or subscriber scheduling.

## Source-grounded lifecycle

This proposal is based on pinned `usb_cam` 0.8.1 commit
`cb2ae6bc0a312de629f84fef933fb3e07dac1ef8` and the exact
M23AX-generated copy of its C++ node. The upstream
`src/ros2/usb_cam_node.cpp` implements:

- `UsbCamNode::take_and_send_image()`, which sets Image/CameraInfo
  headers and calls `m_image_publisher->publish(...)`;
- a `UsbCamNode::~UsbCamNode()` destructor, which releases the
  image objects, timer, service, parameter callback handle and camera
  pointer; and
- `UsbCamNode::update()` timer callback.

M23AY compiled the M23AX per-attempt `M23AX_PUBLISH` stderr
metadata **before** the camera publish call, but does not maintain
a count of calls that returned. The M23BE future source contract
needs exactly one `M23BE_PUBLISHER_FINAL` marker with both counters.
A last observed event index alone cannot prove completion.

## Proposed M23BF source transformation

`ros/m23bf_publisher_completion_source.py` is a no-write,
inert-by-default Python transformer of exactly two **copied**
source files:
- `src/ros2/usb_cam_node.cpp`
- `include/usb_cam/usb_cam_node.hpp`

It assumes M23AX instrumentation has **already** been applied to the
copy. It verifies exact M23AX publication/fatal/source anchors,
specific destructor and timer ordering, source version 0.8.1,
and refuses repeated patching or changed contracts.

The in-memory transform makes these potential future changes:

1. Replaces M23AX's callback-local static
   `m23ax_publication_no` with a per-node
   `std::uint32_t m_m23bf_attempts = 0`; preserves every
   pre-send `M23AX_PUBLISH pub_index` field, acquisition status
   and metadata cap. Separate per-node
   `std::uint32_t m_m23bf_publish_returns = 0` is initialized
   in the same header. **Changing this class layout requires a
   fresh, internally consistent build of all node code.**
2. Increments `m_m23bf_publish_returns` **only after**
   `m_image_publisher->publish(*m_image_msg, *m_camera_info_msg)`
   returns normally. It is not proof of DDS sending, receipt,
   message acknowledgement or camera frame uniqueness.
3. After the original destructor's timer/service/parameter
   release and `delete (m_camera)`, emits
   `M23BE_PUBLISHER_FINAL attempts=N publish_returns=N
   shutdown=clean` with `std::endl` **only if** 3 to
   5000 attempts occurred and both counters match. Otherwise
   emits `M23BF_FATAL=INCOMPLETE_PUBLISHER_TEARDOWN`.
   If the process dies before that destructor code runs, no
   final marker appears and M23BE must block.
4. Preserves the existing compressed MJPEG publisher branch
   (not part of M23AX pre-send attribution). No pixels or bags
   are ever logged by this patch.

**Important qualification:** Destructor completion is NOT itself
proof of normal process termination. The existing M23BE parser also
requires one clean `usb_cam_node_exe` child exit, one clean
`component_container` exit and a complete M23K subscriber
summary, and checks no pre-send event follows the epilogue.
A future binary/launcher review must verify the selected
executor has stopped callbacks before destructor epilogue,
and that no startup/shutdown exception can produce a misleading
clean child exit. This source proposal does NOT establish those
facts. If either cannot be shown, the epilogue mechanism must
be revised before release.

This **does not** add the epilogue to the currently accepted
`localhost/leroy-sensors:m23ay-acquisition-diagnostic-v1`
image ID
`9ff2faf53109f88333e79c998b6946caa49f09fc807d2da422c1adcd47bda571`,
or the later M23BC image ID
`9f5c85981bf0879e27462ded52eda45beb74ba0e938714eaed5b794e4cc6cf90`.
They remain immutable and cannot satisfy the future M23BE
publisher summary without a separately derived image.

## Current Stage A — host Python source-only tests

`ros/m23bf_publisher_completion_source_test.py` contains
**24 synthetic tests**. They generate M23AX-instrumented source
from the original M23AX fixture and add the pinned node
header/destructor contract; they test no mutation, per-node
counters, ordering of attempts / publication-return increments,
teardown record timing, failure on changed source anchors,
incorrect package version, symlinks and default inertness.

From a fresh detached sparse checkout of the exact PR revision,
only run:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bf_publisher_completion_source_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bf_publisher_completion_source.py
```

Include these supporting source files in the sparse checkout:
`ros/m23ax_source_test.py`,
`ros/m23ax_instrument_usb_cam_source.py`,
`ros/m23bf_publisher_completion_source_test.py`,
`ros/m23bf_publisher_completion_source.py`.

**First operator run:** The first source gate at prior PR head
`786f1a72e601dc80340af4edfead02522a472644` stopped
during Python import with `SyntaxError: invalid syntax` at the
first post-transform `cpp.count` assertion, line 112. **Zero M23BF
tests were executed** and no source gate passed. The two adjacent
`if (cpp.count(...)` conditions contained an unmatched opening
parenthesis. These source predicates have been corrected to
`if cpp.count(...) != 1:`; no diagnostic counters, completion
marker semantics or safety checks were removed or weakened.
Only the new pinned revision may be tested, and a complete
24-test rerun remains **PENDING**.

Expected after correction, NOT YET OPERATOR-OBSERVED:
`Ran 24 tests`, `OK`,
`M23BF_STATUS=INERT_NO_WRITES_NO_ROS_NO_DEVICE`.

The script's `--check` mode expects the real, copied
`/opt/m23ax-source/usb_cam` source inside the **immutable
M23AY image**, not a user-host path, and never writes; that
would be a **later separately reviewed disconnected no-device
source gate**, not part of this initial 24-test step.

## Distinct future hold points

After source tests, before any new image build:
- Verify the pinned M23AY source copy in a rootless no-network,
  no-device, readonly Podman; use
  `--tmpfs /tmp:rw,nosuid,nodev,size=16m,notmpcopyup`.
  No ROS, no camera, no hardware is permitted.
- Independently review a separate no-device *derived*
  camera/observer image build that compiles node C++ and header
  together in a new install prefix; check ELF symbols, loader
  closure, pthreads OpenCV mapping, absence of TBB and inert CMD.
- Validate the shutdown/destructor behavior with a bounded
  no-device synthetic test; source code alone does NOT establish
  correct runtime timing or component destruction.
- Add a separate fail-closed bounded-host trace plan that preserves
  raw publisher and subscriber events, source identity and
  complete clean-exit records on both success and failure.
- Re-review physical electrical isolation and require explicit
  human authorization before any further USB camera operation.

Never use `ros/exec.sh` or `ros/robot.launch.py`: these
may open STM32 serial and energize robot actuators.
No camera, MCU, serial/IMU, recorder, XRSLAM/DepthART/VIO,
Nav2, motor power, manipulation, production or robot movement
is authorized by the M23BF source patch.

M23BF_CAMERA_RETEST_AUTHORIZED=FALSE
M23BF_PRODUCTION_ACCEPTANCE=BLOCKED
