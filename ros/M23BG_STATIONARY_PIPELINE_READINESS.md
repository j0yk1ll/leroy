# M23BG — Stationary XRSLAM / DepthART / Nav2 integration readiness gate

**DRAFT / offline source validation only. This is NOT permission to operate
the physical camera, serial/MCU, IMU, recorder, motor, bridge, Nav2,
robot bringup or production stack.**

Priority: converge on a single, reproducible **stationary, bridge-free,
sensor → localization → obstacle costmap demonstration on Leroy**, not
another unrelated round of synthetic sensor tests. The current
M23AU repeated RAW header stamp failure stays BLOCKED; the accepted
M23AY and M23BC images and M23BD audit evidence remain unchanged.
M23BF completed 24/24 corrected synthetic tests and actual pinned
M23AY C++ source no-write check PASS at
`152f77a5f6743c4de55c8fef02f34831efb32990`. That is not an
instrumented camera build, true publisher count, or live root cause.

## The actual source connection graph

| Source | Output / consumer | Present implementation | Still blocked by |
| --- | --- | --- | --- |
| Pinned USB camera + RectifyNode | `/camera/image_rect`, `/camera/camera_info` | Prior camera/rectification and QoS gates | M23AU observed one duplicate RAW header stamp; complete publisher trace and causal attribution missing |
| Timestamped IMU | `/imu/data_raw` | Historical M18–M20 IMU and concurrent-camera observations | New MCU/CDC connection not authorized; physical exposure phase, camera–IMU offset and TF extrinsics not accepted |
| `leroy_xrslam` | `/xrslam/tracking_status`, `/xrslam/localization_valid`, gated `/odom`, `odom → base_link` | Native Jazzy adapter compiles, synthetic monocular and VIO inputs exercised | Real tracking, scale, timestamp/extrinsic calibration, nonidentity pose direction, restart recovery, single TF authority |
| `leroy_depthart` | `/depthart/depth` with original camera acquisition stamp | Historical physical camera/DepthART message contracts; source integration present | Current synchronized live output/latency, accuracy and coverage not independently proven |
| `leroy_nav2_perception.depth_obstacles` | `/leroy_nav2/diagnostics/obstacle_points`, `base_link` | Exact input topic and costmap output are compatible | `publish_obstacles=false`, `extrinsics_validated=false`, `ground_reference_validated=false` by default. Must not bypass without measurements |
| `leroy_nav2_integration` M4 rolling costmap | `odom` global frame + obstacle cloud | Synthetic `/costmap` fixture PASS does not use XRSLAM native `/odom` | Fictitious footprint, fake odometry/depth, not production |
| M15 real-interface command gate | Expects `/odom`, obstacle cloud, `/local_costmap/costmap`, XRSLAM status | ROS types and diagnostic-only inhibit/suppression implemented | M4 fixture probe listens `/costmap`, while M15 real gate assumes `/local_costmap/costmap`. Actual target endpoint unverified |
| M16 passive read-only observer | Nine actual source topics, report JSON | Code exists but has no costmap endpoint coverage | Cannot by itself validate actual costmap, localization geometry or safety |
| Existing `leroy_bringup/launch/robot.launch.py` | Composes camera + optional DepthART/XRSLAM + **hardware bridge** | Actual launch includes `leroy_bridge.launch.py` unconditionally | **Forbidden** for stationary isolated integration; bridge can open STM32 serial and control actuators |

The current XRSLAM node C++ contains a *conditional* `map → odom`
path behind `enable_map_tf`, an external `XRSLAM_MAP_ID` and verified
map-alignment/session API. That is **not proof** that persistent map
save/load, relocalization, occupancy mapping or an appropriate map
server are working on Leroy. The documentation's more conservative
description must not be interpreted as proof of either capability.

## What M23BG actually implements

- `ros/m23bg_stationary_integration_contract.py`: inert-by-default
  stdlib checker of **seven existing pinned source files**; no ROS,
  Podman, import of live ROS packages, motor/serial/device access,
  image pixels, logging of live sensor data, or writes. `--check
  --root <clean detached checkout>` reports grounded interfaces,
  misaligned costmap paths and **explicitly false physical readiness**.
- `ros/m23bg_stationary_integration_contract_test.py`: 23 synthetic
  positive/fail-closed cases. Assert source mutations or unsafe default
  changes do not silently pass, and that no source file is edited.
- Stage A source gate can PASS while `M23BG_REAL_ROBOT_INTEGRATION=BLOCKED`.
  The former means *interfaces found in source*; the latter remains
  blocked by absent real measured proof and physical authorization.

## Proposed release sequence — prioritized to reach real robot

### Milestone 1: offline combined genuine-component ROS graph

Prepare a separate **device-free, network-isolated** Jazzy integration
runtime using the existing installed native `leroy_xrslam`,
`leroy_depthart`, real `depth_obstacles` and actual Nav2
`costmap`, fed ONLY fabricated camera/CameraInfo/IMU/depth.
Avoid the M4 fake `synthetic_odom` and use no bridge, no hardware.
First deliberately require false localization and no metric `/odom`
because `pose_convention_validated=false`. This proves that the
combined graph fails *closed* rather than inventing localization.
Select ONE real costmap topic endpoint and make M15 and observers
match it. Do not activate motor-compatible topics, Nav2 controller,
planner or action server. The source-only M23BG gate is NOT this runtime.

### Milestone 2: real camera + rectifier root-cause clearance

Separately compile M23BF node instrumentation into an isolated derived
image with full ABI/ELF/loader and shutdown validation. Only after
independent motor/manipulator rail disconnection, device identity,
power isolation and explicit permission, consider one bounded
camera-only trial. Demand exact M23AX pre-send events, eventual
M23BE final attempt/return count, M23K per-callback samples and final
counts, same-run publisher/observer provenance, zero duplicate RAW
stamps and verified clean child exits. Remain blocked on ambiguity,
truncation, a >250ms gap or non-monotonicity. No IMU/serial on this test.

### Milestone 3: measured stationary real VIO + obstacle intake

After separate permission for stationary sensor use, verify a
common-camera-time-domain ROS graph: correct rectified
Image+CameraInfo sequence, timestamped IMU `/imu/data_raw`,
measured camera–IMU phase offset and physical extrinsics, validated
XRSLAM pose-direction and metric-scale semantics, exactly one
`odom → base_link` TF source. M16 must observe actual data (not
synthetic), including loss/absence behavior. Independently verify
DepthART accuracy/coverage/latency, ground plane and base-frame
obstacle points before enabling the diagnostic obstacle cloud.
Keep motor power disconnected. Do not bypass the adapter gates
merely to force `/odom` or obstacles to appear.

### Milestone 4: actual stationary Nav2 costmap

Use the same verified odom TF and validated obstacle points with
the actual Nav2 costmap, no controller or bridge. Configure a
measured robot footprint and conservative unknown-space behavior.
Require consistent `/local_costmap/costmap` (or explicitly selected
actual endpoint), observed lethal *and* unknown cells from real data,
freshness/TF coverage, no unexpected clearing when sensors pause,
and bounded recovery on real input loss. A local odom-frame rolling
costmap is NOT a persistent/global map or authorized autonomous
navigation.

### Milestone 5: independent physical motion commissioning

A separate engineering safety decision must cover independent
E-stop and physical safe state, measured wheel mechanics and stopping
distance, command authority and publisher ownership, watchdog
timeouts, recovery/fault matrix, restricted test environment,
supervision, and human permission. The M12 hardware-readiness
register explicitly says NO-GO today. No number of offline tests
substitutes for this evidence.

## M23BG Stage A — allowed now

A clean detached sparse checkout at an exact pinned PR SHA must include:
- `ros/m23bg_stationary_integration_contract.py`
- `ros/m23bg_stationary_integration_contract_test.py`
- the **seven** exact tracked source paths in the `PATHS` dictionary.

Run host Python only:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bg_stationary_integration_contract_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bg_stationary_integration_contract.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bg_stationary_integration_contract.py --check --root .
```

Expect **23** synthetic tests (NOT YET EXECUTED), inert default
`M23BG_STATUS=INERT_NO_ROS_NO_DEVICE_NO_WRITES`, positive source
marker `M23BG_SOURCE_INTERFACE_GATE=PASS`, and simultaneously
`M23BG_REAL_ROBOT_INTEGRATION=BLOCKED`. Any source mismatch or
test failure blocks progress; do not run ROS or hardware to compensate.

**No operation of `ros/exec.sh`, `ros/robot.launch.py`,
`leroy_bringup/launch/robot.launch.py` or `leroy_bridge` is
authorized by this milestone.**

M23BG_CAMERA_RETEST_AUTHORIZED=FALSE
M23BG_STATIONARY_REAL_PIPELINE_READY=FALSE
M23BG_MOTION_AUTHORIZED=FALSE
M23BG_PRODUCTION_ACCEPTANCE=BLOCKED
