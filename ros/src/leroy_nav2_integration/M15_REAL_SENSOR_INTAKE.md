# M15 — XRSLAM/DepthART real-interface software intake and fault-gated diagnostics

**Deliverable:** a real ROS 2 sensor-message intake and command-gating component using the *existing* Leroy sensor message interfaces, plus reproducible disconnected fault replay. This is a software milestone, not a paper-only design. **Actual camera/XRSLAM/bridge hardware has NOT been connected or exercised in M15.**

Unlike M14's synthetic-only subscriptions, M15 explicitly supports a `real_interfaces` input profile matching the ROS output contracts already implemented in the separate `leroy_xrslam` and `leroy_nav2_perception` packages. The second `offline_replay` profile uses exactly the same message validation and gating code but binds its inputs exclusively to diagnostic fake topics so there is never a real `/odom`, motor `/cmd_vel`, camera or serial device in the test.

This is an **incremental deployable software component**: an integration-ready sensor-input and functional velocity-gating stage. It is not yet an integrated autonomous Nav2 robot launch. Real sensor calibration, a physical sensor-data validation session, command arbitration, and bridge integration remain later work. M15's isolated container still uses `LEROY_NAV2_DRY_RUN=1`, `ROS_DOMAIN_ID=167`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; it excludes `leroy_bridge`, XRSLAM binary, the DepthART runtime, physical sensor devices and network access. The running `leroy-ros` container remains untouched.

## Grounded real source interfaces

| Input / expected publisher | Real-topic profile | Required M15 behavior |
| --- | --- | --- |
| XRSLAM `nav_msgs/Odometry` | `/odom`, sole publisher node `/leroy_xrslam` | Require finite metric-shaped pose/twist, normalized quaternion, nonnegative finite covariance diagonals, `odom`/`base_link`, strictly advancing acquisition stamps and ≤1.5s age |
| XRSLAM `std_msgs/Bool` | `/xrslam/localization_valid`, `/leroy_xrslam` | False / unknown immediately inhibits output; true alone does not establish freshness or calibration |
| XRSLAM `std_msgs/String` | `/xrslam/tracking_status`, `/leroy_xrslam` | Exactly `tracking`; other states inhibit and clear stale localization validity |
| TF `TFMessage` | `/tf` | Require finite, normalized, strictly advancing, fresh `odom → base_link`; correlate latest odometry and TF acquisition timestamps to within **0.25s** (M15 software hypothesis) |
| Depth obstacle `PointCloud2` | `/leroy_nav2/diagnostics/obstacle_points`, sole publisher `/depth_obstacles` | Require `base_link`, strictly advancing/fresh stamp, nonempty bounded compact float32 XYZ, finite points; an empty cloud never fabricates free space |
| Rolling `OccupancyGrid` | `/local_costmap/costmap` | Require correct `odom` frame, dimensions/data length and observed lethal **and** unknown cells for current narrow diagnostic acceptance |
| Actual MPPI proposal `Twist` | `/leroy_nav2/diagnostics/proposed_cmd_vel`, sole publisher `/controller_server` | Apply M14 core's finite planar velocity bounds and 0.4s receipt-age limit |
| Output `Twist` | `/leroy_nav2/diagnostics/m15_gated_cmd_vel` | Publish bounded proposal or **exact zero** at 10Hz to a new **unconsumed diagnostic** topic |
| Output `String` status | `/leroy_nav2/diagnostics/m15_gate_status` | Per-tick JSON with block reason, input/output nonzero flags and selected output vector |

The ROS publisher-node names above are graph provenance indicators, **not cryptographic authentication**. A single name matching `leroy_xrslam` does not establish measurement correctness or actual hardware origin. Input validation also cannot establish metric scale merely by checking numeric values.

XRSLAM source behavior: in `monocular` mode it cannot publish metric odometry; in `monocular_imu` mode it requires calibrated input and `pose_convention_validated=true` to publish `/odom`. Tracking failures and session restarts can latch its output off; the M15 gate MUST zero output on absent messages or `localization_valid=false`. The checked-in bringup defaults `use_xrslam:=false`; M15 does NOT override this or assert pose calibration.

DepthART source behavior: the depth adapter accepts `/depthart/depth` plus matching `CameraInfo` and publishes acquisition-stamped obstacle points in `base_link` only when `publish_obstacles`, `extrinsics_validated`, and `ground_reference_validated` are configured appropriately. M15 does **not** turn those flags on, claim that the camera is calibrated, or substitute fictitious TF/clearance. The existing depth adapter is a sensor conversion input stage, not an independent collision-safety function.

## Fail-closed software behavior

`m15_sensor_contract.py` is ROS-independent and validates Odometry, PointCloud2 and TFMessage payloads. Malformed or out-of-order data clears the **current** healthy sample. All three streams require both monotonic receipt freshness and acquisition timestamp freshness; odom and dynamic TF acquisition times must agree within 0.25s.

`m15_real_input_gate.py` reuses M14's actual software gate decision core. It also requires both positive XRSLAM tracking indicators and expected single ROS publisher graph identities for core sensor sources. If localization is false, tracking is not `tracking`, or an expected source is not discovered exactly, it emits exact-zero Twist, logs `BLOCKED_LOCALIZATION` or `BLOCKED_GRAPH`, and resets the one-second recovery dwell. Stale odom/cloud/TF, malformed geometry, map invalidity and stale proposals also select zero. Recovery happens only after all conditions remain healthy through the M14 dwell.

This is not a calibrated obstacle field-of-view assessment, route navigation, motor watchdog or a full command mux. Status inputs from XRSLAM are latched change reports; they are **not** a live heartbeat. Fresh `/odom` and TF are therefore required independently. `Twist` proposals are unstamped: receipt freshness is NOT source-generation freshness. M15's costmap lethal+unknown predicate is intentionally conservative and not a general production costmap semantics implementation.

The M15 gate currently requires the isolation environment even for the `real_interfaces` topic profile, and refuses remapping its diagnostic output onto a motor-compatible topic. Physical deployment packaging, real IO and a production-approved command path have not been implemented by this PR.

## Automated synthetic real-message-shape replay

`m15_replay_source` publishes valid **fake** `nav_msgs/Odometry`, `TFMessage`, `PointCloud2`, XRSLAM-style latched `localization_valid`/`tracking_status`, a fake rolling costmap and a bounded `Twist` proposal under `/leroy_nav2/diagnostics/m15_replay/*`. Its single `Trigger` service causes one four-second **synthetic** blackout. A new `m15_input_probe` independently receives the **actual gated output** and JSON status; it requires healthy nonzero forwarding before the fault, >3 ticks with nonzero input and exact zero output while blocked, >=5 independent received zero Twist samples in the inner blackout, and nonzero forwarding again after recovery.

Three isolated trials:

- `fault:=depth`: stop fake base_link PointCloud2 input; expect `BLOCKED_DEPTH`.
- `fault:=odom`: stop fake odom and matching dynamic TF; expect `BLOCKED_ODOM`.
- `fault:=tracking`: publish `tracking_lost` plus `localization_valid=false` and stop fake odom/TF; expect `BLOCKED_LOCALIZATION`.

**These are real ROS message types but fabricated data.** The replay's nonzero `Twist` does not come from MPPI; the preceding M14 milestone already demonstrated an ACTUAL MPPI controller with enforced diagnostic-depth/odom blackouts. M15 specifically tests the **real-interface message validation and localization-failure contract**. Do not confuse that with testing an actual XRSLAM binary or DepthART camera.

## Build and operator acceptance

From repository root, preserve unrelated local changes:

```bash
git fetch origin
git switch feature/nav2-m15-real-sensor-intake-contract
git pull --ff-only origin feature/nav2-m15-real-sensor-intake-contract
cd ros
podman build --no-cache -f Dockerfile.nav2 -t leroy-nav2:m15-intake .
```

The optional image checks all existing Colcon/pytest tests plus M15's new pure validation tests and installs three console scripts and one ROS launch. It does not copy `leroy_bridge`, `leroy_bringup`, `leroy_xrslam` or real depth drivers.

**Terminal 1 — start the chosen disconnected replay (depth first):**

```bash
podman run --rm -it --name leroy-nav2-m15 \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:m15-intake bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_integration \
      m15_real_input_replay.launch.py fault:=depth
  '
```

**Terminal 2 — independently verify diagnostic Twist selection:**

```bash
podman exec leroy-nav2-m15 bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  if ros2 topic list | grep -Eq "^(/cmd_vel|/odom)$"; then
    echo "FAIL: motor-compatible or physical odom topic" >&2; exit 1
  fi
  ros2 run leroy_nav2_integration m15_input_probe \
    --ros-args -p fault_mode:=depth
'
```

Required output:

```text
M15 WINDOW: mode=depth
M15 INPUT OBSERVATION: mode=depth blocked_gate_ticks=... blocked_input_nonzero_ticks=... blocked_gate_nonzero_ticks=0 blocked_gated_rx_zeros=... recovery_nonzero_forward=...
PASS: M15 real-interface MESSAGE-SHAPE software gate replayed depth outage; nonzero synthetic proposals suppressed to zero and restored after input recovery; no physical sensor or motor path
```

After each trial, Ctrl+C in Terminal 1 and verify both processes exit cleanly. Use a **fresh** container for the next trial. Repeat with `fault:=odom` and `fault_mode:=odom`; then with `fault:=tracking` and `fault_mode:=tracking`. Never send a second trigger to a previously triggered replay source. If a fault test fails, preserve both full terminal logs and do not weaken the predicates or remap a diagnostic output to physical `/cmd_vel`.

The M15 test program does **not** launch Nav2 controller_server, XRSLAM, real camera, depth runtime, or a hardware bridge. It is a focused functional test of sensor intake and zero-output gating; M14 is the distinct MPPI controller test.

## What M15 establishes—and what it does not

Completion means a reproducible ROS software adapter can consume **the same topic/message contracts** expected from real XRSLAM and DepthART, rejects bad data, and gates diagnostic command output through offline fault/recovery regression. This materially advances integration readiness without replacing the actual sensor validation step.

It does **not** show that Leroy's XRSLAM is calibrated, tracks correctly, has a single real TF authority, or that DepthART observations are reliable or collision-complete. The hardware-coded provisional `wheel_velocity_to_pwm_gain: 12.75`, existing bridge software timeout/watchdog, and M14 fictional footprint/inflation (0.650m < 0.721249m circumscribed) are unchanged. No real motor interface has been connected. Physical safety assessment remains separate from these software acceptance claims.
