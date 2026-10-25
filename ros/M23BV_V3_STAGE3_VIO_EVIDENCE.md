# Stage 3 VIO — accepted operator diagnostics and next blockers

**Evidence scope:** Values below come from the operator's text output of the
read-only `m23bv_v3_stage3_vio_log_report.py` on October 25, 2026.
The private log itself was not transferred or independently examined in
this review. The report validated the operator-supplied log SHA-256.

Private run identifier: `stage3_vio_QiKefpTz.log`.

SHA256: `5f3a1129ec679a2c3715514109dd833fdda5cb7cd70699f445ff337456f88e0f`.

## Observed acceptance

| Subsystem / contract | Observation | Interpretation |
| --- | --- | --- |
| Native XRSLAM image ingestion | At least 900 camera frames submitted in `monocular_imu` mode | Real camera frames reached the adapter's native XRSLAM call; **not** evidence of tracking or accurate pose |
| STM32 IMU publisher | 3,500 messages; measured 50.0 Hz | Real IMU source transport passed |
| Independent passive IMU auditor | 300 messages | A subset of the longer publisher session; **not** a 3,200-sample loss figure |
| M18 packet/source contract | `PASS_DIAGNOSTIC_STREAM` | The stream met the script's zero-error/acknowledgment threshold |
| MCU START / STOP | Both `PASS` | The MCU acknowledged the two whitelisted stream-control commands |
| IMU clock mapper | Ready, −28.97 ppm estimated rate error | Relative clock rate estimated, **not** absolute sensor/camera synchronization |
| IMU last mapped age estimate | 1.3 ms | Mapper estimate, **not** independently measured exposure-to-acquisition latency |
| IMU max mapped age estimate | 11.7 ms | Same limitation: estimated age rather than true sensor age |
| Passive real camera auditor | raw 187, CameraInfo 187, rectified 162; 162 matched timestamps | Real paired frames verified in bounded auditor window |
| DepthART | 3 valid depth samples with matched rectified image timestamps | Depth generation/geometry contract passed; throughput/freshness **not yet qualified** |
| XRSLAM tracking state | `initializing` | No observed successful tracking session |
| XRSLAM metric odometry | `NOT_VALIDATED` | Odometry and pose gate remain disabled |
| Nav2 | `UNCONFIGURED` | No planner/controller navigation exercise in this run |
| Motor/recording authority | `NOT_AUTHORIZED` | No driver release, goals, motion or capture permitted |

The XRSLAM native camera frame count and the ROS camera audit counters
come from **different observation scopes**. Likewise, the independent IMU
auditor stopped before the 70-second source session finished; never divide
its 300 receipts by the full-run 3,500 messages to claim packet loss.

## Blocker ranking for the next integration milestone

### 1. XRSLAM estimator has not established tracking

The observed result is `initializing`, not `tracking` or a validated body
pose. The earlier tests intentionally kept the chassis immobilized, which
can deprive visual-inertial initialization of useful parallax/excitation.
The cause is **not yet established**; camera–IMU time offset, extrinsics,
feature geometry, noise tuning and tracking initialization are additional
possibilities. Do not change an initialization threshold or fabricate a
pose to make Nav2 launch.

**Next physical investigation (requires separately approved safety plan):**
a deliberately controlled calibration/excitation session with the camera
and IMU remaining rigidly attached to one another, motor power physically
disconnected and no actuator/bridge process. Review the sensor rig and
how safe motion can be generated before authorizing it; the stationary
VIO approval does **not** authorize sensor-rig or chassis movement.

### 2. Absolute camera–IMU phase and physical extrinsics are unmeasured

The source reports a clock-rate estimate, not the constant absolute
exposure/IMU acquisition phase; the VIO launch used a clearly provisional
`camera_imu_offset_sec=0.0`. The URDF spatial camera–IMU transforms and
nonidentity body-pose/TF convention are also unvalidated.

**Required:** independently measured temporal offset and extrinsics,
then a nonidentity pose-direction check. Maintain
`pose_convention_validated=false` and `enable_map_tf=false` until
independent acceptance evidence exists. Track camera exposure timestamp
semantics explicitly; a mapped MCU age estimate alone cannot establish
temporal calibration.

### 3. DepthART freshness and metric obstacle authority unverified

The short observer saw three depth outputs, but those counts do not
supply a full-session rate/latency distribution. Do not infer a steady
3-per-70-second frame rate (or an accepted collision-safety rate) from
a shorter auditor window. Exact per-frame acquisition/receipt latency
and physical ground/extrinsic geometry still require measurement.

**Required:** real acquisition-to-depth freshness distribution,
validated depth geometry, unknown-space treatment, sensor coverage,
and fail-closed stale-input behavior. Keep the real obstacle adapter
gated (no validated `obstacle_points`) and Nav2 unconfigured.

### 4. End-to-end Nav2 planner/controller operation is still blocked

The existing three-image integration proves real application startup,
ROS communication and input ingestion. The real Nav2 lifecycle did
not configure or activate in this run, and real metric `/odom` and
dynamic `odom -> base_link` were intentionally absent.

**Required:** valid single-owner metric localization/TF, validated
obstacle freshness/footprint, isolated navigation proposal handling and
independent motion safety before considering any physical navigation
test. A separate *no-device* Nav2 lifecycle/configuration test may
isolate plugin/lifecycle errors, but cannot substitute for measured
robot-localization or obstacle evidence.

## Decision

**ACCEPT:** Stage 3 pinned, isolated camera + real IMU + native XRSLAM VIO
ingress and DepthART + unconfigured Nav2 process integration.

**BLOCKED:** autonomous navigation; valid metric odometry; phase/extrinsic
calibration; demonstrated VIO tracking; collision-safe depth obstacles;
motor control and recording.

Continue failure-driven integration work, but do not repeat accepted
sensor-only transport tests merely to gather more proof. Any physical
calibration/excitation session requires a **new and separately reviewed
operator safety/consent boundary**; a GitHub source commit is not that
authorization.

## Next safe integration test: Nav2 real plugin/lifecycle configuration

The physical VIO test ended cleanly, but actual Nav2 nodes have so far
remained `unconfigured`. While localization/extrinsic calibration is
being planned, we can expose **real Nav2 plugin/configuration failures**
without another hardware session.

A new **independent hardware-free** Stage 3 runner starts the pinned
Nav2 image with its existing **explicitly fictional** `simulation.yaml`
parameters, runs actual controller/planner/BT navigator lifecycle
`Configure` transitions one at a time, verifies each becomes
`inactive`, then performs `Cleanup` for all nodes and checks that
they return to `unconfigured`.

```bash
# Stage 3 branch, from ros/; no USB devices required
git pull --ff-only
bash ./m23bv_v3_stage3_nav2_configure_no_device.sh
```

**Not yet run on the operator's host.** It uses the exact pinned
`localhost/leroy-nav2:jazzy` image ID, rootless Podman,
`--network none`, private IPC/PID, readonly filesystem and no mapped
camera or MCU. Actual Nav2 packages start, but the test NEVER requests
lifecycle `Activate`, Nav2 action goals, synthetic odometry/TF, hardware
velocities or recording. It explicitly rejects any publisher of
`/odom` or `/tf` or actuator-compatible `/cmd_vel` endpoints.
(Inert odom/TF **subscriptions** from unactivated Nav2 nodes are expected
and permitted.) Velocity publisher remappings point solely to the
unconsumed diagnostics namespace.

Expected PASS **if all three real server plugins/configurations are
satisfied**: `STAGE3_NAV2_CONFIGURE=PASS_NO_DEVICE_LIFECYCLE`.
On failure, the bounded plugin/lifecycle error log tells us exactly
which installed Nav2 server/configuration to fix. Do not add fake
odometry or activate a node to mask the failure.

This does not unblock live Nav2 control: the real robot still lacks
validated metric localization, calibration, obstacle freshness,
footprint and independently verified motion protection.

## Nav2 Configure/Cleanup audit PASSED; container shutdown BLOCKED

The operator's first no-device Nav2 lifecycle test after the Jazzy
`AMENT_TRACE_SETUP_FILES` / Bash nounset fix reported:

```text
STAGE3_NAV2_CONFIGURE_controller_server=INACTIVE
STAGE3_NAV2_CONFIGURE_planner_server=INACTIVE
STAGE3_NAV2_CONFIGURE_bt_navigator=INACTIVE
STAGE3_NAV2_CLEANUP_bt_navigator=UNCONFIGURED
STAGE3_NAV2_CLEANUP_planner_server=UNCONFIGURED
STAGE3_NAV2_CLEANUP_controller_server=UNCONFIGURED
STAGE3_NAV2_CONFIGURE=PASS_REAL_SERVERS_CONFIGURE_CLEANUP_NO_ACTIVATION
STAGE3_NAV2_OMIT_ODOM_TF_GOALS=CONFIRMED
STAGE3_NAV2_SENSOR_ACCESS=NONE
STAGE3_NAV2_MOTION_AUTHORIZATION=NONE
STAGE3_NAV2_CONFIGURE=BLOCKED_LIFECYCLE_OR_SHUTDOWN
```

**Interpretation:** The actual Nav2 controller/planner/BT navigator
plugins configured, remained INACTIVE, and all returned to UNCONFIGURED.
But the overall Podman run returned nonzero; the previous wrapper could
block indefinitely while waiting for a background `ros2 launch` process
that might ignore the wrapper's SIGINT. No per-process teardown exit
status was preserved by that wrapper. **Do not claim full supervisor
acceptance or a clean shutdown based only on the in-container audit
PASS.** The precise failure mechanism is unconfirmed.

**Targeted follow-up:** `m23bv_v3_stage3_nav2_configure_supervisor.py`
now spawns `ros2 launch` in a dedicated process session, preserving
normal SIGINT handling, and explicitly waits with a fixed deadline. It
collects the audit status, Nav2 launch exit code, and bounded launch
logs on failure. A timeout or abnormal exit remains a failure; no
attempt to hide an observed shutdown crash. The modified
`m23bv_v3_stage3_nav2_configure_no_device.sh` invokes this
supervisor in the same pinned rootless network-none container.
All lifecycle transitions, hardware prohibitions and motor isolation
are unchanged.

Run from `ros/` on the updated Stage 3 branch (NO DEVICES):

```bash
git pull --ff-only
bash -n ./m23bv_v3_stage3_nav2_configure_no_device.sh
bash ./m23bv_v3_stage3_nav2_configure_no_device.sh
```

Expected **only after observed clean shutdown**:
`STAGE3_NAV2_SHUTDOWN=PASS_GRACEFUL_PROCESS_EXIT`,
`STAGE3_NAV2_SUPERVISOR=PASS_AUDIT_AND_CLEAN_SHUTDOWN`,
`STAGE3_NAV2_CONFIGURE=PASS_NO_DEVICE_LIFECYCLE`.
If instead `BLOCKED_SIGINT_TIMEOUT`, `BLOCKED_NAV2_CHILD_CRASH`
or `BLOCKED_LAUNCH_EXIT` appears, inspect the bounded
`STAGE3_NAV2_LAUNCH_LOG` excerpts. Do **not** declare cleanup
successful by ignoring exit codes or forcing the container to terminate.
This new runner version is untested on the operator's host.


## Final Nav2 no-device lifecycle and shutdown: PASS

The operator's latest pinned-image rootless `--network none` run
confirms that the revised bounded supervisor resolved the earlier
overall test failure. The prior `BLOCKED_LIFECYCLE_OR_SHUTDOWN`
output above is retained as **historical** evidence, not the
current result.

```text
STAGE3_NAV2_CONFIGURE_controller_server=INACTIVE
STAGE3_NAV2_CONFIGURE_planner_server=INACTIVE
STAGE3_NAV2_CONFIGURE_bt_navigator=INACTIVE
STAGE3_NAV2_CLEANUP_bt_navigator=UNCONFIGURED
STAGE3_NAV2_CLEANUP_planner_server=UNCONFIGURED
STAGE3_NAV2_CLEANUP_controller_server=UNCONFIGURED
STAGE3_NAV2_CONFIGURE=PASS_REAL_SERVERS_CONFIGURE_CLEANUP_NO_ACTIVATION
STAGE3_NAV2_AUDIT_RC=0
STAGE3_NAV2_SHUTDOWN_LAUNCH_RC=0
STAGE3_NAV2_SHUTDOWN=PASS_GRACEFUL_PROCESS_EXIT
STAGE3_NAV2_SUPERVISOR=PASS_AUDIT_AND_CLEAN_SHUTDOWN
STAGE3_NAV2_CONFIGURE=PASS_NO_DEVICE_LIFECYCLE
STAGE3_NAV2_REAL_ODOM_TF=NOT_SUPPLIED
STAGE3_NAV2_GOALS_ACTIONS_AND_MOTORS=NOT_AUTHORIZED
```

This **accepts Nav2 Jazzy controller, planner and BT navigator plugin
configuration, inactive lifecycle state, reverse cleanup and graceful
process shutdown** under the pinned, disconnected simulation-parameter
fixture. It does *not* activate or exercise controller/planner actions,
validate real localization, safety, obstacle marking, commands,
footprint geometry, or real-robot Nav2 deployment. No physical device
was accessed.

### Remaining highest-priority real-system blocker: tracking and calibration

The Stage 3 real camera/IMU run entered native XRSLAM
`monocular_imu` successfully, but the observed tracking state was
**`initializing`**. Physical geometry in the accepted
`ros/src/leroy_description/urdf/leroy.urdf.xacro` is explicitly
**PROVISIONAL**:

- IMU `base_link -> imu_link`: xyz `(0.095, -0.095, 0.0)` m,
  rpy `(0,0,0)`; IMU height/alignment not measured.
- Front camera `base_link -> front_camera_link`: xyz `(0,0,0)` m,
  rpy pitch `+0.5235987755982988` rad; position is an unmeasured
  zero placeholder, not a real camera origin.
- The `front_camera_link -> front_camera_optical_frame` optical
  frame convention and legacy alias `camera_optical_frame` are
  explicit in the URDF, but their *physical mounting orientation*
  has not been verified.
- Diagnostic XRSLAM launch still uses `camera_imu_offset_sec=0.0`,
  expressly **not an independently measured exposure-vs-IMU phase**.

**Next decision is a new, separately authorized calibration plan,**
not another camera/MCU transport repeat and not real Nav2 activation.
First confirm surveyed sensor mount geometry (including which rigid
body defines `base_link`) and camera exposure timestamp semantics.
Then choose a safely constrained excitation/visual-inertial
calibration method, only with a fresh operator safety assessment and
approval. Prior stationary tests did **not** authorize moving either
the chassis or the camera/IMU rig. Until evidence closes these gaps
keep `pose_convention_validated=false`, `enable_map_tf=false`,
real Nav2 unconfigured, motor power disconnected and no recording
without its own explicit authorization.

