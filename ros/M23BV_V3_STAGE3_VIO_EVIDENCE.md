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
