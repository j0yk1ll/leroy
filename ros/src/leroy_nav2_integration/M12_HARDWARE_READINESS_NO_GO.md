# M12 — Hardware-readiness NO-GO audit, not a motor enablement milestone

**Disposition: NOT AUTHORIZED FOR LIVE ROBOT MOTION.** M1–M11
are isolated, synthetic-only diagnostic milestones. M12 creates a
traceable offline list of missing evidence and commissioning gates;
it does **not** remove any gate, initialize a motor, start ROS,
issue FollowPath on a physical graph, enable `/cmd_vel`, replace the
hardware stack, authorize supervised low-duty driving or certify safety.

The standalone `m12_no_go_audit` executable is ROS-independent
Python (no publishers, subscribers, services, actions, shell/device
operations or ROS graph access). It prints a fixed **NO-GO**
disposition plus 11 BLOCKED / UNVERIFIED evidence gates. It has no
argument, parameter, YAML flag or approval path that emits a GO
decision. A PASS means only *the offline register was generated*.

## Why M11 did not prove physical safety

The real MPPI controller in the optional *disconnected* synthetic
fixture emitted nonzero bounded diagnostic Twist messages even while
the M7 shadow classifier labeled fake depth/odometry stale:

| M11 isolated observation | Timed fake depth | Timed fake odom |
| --- | ---: | ---: |
| Five-second scheduled blackout | yes | yes |
| Nonzero Twist receipts inside full blackout | 50 | 49 |
| Nonzero Twist receipts after first M7 veto, before blackout ended | 39 | 35 |
| First veto after schedule start | 1.0983 s | 1.4921 s |
| First veto after recorded pre-request sensor receipt | 1.4987 s | 1.5040 s |

The messages were not sent to motors, and synthetic odometry does not
follow MPPI control. `geometry_msgs/Twist` is unstamped; a message
*received* after veto need not have been *computed* after veto.
This is nevertheless decisive evidence that **the independent M7
shadow label does not gate the controller's diagnostic output**.

No physical stopping-time budget can be concluded from either
single-trial latency. The budget would also need measured command
inhibition time, actuator/drive response, deceleration, traction and
an operating envelope, plus appropriate uncertainty and foreseeable
faults. The synthetic 0.15m/s / 0.3rad/s MPPI maxima are NOT
physically validated wheel-speed or stopping-distance limits.

## Existing real-world code controls versus missing proof

The separate `leroy_bridge` is **not part of the optional image**.
It already has software checks: finite-command validation, a command
timeout of 0.25s, keepalives at 0.10s, a configured 500ms MCU
watchdog, provisional `wheel_velocity_to_pwm_gain: 12.75`
PWM conversion and an independent code-level PWM-duty cap of 64. These are present in source/config, but **not
evidence of independent, verified mechanical stopping** or a
safety-rated E-stop. The checked-in bridge YAML opts into provisional
motor PWM conversion for prior *supervised low-duty* trials; the
bridge's software defaults may be different. Do not infer that every
deployed image has output disabled without directly inspecting the
actual deployment under an authorized procedure.

The optional Nav2 hardware-policy manifest explicitly retains
`enabled: false`, `bridge_command_output_enabled: false`,
`odometry_validated: false`, `obstacle_coverage_validated: false`,
`footprint_measured: false` and
`operator_acceptance_completed: false`. It is a **policy manifest**,
not a Nav2 ros__parameters file or verified hardware interlock.

The separate XRSLAM integration is opt-in
(`use_xrslam` defaults false), and pose-convention validation and
map TF remain gated. Bridge defaults disable its degraded wheel/IMU
odometry. A real /odom and dynamic TF owner must be validated in a
separate controlled program before any Nav2 deployment.

The synthetic costmap still uses a guessed 1m square footprint,
fictional wheel geometry, 4x4 fake camera points, no free-space
clearing, and a repeatedly observed **0.65m inflation radius less
than the 0.721249m synthetic circumscribed radius**. This is not
a verified physical footprint or obstacle coverage.

## Evidence gates — all remain open

| Gate | Missing required demonstration | Primary source |
| --- | --- | --- |
| G01 Deployment | Explicit separate hardware deployment design; preserve disabled manifest until independent release review | `ros/src/leroy_nav2/config/hardware.disabled.yaml` |
| G02 Physical safe-state | Independent stop chain, verified E-stop and drive behavior through compute/network/power faults | `ros/src/leroy_bridge/README.md` |
| G03 Command ownership | Proven arbitration, fresh-source inhibition, timeout/rearm behavior, and no competing ROS publishers | `ros/src/leroy_nav2_integration/M11_BLACKOUT_WINDOW_METRICS.md` |
| G04 Mechanical model | Measured real wheel sizes, motor signs, load, traction, footprint and stopping envelope; provisional PWM not substituted | `ros/src/leroy_bridge/config/leroy_bridge.yaml` |
| G05 Localization | Metric and TF ownership, IMU/camera phase and extrinsics, scale, localization failure handling | `ros/src/leroy_bringup/README.md` |
| G06 Perception | Real depth coverage, latency and acquisition stamps, blind spots, invalid-surface and lighting cases | `ros/src/leroy_nav2_perception/README.md` |
| G07 Collision geometry | Correct measured footprint, inflation, clearance margins and observed obstacle response | `ros/src/leroy_nav2_integration/config/m5_controller.yaml` |
| G08 Time/distance budget | Risk-derived end-to-end detection–inhibition–mechanical-stop bound under load and foreseeable environment | M11 evidence |
| G09 Fault coverage | Independent fault matrix: stalled process, lost serial, stale TF, clock jump, stale command, restart, power fault | M10/M11 synthetic limits |
| G10 Human/environment | Intended use and exposure assessment, restricted test environment, supervision and applicable requirements | `ros/src/leroy_nav2/README.md` |
| G11 Engineering acceptance | Traceable versions, physical results, independent engineering/operational approval | M11 evidence |

This is a gap register, **not** a risk assessment declaring
acceptable risks, a safety function design specification, a
regulatory approval or a completed test matrix. A responsible
engineer must assess which standards apply to the actual robot
and intended environment. For example, ISO 12100 provides a
risk-assessment framework; ISO 13482 has a specific scope for
personal-care robots and is not automatically applicable to every
mobile platform. Do not assume a Nav2 collision-monitor node can
replace independent physical stopping: Nav2's own Jazzy documentation
states its CPU-based collision monitor provides **no hard real-time
safety certification**:
https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/

## Distinct pathways; none authorized by M12 alone

1. **Design review (no robot connection).** Confirm intended
   environment and mechanical envelope, enumerate hazards, decide
   which independent physical protections and validation methods
   are appropriate. Retain deployed safety function ownership
   outside ROS/MPPI.
2. **Protected unpowered / motors-inhibited inspection.** Under
   separately authorized electrical/mechanical procedures,
   inspect the physical E-stop and drive-safe-state architecture;
   establish a documented independent test plan.
3. **Passive sensor/TF assessment only after explicit approval.**
   This would require a distinct scope, verified ROS graph
   separation and guaranteed motor inhibition before connecting
   any passive real sensor. The M12 implementation does not
   start it or provide commands for it.
4. **Controlled physical commissioning only after all critical
   release gates have independently justified closure.** A
   defined test environment, responsible supervision, mechanical
   stopping bounds and independent safety measures are
   prerequisites—not tasks M12 performs.

There is no number of additional synthetic PASS logs that by itself
proves the independent physical safety function. The gates are
not sequentially waived by merging source code.

## Offline no-go audit acceptance

From repository root on the isolated test branch:

```bash
git fetch origin
git switch docs/nav2-m12-live-release-no-go-audit
git pull --ff-only origin docs/nav2-m12-live-release-no-go-audit
cd ros
podman build --no-cache -f Dockerfile.nav2 -t leroy-nav2:m12-no-go .
```

Build must pass existing optional-package Colcon/pytest and the
registered `m12_no_go_audit` executable. No production
`ros/Dockerfile`, `leroy_bridge` or running `leroy-ros`
should be touched.

Run the non-ROS report in a rootless, disconnected, no-device
test container:

```bash
podman run --rm \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:m12-no-go bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 run leroy_nav2_integration m12_no_go_audit
  '
```

Expected:

```text
M12 READINESS: NO_GO_LIVE_ROBOT_NOT_AUTHORIZED
M12 SCOPE: DISCONNECTED_SYNTHETIC_READINESS_REVIEW_ONLY
M12 GATE: G01 BLOCKED Deployment policy and graph separation
...
M12 GATE: G11 UNVERIFIED Reproducible physical evidence and independent review
PASS: M12 offline NO-GO evidence inventory generated; live robot motion is NOT authorized and no motor path exists
```

The ROS package executable launcher above only invokes pure Python.
It starts no ROS node or action and publishes no message. The
`PASS` line **does not** mean the robot is safe or ready to move.
There is no `GO`, `--arm`, permit flag, telemetry hookup or
motor-compatible topic in this PR.

M12 acceptance requires a successful image/Colcon/pytest result,
the exact NO-GO report with all 11 blockers, and confirmation that the
unrelated host working tree, robot image and running hardware container
were untouched. It requires **no two-terminal controller fixture or
FollowPath action**. Do not modify the hardware disabled manifest to
make tests pass.

## Safety nonclaims

No deployment or hardware behavior has been validated by M12.
Independent E-stop **existence/function is unknown from M1–M11
evidence**, even though bridge docs require a verified device for
supervised low-duty testing. A drive command timeout and MCU
watchdog settings are software/firmware intentions requiring
physical confirmation, not a certified safety channel. Synthetic
fixture cleanup and controller `finalized [4]` cannot establish
power removal, standstill, or lack of drivetrain run-on. Source names
in DDS are metadata, not secure authorization. No autonomous
navigation approval, physical motion experiment, real FollowPath,
hardware test or M13 is initiated by this milestone.
