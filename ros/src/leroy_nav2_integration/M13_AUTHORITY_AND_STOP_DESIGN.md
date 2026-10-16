# M13 — Command authority and independent stopping: OFFLINE design review

**Status: DESIGN ONLY · LIVE ROBOT NO-GO.** This milestone is not a
functioning command mux, motor filter, emergency-stop system, safety
controller, brake, live navigation integration, or permit to move.
There is **no ROS publisher, subscriber, action client, motor bridge
connection, source of physical data, watchdog implementation or
actuator path** in the M13 program. Do not attempt to hook this model
into `/cmd_vel`, connect it to `leroy-ros`, map devices, replace
the hardware image or enable any hardware manifest.

The purpose is to translate M12's open G01/G02/G03/G08/G09 gates into
design-time requirements, a deliberately simplified offline fault
thought experiment, and the evidence a responsible engineer would
need BEFORE considering a separately authorized physical program.

## 1. What the existing repository actually has

| Area | Source evidence | Claim status |
| --- | --- | --- |
| Hardware Nav2 policy | `ros/src/leroy_nav2/config/hardware.disabled.yaml`: `enabled: false`, `bridge_command_output_enabled: false`, `operator_acceptance_completed: false` | Disabled **declaration**; not an independent physical stop |
| Nav2 diagnostic control | M5–M11 controller fixtures remap relative and absolute `cmd_vel` to `/leroy_nav2/diagnostics/proposed_cmd_vel`; ROS graph is isolated | Offline behavior only, no live command authority |
| Physical bridge interface | `leroy_bridge` subscribes to `/cmd_vel` and can emit `set_wheels` over serial | Real hardware-facing path; **never connected to Nav2 by M13** |
| Bridge software timeouts | Config includes `cmd_vel_timeout_s: 0.25`, `watchdog_ms: 500`, `keepalive_period_s: 0.10` | Implemented/configured mechanisms; physical stopping NOT measured here |
| Motor conversion | Bridge YAML uses provisional `wheel_velocity_to_pwm_gain: 12.75` and `max_pwm: 64`; right encoder sign fields remain zero | Not calibrated real speed or deceleration envelope |
| M11 fault observation | 39 depth and 35 odom nonzero diagnostic Twist **receipts after the shadow veto and before the scheduled fake blackout end** | A diagnostic label does not gate MPPI's output |
| Localization/perception | XRSLAM, extrinsics and physical camera coverage remain unvalidated | No real-data navigation readiness |
| Geometry | Fictional square footprint; inflation 0.650m below circumscribed 0.721249m | Unresolved synthetic configuration warning |

The source-known watchdog, PWM cap and timeout must NOT be described
as nonexistent. Equally, a `set_wheels 0 0 0 0` serial request,
controller goal cancellation, Nav2 lifecycle shutdown or MCU watchdog
*setting* cannot by itself establish actual motor torque removal,
physical standstill or an independently verified E-stop. A software
bridge and ROS graph may share the same power, compute, timing and
failure modes.

## 2. Proposed separation of responsibilities — NOT implemented

```text
    AUTONOMOUS PROPOSALS / TELEOP / TEST CLIENTS
                  (untrusted intents)
                            |
                     [design-time]
     single exclusive authority + provenance + age checks
           [not implemented or validated]
                            |
        application command inhibition & arbitration
           [not implemented or validated]
                            |
           REAL HARDWARE BRIDGE (/cmd_vel)
                     [exists separately]
                            |
                    MCU + MOTOR DRIVES
                 [not controlled by M13]

    INDEPENDENT E-STOP / INHIBIT / DRIVE SAFE-STATE
     [physical architecture not verified; must be designed
      and tested without reliance on the ROS/Nav2 path]
```

This diagram is **conceptual only**. No connection line above is an
existing Nav2-to-bridge integration. The physically effective stop
must be designed through risk assessment; cutting drive power is
not automatically safe for every load, mechanism or incline.
Proposed software gating would complement, never replace, that stop.

A future protocol would need robust authority isolation and message
provenance beyond ROS graph node-name metadata: graph publisher
identity is not authentication. `geometry_msgs/Twist` is
**unstamped**, so receipt freshness is not evidence of generation
freshness. Proposal sequencing, command expiry, exclusivity and
authenticated source binding are **requirements to design**, not
features M13 installs.

## 3. Requirements register — OPEN, NOT VALIDATED

All acceptance properties below are provisional **design requirements**
subject to a competent, application-specific risk assessment. There
are deliberately no release/go thresholds or test commands for
actual motors.

| ID | Requirement to be specified and independently verified | M12 gates | Required validation evidence |
| --- | --- | --- | --- |
| **AR-01** | Exactly one exclusive physical command authority; conflict, duplication, unexpected publisher or absent owner cannot retain prior motion intent | G01, G03, G09 | Threat model, graph/dataflow ownership proof, authority arbitration tests |
| **AR-02** | Provenance, replay rejection and acquisition/generation freshness semantics for all command intents; DDS names and unstamped Twist receipt alone do not authenticate | G03, G09 | Protocol architecture review, adversarial source/replay/reordering tests |
| **AR-03** | Missing, invalid, out-of-order or stale depth, odometry, TF, costmap or proposals cause a latched application-level inhibit rather than a hypothetical text label | G03, G05, G06, G09 | Versioned end-to-end fault tests with real data; explicit safe-state acceptance |
| **AR-04** | Controller lifecycle, process restart, ROS partition, serial fault, watchdog expiry and competing operator inputs never silently resume autonomous movement | G03, G09, G10 | Restart, partition, stall, abnormal termination and arbitration matrix |
| **AR-05** | Fault clearing is distinct from supervised manual rearm, with no automatic authorization solely from fresh sensor return | G03, G09, G10 | Reset/rearm human-factors design, held-fault traces and operator signoff |
| **FS-01** | Independent physical E-stop/inhibit path and defined motor drive safe-state, with separation from Nav2, ROS, bridge and common-mode compute faults | G02, G08, G10 | Electrical/mechanical schematics, failure analysis, controlled stop-function verification |
| **FS-02** | Bridge command expiry, MCU watchdog and firmware behavior are measured over loss of command, process, transport, power and malformed data | G02, G08, G09 | Time-stamped stop-output measurements and state evidence; no PWM-to-speed assumption |
| **FS-03** | Stopping and holding safe-state are demonstrated under worst relevant load, speed, surface, traction, slope and obstacle clearance | G02, G04, G08 | Appropriate measured deceleration/distance and uncertainty envelope |
| **FS-04** | Reset after E-stop, clock step, restart or tracking loss cannot re-energize a drivetrain without deliberate authorized procedures | G02, G03, G09, G10 | Fault/recovery/latched-stop tests with witnessed safe-state outcomes |
| **FS-05** | No software, DDS source, controller output or restart can bypass the independent hardware-level protective path | G01, G02, G03, G09 | Independence, common-cause, bypass and single-fault review |
| **IV-01** | Actual odometry scale, wheel/encoder signs, extrinsics, TF ownership and tracking-loss detection are independently measured | G04, G05 | Calibrations, source arbitration, timestamp and error-distribution reports |
| **IV-02** | Physical footprint, depth field of view/blind spots and costmap inflation are validated for intended environment | G06, G07 | Geometric calibration, obstacle/unknown-space and perception fault tests |
| **IV-03** | Bound worst-case detect → application inhibit → drive response → mechanical stop with uncertainties, not just M11's ~1.1–1.5s shadow observation | G02, G08, G09 | Timing budget tied to measured worst-case latency and hazard clearance |
| **IV-04** | Record intended-use hazard analysis, accountable reviewers, evidence versioning and approval before any supervised commissioning | G10, G11 | Signed risk assessment, controlled test plan and independent acceptance record |

*Requirement identifiers are traceability labels only*, not an
implemented safety specification. Independent stopping may need
additional architecture and qualified components, depending on
the hazard analysis and applicable requirements. No ISO compliance
or safety performance level is claimed.

## 4. Fault-response thought experiment (pure Python only)

The `m13_authority_design` executable enumerates nine *fictional*
independent fault cases: unknown physical stop evidence; 0 or 2
command owners; untrusted source; stale odom; stale depth; TF loss;
missing proposal; stale proposal. It returns
`MODEL_WOULD_REJECT_*` strings, never motor commands.

A deterministic fictional trace also demonstrates a **latched**
fault model: start latched → human-review acknowledgment required
→ illustrative 1s M7-derived holdoff → optionally
`MODEL_CONDITIONS_OBSERVED_DIAGNOSTIC_ONLY`. A new fault or
non-monotonic model clock re-latches. The illustrative holdoff is
NOT a physical safe-stopping threshold, stop-distance budget or
recommended deployable configuration. All input flags—including
`independent_stop_evidence=True` and
`proposed_source_authenticated=True` in the fictional baseline—
are arbitrary thought-experiment data, **not real proof**. A
hypothetical positive diagnostic label does not make the design
safe, and the program's release verdict is **always**
`NO_GO_LIVE_ROBOT_NOT_AUTHORIZED`, with M12 G01–G11 still open.

The model deliberately **does not output velocities, authorization
bits, motor commands, ROS messages, emergency-stop electrical
signals or a real command mux**. A future real controller must not
import this model as an interlock. It is a checklist and a negative
regression example, not reusable actuation code.

## 5. Quantitative safety evidence not yet available

No physical stop budget is known. The risk assessment would need a
bound including, at minimum, sensor failure recognition, supervisory
decision, application inhibit propagation, bridge/MCU/drive reaction,
mechanical stopping and uncertainties. The M11 `1.0983s` fake-depth
and `1.4921s` fake-odom times represent **first receipt-side
hypothetical veto onset in single synthetic trials**, not guaranteed
latencies, not independent-stop timing, and not physical braking.

A requirements review should also decide how to handle:
- Loss of all computers or of the serial link during active motion,
  including what happens if the last `set_wheels` packet was nonzero.
- Repeated, delayed, malicious or duplicate command producers, ROS
  partitions and unexpected topic remaps, including stale data after
  restart or recovered network connectivity.
- Real camera blind spots, reflective/glass/near-field regions, stale
  depth, stale TF, or inaccurate pose/metric scale.
- Operator E-stop activation, power interruptions, downhill run-on,
  failure to retain safe-state, and controlled reset after fault.
- Evidence that independent stop still works when the ROS controller
  is wedged, not running, or outputting nonzero proposals.

No synthetic-only PASS can close these independent physical gates.

## 6. Operator validation — NO ROS controller or robot required

At the repo root, preserve unrelated dirty files and never restart
or change the live `leroy-ros` container:

```bash
git fetch origin
git switch feature/nav2-m13-offline-authority-design-review
git pull --ff-only origin feature/nav2-m13-offline-authority-design-review
cd ros
podman build --no-cache -f Dockerfile.nav2 -t leroy-nav2:m13-design .
```

The optional image builds/tests the existing disconnected packages
and validates the M13 console script, with **no additional hardware
dependencies**. Run one Python-only report:

```bash
podman run --rm \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:m13-design bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 run leroy_nav2_integration m13_authority_design
  '
```

Expected output:

```text
M13 DESIGN: NO_GO_LIVE_ROBOT_NOT_AUTHORIZED
M13 SCOPE: OFFLINE_NON_ACTUATING_DESIGN_THOUGHT_EXPERIMENT
M13 CASE: physical_stop_unknown => MODEL_WOULD_REJECT_INDEPENDENT_STOP_UNVERIFIED
M13 CASE: duplicate_command_owner => MODEL_WOULD_REJECT_OWNER_COUNT
...
M13 REARM EXAMPLE: MODEL_WOULD_HOLD_MANUAL_REVIEW -> MODEL_WOULD_HOLD_RECOVERY -> MODEL_CONDITIONS_OBSERVED_DIAGNOSTIC_ONLY
M13 M12-GATES-STILL-OPEN: G01,G02,G03,G04,G05,G06,G07,G08,G09,G10,G11
PASS: M13 offline design thought experiment rejects all 9 fictional fault cases; NO motor permission, hardware safe-stop proof or release gate closure
```

The program does **not create a ROS node**: `ros2 run` here is
just the package's Python executable launcher. Confirm the image
Colcon/pytest build, unchanged hardware manifest, and source
review before marking M13's design-only scope accepted. **No
separate controller fixture, sensor trigger or FollowPath test is
involved**.

## 7. Release disposition

M12 G01–G11 remain unresolved. In particular:
- G01: no approved physical command/dataflow deployment architecture.
- G02: no independently demonstrated physical stop chain.
- G03: no implemented and validated application command gate.
- G08: no measured end-to-end stopping risk budget.
- G09: incomplete real fault matrix and supervised recovery evidence.

The bridge already has software command timeouts and MCU watchdog
configuration, but these are not independent safety certifications.
M13 makes no changes to bridge source/config, ROS live graph,
deployment manifest, or motor behavior.

**Live robot operation: NO-GO.** Review can guide future
engineering only. There is no implied M14 or automatic progression
to passive hardware observation or supervised motion.
