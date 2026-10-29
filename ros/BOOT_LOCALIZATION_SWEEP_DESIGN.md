# Leroy — fail-closed autonomous localization boot (design proposal)

**Status: OFFLINE FAIL-CLOSED MODEL IMPLEMENTED, NO-GO FOR PHYSICAL MOTION.** The implementation is a pure Python, standard-library diagnostic model and synthetic simulation only. It adds no physical actuator command, motor/MCU access, GPIO operation, ROS publisher, bridge binding, enabled navigation stack, control-loop timing or authority exception. The accepted Stage 3 camera+DepthART/costmap+XRSLAM and Stage 4C recorder remain diagnostic-only. No deployment image, firmware, signed motor ingress, power isolation or hardware manifest is modified.

## Desired user behavior

Leroy should eventually boot unattended, check its sensors and immediate surroundings, perform a short localization movement only if independently allowed, and enter navigation only when localization, command and obstacle-protection evidence is valid. If no safe movement can be established, it must stay still and report the reason rather than attempt an unobserved avoidance maneuver.

A ±90-degree scan is useful for **coverage and feature acquisition**, but is **not a sufficient monocular SLAM initializer**. Leroy's current `m23bv_v3_stage3_xrslam_monocular_diag.yaml` requires at least eight keyframes, 50 matches, 10 units of triangulation parallax and 30 landmarks; *pure yaw about the camera optical center* does not create the translational camera baseline needed to triangulate landmark depths. A robot yaw in place may introduce a small camera translation if the optical center is offset from the yaw center, but its scale and parallax are not verified; it cannot be assumed to satisfy initialization, especially with Leroy's provisional camera-center translation. The active Stage 3 sidecar uses **monocular camera-only** mode, and IMU observations cannot initialize that mode. Mono-IMU deployment additionally requires validated camera–IMU clock phase, extrinsics and noise, which Stage 4C's stationary bag did not establish.

The target is therefore **preflight → safe sweep → check SLAM → optional short translation to generate parallax → check SLAM → navigation**, never move merely to make the status flip to `tracking`.

## Implemented no-device prototype (PR #166)

- `ros/boot_localization_core.py`: deterministic, fail-closed startup state machine. Holds an inhibited default, requires every **fictional/supplied** qualification flag to be explicitly true before producing *diagnostic yaw setpoints* (-90°, +90°, return 0° relative to the initial unwrapped yaw), monitors observation freshness and clearance on every tick, requires consecutive stable heading observations, and latches on all faults without automatic rearm. The result has **no velocities, motor commands, signed tokens or physical enable bits**.
- `ros/boot_localization_core_test.py`: 17 standard-library regression tests: sweeps, tracking/metric distinction, safety-gate defaults, 400 ms stale-frame fault, unknown/occupied footprint, clockwise/heading frame, repeated/backwards clocks, target settling, timeouts, and post-READY fault handling.
- `ros/boot_localization_demo.py`: scripted synthetic trial of the full ±90° sweep, absence of blind translation, and latched obstacle fault; demonstrates explicit blocking of an unqualified physical/default observation.
- `.github/workflows/leroy-boot-localization-offline.yml`: device-free CI; no ROS, MCU or motor interface.

Reproduce from `ros/` (offline, no dependencies beyond Python 3.11+):

```bash
python3 -m unittest -v boot_localization_core_test
python3 boot_localization_demo.py
```

Expected demonstration markers:
```text
BOOT_UNQUALIFIED=FAULT_LATCHED:INDEPENDENT_STOP_UNPROVEN
BOOT_SYNTHETIC_SWEEP=PASS_NEEDS_PARALLAX
BOOT_SYNTHETIC_OBSTACLE=PASS_LATCHED
BOOT_MODEL=PASS_NO_ROS_NO_DEVICES_NO_ACTUATION
```

**Explicit limitation:** All positive qualification flags in the demonstration are synthetic fixtures. The active Stage 3 costmap cannot produce measured full-rotation obstacle clearance; no code currently connects real trustable sensor evidence or closed-loop motor output to this model. Tests and CI prove software decision behavior under invented inputs, **not physical collision avoidance, real actuation, or autonomous boot readiness**. The numerical freshness, dwell and waypoint limits are for synthetic test coverage only, not validated robot motion limits.

## Live Stage 3 read-only integration (implemented)

The **shortest next live step** is one bounded hardware-input observation
using the already running `leroy-live-costmap` and `leroy-live-xrslam`
containers. It uses no MCU connection, camera remapping, new DepthART
instance or synthetic substitute for actual sensor messages.

```bash
cd /home/kevin/Code/leroy-boot-sweep/ros
git fetch origin design/boot-localization-sweep
git switch --detach origin/design/boot-localization-sweep
bash ./live_boot_readiness.sh
```

The launcher verifies the parent is a running rootless, isolated Stage 3
network/IPC/PID container, then starts a **temporary read-only,
no-device, no-publisher** sidecar in the same DDS graph (domain 170).
It binds only `live_boot_readiness.py` and
`boot_localization_core.py` read-only into the existing locally built
`localhost/leroy-ros:stage4c-capture-offline` image. No image rebuild,
serial open, rosbag writing, motor GPIO, controller, odometry or TF
publication is needed.

For 12 seconds the observer subscribes to actual
`/camera/image_rect`, `/camera/camera_info`,
`/leroy_nav2/diagnostics/raw_depth_points`,
`/xrslam/tracking_status` and `/xrslam/localization_valid`.
It reports publisher identity, callback counts/ages, camera-info
timestamp correspondence, frame consistency and the raw XRSLAM state
as `LIVE_BOOT_OBSERVATION_JSON=...`.
`LIVE_BOOT_SENSORS=PASS_READ_ONLY` means **only** that selected
real input paths were visible and plausibly active, not that any
obstacle classification, all-around clearance, stopping or metric SLAM
requirements were met. A nonzero exit or
`BLOCKED_INPUTS_INCOMPLETE` indicates a live sensor wiring issue.

The real readiness model is evaluated with **all unproven physical
motion-release flags left false**, and no yaw estimate is invented.
Expect `LIVE_BOOT_MOTION=BLOCKED_UNCOMMISSIONED`; that is the
correct result for today's robot. The output is a diagnostic JSON
line printed to stdout, not a ROS command or a credential.
The overall test exits successfully only on a complete read-only
input observation.

The live source has a narrow host-level no-device guard regression:
`python3 -m unittest -v live_boot_readiness_test`.
There is **no extra 75s MCU recording** or repeat camera FPS test.

## Current repository constraints (block autonomous motion)

1. `ros/src/leroy_nav2_perception/config/live_diagnostic_costmap.yaml` marks unclassified depth surfaces (including floor), has provisional camera TF and a placeholder 1 m square footprint, leaves unknown space unknown, does not clear free space and has no source freshness enforcement. It cannot prove the rotation swept area or a translation corridor is collision-free.
2. Only a forward-facing camera is verified. A robot turning with an extended body or arm sweeps a disk around the yaw center; objects alongside or behind may be missed by the front camera **before the turn starts**. The actual center of rotation, current manipulator envelope and full footprint must be measured and coverage independently established. An in-place turn is not inherently collision-free.
3. M12 describes unresolved physical safe-state, command ownership, footprint/stop budget, real odometry and perception acceptance. M14 gates only **diagnostic** Nav2 proposals; no production-ready authority path connects it to the real bridge. Motor deployment configuration remains disabled. Stage 3 has deliberately **no** serial mapping, odometry or drive authority.
4. A yaw target of ±90° must be **closed-loop**, based on a trusted yaw estimate and verified wheel-odometry/IMU source with measured error and stop response, not a timed twist or open-loop PWM. Raw IMU/MCU metadata alone is not physical yaw calibration. No calibrated movement speed, acceleration, settling-time, braking distance or stale-frame threshold is implied by this design.
5. Stage 4C recorded 919 rectified images in 65 s (≈14 Hz), including gaps up to 400 ms, with 3250/3250 matched 50 Hz IMU and metadata. 15 Hz may be suitable for visual tracking, but sensor freshness and stopping bounds must be assessed at worst-case latency, not the median.

## Proposed startup supervisor (functional state machine, no implementation yet)

| State | Non-actuating condition / intended transition | Failure behavior |
| --- | --- | --- |
| `INHIBITED` | Default on boot/reboot, unverified process state, E-stop change, loss of source, safety fault | Inhibit drive via an *independently proven* path; require a separate authorized rearm |
| `PREFLIGHT` | Verify camera/IMU timestamps, TF and odom owner, command owner/authenticity, physically measured footprint, validated obstacle classifier and complete swept-area coverage, protective stop and watchdog | Remain still and publish fault, never try to drive around an unknown |
| `CHECK_INITIALIZATION` | See whether trusted SLAM status is already tracking with valid metric pose/TF; no motion if ready | Continue only if scan is permitted, else `INHIBITED` |
| `SWEEP` | **Candidate** yaw waypoints relative to the boot heading: `0° → −90° → +90° → 0°`. This is 360° of *cumulative yaw travel*; optimize later for minimal safe movement. Before and continuously during each segment, check a measured rotation-swept footprint plus stop margin for known clearance. Use closed-loop yaw and bounded stop. | Stale/unknown surroundings, occupied footprint, unexpected translation, lost odom/TF, sensor latency fault, overshoot or any stop input: latch inhibit |
| `RECHECK_SLAM` | Stop and settle. Inspect actual XRSLAM tracking and pose validity (not solely frame counts). Distinguish native feature tracking from validated metric localization. | If insufficient parallax, never declare localization initialized |
| `TRANSLATION_PENDING` | A **separate, explicitly qualified** short lateral/forward-back motion could introduce translational parallax after proving a surveyed free corridor, measured wheel/IMU odometry, safe reverse path and stop envelope. Specific distance and speed cannot be chosen from current evidence. | If no independent obstacle coverage/odom, stay still and request assistance; do not blind-translate |
| `READY` | Enter navigation only after physically validated, fresh metric localization, correct TF pose direction and one exclusive drive authority; prove continuous collision monitor and independent stop path. | Any loss latched to inhibited; no automatic restart/rearm |

`SWEEP` should be optional; when the initial environment has too few features, insufficient obstacle coverage or no space to rotate, do not issue any movement. There must be a global bounded startup deadline and no unbounded autonomous retry loops.

## Recommended architecture

```
diagnostic localization startup supervisor
    (startup state, operator report, bounded movement request)
          |
          v
exclusive authenticated intent arbiter + application inhibit
          |
          v
validated local obstacle/footprint/freshness gate
          |
          v
real bridge command protocol with independently proven hardware stop
          |
          v
MCU + motor drives

camera / IMU / wheel encoders -> validated local odometry + SLAM
depth/range/obstacle sensors -> validated swept-envelope checker
independent E-stop/cutoff -> independent physical stop
```

A costmap, raw pointcloud or Nav2 `Collision Monitor` is not a certified physical stop path. Nav2's Jazzy guidance expressly notes that its CPU-based Collision Monitor does not provide hard-real-time safety certification. A new startup supervisor must not publish raw `/cmd_vel`, override signed motor ingress, bypass watchdogs, initialize dynamic TF, or flip `hardware.disabled.yaml`.

## Staged implementation / acceptance

1. **Source-only controller model** (first implementation): pure deterministic state transitions, no ROS or devices; force `INHIBITED` until all independently supplied prerequisites pass. Verify no actuation and no implicit rearm. Test no localization, yaw sweep exact targets, no parallax, stale rect images, unknown costmap, obstacle approach from blind side, source duplication, gyro discontinuity, lost odom, restart, abrupt STOP and simulated controller overshoot.
2. **Isolated synthetic ROS fixture:** use simulated sensors/odometry/obstacles; keep all motion topics remapped to diagnostic-only, unconsumed sinks, with positive/negative audit assertions on actual chosen commands and freshness. Test failure under load and message delays (including 400 ms image gaps).
3. **Physical commissioning of perception + stop path (separate milestone):** measure actual footprint and swept envelope, calibrated obstacles/floor segmentation, clearance/blind spots, closed-loop yaw error, wheel signs/encoder completeness, stopping times, fault-induced inhibition and operator acceptance. No assumption that any scan is collision-free.
4. **Supervised bounded movement trial:** only after 1–3 have documented acceptance; default startup still inhibited, independently released for this approved session. Demonstrate stopping on sensor fault and correct tracking feedback before considering unattended startup.
5. **Unattended startup release:** only after real-world acceptance of collision avoidance, localization and exclusive motor authority; version and monitor every acceptance gate. Do not release for Nav2 without validated metric odometry and recovery behavior.

## Explicit non-goals

- No actuator power-on, autonomous sweep, IMU capture, STM32 serial access, safety gate bypass or current hardware operation.
- No claim pure rotation yields monocular depth/scale.
- No fabricated stop distance, valid camera–IMU extrinsic, camera exposure phase, speed limit, footprint or physical motor safety.
- No SLAM map/odometry publication based solely on camera image rate or stationary 65-second bag.

## References

- `ros/m23bv_v3_stage3_xrslam_monocular_diag.yaml` and `ros/src/leroy_xrslam/README.md`: initialization and scale limitations.
- `ros/src/leroy_nav2_perception/config/live_diagnostic_costmap.yaml`: diagnostic-only cloud and provisional footprint.
- `ros/src/leroy_nav2_integration/M12_HARDWARE_READINESS_NO_GO.md`, `M13_AUTHORITY_AND_STOP_DESIGN.md`, `M14_ENFORCING_GATE_AUDIT.md`: gap register and diagnostic gate boundary.
- Nav2 Jazzy Collision Monitor: https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/
