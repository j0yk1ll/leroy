# M23BV V3 Stage 3 — full-stack diagnostic integration (draft)

**Status: source-only diagnostic candidate.** Stack parent: PR #153 at
`93a399fe0b9b0572d389d4c5139f092f4423ceb1`. Does not modify,
merge, or replace any accepted Stage 1/2A/2B source or branches.

## Goal and test-first approach

Bring the **accepted real USB V3 camera + STM32F401 IMU** together with
**XRSLAM, DepthART, and Nav2** and debug actual runtime failures one at a
time. Do not require a broad new synthetic test campaign before exploring
the real integration graph.

Initial full-stack testing must be **diagnostic only**, without wheel or
manipulator power, bridge actuation, Nav2 motor commands, autonomous goals,
or validated-metric-odometry claims.

## Discovered incompatibilities

1. The accepted V3 sensor image is pinned to
   `f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c`.
   It was not built to contain all optional XRSLAM and Nav2 software.
   Independent `Dockerfile.xrslam` and `Dockerfile.nav2` produce
   separate runtime families. Do not assume one image contains everything.
2. `m20_sensor_only_run.sh` and the Stage 2B physical harness deliberately
   reject odometry, `leroy_xrslam`, `controller_server`, and other Nav2
   graph participants. **Never weaken these accepted exclusions.**
3. `leroy_bringup/launch/robot.launch.py` includes the hardware MCU
   bridge and a different camera/rectifier launch; it is **not** a safe
   shortcut for V3 full-stack testing.
4. XRSLAM `monocular_imu` requires a finite, measured
   `camera_imu_offset_sec` before a calibrated deployment. The M20
   relative clock-rate estimate (-41.95 ppm) does **not** supply that
   offset. Provisional, explicitly labelled offset values are permitted
   only for a diagnostic initialization investigation, never to mark
   localization or pose direction validated.
5. The current URDF camera/IMU extrinsics are **provisional**.
   `pose_convention_validated` stays **false** and `enable_map_tf` stays
   **false**. Consequently, odometry and dynamic `odom -> base_link`
   are not enabled by this candidate.
6. Current Nav2 diagnostics use domain 167 and a separate disconnected
   graph. Any later live-sensor diagnostic integration must be a new
   explicit domain-170 candidate; do not modify existing dry-run guards.
7. DepthART depth estimates and preliminary Nav2 depth-to-obstacle
   adapter are not verified obstacle coverage. Keep
   `publish_obstacles=false`,
   `extrinsics_validated=false`, and
   `ground_reference_validated=false`. No obstacle-clearance claims.
8. Existing Nav2 commands must remain remapped to the **unconsumed**
   `/leroy_nav2/diagnostics/proposed_cmd_vel` topic. An inhibited
   diagnostic output is not a physical emergency stop.

## First action: no-device inventory

On the operator's Arch host, with no changes to running containers:

```bash
cd ros
bash ./m23bv_v3_stage3_runtime_inventory.sh
```

This source-only tool checks the exact accepted V3 image ID, checks the
presence of the legacy XRSLAM and Nav2 images, and uses a rootless
`podman run --network none` with no `--device` or bind mounts to
inspect installed ROS package prefixes, XRSLAM native shared library
linkage, and the DepthART checkpoint. Each inspected container is a
read-only, isolated, inert Bash process. No ROS nodes or MCU serial port
are opened. Missing **optional** images are reported as `ABSENT`
without failing the inventory: that is an actionable packaging gap.

**Not yet executed on the operator's host.** The helper does not build
images, drive sensors, record messages, connect DDS domains, or exercise
XRSLAM/Nav2. Inventory PASS means only that source and image information
was obtained in no-device conditions; it is not full-stack acceptance.

## Next narrow test: DDS between all three existing images

The operator inventory returned:

- V3 sensor image matches the exact accepted pinned ID and contains the
  real USB camera, reliable rectifier, DepthART package and checkpoint.
- Existing XRSLAM image ID:
  `20f9cf399ef331dda4aca093182787f8d622aa297b2d8fbcf730541a6f1cd43a`.
  The native XRSLAM linkage is resolved.
- Existing Nav2 image ID:
  `89b3733f7aa281b6083d9078a11fe10e2f4f096f2aca5511a108d4873a3cc7c4`.
  Controller, planner and BT navigator packages exist.
- All three probes verified no physical devices exposed.

Try existing images in a single **isolated, network=none namespace**
before attempting an expensive combined-image rebuild. The minimal DDS
smoke uses a non-actuating nonce publisher in the V3 image, and nonce
subscribers first in the XRSLAM and then Nav2 images. Only the exact
`m23bv_v3_stage3_dds_probe.py` file is mounted read-only into each
container; no host directory, camera/serial device, socket, or motor
interface is mounted. This is a communication test, not an XRSLAM or
Nav2 launch.

On the Stage 3 branch:

```bash
cd ros
bash -n ./m23bv_v3_stage3_no_device_dds.sh
python3 -c "import ast,pathlib; ast.parse(pathlib.Path('m23bv_v3_stage3_dds_probe.py').read_text())"
bash ./m23bv_v3_stage3_no_device_dds.sh
```

The script pins all three observed image IDs, uses rootless Podman,
FastDDS, ROS_DOMAIN_ID=170, LOCALHOST discovery, `--network none` for
the parent and namespace joining for both subscribers, a read-only
filesystem and dropped privileges. It attempts bounded cleanup of
its uniquely named publisher container even after failure.

Expected successful markers (NOT YET OBSERVED):
`M23BV_V3_STAGE3_DDS_XRSLAM=RECEIVED_MATCHED_NONCE`,
`M23BV_V3_STAGE3_DDS_NAV2=RECEIVED_MATCHED_NONCE`, and
`M23BV_V3_STAGE3_DDS=PASS_THREE_IMAGE_ISOLATED_TRANSPORT`.

A failed DDS smoke should be investigated as a focused namespace,
transport or RMW issue. **Do not add `--network host`, map hardware,
relax Stage 2B guards, or join the existing robot container to work
around it.** No new physical sensor experiment is authorized by
passing this probe.

## Verified DDS handoff and next actual-application smoke

The operator reported `M23BV_V3_STAGE3_DDS=PASS_THREE_IMAGE_ISOLATED_TRANSPORT`.
Both XRSLAM and Nav2 images received the correctly identified
nonce from the pinned V3 sensor image. All three image pins matched.
This proves isolated ROS 2 communication, **not** camera/IMU processing
or application startup.

A new, bounded, **no-device** smoke now launches actual DepthART,
XRSLAM and Nav2 processes across those three existing images. It never
opens camera or serial devices, never starts the bridge, never enables
metric odometry, and never sends Nav2 goals. It uses:

- Real DepthART ROS node in the V3 sensor image, with no image publisher.
- Real XRSLAM ROS 2 adapter in the XRSLAM image, using a deliberately
  startup-only `monocular` YAML. The device template is intentionally
  incomplete: no CameraInfo/TF is provided so the adapter must remain in
  `waiting_calibration`, `localization_valid=false`, with **zero**
  published odometry messages.
- Real Nav2 `controller_server`, `planner_server`, `bt_navigator`,
  lifecycle manager in the Nav2 image. All lifecycle nodes MUST stay
  `unconfigured`. Controller velocity names are remapped to the
  unconsumed diagnostic topic and the graph audit rejects
  `/cmd_vel`, `/cmd_vel_nav` or an MCU bridge.
- A ROS graph audit which checks exact expected nodes, status, absence
  of emitted metric odometry and unconfigured lifecycle services.
  All containers use parent `--network none` plus isolated IPC joins
  and no hardware-device mappings; all inputs are read-only file mounts.

On the Stage 3 checkout, from `ros/`:

```bash
git pull --ff-only
bash -n ./m23bv_v3_stage3_no_device_apps.sh
python3 - <<'PY'
import ast
from pathlib import Path
for file in ('m23bv_v3_stage3_nav2_smoke.launch.py',
             'm23bv_v3_stage3_app_smoke_audit.py'):
    ast.parse(Path(file).read_text())
print('STAGE3_APP_SOURCE_SYNTAX=PASS')
PY
bash ./m23bv_v3_stage3_no_device_apps.sh
```

On success the audit expects XRSLAM `waiting_calibration`, false
localization, three `unconfigured` Nav2 lifecycle nodes, a discovered
DepthART node, and
`M23BV_V3_STAGE3_APPS=PASS_REAL_APP_STARTUP_NO_DEVICE`.
This is **not yet observed**. If an application crashes, the runner
prints bounded per-container logs for diagnosis and cleans up its own
containers. Do not substitute synthetic IMU/camera measurements, guessed
extrinsics, a fake pose, or Nav2 lifecycle activation to force success.

After a successful application startup test, the remaining engineering
work is a separately reviewed V3 physical-sensor integration supervisor
that permits XRSLAM/Nav2 diagnostic consumers WITHOUT weakening accepted
Stage 2B source graph guards. Any physical run will require new
operator-performed motor-power isolation, E-stop verification, a fresh
single-use explicit YES, stable USB ID verification, and strict MCU
START/STOP acknowledgement. A diagnostic startup PASS confers no motor
authority, autonomous navigation, calibrated localization or recording
permission.

## Follow-up integration run (separate review and authorization)

After inspecting the inventory output:

1. Make an **additive** derived image from approved, immutable artifacts.
   Explicitly verify native XRSLAM binary, pinned camera driver, V3
   rectifier, DepthART model and Nav2 plugin ABI before sensor exposure.
   Never silently replace the accepted V3 camera/IMU runtime.
2. Design and review a separate V3 sensor supervisor that retains the
   START/STOP acknowledgment protocol and camera identity pins, but
   permits approved diagnostic consumers. The old M20 and Stage 2B
   supervisors stay unchanged.
3. Explicitly authorize each physical session. Verify the two motor
   rails disconnected, robot immobilized, E-stop functional, a present
   operator with immediate power-disconnect ability, correct camera
   physical USB path and MCU by-id, rootless device isolation, and one
   fresh uppercase `YES`. Opening the MCU CDC port/DTR can reset
   motor-capable firmware. The approved MCU commands remain only
   `set_imu_stream_hz 50\\r\\n` and
   `set_imu_stream_hz 0\\r\\n`; a missed STOP ACK is an unknown
   streaming state, not grounds for blind retry.
4. Start V3 camera/IMU and instrument XRSLAM tracking/initialization,
   TF graph, DepthART timing/geometry, Nav2 costmap/lifecycle and
   diagnostic-only MPPI proposals. No recording or sensor-rig motion
   without separate opt-in.
5. Fix the first observed failure in a small new commit on this
   **Stage 3** branch. Rerun the minimal relevant diagnostic.
   Do not bypass sensor freshness, pose validation, TF ownership,
   obstacle or physical actuation gates to force a green status.

## Evidence required before claiming a full-stack test

- Exact SHA and image IDs; actual operator-host stdout/stderr.
- Camera/IMU per-topic message counts, rates, and timestamp health.
- XRSLAM adapter/library launch outcome and tracking state.
- DepthART inference outcome and whether any real valid depth appears.
- Nav2 lifecycle state and remapped-only proposal endpoints.
- Explicit graph proof of **no** MCU bridge, physical `/cmd_vel`
  connection, or competing odom/TF authority.
- Clean shutdown, both MCU ACKs, and any terminal fault.

Current result: **NOT RUN**. No full-stack execution, localization
accuracy, calibrated extrinsics, calibrated temporal offset,
physical obstacle validity, or motion safety has been established.
