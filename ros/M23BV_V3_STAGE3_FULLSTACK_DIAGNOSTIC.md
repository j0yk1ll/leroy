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

## Observed application startup acceptance and next real camera trial

The operator confirmed
`M23BV_V3_STAGE3_APPS=PASS_REAL_APP_STARTUP_NO_DEVICE`
with the three exact pinned image IDs, real DepthART and XRSLAM
processes, unconfigured controller/planner/navigator, zero real
sensor input and no motor access. This is a genuine application
startup PASS, **not** VIO or Nav2 navigation acceptance.

**Next experiment is real camera only at the physical boundary:**
the V3 camera and rectifier run unchanged inside the accepted sensor
image, concurrent with real DepthART; XRSLAM and Nav2 run in
different containers joined exclusively to the isolated
`--network none` / IPC namespace. No MCU serial is mapped,
opened, reset, or commanded. The exact real front-camera port, camera
serial/udev IDs, accepted V3 image, calibration SHA-256, source SHA
and private log directory are verified by the new operator host runner.

The new passive DDS auditor requires real 640x480 RGB8
raw/CameraInfo/rectified messages with 90+ paired stamps,
at least one 640x480 32FC1 DepthART output **bearing a matching
rectified frame stamp**, actual XRSLAM and DepthART subscriptions
to the camera topics, and real Nav2 lifecycle nodes left
unconfigured. It also rejects any unexpected XRSLAM localization
validity, odometry messages, hardware-compatible command topics
and hardware bridge.

**XRSLAM is deliberately configured only for `monocular`
`waiting_calibration`, with no static TF broadcaster and an
explicitly incomplete device stub.** This checks actual ROS
image-topic handoff and DepthART inference while keeping XRSLAM
metric pose/TF gated; it does **not** exercise XRSLAM frame
submission or initialize a real visual-inertial estimator.
A separate follow-up needs real sensor calibration and a reviewed
XRSLAM template before the VIO initialization test.

### Required source-only preflight (no camera or MCU access)

On the up-to-date Stage 3 branch from `ros/`:

```bash
git pull --ff-only
bash -n m23bv_v3_stage3_physical_camera_host.sh
python3 -B - <<'PY'
import ast
from pathlib import Path
for name in (
    'm23bv_v3_stage3_physical_camera_supervisor.py',
    'm23bv_v3_stage3_live_camera_audit.py',
    'm23bv_v3_stage3_nav2_smoke.launch.py',
):
    ast.parse(Path(name).read_text(), filename=name)
    print('STAGE3_REAL_CAMERA_AST_PASS=' + name)
PY
```

**This candidate has not yet passed operator-run preflight or
physical runtime.** Inspect the source and safety boundary prior to
a trial. The prior Stage 2B approval does not authorize Stage 3.

### Operator-owned real-camera test: fresh authorization required

Only when both motor-power rails are PHYSICALLY disconnected,
chassis secured, E-stop tested, verified front camera visually
identified on USB port 3.4.2, calibration provenance confirmed,
and an operator is present to disconnect power:

```bash
# Review the exact visual stream FIRST, independently.
# Enter the actual recently verified /dev/videoN (do not blindly
# copy a historical /dev/video8 or select by-id: both Wed Camera
# units share their USB serial).
export M23BV_STAGE3_CAMERA_VISUALLY_VERIFIED_NODE=/dev/videoN
# Set an EXISTING owner-private mode-0700 log directory.
export M23BV_LOG_DIR=/absolute/private/log-directory
bash ./m23bv_v3_stage3_physical_camera_host.sh
```

The script itself requires an interactive uppercase `YES`
once per attempt and rechecks the stable camera identity after
consent. It launches only the pinned camera video node inside
Podman, with `--network none`, no MCU device, read-only
rootfs, dropped capabilities, and read-only source/calibration
mounts. It stops all sidecars and checks camera and DepthART
shutdown, retaining only private application logs and a
one-time approval record. **No recording of sensor frames**
and no motion commands are allowed.

If source preflight fails, do not invoke the physical runner.
If the real trial fails, supply the error and private log
extract for a targeted fix. A successful camera-only trial does
not authorize opening the motor-capable STM32F401, fabricating a
temporal offset, declaring physical obstacle safety, publishing
metric odometry, or running Nav2 actions.

## Operator-observed real-camera acceptance and real XRSLAM frame test

The operator reported a successful supervised Stage 3 camera-only trial:

```text
M23BV_STAGE3_REAL_CAMERA_AUDIT=PASS
M23BV_STAGE3_REAL_CAMERA_READY=PASS
M23BV_STAGE3_REAL_CAMERA_SHUTDOWN=PASS
STAGE3_LIVE_CAMERA=PASS_DIAGNOSTICS_ONLY
```

Private log: `stage3_camera_vFVFv85b.log`; SHA-256
`dcbd362a9b7389ed75313a7f4368d7e894d50c1c5ff6d5551a6f443376849a23`.
The log contents were not available for independent inspection in this
review; do not claim its per-stream counts or latency measurements.

**The next candidate is additive**, leaving that accepted operator
camera-only runner unchanged:

- `m23bv_v3_stage3_xrslam_monocular_diag.yaml` comes from
  pinned `j0yk1ll/xrslam` `53adcdaf...` EuRoC SLAM tuning, with
  `sensor.mode=monocular`, `initializer.refine_imu=false`.
- `m23bv_v3_stage3_xrslam_euroc_template_UNCALIBRATED.yaml`
  contains the pinned EuRoC sensor template with its nonstandard
  OpenCV YAML directive removed for YAML-CPP parsing. Its
  **noise values are not Leroy measurements**. The existing adapter
  overwrites camera intrinsics using actual rectified CameraInfo,
  and extrinsic q_bc/p_bc using provisional URDF static TF.
- `m23bv_v3_stage3_xrslam_monocular_diag.launch.py` starts the real
  Leroy robot_state_publisher and XRSLAM adapter. Camera frames should
  be submitted to the native `libxrslam.so` using actual V3 timestamps.
  **Metric odometry and map TF remain hard-disabled**. The camera/IMU
  transform remains provisional and is never accepted as a real
  calibrated VIO deployment.
- `m23bv_v3_stage3_xrslam_monocular_audit.py` checks actual camera
  image/depth timestamps, static TF authority, XRSLAM status
  `initializing` or `pose_convention_unverified`, Nav2 lifecycle
  unconfigured, `/odom` message count zero, false localization and
  no `/cmd_vel`/bridge.
- `m23bv_v3_stage3_xrslam_monocular_host.sh` uses the same pinned
  V3 camera-only hardware isolation and fresh per-attempt physical
  consent, and additionally requires the XRSLAM C++ log message
  confirming **native camera frame submission**. The inherited
  Stage 3 camera-only supervisor still owns camera/depth shutdown.

This is the first **actual XRSLAM algorithm-ingress** experiment.
A stationary camera can remain at `initializing` for lack of
parallax, so do not force motion or fabricate input to obtain tracking
success. The target is valid XRSLAM native frame submission, not
calibrated localization or autonomous Nav2 navigation.

### Source-only preflight: run before touching devices

On the current Stage 3 branch, from `ros/`:

```bash
git pull --ff-only
bash -n m23bv_v3_stage3_xrslam_monocular_host.sh
python3 -B - <<'PY'
import ast
from pathlib import Path
for name in ('m23bv_v3_stage3_xrslam_monocular_diag.launch.py',
             'm23bv_v3_stage3_xrslam_monocular_audit.py'):
    ast.parse(Path(name).read_text(), filename=name)
    print("STAGE3_MONOCULAR_SOURCE_PASS=" + name)
PY
```

Run the **no-device** XRSLAM image dependency/template check:

```bash
podman run --rm --pull=never --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,size=64m \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_xrslam_monocular_diag.yaml,dst=/opt/leroy_stage3_slam.yaml,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_xrslam_euroc_template_UNCALIBRATED.yaml,dst=/opt/leroy_stage3_device.yaml,readonly" \
  --entrypoint /bin/bash localhost/leroy-ros:xrslam -c '
    set -e
    . /opt/ros/jazzy/setup.sh
    . /leroy_ws/install/setup.sh
    test ! -e /dev/video2 && test ! -e /dev/ttyACM0
    ros2 pkg prefix leroy_description >/dev/null
    ros2 pkg prefix leroy_xrslam >/dev/null
    python3 - <<"PY"
import yaml
from pathlib import Path
s = yaml.safe_load(Path("/opt/leroy_stage3_slam.yaml").read_text())
d = yaml.safe_load(Path("/opt/leroy_stage3_device.yaml").read_text())
assert s["sensor"]["mode"] == "monocular"
assert s["initializer"]["refine_imu"] is False
assert {"imu", "cam0"} <= set(d)
assert all(k in d["cam0"] for k in ("intrinsics", "extrinsic", "noise"))
print("STAGE3_MONOCULAR_NO_DEVICE_ASSETS=PASS")
PY
  '
```

No physical camera run has been performed for **this new
monocular native-frame candidate**. Do not launch it without review
of the sources, motor-power rails physically disconnected,
secured chassis, tested E-stop, supervised emergency power disconnect,
fresh visual camera identification and calibration attestation, an
existing owner-private `M23BV_LOG_DIR` and a new uppercase
`YES` for **this attempt**. No MCU serial device is mounted.

Once those requirements are met, the proposed physical invocation
(on the Stage 3 branch) is:

```bash
export M23BV_STAGE3_CAMERA_VISUALLY_VERIFIED_NODE=/dev/videoN
export M23BV_LOG_DIR=/absolute/owner-private/0700-log-directory
bash ./m23bv_v3_stage3_xrslam_monocular_host.sh
```

The node value must be replaced with the *newly visually verified*
physical front-camera stream, not a historical device index.
Supply the bounded terminal verdict and private log excerpts on
failure. The next step after observing a real native-frame result
will be testing the IMU intake with a **separate physical operator
authorization** and strictly measured/labelled timing. Do not
claim validated metric odometry, obstacle authority, Nav2 drive
readiness or a released actuator path.

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
