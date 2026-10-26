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

## Observed XRSLAM monocular native-frame PASS and next VIO candidate

The operator reported a successful new real-camera native XRSLAM test:

```text
M23BV_STAGE3_MONOCULAR_AUDIT=PASS_GRAPH_AND_INFERENCE
Submitted 1 camera frames to XRSLAM (monocular)
Submitted 300 camera frames to XRSLAM (monocular)
Submitted 600 camera frames to XRSLAM (monocular)
M23BV_STAGE3_REAL_CAMERA_READY=PASS
M23BV_STAGE3_REAL_CAMERA_SHUTDOWN=PASS
STAGE3_MONOCULAR=PASS_REAL_FRAMES_NATIVE_XRSLAM_NO_ODOM
```

This is evidence of **at least 600 real frames entering native
XRSLAM**, as logged by the adapter—not a claim of tracking accuracy,
metric scale, pose-direction validation or Nav2 driving capability.
The private log is `stage3_monocular_txkmGUYu.log`,
SHA256 `98a0b8181bc335cdeb3563119773b07bf60fd0f812233dd45daee7eac15532b1`.
Only the operator-supplied terminal excerpt was reviewed.

### Next distinct experiment: real STM32 IMU + camera, VIO diagnostic

A new additive **four-container** source candidate is now committed.
No new physical run has occurred or been authorized by these files.

- `m23bv_v3_stage3_xrslam_vio_diag.yaml`: sensor
  `mode=monocular_imu`, upstream diagnostic initializer with IMU
  refinement. It is **NOT a calibrated Leroy configuration**.
- `m23bv_v3_stage3_xrslam_vio_diag.launch.py`: actual XRSLAM +
  provisional static robot URDF TF, real camera/IMU subscriptions,
  `camera_imu_offset_sec=0.0` **explicitly provisional**, metric
  `pose_convention_validated=false` and `enable_map_tf=false`.
  Previous Stage 2B -41.95 ppm clock-rate estimate DOES NOT measure
  camera/IMU absolute phase; do not promote the 0.0 placeholder.
- `m23bv_v3_stage3_vio_imu_sender.py`: separately supervised M18
  packet parser/clock mapper, single MCU CDC open and only allowlisted
  START 50Hz/STOP 0Hz serial writes. Graph guards reject bridge/motor
  command topics, unknown odom/TF publishers, or missing XRSLAM subscriber
  **before serial opening and periodically during streaming**.
  All code paths after an open attempt STOP; **STOP ACK is mandatory**.
- `m23bv_v3_stage3_vio_audit.py`: confirms real stamped camera
  frame pairs + DepthART output + real IMU messages from one M18
  publisher and an XRSLAM IMU subscriber, while checking unconfigured
  Nav2 and zero metric odometry.
- `m23bv_v3_stage3_vio_host.sh`: new, independent operator-gated
  candidate, preserving the previously proven camera/depth supervisor.
  Parent maps only camera; XRSLAM and Nav2 map no devices; a FOURTH
  short-lived Podman container maps only the exact MCU, uses rootless
  crun `--group-add keep-groups`, the corrected M18 source, and
  physically isolated ROS-domain/IPC namespaces. Missing MCU STOP ACK
  is an explicit unknown MCU state and cannot PASS.

The VIO test's PASS target is native XRSLAM camera frame submission
in `monocular_imu` mode **plus** 70-second real MCU diagnostics and
both acknowledged serial commands, without metric odometry. Tracking
may remain `initializing` on a stationary robot. The test does NOT
authorize moving the sensor rig, Nav2 actions, motor energization,
recording, unverified obstacle clearance, calibrated pose, or drive.

### Mandatory NEXT operator step: strictly no-device VIO preflight

From `ros/` on the updated Stage 3 branch:

```bash
git pull --ff-only
bash ./m23bv_v3_stage3_vio_preflight.sh
```

The new preflight performs shell/AST source checks and rootless,
network-none, readonly package/YAML/M18 import checks in the two
pinned images, with **zero camera or MCU mappings or serial opens**.
Expected result (NOT YET OBSERVED):

```text
STAGE3_VIO_AST_PASS=m23bv_v3_stage3_vio_imu_sender.py
STAGE3_VIO_AST_PASS=m23bv_v3_stage3_vio_audit.py
STAGE3_VIO_AST_PASS=m23bv_v3_stage3_xrslam_vio_diag.launch.py
STAGE3_VIO_SOURCE_BOUNDARIES=PASS
STAGE3_VIO_M18_IMPORT_NO_DEVICE=PASS
STAGE3_VIO_XRSLAM_YAML_NO_DEVICE=PASS
STAGE3_VIO_PREFLIGHT=PASS_NO_DEVICE_SOURCE_AND_IMPORTS
STAGE3_VIO_MCU_CDC=NOT_OPENED
```

**Do not run the new physical VIO host script until preflight passes
and its exact MCU identity is independently checked.** The existing
private `M23BV_LOG_DIR` and freshly visually verified camera
`M23BV_STAGE3_CAMERA_VISUALLY_VERIFIED_NODE` remain prerequisites.
The new MCU ID must come from
`udevadm info --query=property --name=<actual /dev/ttyACM*>`,
then be supplied as `M23BV_STAGE3_MCU_ID_SERIAL`; it is NOT guessed
from the by-id name.

Each physical VIO attempt requires a fresh uppercase `YES` after
human confirmation that BOTH motor power rails are **physically
disconnected**, chassis secured, E-stop tested and operator present
for immediate disconnect. Opening STM32 CDC/DTR may itself energize
motor-capable firmware. If STOP is unacknowledged, assume MCU state
UNKNOWN and intervene physically; do not blindly retry. The Stage 2B
and Stage 3 earlier camera runs are not authority for this test.

## Operator-reported physical VIO result: PASS; log-only next action

The operator reports:

```text
STAGE3_VIO_PARENT_NETWORK=NONE
STAGE3_VIO_APPS=CAMERA_DEPTHART_XRSLAM_NAV2_STATIC_TF
STAGE3_VIO_IMU_START_ACK=PASS
M23BV_STAGE3_VIO_AUDIT=PASS_REAL_CAMERA_IMU_GRAPH_NO_ODOM
STAGE3_VIO_IMU_STOP_ACK=PASS
STAGE3_VIO_IMU_VERDICT=PASS_DIAGNOSTIC_STREAM
Submitted 1 camera frames to XRSLAM (monocular_imu)
Submitted 300 camera frames to XRSLAM (monocular_imu)
Submitted 600 camera frames to XRSLAM (monocular_imu)
Submitted 900 camera frames to XRSLAM (monocular_imu)
M23BV_STAGE3_REAL_CAMERA_SHUTDOWN=PASS
STAGE3_VIO=PASS_REAL_CAMERA_IMU_NATIVE_FRAMES_NO_ODOM
```

Private on-host log: `stage3_vio_QiKefpTz.log`;
SHA256 `5f3a1129ec679a2c3715514109dd833fdda5cb7cd70699f445ff337456f88e0f`.
The full log has **not** been independently inspected. These observations
prove the graph's real camera and MCU IMU ingress, native XRSLAM
camera-frame submissions in VIO mode, M18 source streaming and
acknowledged MCU STOP. The current adapter does not individually
log that its native XRSLAM gyroscope and accelerometer forwarding
paths consumed every IMU sample. No tracking accuracy, absolute
camera–IMU phase or metric odometry has been validated. Nav2 remains
unconfigured with no motor commands.

### Next: read-only private log health review, no new physical run

On the updated Stage 3 branch from `ros/`, with the **existing**
operator-private log still on disk:

```bash
git pull --ff-only
python3 -B -c "import ast,pathlib; ast.parse(pathlib.Path('m23bv_v3_stage3_vio_log_report.py').read_text())"
python3 -B ./m23bv_v3_stage3_vio_log_report.py \
  --log /home/kevin/.local/state/leroy-m23bv/stage3_vio_QiKefpTz.log \
  --expect-sha256 5f3a1129ec679a2c3715514109dd833fdda5cb7cd70699f445ff337456f88e0f
```

The log reviewer opens **only the previously recorded private text
log** and validates its SHA-256 and owner-private permissions. It
prints bounded scalar counters/health/status and does not print
raw camera, IMU, serial or other log content. It does not launch
Podman, ROS, devices, a recorder or actuators. No motion or
physical authorization is involved.

The next true bottleneck for navigation is validated temporal
camera–IMU alignment, physical extrinsics/TF ownership, actual
metric localization with established pose convention, and verified
depth-obstacle geometry/freshness. Do not enable Nav2 autonomous
control to compensate for these. Address the **specific observed
input/timing/tracking failure** after reviewing log scalars;
avoid repeating already accepted physical transport tests.

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

## Next hardware-free calibration provenance inventory

After the accepted physical VIO ingress and no-device Nav2
Configure/Cleanup + graceful shutdown results, the remaining
navigation-critical blocker is **not sensor transport or Nav2
package startup**. XRSLAM has only shown `initializing`, and
camera–IMU temporal offset and rigid extrinsics have not been measured.

The next *source-only* no-device operation, performed from `ros/`
after `git pull --ff-only`, is:

```bash
bash ./m23bv_v3_stage3_calibration_provenance_host.sh
```

The wrapper verifies exact accepted source Git blobs and image
calibration SHA, traces usb_cam/V3 rectifier timestamp lineage and the
M18 clock-rate mapper, checks provisional `base_link` sensor
transforms/pose gates, and inventories the **installed** usb_cam package
version and executable digest from the accepted sensor container using
rootless `podman --network none` with NO devices. It explicitly
returns `NOT_MEASURED` for camera exposure phase and extrinsics.

See `ros/M23BV_V3_STAGE3_CALIBRATION_PROVENANCE.md` for the source
evidence and stationary-robot measurement worksheet. That static
inspection is NOT camera–IMU calibration and does not authorize
moving the chassis, physically exciting the camera/IMU rig,
recording, accessing MCU serial or activating Nav2. A new physical
calibration plan and consent must precede any of those actions.

## Effective usb_cam driver check after underlay-header finding

**Do not rebuild or retag the accepted sensor image.** An operator
source/header inventory reproduced the upstream usb_cam 0.8.1
`tv_usec/1000.0` error in a Debian-installed header, but the
existing Leroy `ros/Dockerfile` already builds and installs the
patched `usb_cam` source into `/leroy_ws/install/usb_cam`.
`ros/CAMERA_TIMESTAMP_FIX.md` documents earlier camera timing
improvements with that overlay. The pinned Stage 3 V3 sensor image
inherits the M16 sensor-only build recipe.

The next **strictly hardware-free** step from `ros/` is:

```bash
git pull --ff-only
bash -n ./m23bv_v3_stage3_usb_cam_effective_overlay.sh
bash ./m23bv_v3_stage3_usb_cam_effective_overlay.sh
```

It sources ROS/leroy/V3 launch environments inside the exact pinned
sensor image in the same order as the accepted parent runner and
verifies which executable and shared library the ROS package lookup
selects. The older installed Debian package checksum was **not**
a suitable proxy for the effective runtime executable. The script
is a new candidate: **not yet executed by the operator**.

The original 0.8.1 source bug alone does not prove this accepted
V3 integration suffers a camera epoch shift. Absolute exposure phase,
camera–IMU time offset and physical mounting geometry remain
unmeasured, independent of which installed overlay wins. No sensors
or serial are opened by this check.

## YAML-configured sensor transforms (supersedes CAD audit proposal)

Leroy's hardware-specific fixed IMU, front-camera and workspace-camera
mounting coordinates are now editable in one YAML file:

`ros/src/leroy_description/config/sensor_mounts.yaml`

`leroy.urdf.xacro` consumes these values; the description package
installs the YAML. The previous numbers were migrated without changing
their values: IMU (+0.095, -0.095, 0.0) m, zero robot-aligned
IMU-link rotation; front camera +30° pitch with currently zero
translation; workspace camera +60° pitch with currently zero
translation. These are **configured but provisional**. The camera
optical-frame axis-conversion joints are ROS conventions and remain
in Xacro; the physical IMU chip-axis mapping is already applied
before the MCU publishes the robot-aligned IMU messages.

The new isolated no-device source test is
`bash ./m23bv_v3_stage3_sensor_mounts_yaml_no_device.sh`,
from `ros/` on the Stage 3 branch. It validates that Xacro expansion
matches the YAML and that changing a value in a **temporary YAML
copy** changes the generated transform. This candidate is **not yet
operator-tested**. The old YAML-less provenance check remains pinned
to its historical Xacro source and will intentionally block after
this new source revision; prior physical acceptance is unaffected.

No CAD-based extraction, additional MCU access, camera recording,
sensor-rig motion, pose validation, Nav2 activation or motor control
is authorized by this change.

## Direct YAML static TF (supersedes previous Xacro subsection)

The user selected **one YAML configuration and six standard
`tf2_ros/static_transform_publisher` nodes**, not URDF/Xacro.
Current authoritative source is
`ros/src/leroy_description/config/sensor_mounts.yaml`;
`ros/src/leroy_description/launch/description.launch.py`
loads it and generates exactly the existing six fixed frame edges.
Source Xacro was deleted and the old YAML-to-Xacro test deleted.
`robot_state_publisher` is no longer used by this package.
The original Stage 3 physical runner and immutable images were not
changed or rebuilt, so their historic execution is unaffected.

The replacement test, not yet operator-run, is:

```bash
# If the current shell is in repository root:
bash ros/m23bv_v3_stage3_sensor_tf_yaml_no_device.sh
# If inside ros/ instead:
# bash ./m23bv_v3_stage3_sensor_tf_yaml_no_device.sh
```

It runs source checks in the pinned no-device ROS Jazzy image, verifies
all six transforms and conventional camera frame IDs, creates six
`tf2_ros` action descriptions **without starting ROS nodes**, checks
an ephemeral 1 mm YAML adjustment and rejects malformed/unsafe values.
A PASS does not prove active TF publication or spatial calibration.
Do NOT run the old deleted `m23bv_v3_stage3_sensor_mounts_yaml_no_device.sh`.
The older pinned URDF provenance tool also belongs to the prior
source revision; do not weaken its historical pins.

If `git status --short` shows unrelated local deletions, preserve
them; do not use `git reset --hard` or `git clean` just to perform
this test. This new focused test verifies only its own committed
source inputs.

XRSLAM offset 0.0 s, pose validity, map TF and real Nav2 goals remain
gated; no MCU or motor access is authorized.

## Stage 3 YAML static TF host test — launch log tmpfs correction

The operator's first no-device run reported:
```text
STAGE3_SENSOR_TF_FRAME_TREE=PASS_SIX_UNCHANGED_STATIC_EDGES
STAGE3_SENSOR_TF_IMU=PASS_ALREADY_ROBOT_ALIGNED
STAGE3_SENSOR_TF_OPTICAL_CONVENTIONS=PASS
OSError: [Errno 30] Read-only file system: '/root/.ros/log'
STAGE3_SENSOR_TF=BLOCKED_VALIDATION_FAILED
```

This is a **test-container logging destination issue**, not a TF
geometry mismatch. Constructing a Jazzy `launch_ros.actions.Node`
creates a launch logger even though the test never starts the node.
The rootless container is deliberately read-only and had no writable
`/root/.ros/log`.

The test now sets `ROS_LOG_DIR=/tmp/leroy-ros-launch-logs` and
`ROS_HOME=/tmp/leroy-ros-home` inside the same 64 MiB
**ephemeral container `/tmp` tmpfs** before importing the launch,
and guards those paths. There are no new host mounts, exposed
devices, capabilities, ROS nodes, network connections or motion.

A successful retry must show the new
`STAGE3_SENSOR_TF_LAUNCH_LOG_DIR=EPHEMERAL_CONTAINER_TMPFS`
marker, the publisher/action and override/invalid-config PASS markers,
and only then
`STAGE3_SENSOR_TF=PASS_YAML_STATIC_TF_NO_DEVICE`.
Do not declare the previously blocked run as a PASS.

## Stage 3 YAML TF source test — package lookup fixture correction

The operator retried with the corrected ROS log destination. Their
output confirmed successful:
```text
STAGE3_SENSOR_TF_LAUNCH_LOG_DIR=EPHEMERAL_CONTAINER_TMPFS
STAGE3_SENSOR_TF_FRAME_TREE=PASS_SIX_UNCHANGED_STATIC_EDGES
STAGE3_SENSOR_TF_IMU=PASS_ALREADY_ROBOT_ALIGNED
STAGE3_SENSOR_TF_OPTICAL_CONVENTIONS=PASS
```

The next test step attempted to generate the launch description and
correctly stopped with:

```text
ament_index_python.packages.PackageNotFoundError:
"package 'leroy_description' not found, searching: ['/opt/ros/jazzy']"
STAGE3_SENSOR_TF=BLOCKED_VALIDATION_FAILED
```

**This does not implicate the YAML values or TF structure.** The pinned
sensor image was built before this new source-only description package
revision and does not contain an installed `leroy_description` package
in its ament resource index. An installed launch normally discovers its
package share directory through that index.

The read-only test harness now creates a **minimal temporary ament
index fixture** entirely in the preexisting container `/tmp` tmpfs,
registers `leroy_description`, copies the checked-in YAML to the
temporary package share path, and temporarily prepends the fixture to
`AMENT_PREFIX_PATH`. It verifies the package lookup resolves and
the installed-source fixture YAML bytes match the mounted original,
then continues with action construction, YAML override and negative
validation. It neither installs `leroy_description` into the immutable
image nor builds/starts any nodes.

Expected additional markers **not yet operator-observed**:

```text
STAGE3_SENSOR_TF_AMENT_INDEX=EPHEMERAL_SOURCE_FIXTURE_NO_INSTALL
STAGE3_SENSOR_TF_PACKAGE_LOOKUP=PASS_EPHEMERAL_AMENT_INDEX_ONLY
```

The real runtime package install/startup and live TF publication remain
**NOT TESTED**. A future deployment validation will require a separate
reviewed image/package build; the accepted pinned images are unchanged.
