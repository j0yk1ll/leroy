# M16 — Passive observation of **actual** Leroy ROS sensor streams

**Goal:** obtain recorded, bounded, actual ROS sensor-transport evidence instead
of another fabricated-sensor PASS. Implement a read-only ROS 2 participant to
monitor the **existing** deployed camera, IMU, DepthART, XRSLAM, TF and obstacle
message topics (where present), verify concrete message shapes/time behavior
and record publisher graph identities. This is a software integration
diagnostic, not a hardware controller or an assertion of calibration.

**Actual source data is not yet collected.** The user must run the M16 passive
monitor; an unobserved stream is reported as **unverified**, not synthesized
or declared healthy.

## What exists already in the checked-in source

- Existing `leroy_bringup/launch/robot.launch.py` has
  `use_xrslam:=false` by default. Do **not** set
  `xrslam_pose_convention_validated` to true just to get odometry.
  `leroy_xrslam` only publishes `/odom` and dynamic
  `odom -> base_link` when its own validated pose-tracking conditions hold.
  It reports `/xrslam/tracking_status` and latched
  `/xrslam/localization_valid`, which are **unstamped status flags**.
  A latched true does not prove fresh odometry or pose correctness.
- The actual `leroy_bridge` publishes `/imu/data_raw`; the robot launch
  contains the bridge and camera driver. The bridge also owns physical
  `/cmd_vel`. M16 **only subscribes to the IMU**; it does not create
  `/cmd_vel`, send goals, change parameters, or interact with actuators.
  M16 never starts, stops or modifies `leroy-ros`.
- The existing `leroy_depthart` driver is expected to publish
  `/depthart/depth` as image `32FC1`, with an acquisition stamp copied
  from the input rectified camera frame. The corresponding
  `/camera/camera_info` is required to use the depth for navigation.
  The optional `leroy_nav2_perception/depth_obstacles` adapter publishes
  `/leroy_nav2/diagnostics/obstacle_points` **only if**
  `publish_obstacles`, `extrinsics_validated` and
  `ground_reference_validated` have been appropriately configured.
  M16 does not override any such configuration.
- The bridge normally does not own `/odom` or odom TF: XRSLAM is intended
  as the single owner. Report any duplicate or unexpected /odom publisher
  rather than assuming harmlessness.
- The development image **omits** XRSLAM, DepthART, camera drivers,
  firmware, motor bridge and all physical USB devices. It can observe ROS
  publishers reachable over DDS, but cannot make an unavailable source
  appear.

## Implemented read-only component

`m16_passive_observer.py` is a ROS node with **nine subscriptions**, no
application publisher, no action client, no service client, no timer and no
lifecycle/parameter action. It disables ROSout and parameter services.
Inputs are hard-named and remaps rejected. It processes bounded summaries,
NOT sensor frame captures or message payload dumps. Observed inputs:

| Stream | Topic |
|---|---|
| Rectified camera image | `/camera/image_rect` |
| Camera intrinsics | `/camera/camera_info` |
| DepthART output | `/depthart/depth` |
| Raw MCU IMU | `/imu/data_raw` |
| XRSLAM status | `/xrslam/tracking_status` |
| XRSLAM localization flag | `/xrslam/localization_valid` |
| XRSLAM metric-shaped odometry | `/odom` |
| Dynamic `odom -> base_link` TF within `/tf` | `/tf` |
| Optional marked base-frame obstacle cloud | `/leroy_nav2/diagnostics/obstacle_points` |

Checks include strictly increasing nonzero acquisition stamps, basic
ROS-time age/future-skew constraints, bounded receipt rates and gaps,
camera image dimensions and encoding, exact `32FC1` layout, a bounded
64-pixel positive-finite depth sample fraction, valid CameraInfo.P,
a bounded 512-entry CameraInfo frame/stamp history to match delayed
DepthART frames, finite IMU angular velocity and acceleration in
`imu_link`, XRSLAM status flags, finite/normalized odometry and
dynamic TF and stamp agreement <=0.25s, and bounded finite XYZ obstacle
clouds using the M15 payload validator. It reports raw publisher
node/namespace/type claims and warnings for unexpected source ownership.

`m16_passive_contract.py` contains pure Python validation/statistics;
`test_m16_passive_observer.py` tests malformed frames, finite sample
checks, stale/regressed timestamps, bounded runtime, graph metadata,
read-only ROS construction and optional image registration.

A single, machine-readable result line begins
`M16_PASSIVE_REPORT_JSON=`; the compact verdict line is
`M16_VERDICT=...`. Possible outcomes:

- `OBSERVED_MESSAGE_CONTRACTS_ONLY`: all four *limited* stream groups
  were actually observed fresh with their expected ROS message contract,
  XRSLAM tracking indicators were true, and the checked publisher graph
  identities did not conflict. This is **not** proof of calibrated geometry,
  robot pose accuracy, obstacle completeness, actuation readiness, or safety.
- `INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS`: one or more streams were not
  published, not reachable by DDS, stale/invalid, unrecognized or the
  XRSLAM state was not valid. Look at the JSON for the specific reason.
  In the current default robot bringup, XRSLAM may be disabled and
  obstacle clouds unpublished; incomplete is therefore **expected**.

No actual frame payload, media recording, environment secret, serial
bytes or hardware identifier is stored by the monitor. JSON contains
only observed ROS endpoint names/types, numeric sensor statistics and
small diagnostic state strings. The log still reveals ROS graph
topology, so handle it as project data.

## Stage 1 — Build and check registration (no robot access)

From **repository root**, without touching the active `leroy-ros` container:

```bash
git fetch origin
git switch feature/nav2-m16-passive-real-sensor-observer
git pull --ff-only origin feature/nav2-m16-passive-real-sensor-observer
cd ros
podman build --no-cache -f Dockerfile.nav2 -t leroy-nav2:m16-passive .
```

`Dockerfile.nav2` runs the existing five optional packages' Colcon/pytest
suite and verifies the installed M16 console entry point. The build does
not require or start any real ROS sensor and does not replace the
running `leroy-ros` image.

## Stage 2 — Read-only deployed ROS environment preflight

Run on the **host** from the `ros/` directory. This command **inspects**
the existing container's environment and Podman network mode; it does
not modify or restart that container.

```bash
podman inspect leroy-ros --format 'M16 preflight: container={{.Name}} status={{.State.Status}} network={{.HostConfig.NetworkMode}}'
podman exec leroy-ros bash -lc '
  printf "M16 preflight: ROS_DOMAIN_ID=%s\n" "${ROS_DOMAIN_ID:-0}"
  printf "M16 preflight: ROS_AUTOMATIC_DISCOVERY_RANGE=%s\n" "${ROS_AUTOMATIC_DISCOVERY_RANGE:-unset}"
'
```

If `leroy-ros` is not a running container with this name, the preflight
fails and you cannot infer a valid live ROS data path. Preserve that
output rather than guessing the current deployment network.

## Stage 3 — ONE bounded passive live observation

This is deliberately different from previous `--network none`
**synthetic** fixtures. To see actual ROS publishers, the observer
temporarily joins **host networking** for DDS discovery, but still has
**no devices**, no host path mounts, private IPC/PID namespaces,
no Linux capabilities, `no-new-privileges` and a read-only
filesystem. The host's `leroy-ros` must itself be discoverable via
host networking/ROS DDS; if its Podman network is private, an incomplete
report is evidence of discovery limits, not evidence that sensors are
absent. Do not alter bridge networking as part of M16.

Use exact commands on the host from `ros/`:

```bash
ROS_DOMAIN_ID_FOR_M16="$(podman exec leroy-ros bash -lc 'printf "%s" "${ROS_DOMAIN_ID:-0}"')"
set -o pipefail
podman run --rm --name leroy-nav2-m16-passive \
  --network host --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=128m \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_FOR_M16" \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e ROS_LOG_DIR=/tmp/m16-ros-logs \
  -e LEROY_NAV2_DRY_RUN=1 \
  -e LEROY_M16_DURATION_S=45 \
  leroy-nav2:m16-passive bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 run leroy_nav2_integration m16_passive_observer
  ' 2>&1 | tee m16_actual_sensor_report.log
```

The observer ends itself after 45 seconds; the Podman
`--rm` container is removed. If interrupted or hung, press Ctrl+C
in that command's terminal. In a **second host terminal**, if
required, stop **only** the dedicated observer:

```bash
podman stop --time 5 leroy-nav2-m16-passive
```

This does **not** stop or modify `leroy-ros`. A `M16_PASSIVE_REPORT_JSON`
line plus `M16_VERDICT` must appear for completed runs. Keep the saved
`ros/m16_actual_sensor_report.log` and share the report line with the
software developer. Capture no screenshots or raw sensor payloads.

### If the report is incomplete

Inspect `topics.<stream>.received`, `accepted`, `invalid`,
`fresh_valid_receipt`, and `publishers`. Key distinctions:

- **Publisher absent**: source not started or unreachable over DDS; no
  claim about real sensor failure. M16 does not enable XRSLAM or the
  DepthART obstacle adapter automatically.
- **Publisher present, zero received**: QoS/discovery/network/source
  timing mismatch or inactive driver; do not silently simulate data.
- **Samples received but invalid/stale**: message shape, timestamp
  source, calibration contract or timing requires engineering work.
- **XRSLAM `localization_valid=false` or no `/odom`**: do not
  enable `pose_convention_validated` without the source-specific
  pose-direction, temporal offset and metric-scale verification.
- **Camera/depth timestamp mismatch**: the exact stamped
  CameraInfo cache may be missing the delayed depth frame or there
  is a real timing/transport mismatch. Probe further; never
  rewrite timestamps to force a pass.
- **Unexpected/duplicate odom publisher**: investigate graph
  ownership before integration.

The code's thresholds are **software screening heuristics**, not
calibration or navigation performance acceptance. `/odom` confidence,
pose-direction, camera-IMU offset, real camera-to-base TF, DepthART
metric accuracy, obstacle coverage, costmap configuration and actual
command multiplexing remain separate integration tasks.

## M16 acceptance boundary

M16 implementation acceptance requires optional-image build/pytest pass
plus proof that a 45s **actual-source** observation ran to completion
and produced machine-readable coverage and provenance of the
nine checked topic contracts. **Positive all-source data acceptance**
requires those inputs to actually be present and meet the above
conditions; `INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS` is an honest
interim result, not a source-quality PASS. A completed monitor with
no XRSLAM data still represents successful **tool execution**, not
completed real-sensor integration. Do not claim full real-sensor
validation until actual source observations support it.

This step contains **no** Nav2 FollowPath, MPPI controller startup,
bridge command topic, motor action, physical operating test, or
independent stop-safety assertion. M14/M15 offline fault-gate
results remain valid but are separate evidence.
