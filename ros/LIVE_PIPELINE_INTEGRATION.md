# Leroy: live end-to-end integration, provisional depth (Stages 1–3)

Goal: connect the **real** front camera, rectifier, CPU DepthART model and
existing depth-to-PointCloud2 ROS adapter without synthetic messages.
For this integration pass, treat DepthART metric distance estimates as
provisionally correct; accuracy and speed improvements are separate work.

## What is integrated now

```text
verified /dev/video2 + real front_camera.yaml
  -> /camera/image_raw + /camera/camera_info
  -> /camera/image_rect
  -> CPU leroy_depthart -> /depthart/depth (32FC1, acquisition stamp)
  -> leroy_nav2_perception/depth_obstacles
  -> /leroy_nav2/diagnostics/raw_depth_points (optical-frame PointCloud2)
  -> live_pipeline_probe (read-only exact-frame/timestamp test)
```

The adapter deliberately runs with `publish_obstacles: false` until the
*real* camera-to-base transform and floor plane are measured; this setting
does **not** represent absent obstacles. The live chain has no Nav2
controller, XRSLAM odom, command-to-motor routing or safety authority.
No mock odometry, guessed transform or synthetic obstacle is substituted.

## Run on Arch/rootless Podman

Before starting, stop the manual motor-commissioning process and cut
motor-driver power using the independent cutoff. This is a stationary,
sensor-only experiment. Confirm the camera device really is `/dev/video2`
and the checked-in camera calibration still corresponds to that device.
There must not be another camera process holding the device.

Update the feature branch without resetting unrelated local changes:

```bash
cd /home/kevin/Code/leroy
git pull --ff-only
cd ros
podman build -f Dockerfile.live-perception -t leroy-ros:perception .
```

In terminal 1:

```bash
podman run --rm -it --name leroy-live-perception \
  --network=none --ipc=private --pid=private \
  --group-add keep-groups --device=/dev/video2 \
  --cap-drop=all --security-opt=no-new-privileges \
  -e ROS_DOMAIN_ID=170 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e LEROY_LIVE_PERCEPTION_ONLY=1 \
  --entrypoint bash localhost/leroy-ros:perception -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_perception live_depth_pipeline.launch.py
  '
```

In terminal 2, after the startup logs show camera and DepthART ready:

```bash
podman exec leroy-live-perception bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  ros2 run leroy_nav2_perception live_pipeline_probe
'
```

The probe has **no publishers**. It must observe five nonempty optical
point clouds matched to real DepthART images and valid CameraInfo by
exact acquisition timestamp and frame. It prints `LIVE_DEPTH_CHAIN=PASS`
with mean acquisition age or `INCOMPLETE` / `BLOCKED` with an error.
PASS verifies **transport and basic shape** only, not distance accuracy,
physical obstacle clearance, or collision-safe movement.

If startup or the probe fails, record stdout and the
`depth_obstacles` node's `missing_exact_timestamp_camera_info`,
`stale`, or message-format logs. Do **not** change acquisition stamps to
make them match, toggle extrinsics validation, or inject fictional TF.
Exit terminal 1 with Ctrl+C.

Optional no-device test (after image build):

```bash
podman run --rm --network none --entrypoint bash \
  localhost/leroy-ros:perception -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    cd /leroy_ws
    PYTHONPATH="$PWD/src/leroy_nav2_perception:$PYTHONPATH" \
      python3 -m unittest discover \
        -s src/leroy_nav2_perception/test \
        -p test_live_pipeline_contract.py -v
  '
```

## Observed Stage 2 result — PASS (2026-10-27)

The operator successfully completed the isolated, real camera DepthART
to Nav2 rolling costmap integration. A corrected two-stage Nav2
`ObstacleLayer` height filter was verified live without motor access:

```text
Original layer Z bounds: min=0.0 max=2.0
Changed layer Z bounds:  min=-3.0 max=3.0
LIVE_DEPTH_COSTMAP_GRAPH=READY required_publishers_verified
LIVE_DEPTH_COSTMAP=PASS real_optical_clouds=3 costmap_grids=3 cloud_age_last_s=0.697 lethal_cells=true unknown_cells=true frame=base_link
LIVE_DEPTH_COSTMAP_NOTE=provisional_TF_marks_unclassified_surfaces_not_for_motion
```

The fix is committed in `live_diagnostic_costmap.yaml`. The real
point-cloud-to-Nav2 costmap route is proved. This is NOT physical obstacle
classification: provisional camera pitch, missing floor datum, sample depth
uncertainty, fixed diagnostic footprint and stale-layer persistence remain.
No robot motor power, odometry localization or autonomous motion is
authorized by this result.

### Stage 3 publisher identity / shared PID follow-up (2026-10-27)

After rebuilding the costmap with shareable IPC, the operator observed
`Transitioning successful` twice but the Stage 2 acceptance probe
failed immediately:

```text
LIVE_DEPTH_COSTMAP_GRAPH=UNSAFE unexpected_publishers_on=/leroy_nav2/diagnostics/raw_depth_points [('_NODE_NAMESPACE_UNKNOWN_', '_NODE_NAME_UNKNOWN_', 'sensor_msgs/msg/PointCloud2')]
```

The identified raw cloud *topic* and type are present, but DDS returned
unknown identity metadata. This is not proof that the publisher was
unauthorized; equally it is not acceptable evidence of the required
source's identity. The probe now treats the matching single endpoint
with missing ROS node metadata as `WAITING` with
`publisher_identity_unverified`, cannot PASS it, and times out to
`INCOMPLETE` after 90 seconds if discovery does not recover.
Unexpected **named** publishers, duplicates, wrong types, actuator
topics and forbidden nodes still block immediately; regression tests
cover all those cases.

The first shareable IPC sidecar still did not demonstrate XRSLAM frame
ingestion. An additional possible cause is **PID namespace collision**
between multiple Fast DDS processes sharing IPC/SHM. The XRSLAM
launcher now joins only the existing isolated parent's **PID** namespace
in addition to its network and IPC namespace:
`--pid "container:$parent"`, not `--pid=host`.
The parent retains `--pid=private`; the sidecar checks the parent
is not in host PID mode. This change has **not** been run on physical
hardware yet.

Before rebuilding or changing the proven costmap again, STOP only the
XRSLAM sidecar and inspect the **parent's** raw cloud graph:

```bash
podman stop leroy-live-xrslam || true
podman exec leroy-live-costmap bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  ros2 node list
  ros2 topic info /leroy_nav2/diagnostics/raw_depth_points --verbose
  ros2 lifecycle get /costmap
'
```

If the raw cloud publisher is named `depth_obstacles`, rerun
`ros2 run leroy_nav2_perception live_costmap_probe` inside the
parent and require `PASS` **before** starting XRSLAM. If it remains
anonymous, treat Stage 2's *new container* as not yet accepted; do
not start XRSLAM or relax the probe. Inspect DDS state and compare
parent/child endpoint identity with both graph tools.

Then, from `ros/`, `git pull --ff-only` and run
`python3 -m unittest -v live_stage3_sidecar_static_test`; the
corrected `bash ./live_stage3_monocular_sidecar.sh` requires no
XRSLAM image rebuild. Check new output for a real
`Submitted 1 camera frames to XRSLAM (monocular)` and readable
latched tracking messages. Rerun the costmap probe after starting
the sidecar. Neither native XRSLAM submission nor monocular status
proves metric localization.

### Stage 3 DDS transport correction after real cross-container test

A Stage 2 parent with `--network=none --ipc=private` passed the
real camera-to-costmap probe. The first XRSLAM sidecar attempt failed
because Podman refused to join a non-shareable parent IPC namespace.
The second attempt **started XRSLAM** with the same network namespace
but private IPC and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. The operator
observed only partial DDS discovery: the XRSLAM process logged
`XRSLAM mode=monocular; IMU=disabled; pose output gated=yes`, but
its publisher and subscription endpoints appeared to the other
container as `_NODE_NAME_UNKNOWN_`; neither the latched
`/xrslam/tracking_status` nor `/xrslam/localization_valid` could
be received, and no real camera-frame submission was observed.
**Stage 3 did not PASS.** The cause is consistent with the Fast DDS
same-network/different-IPC shared-memory transport caveat documented
by eProsima, rather than evidence about XRSLAM feature tracking.

The supported Stage 3 configuration now shares ONLY the **existing
isolated parent network, its explicitly shareable IPC namespace and
its private PID namespace**. No host network, IPC or PID namespace is
used. The sidecar uses the
parent Fast DDS transport selection (no conflicting UDP-only override).
The script fails before starting if the parent IPC is not
`shareable`. This requires a controlled Stage 2 restart; an existing
container cannot have its IPC namespace changed in-place.

Keep motor power physically disconnected and STOP any XRSLAM sidecar
before stopping its camera parent. Rebuild from the corrected Stage 2
source, so the previously operator-validated Nav2 height filter
`[-3.0, 3.0]` is persistent rather than only a runtime parameter:

```bash
cd /home/kevin/Code/leroy
git pull --ff-only
cd ros
podman stop leroy-live-xrslam || true
podman stop leroy-live-costmap
podman rm leroy-live-costmap
podman build -f Dockerfile.live-costmap -t leroy-ros:costmap .
```

Terminal 1, the same non-actuating Stage 2 camera/costmap experiment,
now with **`--ipc=shareable`** (and a larger private shared-memory
filesystem, not the host's):

```bash
podman run -it \
  --name leroy-live-costmap \
  --network=none --ipc=shareable --shm-size=256m --pid=private \
  --group-add keep-groups --device=/dev/video2 \
  --cap-drop=all --security-opt=no-new-privileges \
  -e ROS_DOMAIN_ID=170 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e LEROY_LIVE_PERCEPTION_ONLY=1 \
  -e LEROY_LIVE_COSTMAP_DIAGNOSTIC=1 \
  --entrypoint bash localhost/leroy-ros:costmap -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_perception live_depth_costmap.launch.py
  '
```

Verify the parent IPC mode on the host:

```bash
podman inspect --format '{{.HostConfig.IpcMode}}' leroy-live-costmap
# Expected: shareable
```

Terminal 2, explicitly configure/activate the standalone costmap,
then **re-establish Stage 2 PASS on this freshly started graph**:

```bash
podman exec leroy-live-costmap bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  ros2 lifecycle set /costmap configure
  ros2 lifecycle set /costmap activate
  ros2 run leroy_nav2_perception live_costmap_probe
'
```

Only after this is `LIVE_DEPTH_COSTMAP=PASS`, Terminal 3:

```bash
python3 -m unittest -v live_stage3_sidecar_static_test
bash ./live_stage3_monocular_sidecar.sh
```

Expect `Submitted 1 camera frames to XRSLAM (monocular)` and a
readable `/xrslam/tracking_status` (likely `initializing` with
`/xrslam/localization_valid=false`). Confirm
`/leroy_xrslam` becomes a **named** ROS graph node in both containers
and rerun the Stage 2 probe concurrently. These conditions have **not
yet been observed** in the newly shared IPC/PID layout. If the graph
remains anonymous, inspect middleware selection and transport settings
rather than enabling odometry, fake poses or extra TF owners.

## Stage 3: run XRSLAM monocular alongside the already working live costmap

This stage is about **concurrent application integration**. The historical
Stage 3 physical VIO experiment already proved real camera and STM32 IMU
messages were submitted to native XRSLAM, but localization stayed
`initializing`. It did NOT establish validated metric odometry. See
`M23BV_V3_STAGE3_VIO_EVIDENCE.md`. Do not repeat motor-capable
serial/IMU physical work simply to test concurrent camera subscription.

Prerequisites: Stage 2 `leroy-live-costmap` still running, isolated
`--network none`, with camera and costmap publishing, and motor-driver
power disconnected. Also require the existing previously built
`localhost/leroy-ros:xrslam` image. Check locally:

```bash
podman ps --filter name=leroy-live-costmap
podman image exists localhost/leroy-ros:xrslam
```

Terminal 3, from the checked-out `ros/` directory:

```bash
bash ./live_stage3_monocular_sidecar.sh
```

This guarded script starts ONLY one new `leroy_xrslam_node` in a
read-only, **device-free** container sharing the parent's isolated DDS
network/IPC. It reuses existing /camera/image_rect, CameraInfo and the
single six-owner /tf_static tree without mapping camera or MCU to the
sidecar or launching another static TF publisher. The normalized upstream
EuRoC tuning is **UNCALIBRATED**. The ROS adapter runs
`monocular`, `pose_convention_validated=false` and
`enable_map_tf=false`. Do not lift these restrictions to make a
stationary monocular estimator publish metric odometry.

In a fourth terminal, read the latched XRSLAM results through the already
isolated parent graph:

```bash
podman exec leroy-live-costmap bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  ros2 node list | grep -Fx /leroy_xrslam
  ros2 topic info /camera/image_rect --verbose
  ros2 topic echo /xrslam/tracking_status std_msgs/msg/String \
    --once --qos-durability transient_local
  ros2 topic echo /xrslam/localization_valid std_msgs/msg/Bool \
    --once --qos-durability transient_local
'
```

Expected initial state `waiting_calibration` or `initializing`;
`localization_valid` must remain `false`. In the XRSLAM sidecar terminal
the native adapter should log `Submitted N camera frames to XRSLAM`
with `N > 0`. Also reconfirm the existing Stage 2 costmap remains live
(e.g. rerun `live_costmap_probe` in the parent). Only with all these
independent observations is the **concurrent graph** integrated.
XRSLAM native camera submission is NOT successful tracking. No `/odom`
messages or `odom -> base_link` TF should be emitted at this stage.

Stop the sidecar with Ctrl+C. This does not stop Stage 2's camera or
costmap. If the local XRSLAM image is absent or has incompatible ROS
middleware, the script blocks; investigate rather than replacing
calibration with defaults or mapping the MCU into the new container.

Next integration: incorporate the **already accepted real IMU publishing
path** under its separate physical-hardware guard, and then validate
tracking/pose direction before Nav2 planner/controller outputs can use
odometry. Merely showing the XRSLAM package running doesn't validate pose.

## Stage 2: real DepthART points into a Nav2 diagnostic costmap

Stage 1 was observed on Leroy: `LIVE_DEPTH_CHAIN=PASS matched=5`,
`acquisition_age_mean_s=0.825`, `shape_faults=0`. Proceed with the
actual-data costmap wiring rather than further tuning the CPU depth model.

The existing ROS adapter's **validated** obstacle topic stays DISABLED.
Stage 2 deliberately connects the already verified raw optical point cloud
to the Nav2 `ObstacleLayer`, which transforms points into a `base_link`
costmap using the checked-in **PROVISIONAL** fixed camera transform.
With no measured camera extrinsics or floor plane, these are marks for sampled
depth **surfaces**, not classified robot obstacles. The costmap does not
clear unknown space or authorize motion; it can retain stale marks.

```text
Real camera -> DepthART -> /leroy_nav2/diagnostics/raw_depth_points
                               | (real optical PointCloud2)
                               +-> Nav2 ObstacleLayer (provisional TF)
                                   -> /costmap (base_link OccupancyGrid)
```

With the manual motor controller stopped, motor-driver power disconnected,
and the front camera device verified as `/dev/video2`, build from `ros/`:

```bash
cd /home/kevin/Code/leroy
git pull --ff-only
cd ros
podman build -f Dockerfile.live-costmap -t leroy-ros:costmap .
```

This image is incremental from the Stage 1 `localhost/leroy-ros:perception`
image; keep it available locally. It installs Nav2's costmap library, and
uses the existing `leroy_nav2_costmap_fixture` executable with its experimental
callback-group cleanup behavior. That workaround is not yet proven on the
new real-data build.

### Costmap grid publishes but contains only unknown cells

A live run demonstrated all required publishers, `124` real point clouds,
`118` actual costmap grids, and `unknown=True`, but `lethal=False`.
The cause found in Nav2 Jazzy's `ObstacleLayer::updateBounds()` is a
**second height filter** at the layer level, independent of the configured
`raw_depth_surfaces.min_obstacle_height=-3.0` and
`max_obstacle_height=3.0` source filter. With the layer's default
`[0.0, 2.0]` interval, the provisionally downward-pitched camera can
project valid distance surfaces into **negative base_link Z**, where
Nav2 discards them before marking costmap cells.

Stage 2 now explicitly sets `obstacle_layer.min_obstacle_height=-3.0`
and `obstacle_layer.max_obstacle_height=3.0` and tests for those fields
during image build. Both settings are diagnostic only; marks can represent
floors and other unclassified surfaces, **not validated obstacles**.

To test this without rebuilding the existing *active* Stage 2 container:

```bash
podman exec leroy-live-costmap bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  ros2 param get /costmap obstacle_layer.min_obstacle_height
  ros2 param get /costmap obstacle_layer.max_obstacle_height
  ros2 param set /costmap obstacle_layer.min_obstacle_height -3.0
  ros2 param set /costmap obstacle_layer.max_obstacle_height 3.0
  ros2 run leroy_nav2_perception live_costmap_probe
'
```

The runtime adjustment does not persist across a container restart;
use a new `git pull --ff-only` and image rebuild for the corrected YAML.
The motor disconnected safety constraints remain in force.

### After costmap lifecycle activation: DDS graph discovery behavior

The operator successfully configured and activated `/costmap`, but
`live_costmap_probe` returned `BLOCKED_UNSAFE_OR_MISSING_GRAPH`
immediately. The first probe collapsed two independent conditions:
a prohibited actuator graph (fail immediately) and the required sensor/
costmap publishers not yet visible to ROS graph discovery (wait).

The probe now reports:
- `LIVE_DEPTH_COSTMAP_GRAPH=WAITING missing_publishers=[...]` while
  approved publishers have not yet been discovered;
- `LIVE_DEPTH_COSTMAP_GRAPH=READY required_publishers_verified` when
  exactly one approved depth cloud publisher and costmap publisher exist;
- `LIVE_DEPTH_COSTMAP=BLOCKED_UNSAFE_GRAPH ...` for prohibited command
  topics, nodes, or unexpected/duplicate source identities;
- `LIVE_DEPTH_COSTMAP=INCOMPLETE ...` if 90 seconds elapse without
  real clouds and a real grid with lethal/unknown cells.

Waiting for discovery does not bypass a missing sensor, fabricate a
costmap or permit motion. Source identities are checked on every spin
and are required at the exact time of PASS. The Stage 2 container build
runs regression tests for the permitted discovery delay and immediate
rejection of forbidden or unexpected endpoints.

Rebuild the image and **recreate** (not just re-exec into) the Stage 2
container: `podman exec` against an already running container will
continue to execute the old probe installed in that container.

### TF regular-file install fix (after Stage 2 initial camera/costmap fixes)

A subsequent run failed before any camera startup with
`BLOCKED_SENSOR_TF_CONFIG_NOT_REGULAR_FILE`. The launch was using an
installed sensor mount YAML created as a **symlink** by Colcon
`--symlink-install`. The loader intentionally rejects symlinks; changing
that validation would weaken the existing configuration checks.

The Stage 2 Dockerfile now copies the checked-in
`src/leroy_description/config/sensor_mounts.yaml` into the image as the
regular file `/etc/leroy/sensor_mounts.yaml`, checks it is not a symlink
and matches the source, and the Stage 2 launch passes its path through the
existing `sensor_mounts_file` launch argument. No TF data, physical
mount claims, or motor permissions were changed.

The original `leroy-live-perception` container was observed still holding
`/dev/video2` during this run. Stop that container before starting Stage 2:

```bash
podman stop leroy-live-perception
fuser -v /dev/video2 || true
```

After `git pull --ff-only`, rebuild `leroy-ros:costmap` from
`Dockerfile.live-costmap` and rerun the existing Stage 2 procedure.

### Startup failure fixes and camera ownership

The first live Stage 2 run exposed three distinct facts:

- Nav2 `costmap.height` and `costmap.width` parameters must be integer
  YAML scalars (`8`, not `8.0`). Invalid double dimensions abort
  costmap construction before lifecycle configuration.
- The log started `robot_state_publisher`, revealing an older installed
  description launch in the previous image. `Dockerfile.live-costmap`
  now recopies and rebuilds `leroy_description` from this branch and
  checks for `static_transform_publisher` in the installed launch.
- The camera aborted with a thrown `char*` while starting `/dev/video2`.
  The log alone does not identify the cause. Check for an existing camera
  owner or another `leroy-live-perception` container, then retest.
  The unrelated `Cannot open device /dev/video7` etc. lines are
  enumeration attempts, not proof that `/dev/video2` is misconfigured.

Before rebuilding/restarting Stage 2, stop the failing Stage 2 launch
(Ctrl+C), ensure other camera test containers have been stopped, and check:

```bash
podman ps --format '{{.Names}}'
fuser -v /dev/video2 || true
```

Do not kill an unknown process blindly. Stop only the camera process you
control. Verify the expected physical USB camera is still actually mapped
to `/dev/video2`.

Rebuild `localhost/leroy-ros:costmap` using the above command; the
Dockerfile now runs source-level static regression tests as part of build.
On successful launch, expect six `leroy_static_tf_*` publishers rather
than `robot_state_publisher`. If the camera still aborts with no competing
owner, rerun the previously passing Stage 1 live perception image as a
control and retain the exact `usb_cam` startup log; do not assume the
costmap itself caused the camera failure.

Terminal 1 — isolated sensor and costmap processes, no MCU serial:

```bash
podman run --rm -it --name leroy-live-costmap \
  --network none --ipc private --pid private \
  --group-add keep-groups --device=/dev/video2 \
  --cap-drop=all --security-opt=no-new-privileges \
  -e ROS_DOMAIN_ID=170 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e LEROY_LIVE_PERCEPTION_ONLY=1 \
  -e LEROY_LIVE_COSTMAP_DIAGNOSTIC=1 \
  --entrypoint bash localhost/leroy-ros:costmap -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_perception live_depth_costmap.launch.py
  '
```

Terminal 2 — configure/activate this *diagnostic* costmap only, then
observe actual marking of occupied and unknown cells:

```bash
podman exec leroy-live-costmap bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  ros2 lifecycle get /costmap
  ros2 lifecycle set /costmap configure
  ros2 lifecycle set /costmap activate
  ros2 run leroy_nav2_perception live_costmap_probe
'
```

Expected final label:
`LIVE_DEPTH_COSTMAP=PASS ... lethal_cells=true unknown_cells=true ...`.
A `PASS` means only that **real optical points** reached Nav2 and produced
costmap marks without synthetic depth or odometry; it is NOT obstacle accuracy,
measured floor clearance, safe planning, collision avoidance, or a motor enable.
If `INCOMPLETE`, keep the node logs; do not inject synthetic data or mark
the provisional camera transform validated.

Terminate terminal 1 using Ctrl+C; the experimental costmap runner may
still expose the previously observed Jazzy destructor issue. Record any
shutdown errors rather than declaring clean lifecycle behavior.

## Remaining full-pipeline integrations (do not claim completed)

1. **Calibrated base-frame obstacles:** measure camera extrinsics/floor, then
   enable `/leroy_nav2/diagnostics/obstacle_points` on live data.
   A point cloud from just one forward camera is not complete 360-degree
   protection for mecanum strafing/reversing.
2. **XRSLAM:** run the existing camera/IMU adapter with real calibration;
   validate metric pose and the unique `odom -> base_link` TF owner.
   Do not use known-broken right wheel encoders for odometry.
3. **Nav2:** connect real obstacles, validated odom and costmap into
   existing planner/controller on a **diagnostic output topic**,
   showing proposed and inhibited commands before hardware connection.
   Existing offline fixtures don't establish this live interface.
4. **Enforced supervised actuation:** only once live faults,
   stale-depth handling, command ownership, contact/traction and physical
   stopping are measured. The independent cutoff remains mandatory;
   MCU ACK is not proof of a safe physical stop.

For now the observed ~0.7s depth acquisition age plus occasional >1s
inter-frame gaps rule out treating CPU DepthART as a high-rate safety
sensor. This is an integration-stage warning, not a demand to optimize
DepthART before moving on to the other components.
