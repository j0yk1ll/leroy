# Leroy: live end-to-end integration, provisional depth (Stage 1)

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
