# Leroy Milestone 2 — DepthART depth-to-obstacle diagnostics (ROS 2 Jazzy)

**Not collision-safe, not live-ready, and not a motion stack.**
Do not connect this adapter, costmap, or the Nav2 controller to Leroy's
hardware `/cmd_vel`. The running `leroy-ros` container stays untouched.
No new calibration is asserted; physical motor power remains inhibited.

## Input/output contract

```text
/depthart/depth (32FC1, estimated metres, image acquisition timestamp)
        + /camera/camera_info (rectified CameraInfo.P, SAME stamp/frame)
               |
               v
     leroy_nav2_perception/depth_obstacles
               |
               +--> /leroy_nav2/diagnostics/raw_depth_points
               |      (PointCloud2, optical frame, acquisition stamp)
               |
               +--> [DISABLED BY DEFAULT] validated TF + floor-height gate
                         |
                         v
                   /leroy_nav2/diagnostics/obstacle_points
                         (PointCloud2, base_link, acquisition stamp)
                         |
                         v
               Standalone Nav2 Jazzy costmap (synthetic fixture only)
               obstacle_layer (marking=true, clearing=false)
                      + inflation_layer
```

Default ROS params (`config/depth_diagnostics.yaml`) set:
`publish_obstacles: false`, `extrinsics_validated: false`,
`ground_reference_validated: false`. Both validation flags must be true
before publishing marked obstacle points. This is a **configuration
assertion**, not automatic proof of real calibration. Only the explicitly
synthetic fixture sets them true. DO NOT apply fixture params/TF on robot.

The adapter matches CameraInfo to the exact acquisition stamp and frame,
checks width/height and unbinned/full-frame ROI, uses **CameraInfo.P** rather
than inventing intrinsics, validates 32FC1 endianness/row stride, rejects
nonfinite and nonpositive/out-of-range pixels, subsamples depth, retains
the original optical frame and acquisition stamp on the raw cloud,
and transforms to `base_link` only using TF at that acquisition time.
There is no silent fallback transform. Ground-height selection is only an
approximate geometry gate, not terrain classification or safety clearance.
Missing TF, CameraInfo, stale/future/out-of-order depth, or empty point
sets are dropped without emitting an empty cloud to clear an obstacle.

The adapter logs acquisition age, last depth arrival staleness and
accepted/rejected frame counts every 5 s. The old DepthART observations
of ~1–2 FPS and ~600 ms inference latency are NOT performance acceptance.
Measure new rates and timing on the real robot in a later passive stage.
Unobserved space and fields of view outside the single forward camera
(including rear and side blind spots) are **not known free**.
Reflective, glass, textureless, floor and near-camera returns remain
unreliable. Camera angle and camera-to-base transform are PROVISIONAL.
Stale markings may be retained; Nav2 costmap cannot by itself enforce a
stop on missing input. The fail-closed motion gate is a later milestone.

## Host: build entirely separate Nav2 image

From your Arch Linux checkout, with no changes to the running container:

```bash
git fetch origin
git switch feature/nav2-m2-depthart-diagnostic-obstacles
cd ros
podman build -f Dockerfile.nav2 -t leroy-nav2:jazzy .
```

The build compiles both ROS packages and runs their unit/configuration tests.
Nothing runs automatically and the Nav2 image contains no hardware bridge,
DepthART runtime, motors, or XRSLAM. No camera or serial devices are mapped.

## Offline diagnostic costmap fixture (host terminal 1)

```bash
podman run --rm -it --name leroy-nav2-m2 \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:jazzy bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_perception depth_fixture.launch.py
  '
```

This publishes **synthetic** 4x4 depth and CameraInfo, plus precisely
specified **synthetic-only** `base_link -> sim_camera_optical_frame` TF.
The fixture assumes a fictitious floor at base_link z=0 and camera z=0.2.
Nothing corresponds to Leroy's actual sensor mounting or robot odometry.

## Host terminal 2: manually activate ONLY the costmap

```bash
podman exec leroy-nav2-m2 bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  echo "--- versions ---"
  dpkg-query -W ros-jazzy-nav2-costmap-2d ros-jazzy-sensor-msgs-py
  echo "--- synthetic diagnostics ---"
  ros2 topic echo /leroy_nav2/diagnostics/raw_depth_points --field header --once
  ros2 topic echo /leroy_nav2/diagnostics/obstacle_points --field header --once
  echo "--- activate isolated costmap ONLY ---"
  ros2 lifecycle get /costmap
  ros2 lifecycle set /costmap configure
  ros2 lifecycle set /costmap activate
  echo "--- inspect costmap ---"
  ros2 lifecycle get /costmap
  ros2 topic list
  ros2 topic info /costmap/costmap --verbose
  echo "--- prohibited topic ---"
  if ros2 topic list | grep -qx /cmd_vel; then
    echo "FAIL: prohibited /cmd_vel topic exists"
    exit 1
  fi
  echo "PASS: /cmd_vel absent in offline fixture graph"
'
```

The activated node is ONLY a costmap: no controller/planner/action server
is ever launched in M2. Verify costmap topics and obstacle marking in
RViz or inspect its message. Config `fixture_costmap.yaml` uses a static
sensor/robot frame and does **not** publish `odom -> base_link`.
To avoid requiring GUI/network sharing, all RViz displays are optional.
If the standalone costmap executable/lifecycle naming differs with the
installed Jazzy packages, paste the exact console output; do not add fake
`odom`, connect real sensors, or start a controller to work around it.

## Run test suite / collect evidence (host)

```bash
podman run --rm --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:jazzy bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    cd /leroy_ws
    colcon test --packages-select leroy_nav2 leroy_nav2_perception \
      --event-handlers console_direct+
    colcon test-result --verbose
  '
```

To stop fixture: Ctrl-C in terminal 1. Preserve real `leroy-ros`.
Record launch logs, TF topics, point cloud field/width, `/costmap/costmap`
availability and sample lethal/inflated cells; screenshot RViz only if
running entirely within a safe isolated display environment.

## Milestone 2 limits / pending acceptance

- ROS-free geometry, timestamp/floor filtering and config contract tests
  cover deterministic synthetic evidence, but cannot validate real depth.
- Actual operator `colcon test`, package install, standalone costmap
  lifecycle, obstacle marking and RViz observations **pending**.
- Loss of sensor input is reported and NEVER interpreted as proof of
  cleared/free space; there is **no hardware motion authority**.
- Real camera extrinsics, ground plane, sensor age distribution and
  occluded-area handling remain unverified. No passive live observation
  has been authorized; do that only in Milestone 5.
- Do not merge or advance to Milestone 3 until operator evidence reviewed
  and explicitly approved.
