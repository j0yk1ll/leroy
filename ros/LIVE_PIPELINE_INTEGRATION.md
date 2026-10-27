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

## Remaining full-pipeline integrations (do not claim completed)

1. **Actual base-frame obstacles:** measure camera extrinsics/floor, then
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
