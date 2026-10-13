# Building the optional XRSLAM-enabled Jazzy image

The default `Dockerfile` builds Leroy's camera, MCU bridge, robot description,
and DepthART **without** XRSLAM. `Dockerfile.xrslam` extends that image,
builds the independent native `libxrslam.so`, and installs the ROS 2 package
`leroy_xrslam`. It does not modify the XRSLAM source repository or enable
localization by default.

## Build

From the host's `leroy/ros` directory, after pulling the latest `main`:

```bash
podman build -f Dockerfile -t leroy-ros:jazzy .
podman build -f Dockerfile.xrslam -t leroy-ros:xrslam .
```

The base image also builds a patched, pinned `usb_cam` 0.8.1
in `/leroy_ws` to correct a confirmed V4L2 timestamp conversion bug.
The XRSLAM image inherits that fixed driver. See
[`CAMERA_TIMESTAMP_FIX.md`](CAMERA_TIMESTAMP_FIX.md) for verification
and remaining synchronization limitations.

The XRSLAM native source is pinned to
`53adcdafd065ec1b2239b9d000205ea243e8342d`. Its CMake build downloads
pinned third-party dependencies, so the first build needs network access and
can take substantial time. The resulting image contains the same five core
Leroy ROS packages as the base plus `leroy_xrslam`.

The additional build can be tuned without changing the base image:

```bash
podman build -f Dockerfile.xrslam -t leroy-ros:xrslam \
  --build-arg XRSLAM_BUILD_JOBS=2 .
```

## Device-free smoke check

These checks do **not** launch the robot, require USB devices, or run VIO:

```bash
podman run --rm leroy-ros:xrslam bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  EXE="$(ros2 pkg prefix leroy_xrslam)/lib/leroy_xrslam/leroy_xrslam_node"
  ros2 pkg executables leroy_xrslam
  ldd "$EXE" | grep libxrslam.so
  if ldd "$EXE" | grep -q "not found"; then
    echo "Missing shared library" >&2
    exit 1
  fi
'
```

Expected executable: `leroy_xrslam leroy_xrslam_node`. The shared library
should resolve under `/opt/xrslam/lib/`.

## Expected failure tests

Missing calibration and enabling map TF without verified map identity
**must fail** at startup:

```bash
podman run --rm leroy-ros:xrslam bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash

  ros2 run leroy_xrslam leroy_xrslam_node
  first=$?

  ros2 run leroy_xrslam leroy_xrslam_node --ros-args \
    -p enable_map_tf:=true
  second=$?

  test "$first" -eq 1 && test "$second" -eq 1
'
```

The expected errors are `Configure SLAM YAML, calibrated device template...`
and `enable_map_tf needs map_id and a distinct map_frame`, respectively.
Passing these negative tests only validates startup gates—not sensor
synchronization, odometry correctness, loss-of-tracking behavior, or mapping.

## Safety and readiness

- `use_xrslam` remains **false** by default in `leroy_bringup`.
- Do **not** enable pose publication or `odom -> base_link` TF until
  camera–IMU timing, physical extrinsics, pose direction, and tracking-loss
  behavior have been measured and verified on the robot.
- Keep `max_wheel_angular_velocity_rad_s: 0.0` and degraded odometry disabled.
- The ROS adapter does not implement a persistent-map Nav2 pipeline.
- Production builds should additionally pin the ROS base image by digest
  and archive third-party source artifacts, if full supply-chain repeatability
  is required.

For the adapter's calibration and ROS interface details, see
`src/leroy_xrslam/README.md`.
