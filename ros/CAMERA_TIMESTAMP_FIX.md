# Jazzy USB camera timestamp correction

## Why Leroy overlays `usb_cam`

Ubuntu 24.04 / ROS 2 Jazzy's `ros-jazzy-usb-cam` 0.8.1 binary includes
a V4L2 monotonic-to-wall-clock conversion error in
`include/usb_cam/utils.hpp`:

```cpp
epoch_time.tv_sec * 1000000 + epoch_time.tv_usec / 1000.0;
```

The `timeval.tv_usec` field **already contains microseconds**. Dividing it
by 1000 causes an apparent camera header age that changes depending on the
fractional second at which the `usb_cam` process starts.

The corrected expression is:

```cpp
epoch_time.tv_sec * 1000000 + epoch_time.tv_usec;
```

A rootless Podman test with Leroy's Wed camera (`/dev/video2`) reproduced
session-dependent raw-camera delays between approximately 110 and 971 ms
with the packaged binary. A locally built, one-line-patched `usb_cam`
0.8.1 overlay reduced the delay to approximately 50–60 ms and remained
consistent across restarted camera sessions.

The base `ros/Dockerfile` therefore fetches upstream
`ros-drivers/usb_cam` 0.8.1 at commit
`cb2ae6bc0a312de629f84fef933fb3e07dac1ef8`, applies exactly
this replacement, builds into `/leroy_ws/install/usb_cam`, and
checks that ROS resolves the overlaid package and linked shared library.

The distro package remains installed in `/opt/ros/jazzy` as an underlay;
the source overlay is used when `/leroy_ws/install/setup.bash` is sourced.

## Building

From `leroy/ros` on the host, after merging the change and pulling `main`:

```bash
podman build -f Dockerfile -t leroy-ros:jazzy .
podman build -f Dockerfile.xrslam -t leroy-ros:xrslam .
```

The XRSLAM image inherits the corrected driver from the base image. Podman
build caching should reuse earlier base layers when their inputs have not
changed. This Dockerfile update itself has **not** been validated by an
end-to-end image rebuild yet.

## Verify selection without hardware

```bash
podman run --rm leroy-ros:xrslam bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  test "$(ros2 pkg prefix usb_cam)" = /leroy_ws/install/usb_cam
  EXE="$(ros2 pkg prefix usb_cam)/lib/usb_cam/usb_cam_node_exe"
  ldd "$EXE" | grep libusb_cam
  ! ldd "$EXE" | grep -q "not found"
'
```

If testing the base image alone, replace `leroy-ros:xrslam` with
`leroy-ros:jazzy`.

## Hardware retest

Bring up the robot with `use_depthart:=false use_xrslam:=false`,
or launch only `usb_cam`, and monitor:

```bash
ros2 topic delay /camera/image_raw -w 30
```

Repeat across several camera *process* restarts, not merely restarted
delay subscribers. On the test laptop, the patched 0.8.1 driver achieved
approximately 50–60 ms apparent delay. This is **not** a guaranteed
latency bound or a camera–IMU temporal calibration.

## Remaining limitations

- The laptop camera still captured at approximately 19.9 FPS despite a
  configured V4L2 30 FPS mode. That is a separate issue.
- The STM32 timestamp-to-ROS-clock mapping currently uses a one-time offset
  and has not been validated for drift.
- Hardware-calibrated camera–IMU extrinsics, relative timing, pose
  conventions, and tracking-loss behavior are still prerequisites for VIO.
- Keep XRSLAM localization and motors disabled during these tests.
