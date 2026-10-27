#!/usr/bin/env bash
# Concurrent real-camera DepthART costmap + XRSLAM monocular diagnostics.
# Join the EXISTING isolated Stage 2 container's DDS graph; no device maps.
# Never publishes odometry/pose TF, commands motors, or configures a controller.
set -Eeuo pipefail
cd "$(dirname "$0")"

fail() { printf 'LIVE_STAGE3_XRSLAM=BLOCKED_%s\n' "$1" >&2; exit 2; }
[[ "$(id -u)" != 0 ]] || fail ROOT_HOST
command -v podman >/dev/null || fail PODMAN_MISSING
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
    fail NOT_ROOTLESS

parent=leroy-live-costmap
image=localhost/leroy-ros:xrslam
sidecar=leroy-live-xrslam
[[ "$(podman inspect --format '{{.State.Running}}' "$parent" 2>/dev/null)" == true ]] ||
    fail COSTMAP_PARENT_NOT_RUNNING
[[ "$(podman inspect --format '{{.HostConfig.NetworkMode}}' "$parent")" == none ]] ||
    fail COSTMAP_PARENT_NOT_NETWORK_ISOLATED
parent_env="$(podman inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$parent")"
for requirement in     ROS_DOMAIN_ID=170     ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST     LEROY_LIVE_PERCEPTION_ONLY=1     LEROY_LIVE_COSTMAP_DIAGNOSTIC=1
do
    grep -Fxq "$requirement" <<<"$parent_env" || fail PARENT_ENVIRONMENT_MISMATCH
done
podman exec "$parent" test ! -e /dev/ttyACM0 || fail MCU_VISIBLE_IN_PARENT
podman image exists "$image" || fail EXISTING_XRSLAM_IMAGE_MISSING
if podman container exists "$sidecar"; then
    fail SIDECAR_NAME_IN_USE
fi
for file in     m23bv_v3_stage3_xrslam_monocular_diag.yaml     m23bv_v3_stage3_xrslam_euroc_template_UNCALIBRATED.yaml
do
    [[ -f "$file" && ! -L "$file" ]] || fail DIAGNOSTIC_CONFIG_MISSING
done

printf '%s\n'   'Leroy Stage 3: reuse existing real camera + Nav2 diagnostic costmap.'   'XRSLAM monocular joins isolated DDS graph with NO device mapping.'   'No IMU, MCU/serial access, motor bridge, odometry authorization, map TF,'   'navigation goals or physical motion. Metric localization NOT validated.'   'Camera and TF calibrations remain provisional.'
printf 'LIVE_STAGE3_XRSLAM=STARTING_MONOCULAR_DIAGNOSTIC_ONLY\n'

# Only join the network namespace. Stage 2 was launched with --ipc private,
# so Podman cannot join its IPC namespace (the donor is not shareable).
# Give XRSLAM private IPC and force Fast DDS UDPv4, disabling SHM transport
# on this participant; Stage 2's default Fast DDS includes UDPv4.
# We intentionally do not start another camera, DepthART or static TF owner.
exec podman run --rm -it --pull=never --name "$sidecar" \
    --network "container:$parent" --ipc private --pid private \
    --read-only --cap-drop=all --security-opt=no-new-privileges \
    --tmpfs /tmp:rw,nosuid,nodev,size=256m \
    --mount "type=bind,src=$PWD/m23bv_v3_stage3_xrslam_monocular_diag.yaml,dst=/opt/leroy_stage3_slam.yaml,readonly" \
    --mount "type=bind,src=$PWD/m23bv_v3_stage3_xrslam_euroc_template_UNCALIBRATED.yaml,dst=/opt/leroy_stage3_device.yaml,readonly" \
    -e ROS_DOMAIN_ID=170 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -e ROS_LOG_DIR=/tmp/leroy-xrslam-ros \
    -e XDG_CACHE_HOME=/tmp/leroy-xrslam-cache \
    --entrypoint bash "$image" -lc '
      set -e
      source /opt/ros/jazzy/setup.bash
      source /leroy_ws/install/setup.bash
      test ! -e /dev/ttyACM0
      test ! -e /dev/video2
      exec ros2 run leroy_xrslam leroy_xrslam_node --ros-args \
        -p slam_yaml:=/opt/leroy_stage3_slam.yaml \
        -p device_template_yaml:=/opt/leroy_stage3_device.yaml \
        -p image_topic:=/camera/image_rect \
        -p camera_info_topic:=/camera/camera_info \
        -p camera_optical_frame:=camera_optical_frame \
        -p base_frame:=base_link \
        -p imu_frame:=imu_link \
        -p odom_frame:=odom \
        -p pose_convention_validated:=false \
        -p enable_map_tf:=false
    '
