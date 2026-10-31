#!/usr/bin/env bash
# Reproduce the operator-validated camera -> DepthART -> Nav2 diagnostic
# costmap + XRSLAM monocular graph, WITHOUT MCU, motor or odometry authority.
# Fail closed. Only the containers started by this invocation are stopped
# if startup or verification fails; no existing containers are replaced.
set -Eeuo pipefail
cd "$(dirname "$0")/../../.."

fail() { printf 'LIVE_STAGE3_START=BLOCKED_%s\n' "$1" >&2; exit 2; }
[[ $# -eq 0 ]] || fail UNEXPECTED_ARGUMENTS
[[ "$(id -u)" -ne 0 ]] || fail ROOT_HOST
command -v podman >/dev/null || fail PODMAN_MISSING
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] || fail NOT_ROOTLESS
[[ "${LEROY_MOTOR_POWER_DISCONNECTED-}" == 1 ]] || fail PHYSICAL_MOTOR_DISCONNECT_CONFIRMATION_REQUIRED
# The operator visually verified the front camera on HOST /dev/video8.
# Keep /dev/video2 ONLY as the existing in-container usb_cam interface.
# Never silently fall back to a different host video node.
camera=/dev/video8
[[ -c "$camera" && ! -L "$camera" ]] || fail VERIFIED_CAMERA_DEVICE_MISSING
camera_rdev="$(stat -Lc '%t:%T' -- "$camera")" || fail CAMERA_DEVICE_STAT
[[ ! -e /dev/ttyACM0 || -c /dev/ttyACM0 ]] || fail UNEXPECTED_MCU_DEVICE
for image in localhost/leroy-ros:costmap localhost/leroy-ros:xrslam; do
    podman image exists "$image" || fail IMAGE_NOT_BUILT
done
for name in leroy-live-costmap leroy-live-xrslam; do
    if podman container exists "$name"; then fail EXISTING_CONTAINER_NAME_IN_USE; fi
done

# This confirmation is an operator statement, NOT automatic electrical
# verification. Motor driver rails must be independently disconnected.
echo "Motor-driver power must already be physically disconnected."
echo "Starting ONLY the isolated real camera diagnostic; no serial mapping."
echo "LIVE_STAGE3_CAMERA_MAPPING=HOST_/dev/video8_TO_CONTAINER_/dev/video2"
parent_started=0
sidecar_started=0
cleanup_if_failed() {
    rc=$?
    trap - EXIT
    if (( rc != 0 )); then
        echo "LIVE_STAGE3_START=FAILED_ROLLING_BACK" >&2
        if (( sidecar_started )); then
            podman logs --tail 30 leroy-live-xrslam >&2 || true
            podman stop leroy-live-xrslam >/dev/null || true
        fi
        if (( parent_started )); then
            podman logs --tail 30 leroy-live-costmap >&2 || true
            podman stop leroy-live-costmap >/dev/null || true
        fi
    fi
    exit "$rc"
}
trap cleanup_if_failed EXIT

# Refuse a replaced/hotplugged host camera before starting the container.
[[ -c "$camera" && "$(stat -Lc '%t:%T' -- "$camera")" == "$camera_rdev" ]] ||
    fail CAMERA_CHANGED_BEFORE_START
# Only the visually verified HOST /dev/video8 is mapped, to the existing
# INNER /dev/video2 interface expected by usb_cam's launch parameters.
# No host networking, IPC, PID, motor-capable STM32 or command bridge.
podman run --rm --detach --pull=never --name leroy-live-costmap \
    --network=none --ipc=shareable --shm-size=256m --pid=private \
    --group-add keep-groups --device="$camera:/dev/video2:rw" \
    --cap-drop=all --security-opt=no-new-privileges \
    -e ROS_DOMAIN_ID=170 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    -e LEROY_LIVE_PERCEPTION_ONLY=1 \
    -e LEROY_LIVE_COSTMAP_DIAGNOSTIC=1 \
    --entrypoint bash localhost/leroy-ros:costmap -lc '
      set -e
      source /opt/ros/jazzy/setup.bash
      source /leroy_ws/install/setup.bash
      test ! -e /dev/ttyACM0
      exec ros2 launch leroy_nav2_perception live_depth_costmap.launch.py
    ' >/dev/null
parent_started=1

# Nav2 lifecycle transitions are deliberately explicit; never launch
# controllers, planners, odom TF, motors or synthetic sensor fixtures.
ready=0
for attempt in $(seq 1 45); do
    if podman exec leroy-live-costmap bash -lc '
      source /opt/ros/jazzy/setup.bash
      source /leroy_ws/install/setup.bash
      ros2 lifecycle get /costmap
    ' 2>/dev/null | grep -q 'unconfigured \[1\]'; then
        ready=1
        break
    fi
    sleep 2
done
(( ready )) || fail COSTMAP_LIFECYCLE_NOT_DISCOVERED

podman exec leroy-live-costmap bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    installed="$(ros2 pkg prefix leroy_nav2_perception)/share/leroy_nav2_perception/config/live_diagnostic_costmap.yaml"
    python3 - "$installed" <<"PY"
import sys
import yaml
with open(sys.argv[1], encoding="utf-8") as f:
    params = yaml.safe_load(f)["costmap"]["ros__parameters"]
layer = params["obstacle_layer"]
source = layer["raw_depth_surfaces"]
assert (float(layer["min_obstacle_height"]),
        float(layer["max_obstacle_height"])) == (-3.0, 3.0), "layer-wide Z window stale"
assert (float(source["min_obstacle_height"]),
        float(source["max_obstacle_height"])) == (-3.0, 3.0), "cloud source Z window stale"
print("LIVE_STAGE3_INSTALLED_COSTMAP_CONFIG=PASS_BOTH_HEIGHT_WINDOWS")
PY
    ros2 lifecycle set /costmap configure
    ros2 lifecycle set /costmap activate
    ros2 lifecycle get /costmap | grep -F "active [3]"
    ros2 param get /costmap obstacle_layer.min_obstacle_height | grep -Fx "Double value is: -3.0"
    ros2 param get /costmap obstacle_layer.max_obstacle_height | grep -Fx "Double value is: 3.0"
    ros2 run leroy_nav2_perception live_costmap_probe
'

# XRSLAM reuses existing camera/TF DDS. The sidecar script maps ZERO
# devices and joins only the already isolated parent network+IPC+PID.
bash ./src/leroy_bringup/scripts/live_stage3_monocular_sidecar.sh --detach >/dev/null
sidecar_started=1

native_frames=0
for attempt in $(seq 1 45); do
    if podman logs leroy-live-xrslam 2>&1 |
            grep -Fq 'Submitted 1 camera frames to XRSLAM (monocular)'; then
        native_frames=1
        break
    fi
    sleep 2
done
(( native_frames )) || fail XRSLAM_NATIVE_FRAME_INGEST_NOT_VERIFIED

podman exec leroy-live-costmap bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 node list | grep -Fx /leroy_xrslam
    timeout 15s ros2 topic echo /xrslam/tracking_status std_msgs/msg/String \
      --once --qos-reliability reliable --qos-durability transient_local | grep -F "data:"
    timeout 15s ros2 topic echo /xrslam/localization_valid std_msgs/msg/Bool \
      --once --qos-reliability reliable --qos-durability transient_local | grep -Fx "data: false"
    # A publisher endpoint may exist, but publishing *any* odom message
    # is prohibited in the camera-only acceptance run.
    if timeout 6s ros2 topic echo /odom nav_msgs/msg/Odometry --once; then
        echo "LIVE_STAGE3_ODOM=BLOCKED_UNEXPECTED_MESSAGE" >&2
        exit 3
    else
        code=$?
        [ "$code" -eq 124 ] || exit "$code"
    fi
    echo "LIVE_STAGE3_ODOM=PASS_NO_MESSAGES_IN_6_SECONDS"
    ros2 run leroy_nav2_perception live_costmap_probe
'

echo "LIVE_STAGE3_START=PASS_CAMERA_ONLY_CONCURRENT_DDS"
echo "Tracking and metric localization are NOT validated; motors remain disabled."
echo "For shutdown: podman stop leroy-live-xrslam && podman stop leroy-live-costmap"
