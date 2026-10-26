#!/usr/bin/env bash
# Stage 3: install current Leroy description in disposable container tmpfs,
# launch six REAL static TF publishers and observe /tf_static. NO hardware.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "$0")"
fail() { echo "STAGE3_SENSOR_TF_INTEGRATION=BLOCKED_$1" >&2; exit 2; }

[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] ||
  fail WRONG_BRANCH
# Verify ONLY the source inputs used here; respect unrelated local deletions.
for file in src/leroy_description/CMakeLists.txt \
            src/leroy_description/package.xml \
            src/leroy_description/launch/description.launch.py \
            src/leroy_description/config/sensor_mounts.yaml \
            m23bv_v3_stage3_sensor_tf_live_verify.py; do
  [[ -f "$file" && ! -L "$file" ]] || fail SOURCE_MISSING
  [[ "$(git hash-object "$file")" == "$(git rev-parse "HEAD:ros/$file")" ]] ||
    fail INPUT_SOURCE_DRIFT
done
[[ ! -e src/leroy_description/urdf/leroy.urdf.xacro ]] ||
  fail LEGACY_URDF_PRESENT
[[ "$(id -u)" != 0 ]] || fail ROOT_NOT_ALLOWED
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail ROOTLESS_REQUIRED

sensor=localhost/leroy-sensors:m23bv-reliable-rectifier-v3
expected=f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c
actual="$(podman image inspect "$sensor" --format '{{.Id}}')" || fail IMAGE_MISSING
actual="$(printf '%s' "$actual" | sed 's/^sha256://')"
[[ "$actual" == "$expected" ]] || fail IMAGE_DRIFT
echo "STAGE3_SENSOR_TF_INTEGRATION_PINNED_SENSOR_IMAGE=$actual"

# All mounts are read-only repository source only. No --device, no
# /run/podman.sock, no host ROS socket, no host workspace, no network.
# The new package build/install/logs live only in disposable /tmp tmpfs.
podman run --rm --interactive --pull=never --network none \
  --ipc private --pid private \
  --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=384m \
  --env PYTHONDONTWRITEBYTECODE=1 \
  --env ROS_HOME=/tmp/leroy-ros-home \
  --env ROS_LOG_DIR=/tmp/leroy-ros-logs \
  --env XDG_CACHE_HOME=/tmp/leroy-cache \
  --env ROS_DOMAIN_ID=170 \
  --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --mount type=bind,src="$PWD/src/leroy_description",dst=/opt/leroy_tf_source,readonly \
  --mount type=bind,src="$PWD/m23bv_v3_stage3_sensor_tf_live_verify.py",dst=/opt/leroy_tf_live_verify.py,readonly \
  --entrypoint /bin/bash "$sensor" -s <<'LEROY_TF_INTEGRATION_CONTAINER' || fail CONTAINER_BUILD_OR_RUNTIME_TEST
set -Eeuo pipefail
shopt -s nullglob
for device in /dev/video* /dev/ttyACM* /dev/ttyUSB*; do
  echo "STAGE3_SENSOR_TF_INTEGRATION=BLOCKED_PHYSICAL_DEVICE_VISIBLE:$device" >&2
  exit 2
done
test "$ROS_DOMAIN_ID" = 170
test "$ROS_AUTOMATIC_DISCOVERY_RANGE" = LOCALHOST
test "$RMW_IMPLEMENTATION" = rmw_fastrtps_cpp
test "$ROS_LOG_DIR" = /tmp/leroy-ros-logs
test "$ROS_HOME" = /tmp/leroy-ros-home
test "$XDG_CACHE_HOME" = /tmp/leroy-cache
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME" "$XDG_CACHE_HOME"
. /opt/ros/jazzy/setup.sh
command -v colcon >/dev/null
command -v ros2 >/dev/null
python3 -B -c 'import rclpy, launch_ros, tf2_msgs, yaml'
mkdir -p /tmp/leroy_static_tf_ws/src
cp -a /opt/leroy_tf_source /tmp/leroy_static_tf_ws/src/leroy_description
test ! -e /tmp/leroy_static_tf_ws/src/leroy_description/urdf
cd /tmp/leroy_static_tf_ws
colcon --log-base /tmp/leroy_static_tf_ws/log build \
  --base-paths /tmp/leroy_static_tf_ws/src \
  --packages-select leroy_description \
  --merge-install \
  --build-base /tmp/leroy_static_tf_ws/build \
  --install-base /tmp/leroy_static_tf_ws/install \
  --cmake-args -DBUILD_TESTING=OFF \
  --event-handlers console_direct+
. /tmp/leroy_static_tf_ws/install/local_setup.sh
test "$(ros2 pkg prefix leroy_description)" = /tmp/leroy_static_tf_ws/install
echo STAGE3_SENSOR_TF_INTEGRATION_BUILD=PASS_EPHEMERAL_PACKAGE_INSTALL
python3 -B /opt/leroy_tf_live_verify.py
echo STAGE3_SENSOR_TF_INTEGRATION_CONTAINER_DEVICES=NONE
LEROY_TF_INTEGRATION_CONTAINER

echo STAGE3_SENSOR_TF_INTEGRATION=PASS_REAL_TF_STATIC_NO_DEVICE
echo STAGE3_SENSOR_TF_INTEGRATION_HOST_DEVICES=NOT_MAPPED
echo STAGE3_SENSOR_TF_INTEGRATION_PINNED_IMAGE=UNCHANGED
echo STAGE3_SENSOR_TF_INTEGRATION_GEOMETRY=PROVISIONAL
echo STAGE3_SENSOR_TF_INTEGRATION_MCU_CAMERA_MOTORS=NOT_AUTHORIZED
