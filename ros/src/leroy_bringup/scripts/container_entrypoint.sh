#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

# Prefer an explicit, stable camera device. If unset and exactly one video
# device exists, use it rather than assuming /dev/video2 survives USB replug.
# Multiple devices require CAMERA_DEVICE to avoid selecting the wrong camera.
if [[ -n "${CAMERA_DEVICE:-}" ]]; then
    camera_device="${CAMERA_DEVICE}"
else
    shopt -s nullglob
    video_devices=(/dev/video[0-9]*)
    shopt -u nullglob
    if (( ${#video_devices[@]} != 1 )); then
        echo "ERROR: Expected one /dev/video* device; found ${#video_devices[@]}. Set CAMERA_DEVICE explicitly." >&2
        exit 1
    fi
    camera_device="${video_devices[0]}"
fi
CAMERA_DEVICE="${camera_device}"
CAMERA_INFO_FILE="${CAMERA_INFO_FILE:-$(ros2 pkg prefix --share leroy_sensors)/config/front_camera.yaml}"

if [[ ! -e "${CAMERA_DEVICE}" ]]; then
    echo "ERROR: Camera device does not exist: ${CAMERA_DEVICE}"
    exit 1
fi

if [[ ! -r "${CAMERA_DEVICE}" || ! -w "${CAMERA_DEVICE}" ]]; then
    echo "ERROR: Cannot access camera device: ${CAMERA_DEVICE}"
    exit 1
fi

if [[ ! -f "${CAMERA_INFO_FILE}" ]]; then
    echo "ERROR: Camera calibration file not found: ${CAMERA_INFO_FILE}"
    exit 1
fi

echo "Starting Leroy bringup: camera, rectifier, sensor TF and health nodes."
echo "MCU bridge is disabled in viz; the leroy-imu container owns serial."
exec ros2 launch leroy_bringup robot.launch.py \
    video_device:="${CAMERA_DEVICE}" \
    camera_info_file:="${CAMERA_INFO_FILE}" \
    use_mcu_bridge:=false \
    "$@"