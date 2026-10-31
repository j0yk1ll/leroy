#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video2}"
CAMERA_INFO_FILE="${CAMERA_INFO_FILE:-/config/camera/front_camera.yaml}"

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

exec ros2 launch leroy_bringup robot.launch.py \
    video_device:="${CAMERA_DEVICE}" \
    camera_info_file:="${CAMERA_INFO_FILE}" \
    "$@"