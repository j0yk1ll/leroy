#!/usr/bin/env bash
# M23O: human-authorized physical Wed USB camera + image_proc ONLY.
# No STM32, recorder, IMU, motor bridge or production QoS changes.
set -Eeuo pipefail
: "${M23K_HOST_CAMERA_BY_ID:?Set the verified Wed camera by-id path}"
: "${M23O_OPERATOR_CAMERA_ONLY_APPROVED:?Must equal YES}"
if [[ "$M23O_OPERATOR_CAMERA_ONLY_APPROVED" != "YES" ]]; then
  echo "M23O_HOST_PRECONDITION=APPROVAL_REQUIRED" >&2
  exit 2
fi
if [[ "$M23K_HOST_CAMERA_BY_ID" != "/dev/v4l/by-id/usb-Wed_Camera_Wed_Camera_20240420112206-video-index0" ||
      ! -L "$M23K_HOST_CAMERA_BY_ID" ]]; then
  echo "M23O_HOST_PRECONDITION=VERIFIED_WED_CAMERA_REQUIRED" >&2
  exit 2
fi
CAMERA_DEV="$(readlink -f -- "$M23K_HOST_CAMERA_BY_ID")"
if [[ ! "$CAMERA_DEV" =~ ^/dev/video[0-9]+$ || ! -c "$CAMERA_DEV" ]]; then
  echo "M23O_HOST_PRECONDITION=INVALID_CAMERA_DEVICE" >&2
  exit 2
fi
if [[ "$(podman inspect leroy-ros --format '{{.State.Status}}')" != "exited" ]]; then
  echo "M23O_HOST_PRECONDITION=PRODUCTION_CONTAINER_MUST_BE_STOPPED" >&2
  exit 2
fi
podman image exists localhost/leroy-sensors:m23o-rect-camera-qos
command -v fuser >/dev/null || {
  echo "M23O_HOST_PRECONDITION=FUSER_MISSING" >&2
  exit 2
}
if fuser "$CAMERA_DEV" >/dev/null 2>&1; then
  echo "M23O_HOST_PRECONDITION=CAMERA_BUSY" >&2
  exit 2
fi
echo "M23O_CAMERA_BY_ID=$M23K_HOST_CAMERA_BY_ID"
echo "M23O_CAMERA_SELECTED=$CAMERA_DEV"
echo "M23O_CAMERA_WITH_RECTIFIER_ONLY=TRUE"
echo "M23O_SUB_QOS=reliable_100"
read -r -p "Type AUTHORIZE_M23O to launch ONLY the isolated camera and rectifier: " ANSWER
if [[ "$ANSWER" != "AUTHORIZE_M23O" ]]; then
  echo "M23O_HOST_PRECONDITION=OPERATOR_DECLINED" >&2
  exit 2
fi
exec podman run --rm --name leroy-m23o-camera-rect-qos \
  --network none --ipc private --pid private \
  --shm-size 512m \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --group-add keep-groups \
  --device "$CAMERA_DEV":/dev/video2 \
  -e M23K_PROBE_MODE=live \
  -e M23K_PROBE_VARIANT=full_mjpeg \
  -e M23K_OPERATOR_CAMERA_ONLY_APPROVED=YES \
  -e M23O_RECT_QOS_ONLY=1 \
  -e M23O_SUB_QOS=reliable_100 \
  localhost/leroy-sensors:m23o-rect-camera-qos
