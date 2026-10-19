#!/usr/bin/env bash
# M23N: operator camera-only ROS subscriber QoS check. NO STM32.
set -Eeuo pipefail
: "${M23K_HOST_CAMERA_BY_ID:?Set existing Wed front camera by-id symlink}"
: "${M23N_SUB_QOS:?Choose best_effort_100 or reliable_100}"
: "${M23N_OPERATOR_CAMERA_ONLY_APPROVED:?Must equal YES}"
if [[ "$M23N_OPERATOR_CAMERA_ONLY_APPROVED" != "YES" ]]; then
  echo "M23N_HOST_PRECONDITION=OPERATOR_APPROVAL_REQUIRED" >&2
  exit 2
fi
case "$M23N_SUB_QOS" in
  best_effort_100|reliable_100) ;;
  *) echo "M23N_HOST_PRECONDITION=UNSUPPORTED_QOS" >&2; exit 2 ;;
esac
if [[ "$M23K_HOST_CAMERA_BY_ID" != "/dev/v4l/by-id/usb-Wed_Camera_Wed_Camera_20240420112206-video-index0" ||
      ! -L "$M23K_HOST_CAMERA_BY_ID" ]]; then
  echo "M23N_HOST_PRECONDITION=VERIFIED_FRONT_CAMERA_ID_REQUIRED" >&2
  exit 2
fi
CAMERA_DEV="$(readlink -f -- "$M23K_HOST_CAMERA_BY_ID")"
if [[ ! "$CAMERA_DEV" =~ ^/dev/video[0-9]+$ || ! -c "$CAMERA_DEV" ]]; then
  echo "M23N_HOST_PRECONDITION=INVALID_CAMERA_DEVICE" >&2
  exit 2
fi
if [[ "$(podman inspect leroy-ros --format '{{.State.Status}}')" != "exited" ]]; then
  echo "M23N_HOST_PRECONDITION=PRODUCTION_CONTAINER_RUNNING" >&2
  exit 2
fi
podman image exists localhost/leroy-sensors:m23n-real-camera-qos
command -v fuser >/dev/null || { echo "M23N_HOST_PRECONDITION=FUSER_MISSING" >&2; exit 2; }
if fuser "$CAMERA_DEV" >/dev/null 2>&1; then
  echo "M23N_HOST_PRECONDITION=CAMERA_BUSY" >&2
  exit 2
fi
echo "M23N_CAMERA_BY_ID=$M23K_HOST_CAMERA_BY_ID"
echo "M23N_CAMERA_SELECTED=$CAMERA_DEV"
echo "M23N_RECEIVER_QOS_SELECTED=$M23N_SUB_QOS"
echo "M23N_CAMERA_ONLY_NO_MCU=TRUE"
read -r -p "Type AUTHORIZE_M23N for camera-only QoS diagnosis: " ANSWER
if [[ "$ANSWER" != "AUTHORIZE_M23N" ]]; then
  echo "M23N_HOST_PRECONDITION=OPERATOR_DECLINED" >&2
  exit 2
fi
exec podman run --rm --name leroy-m23n-camera-qos \
  --network none --ipc private --pid private \
  --shm-size 512m \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --group-add keep-groups \
  --device "$CAMERA_DEV":/dev/video2 \
  -e M23K_PROBE_MODE=live \
  -e M23K_PROBE_VARIANT=raw_only_mjpeg \
  -e M23K_OPERATOR_CAMERA_ONLY_APPROVED=YES \
  -e M23N_CAMERA_QOS_ONLY=1 \
  -e M23N_SUB_QOS="$M23N_SUB_QOS" \
  localhost/leroy-sensors:m23n-real-camera-qos
