#!/usr/bin/env bash
# Operator-initiated camera-only diagnostic; no MCU or motor interface.
set -Eeuo pipefail
: "${M23K_HOST_CAMERA_BY_ID:?Set current front camera /dev/v4l/by-id/... index0}"
: "${M23K_OPERATOR_CAMERA_ONLY_APPROVED:?Set YES for operator approval}"
if [[ "$M23K_OPERATOR_CAMERA_ONLY_APPROVED" != "YES" ]]; then
  echo "M23K_HOST_PRECONDITION=OPERATOR_APPROVAL_REQUIRED" >&2
  exit 2
fi
if [[ "$M23K_HOST_CAMERA_BY_ID" != /dev/v4l/by-id/*-video-index0 ||
      ! -L "$M23K_HOST_CAMERA_BY_ID" ]]; then
  echo "M23K_HOST_PRECONDITION=STABLE_CAMERA_ID_REQUIRED" >&2
  exit 2
fi
CAMERA_DEV="$(readlink -f -- "$M23K_HOST_CAMERA_BY_ID")"
if [[ ! "$CAMERA_DEV" =~ ^/dev/video[0-9]+$ || ! -c "$CAMERA_DEV" ]]; then
  echo "M23K_HOST_PRECONDITION=INVALID_CAMERA_DEVICE" >&2
  exit 2
fi
if [[ "$(podman inspect leroy-ros --format '{{.State.Status}}')" != "exited" ]]; then
  echo "M23K_HOST_PRECONDITION=PRODUCTION_CONTAINER_RUNNING" >&2
  exit 2
fi
podman image exists localhost/leroy-sensors:m23k-camera-path-probe
command -v fuser >/dev/null || { echo "M23K_HOST_PRECONDITION=FUSER_MISSING" >&2; exit 2; }
if fuser "$CAMERA_DEV" >/dev/null 2>&1; then
  echo "M23K_HOST_PRECONDITION=CAMERA_BUSY" >&2
  exit 2
fi
echo "M23K_CAMERA_BY_ID=$M23K_HOST_CAMERA_BY_ID"
echo "M23K_CAMERA_SELECTED=$CAMERA_DEV"
echo "M23K_CAPTURE_PATH=CAMERA_ONLY_NO_MCU"
read -r -p "Type AUTHORIZE_M23K for camera-only diagnosis: " ANSWER
if [[ "$ANSWER" != "AUTHORIZE_M23K" ]]; then
  echo "M23K_HOST_PRECONDITION=OPERATOR_DECLINED" >&2
  exit 2
fi
exec podman run --rm --name leroy-m23k-camera-probe \
  --network none --ipc private --pid private \
  --shm-size 512m \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --group-add keep-groups \
  --device "$CAMERA_DEV":/dev/video2 \
  -e M23K_PROBE_MODE=live \
  -e M23K_OPERATOR_CAMERA_ONLY_APPROVED=YES \
  localhost/leroy-sensors:m23k-camera-path-probe
