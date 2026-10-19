#!/usr/bin/env bash
# M23H operator-initiated physical sensor capture. NEVER run in CI.
# Maps only verified front camera and STM32 CDC into isolated M20/rosbag2.
set -Eeuo pipefail

: "${M23H_HOST_RECORDINGS:?Set existing offline output directory}"
: "${M23H_HOST_CAMERA:?Set current verified host /dev/videoN camera}"
: "${M23H_MCU_BY_ID:?Set current STM32 /dev/serial/by-id/... link}"
: "${M23H_SESSION_ID:?Set unique safe session identifier}"

if [[ "${M23H_OPERATOR_CAPTURE_APPROVED:-}" != "YES" ]]; then
  echo "M23H_HOST_PRECONDITION=NO_OPERATOR_APPROVAL" >&2
  exit 2
fi
if [[ "$M23H_HOST_RECORDINGS" != /* ||
      "$M23H_HOST_RECORDINGS" == "/" ||
      ! -d "$M23H_HOST_RECORDINGS" || -L "$M23H_HOST_RECORDINGS" ||
      ! -w "$M23H_HOST_RECORDINGS" ]]; then
  echo "M23H_HOST_PRECONDITION=OUTPUT_DIRECTORY_INVALID" >&2
  exit 2
fi
if [[ ! "$M23H_SESSION_ID" =~ ^[a-z0-9][a-z0-9_-]{0,47}$ ]]; then
  echo "M23H_HOST_PRECONDITION=SESSION_INVALID" >&2
  exit 2
fi
if [[ ! "$M23H_HOST_CAMERA" =~ ^/dev/video[0-9]+$ ||
      ! -c "$M23H_HOST_CAMERA" ]]; then
  echo "M23H_HOST_PRECONDITION=CAMERA_DEVICE_INVALID" >&2
  exit 2
fi
if [[ ! "$M23H_MCU_BY_ID" == /dev/serial/by-id/* ||
      ! -L "$M23H_MCU_BY_ID" ]]; then
  echo "M23H_HOST_PRECONDITION=MCU_STABLE_ID_MISSING" >&2
  exit 2
fi
MCU_DEV="$(readlink -f -- "$M23H_MCU_BY_ID")"
if [[ ! "$MCU_DEV" =~ ^/dev/ttyACM[0-9]+$ || ! -c "$MCU_DEV" ]]; then
  echo "M23H_HOST_PRECONDITION=MCU_ID_NOT_ACM" >&2
  exit 2
fi
if [[ "$MCU_DEV" == "$M23H_HOST_CAMERA" ]]; then
  echo "M23H_HOST_PRECONDITION=DUPLICATE_DEVICE" >&2
  exit 2
fi
if [[ "$(podman inspect leroy-ros --format '{{.State.Status}}')" != "exited" ]]; then
  echo "M23H_HOST_PRECONDITION=PRODUCTION_CONTAINER_RUNNING" >&2
  exit 2
fi
podman image exists localhost/leroy-sensors:m23h-bounded-recording
if ! command -v fuser >/dev/null; then
  echo "M23H_HOST_PRECONDITION=DEVICE_OWNERSHIP_TOOL_MISSING" >&2
  exit 2
fi
if fuser "$MCU_DEV" "$M23H_HOST_CAMERA" >/dev/null 2>&1; then
  echo "M23H_HOST_PRECONDITION=SENSOR_DEVICE_BUSY" >&2
  exit 2
fi
if [[ -e "$M23H_HOST_RECORDINGS/m23g_live_$M23H_SESSION_ID" ||
      -e "$M23H_HOST_RECORDINGS/m23h_capture_$M23H_SESSION_ID.log" ]]; then
  echo "M23H_HOST_PRECONDITION=SESSION_ALREADY_EXISTS" >&2
  exit 2
fi

echo "M23H_HOST_CAMERA_SELECTED=$M23H_HOST_CAMERA"
echo "M23H_MCU_STABLE_ID=$M23H_MCU_BY_ID"
echo "M23H_MCU_DEVICE_SELECTED=$MCU_DEV"
echo "M23H_CAPTURE=120_SECONDS_EXACT_THREE_SENSOR_TOPICS"
read -r -p "Confirm isolated physical state and approval. Type AUTHORIZE_M23H: " ANSWER
if [[ "$ANSWER" != "AUTHORIZE_M23H" ]]; then
  echo "M23H_HOST_PRECONDITION=OPERATOR_DECLINED" >&2
  exit 2
fi

# No bridge, Nav2, XRSLAM, motion topics or host ROS daemon.
exec podman run --rm --name leroy-m23h-real-sensor-record \
  --network none --ipc private --pid private \
  --shm-size 512m \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --group-add keep-groups \
  --device "$M23H_HOST_CAMERA":/dev/video2 \
  --device "$MCU_DEV":/dev/ttyACM0 \
  --volume "$M23H_HOST_RECORDINGS":/recordings:rw \
  -e M23H_SELFTEST=0 \
  -e M23H_OPERATOR_CAPTURE_APPROVED=YES \
  -e M23H_SESSION_ID="$M23H_SESSION_ID" \
  localhost/leroy-sensors:m23h-bounded-recording
