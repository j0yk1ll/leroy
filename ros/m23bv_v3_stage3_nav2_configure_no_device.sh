#!/usr/bin/env bash
# Stage 3 isolated Nav2 application Configure/Cleanup only.
# No sensors, serial, image devices, /odom, TF, goals, fake measurements
# or actuator topics. NEVER transitions a lifecycle node to Active.
set -Eeuo pipefail
cd "$(dirname -- "$0")"
fail() { echo "STAGE3_NAV2_CONFIGURE=BLOCKED_$1" >&2; exit 2; }
[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] ||
  fail BRANCH
[[ -z "$(git status --porcelain)" ]] || fail DIRTY_SOURCE
for source in m23bv_v3_stage3_nav2_configure_no_device.launch.py \
              m23bv_v3_stage3_nav2_configure_no_device_audit.py; do
  [[ -f "$source" && ! -L "$source" ]] || fail SOURCE_MISSING
  [[ "$(git hash-object "$source")" == "$(git rev-parse "HEAD:ros/$source")" ]] ||
    fail SOURCE_DRIFT
done
python3 -B - <<'PY'
import ast
from pathlib import Path
for path in (
    "m23bv_v3_stage3_nav2_configure_no_device.launch.py",
    "m23bv_v3_stage3_nav2_configure_no_device_audit.py",
):
    ast.parse(Path(path).read_text(), filename=path)
    print("STAGE3_NAV2_CONFIGURE_SOURCE_PASS=" + path, flush=True)
PY
command -v podman >/dev/null 2>&1 || fail PODMAN_MISSING
[[ "$(id -u)" != 0 ]] || fail ROOT_DISALLOWED
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail NOT_ROOTLESS
[[ -z "$(podman ps --format '{{.Names}}')" ]] || fail OTHER_CONTAINERS_RUNNING
image=localhost/leroy-nav2:jazzy
image_id=89b3733f7aa281b6083d9078a11fe10e2f4f096f2aca5511a108d4873a3cc7c4
observed="$(podman image inspect "$image" --format '{{.Id}}')" ||
  fail IMAGE_MISSING
observed="$(printf '%s' "$observed" | sed 's/^sha256://')"
[[ "$observed" == "$image_id" ]] || fail IMAGE_PIN_DRIFT
echo "STAGE3_NAV2_CONFIGURE_IMAGE_ID=$image_id"

# Image entrypoint is forcibly overridden. Only this Nav2 image runs;
# no camera, STM32, bridge, host network, host IPC or runtime device mounts.
timeout --foreground --signal=INT --kill-after=12s 110s \
  podman run --rm --pull=never \
    --network none --ipc private --pid private \
    --read-only --cap-drop all --security-opt no-new-privileges \
    --tmpfs /tmp:rw,nosuid,nodev,size=256m \
    --mount "type=bind,src=$PWD/m23bv_v3_stage3_nav2_configure_no_device.launch.py,dst=/opt/stage3_nav2_configure.launch.py,readonly" \
    --mount "type=bind,src=$PWD/m23bv_v3_stage3_nav2_configure_no_device_audit.py,dst=/opt/stage3_nav2_configure_audit.py,readonly" \
    --env ROS_DOMAIN_ID=170 \
    --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --env ROS_LOG_DIR=/tmp/stage3-nav2-ros \
    --env PYTHONDONTWRITEBYTECODE=1 \
    --env M23BV_STAGE3_NAV2_CONFIGURE_NO_DEVICE=1 \
    --entrypoint /bin/bash "$image" -c '
set -Eeuo pipefail
. /opt/ros/jazzy/setup.sh
. /leroy_ws/install/setup.sh
test ! -e /dev/video2 && test ! -e /dev/ttyACM0
ros2 launch /opt/stage3_nav2_configure.launch.py >/tmp/stage3-nav2-launch.log 2>&1 &
launch_pid=$!
cleanup() {
  trap - EXIT INT TERM
  kill -INT "$launch_pid" >/dev/null 2>&1 || true
  wait "$launch_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap "exit 130" INT
trap "exit 143" TERM
if ! python3 -B /opt/stage3_nav2_configure_audit.py; then
  echo "--- NAV2 CONFIGURE FAILURE: BOUNDED CONTAINER LOG ---" >&2
  tail -n 90 /tmp/stage3-nav2-launch.log >&2 || true
  exit 2
fi
if ! kill -0 "$launch_pid" >/dev/null 2>&1; then
  echo "STAGE3_NAV2_CONFIGURE=BLOCKED_NAV2_LAUNCH_EARLY_EXIT" >&2
  tail -n 60 /tmp/stage3-nav2-launch.log >&2 || true
  exit 2
fi
' || fail LIFECYCLE_OR_SHUTDOWN

echo 'STAGE3_NAV2_CONFIGURE=PASS_NO_DEVICE_LIFECYCLE'
echo 'STAGE3_NAV2_REAL_ODOM_TF=NOT_SUPPLIED'
echo 'STAGE3_NAV2_GOALS_ACTIONS_AND_MOTORS=NOT_AUTHORIZED'
