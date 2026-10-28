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
              m23bv_v3_stage3_nav2_configure_no_device_audit.py \
              m23bv_v3_stage3_nav2_configure_supervisor.py; do
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
    "m23bv_v3_stage3_nav2_configure_supervisor.py",
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
timeout --foreground --signal=TERM --kill-after=12s 155s \
  podman run --rm --pull=never \
    --network none --ipc private --pid private \
    --read-only --cap-drop all --security-opt no-new-privileges \
    --tmpfs /tmp:rw,nosuid,nodev,size=256m \
    --mount "type=bind,src=$PWD/m23bv_v3_stage3_nav2_configure_no_device.launch.py,dst=/opt/stage3_nav2_configure.launch.py,readonly" \
    --mount "type=bind,src=$PWD/m23bv_v3_stage3_nav2_configure_no_device_audit.py,dst=/opt/stage3_nav2_configure_audit.py,readonly" \
    --mount "type=bind,src=$PWD/m23bv_v3_stage3_nav2_configure_supervisor.py,dst=/opt/stage3_nav2_configure_supervisor.py,readonly" \
    --env ROS_DOMAIN_ID=170 \
    --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --env ROS_LOG_DIR=/tmp/stage3-nav2-ros \
    --env PYTHONDONTWRITEBYTECODE=1 \
    --env M23BV_STAGE3_NAV2_CONFIGURE_NO_DEVICE=1 \
    --entrypoint /bin/bash "$image" -c '
# ROS 2 Jazzy generated setup.sh can read optional AMENT_TRACE_SETUP_FILES
# before setting it. Source trusted, pinned image overlays with nounset
# disabled, then restore nounset for every subsequent runtime operation.
set -Eeo pipefail
. /opt/ros/jazzy/setup.sh
. /leroy_ws/install/setup.sh
set -u
test ! -e /dev/video2 && test ! -e /dev/ttyACM0
# A separate new process session prevents noninteractive Bash children
# inheriting ignored SIGINT. Supervisor checks BOTH audit and bounded exit.
exec python3 -B /opt/stage3_nav2_configure_supervisor.py
' || fail LIFECYCLE_OR_SHUTDOWN

echo 'STAGE3_NAV2_CONFIGURE=PASS_NO_DEVICE_LIFECYCLE'
echo 'STAGE3_NAV2_REAL_ODOM_TF=NOT_SUPPLIED'
echo 'STAGE3_NAV2_GOALS_ACTIONS_AND_MOTORS=NOT_AUTHORIZED'
