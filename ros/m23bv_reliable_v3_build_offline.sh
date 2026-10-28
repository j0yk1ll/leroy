#!/usr/bin/env bash
# No-device V3 explicit RELIABLE rectifier package compiled atop pinned SIGINT-v2 image.
# Never retag accepted M23BV, v1, v2, or M23H; fails closed without dev tools.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "${BASH_SOURCE[0]}")"
PARENT=localhost/leroy-sensors:m23bv-camera-ready-sigint-v2
EXPECTED_ID=5cdf67f984995003180741c6d14883caf97acfa2b9611e37b34f6015c0dcbc9e
OUTPUT=localhost/leroy-sensors:m23bv-reliable-rectifier-v3
: "${M23BV_LOG_DIR:?Set existing owner-private log directory}"
if [[ "$M23BV_LOG_DIR" != /* || ! -d "$M23BV_LOG_DIR" ||
      -L "$M23BV_LOG_DIR" || ! -w "$M23BV_LOG_DIR" ]]; then
  echo 'M23BV_RELIABLE_V3_BUILD=BLOCKED_LOG_DIR' >&2
  exit 2
fi
actual="$(podman image inspect "$PARENT" --format '{{.Id}}')"
actual="${actual#sha256:}"
if [[ "$actual" != "$EXPECTED_ID" ]]; then
  echo 'M23BV_RELIABLE_V3_BUILD=BLOCKED_UNPINNED_PARENT' >&2
  exit 2
fi
status="$(podman inspect leroy-ros --format '{{.State.Status}}' 2>/dev/null || true)"
if [[ "$status" == running ]]; then
  echo 'M23BV_RELIABLE_V3_BUILD=BLOCKED_PRODUCTION_RUNNING' >&2
  exit 2
fi
# Refuse an accidental overwrite of an existing candidate tag.
if podman image exists "$OUTPUT"; then
  echo 'M23BV_RELIABLE_V3_BUILD=BLOCKED_CANDIDATE_TAG_EXISTS' >&2
  exit 2
fi

bash ./m23bv_offline_gate.sh
log="$(mktemp --tmpdir="$M23BV_LOG_DIR" m23bv_reliable_v3_build_XXXXXXXX.log)"
set +e
podman build --network none --pull=never --no-cache \
  -f Dockerfile.m23bv-reliable-rectifier-v3 \
  -t "$OUTPUT" . >"$log" 2>&1
rc=$?
set -e
printf 'M23BV_RELIABLE_V3_BUILD_EXIT=%s\n' "$rc"
printf 'M23BV_RELIABLE_V3_BUILD_LOG=%s\n' "$log"
sha256sum "$log"
grep -E 'M23BV_|ERROR|Error|failed|FAILED' "$log" | tail -n 45 || true
if [[ "$rc" != 0 ]] || \
   ! grep -qx 'M23BV_V3_RELIABLE_RECTIFIER_OFFLINE_COMPILE=PASS' "$log"; then
  echo 'M23BV_RELIABLE_V3_CANDIDATE_BUILD=BLOCKED' >&2
  exit 2
fi
printf 'M23BV_RELIABLE_V3_CANDIDATE_IMAGE_ID=%s\n' \
  "$(podman image inspect "$OUTPUT" --format '{{.Id}}')"

runlog="$(mktemp --tmpdir="$M23BV_LOG_DIR" m23bv_reliable_v3_selftest_XXXXXXXX.log)"
set +e
podman run --rm --pull=never --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  "$OUTPUT" >"$runlog" 2>&1
run_rc=$?
set -e
printf 'M23BV_RELIABLE_V3_SELFTEST_EXIT=%s\n' "$run_rc"
printf 'M23BV_RELIABLE_V3_SELFTEST_LOG=%s\n' "$runlog"
sha256sum "$runlog"
if [[ "$run_rc" != 0 ]] || \
   ! grep -qx 'M23BV_OFFLINE_UNIT_TESTS=PASS' "$runlog"; then
  echo 'M23BV_RELIABLE_V3_CANDIDATE_SELFTEST=BLOCKED' >&2
  exit 2
fi
echo 'M23BV_RELIABLE_V3_CANDIDATE_BUILD=PASS'
echo 'M23BV_RELIABLE_V3_CANDIDATE_ROS_GRAPH_AND_LIFECYCLE=NOT_TESTED'
echo 'M23BV_RELIABLE_V3_CANDIDATE_PHYSICAL_CAPTURE=NOT_AUTHORIZED'
echo 'M23BV_PHYSICAL_ACCESS=NONE'
