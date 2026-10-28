#!/usr/bin/env bash
# HARDWARE-FREE: gated, pinned source images and owner-private immutable logs.
# Deliberately no host physical-capture code, device mapping, or ROS sensor launch.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "${BASH_SOURCE[0]}")"

: "${M23BV_M23H_EXPECTED_IMAGE_ID:?Require separately verified 64-hex M23H image ID}"
: "${M23BV_LOG_DIR:?Require an existing owner-private directory for logs}"
if [[ ! "$M23BV_M23H_EXPECTED_IMAGE_ID" =~ ^[[:xdigit:]]{64}$ ]]; then
  echo 'M23BV_BUILD=INVALID_M23H_EXPECTED_ID' >&2
  exit 2
fi
if [[ "$M23BV_LOG_DIR" != /* || ! -d "$M23BV_LOG_DIR" ||
      -L "$M23BV_LOG_DIR" || ! -w "$M23BV_LOG_DIR" ]]; then
  echo 'M23BV_BUILD=INVALID_LOG_DIRECTORY' >&2
  exit 2
fi
check_id() {
  local tag="$1" expected="$2" actual
  actual="$(podman image inspect "$tag" --format '{{.Id}}')" || return 2
  actual="${actual#sha256:}"
  if [[ "${actual,,}" != "${expected,,}" ]]; then
    echo "M23BV_IMAGE_ID_MISMATCH=$tag" >&2
    return 2
  fi
  printf 'M23BV_SOURCE_IMAGE_ID=%s:%s\n' "$tag" "$actual"
}
check_id localhost/leroy-ros:xrslam-pthreads-opencv-v2 \
  40351f56ff4fa54da881178050b91e74db0450b45b07a51ca7bc67b7590112d1
check_id localhost/leroy-sensors:m23h-bounded-recording \
  "$M23BV_M23H_EXPECTED_IMAGE_ID"

# No production container startup, no host network, no device mappings.
status="$(podman inspect leroy-ros --format '{{.State.Status}}' 2>/dev/null || true)"
if [[ "$status" == 'running' ]]; then
  echo 'M23BV_BUILD=PRODUCTION_CONTAINER_RUNNING' >&2
  exit 2
fi

build_log="$(mktemp --tmpdir="$M23BV_LOG_DIR" m23bv_build_XXXXXXXX.log)"
run_log="$(mktemp --tmpdir="$M23BV_LOG_DIR" m23bv_selftest_XXXXXXXX.log)"
set +e
podman build --network none --no-cache \
  -f Dockerfile.m23bv-capture-prep \
  -t localhost/leroy-sensors:m23bv-capture-prep . \
  >"$build_log" 2>&1
build_rc=$?
set -e
printf 'M23BV_BUILD_EXIT=%s\n' "$build_rc"
printf 'M23BV_BUILD_LOG=%s\n' "$build_log"
sha256sum "$build_log"
if [[ "$build_rc" != 0 ]]; then
  echo 'M23BV_BUILD=BLOCKED' >&2
  exit 2
fi
new_id="$(podman image inspect localhost/leroy-sensors:m23bv-capture-prep --format '{{.Id}}')"
printf 'M23BV_DERIVED_IMAGE_ID=%s\n' "$new_id"

set +e
podman run --rm --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  localhost/leroy-sensors:m23bv-capture-prep \
  >"$run_log" 2>&1
run_rc=$?
set -e
printf 'M23BV_SELFTEST_EXIT=%s\n' "$run_rc"
printf 'M23BV_SELFTEST_LOG=%s\n' "$run_log"
sha256sum "$run_log"
if [[ "$run_rc" != 0 ]] || ! grep -q 'M23BV_OFFLINE_UNIT_TESTS=PASS' "$run_log"; then
  echo 'M23BV_SELFTEST=BLOCKED' >&2
  exit 2
fi
echo 'M23BV_NO_DEVICE_BUILD_AND_UNIT_TEST=PASS'
echo 'M23BV_REAL_ROS_GRAPH_OR_BAG_ACCEPTANCE=NOT_TESTED'
echo 'M23BV_PHYSICAL_ACCESS=NONE'
