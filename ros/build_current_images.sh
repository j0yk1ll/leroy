#!/usr/bin/env bash
# Rebuild the CURRENT non-actuating Stage 3 + persistent IMU/yaw image family
# from this checkout. No host devices or motors are started by this script.
set -Eeuo pipefail
cd "$(dirname "$0")"
if [[ $# -ne 1 || "$1" != --build ]]; then
  echo "Usage: bash ./build_current_images.sh --build" >&2
  exit 2
fi
[[ "$(id -u)" != 0 ]] || { echo "Use rootless Podman" >&2; exit 2; }
command -v podman >/dev/null || { echo "Podman unavailable" >&2; exit 2; }
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  { echo "Rootless Podman required" >&2; exit 2; }
for required in Dockerfile Dockerfile.live-perception Dockerfile.live-costmap \
  Dockerfile.xrslam Dockerfile.live-stage4-imu Dockerfile.live-stage4c-capture; do
  [[ -f "$required" ]] || { echo "Missing build input $required" >&2; exit 2; }
done
# Keep the proven Stage 3 build sequence unchanged.
bash ./live_stage3_build.sh --build
# The live_boot_session / live_boot_yaw launchers depend on these tags,
# which were NOT produced by the older Stage 3-only build script.
podman build -f Dockerfile.live-stage4-imu \
  -t localhost/leroy-ros:stage4b-imu .
podman tag localhost/leroy-ros:stage4b-imu \
  localhost/leroy-ros:stage4-imu
podman build -f Dockerfile.live-stage4c-capture \
  -t localhost/leroy-ros:stage4c-capture-offline .
for image in jazzy perception costmap xrslam stage4-imu stage4b-imu stage4c-capture-offline; do
  podman image exists "localhost/leroy-ros:$image" ||
    { echo "Missing rebuilt image $image" >&2; exit 2; }
done
echo 'LEROY_CURRENT_IMAGES=BUILT_FROM_CHECKOUT_NO_HARDWARE_LAUNCHED'
