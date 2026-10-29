#!/usr/bin/env bash
# Build the complete current Leroy sensor runtime, independent of old
# stage milestones. No host devices or motors are started by this script.
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
# Build the complete family in dependency order. The stage-labelled
# image tags below are internal compatibility contracts for the already
# physically validated source wrappers; they are not distinct user modes.
podman build -f Dockerfile -t localhost/leroy-ros:jazzy .
podman build -f Dockerfile.live-perception -t localhost/leroy-ros:perception .
podman build -f Dockerfile.live-costmap -t localhost/leroy-ros:costmap .
podman build -f Dockerfile.xrslam -t localhost/leroy-ros:xrslam .
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
