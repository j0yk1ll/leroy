#!/usr/bin/env bash
# Rebuild the sensor-only Stage 3 image family from this exact checkout.
# Does not run containers or enable any hardware.
set -Eeuo pipefail
cd "$(dirname "$0")"

if [[ "${1-}" != "--build" || $# -ne 1 ]]; then
    printf 'Usage: %s --build\n' "$0" >&2
    exit 2
fi
[[ "$(id -u)" -ne 0 ]] || { echo "BLOCKED: use rootless Podman" >&2; exit 2; }
command -v podman >/dev/null
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
    { echo "BLOCKED: Podman is not rootless" >&2; exit 2; }

# The base image retains the pinned patched usb_cam driver and CPU DepthART.
# The costmap image builds only perception/description/fixture packages.
# The separate XRSLAM image is not asked to start an additional camera.
podman build -f Dockerfile -t localhost/leroy-ros:jazzy .
podman build -f Dockerfile.live-perception -t localhost/leroy-ros:perception .
podman build -f Dockerfile.live-costmap -t localhost/leroy-ros:costmap .
podman build -f Dockerfile.xrslam -t localhost/leroy-ros:xrslam .

echo 'LIVE_STAGE3_IMAGES=BUILT_NOT_LAUNCHED'
