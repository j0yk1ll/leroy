#!/usr/bin/env bash
# Build only the production base. Never launches hardware or a ROS graph.
set -Eeuo pipefail
cd "$(dirname "$0")"
[[ "${1:-}" == --build && $# -eq 1 ]] || { echo 'Usage: bash ./build_current_images.sh --build' >&2; exit 2; }
[[ "$(id -u)" != 0 ]] || { echo 'Rootless Podman required' >&2; exit 2; }
command -v podman >/dev/null || { echo 'Podman unavailable' >&2; exit 2; }
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] || { echo 'Rootless Podman required' >&2; exit 2; }
podman build -f Dockerfile -t localhost/leroy-ros:production .
echo 'LEROY_PRODUCTION_IMAGE=BUILT_MOTION_INHIBITED'
