#!/usr/bin/env bash
set -Eeuo pipefail
cd "$(dirname "$0")"
[[ "${1:-}" == --build && $# -eq 1 ]] || { echo 'Usage: bash ./build.sh --build' >&2; exit 2; }
[[ "$(id -u)" != 0 ]] || { echo 'Rootless Podman required' >&2; exit 2; }
command -v podman >/dev/null || { echo 'Podman unavailable' >&2; exit 2; }
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] || { echo 'Rootless Podman required' >&2; exit 2; }
podman build -f Dockerfile -t localhost/leroy-ros:production .
