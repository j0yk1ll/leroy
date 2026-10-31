#!/usr/bin/env bash
# Leroy production operator interface. Motion remains fail-closed until
# physical commissioning and a single supervised hardware command owner exist.
set -Eeuo pipefail
cd "$(dirname "$0")"
usage() {
  printf '%s\n' 'Usage: bash ./leroy.sh {check|build|run|status|stop|help}'     '  check  Validate production source and fail-closed contracts'     '  build  Build production ROS base image (no devices opened)'     '  run    Refused until the production supervisor and hardware are commissioned'     '  status Show production container state'     '  stop   Stop the production container, if present'
}
case "${1:-help}" in
  help|--help|-h) usage ;;
  check) exec bash ./check_current_checkout.sh ;;
  build) exec bash ./build_current_images.sh --build ;;
  run)
    echo 'LEROY_RUNTIME=BLOCKED_PRODUCTION_MOTION_NOT_COMMISSIONED' >&2
    exit 2 ;;
  status)
    if ! command -v podman >/dev/null; then
      echo 'LEROY_RUNTIME=PODMAN_UNAVAILABLE'
    elif podman container exists leroy-production; then
      podman inspect -f 'LEROY_RUNTIME={{.State.Status}}' leroy-production
    else
      echo 'LEROY_RUNTIME=STOPPED'
    fi ;;
  stop)
    if command -v podman >/dev/null && podman container exists leroy-production; then
      echo 'LEROY_RUNTIME=STOP_UNVERIFIED_DISCONNECT_MOTOR_POWER' >&2
      podman stop -t 10 leroy-production >/dev/null || exit 2
      exit 2
    fi
    echo 'LEROY_RUNTIME=STOPPED_NO_MOTION_AUTHORITY' ;;
  *) usage >&2; exit 2 ;;
esac
