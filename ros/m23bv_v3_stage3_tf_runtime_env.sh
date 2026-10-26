#!/usr/bin/env bash
# Source this from bash in the Stage 3 TF-only candidate runtime.
# It adds the new YAML-only description as an ament overlay above the
# EXISTING Leroy Jazzy/XRSLAM/DepthART runtime without mutating it.
# Sourcing does not start ROS nodes, camera, MCU, bridge or motors.
# ROS generated setups read unset optional variables with nounset enabled.
# Temporarily turn off -u only around the trusted image setup scripts.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "STAGE3_TF_RUNTIME_ENV=BLOCKED_MUST_SOURCE_NOT_EXECUTE" >&2
  exit 2
fi
case "$-" in *u*) _leroy_stage3_tf_restore_u=1 ;; *) _leroy_stage3_tf_restore_u=0 ;; esac
set +u
. /opt/ros/jazzy/setup.sh
. /leroy_ws/install/setup.sh
. /opt/leroy_stage3_tf_ws/install/local_setup.sh
if [[ "$_leroy_stage3_tf_restore_u" == 1 ]]; then
  set -u
fi
unset _leroy_stage3_tf_restore_u
