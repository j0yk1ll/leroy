#!/usr/bin/env bash
# Read-only, isolated, one-scenario XRSLAM bias-covariance sensitivity replay.
# No camera/serial devices, motor bridge, Nav2 or pose authority are started.
set -Eeuo pipefail

usage() {
  echo "Usage: $0 {low|mid|high} [bag_dir] [output_root]" >&2
  echo "Run one scenario at a time; NOT calibration. Requires isolated ROS_DOMAIN_ID=191." >&2
  exit 2
}
[[ $# -ge 1 && $# -le 3 ]] || usage
scenario="$1"
bag="${2:-/leroy_ws/calibration/leroy_cam_imu_05}"
results="${3:-/leroy_ws/calibration/xrslam_bias_sensitivity}"
case "$scenario" in
  low)  bg="1e-12"; ba="1e-10" ;;
  mid)  bg="1e-10"; ba="1e-8" ;;
  high) bg="1e-8";  ba="1e-6" ;;
  *) usage ;;
esac

# ROS setup scripts may read unset environment variables. Source with nounset
# disabled, then restore strict checking for the experiment itself.
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=191
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

[[ -f "$bag/metadata.yaml" ]] || { echo "Missing ROS bag metadata: $bag" >&2; exit 2; }
[[ -f /opt/xrslam/lib/libxrslam.so ]] || { echo "Missing custom XRSLAM runtime" >&2; exit 2; }
if [[ "$bag" != /leroy_ws/calibration/leroy_cam_imu_05 ]]; then
  echo "WARNING: bag must contain corrected camera stamps and three required topics." >&2
fi
# Fail closed rather than join another experiment/live ROS graph.
topics="$(ros2 topic list --no-daemon 2>/dev/null || ros2 topic list)"
if grep -Eq '^(/camera/image_rect|/imu/data_raw|/xrslam/tracking_status|/odom)$' <<<"$topics"; then
  echo "ROS_DOMAIN_ID=191 is not isolated. Stop existing nodes in that domain." >&2
  exit 3
fi

run_dir="$results/$scenario"
[[ ! -e "$run_dir" ]] || { echo "Output already exists: $run_dir" >&2; exit 3; }
mkdir -p "$run_dir"
printf '{"scenario":"%s","domain":191,"bag":"%s","imu_cov_g_diagonal":6.850035236e-8,"imu_cov_a_diagonal":1.084722832e-5,"imu_cov_bg_diagonal":%s,"imu_cov_ba_diagonal":%s,"camera_imu_offset_sec":0.0,"estimated_bias_covariances":false,"pose_authority":false}\n' \
  "$scenario" "$bag" "$bg" "$ba" > "$run_dir/inputs.json"

pids=()
cleanup() {
  local pid
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

ros2 launch leroy_description description.launch.py \
  > "$run_dir/tf.log" 2>&1 &
pids+=("$!")
sleep 3
if ! kill -0 "${pids[0]}" 2>/dev/null; then
  echo "Static TF launch exited; see $run_dir/tf.log" >&2
  exit 4
fi

# Capture tracking status and the fail-closed localization validity signal.
ros2 bag record -o "$run_dir/status_bag" --topics \
  /xrslam/tracking_status /xrslam/localization_valid \
  > "$run_dir/status_record.log" 2>&1 &
pids+=("$!")

ros2 launch leroy_localization xrslam_visual_inertial.launch.py \
  camera_imu_offset_sec:=0.0 \
  imu_cov_g_diagonal:=6.850035236e-8 \
  imu_cov_a_diagonal:=1.084722832e-5 \
  imu_cov_bg_diagonal:="$bg" \
  imu_cov_ba_diagonal:="$ba" \
  > "$run_dir/xrslam.log" 2>&1 &
pids+=("$!")
sleep 5
if ! kill -0 "${pids[2]}" 2>/dev/null; then
  echo "XRSLAM launch exited; see $run_dir/xrslam.log" >&2
  exit 4
fi

# Replay recorded header timestamps unchanged. No /clock or sim-time rewrites.
# Lower replay rate is purely to reduce input queue overflow, not camera FPS tuning.
set +e
ros2 bag play "$bag" --rate 0.25 --delay 3 \
  > "$run_dir/playback.log" 2>&1
play_status=$?
set -e
sleep 2
echo "Replay scenario '$scenario' finished with exit=$play_status"
echo "Logs/status bag: $run_dir"
exit "$play_status"
