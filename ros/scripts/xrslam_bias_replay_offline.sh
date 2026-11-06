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

# Each launched ROS command owns a separate session/process group.
# ROS 2 launch and rosbag can spawn grandchildren which outlive their parents.
# Signal ONLY verified process-group leaders created here, not the live graph.
command -v setsid >/dev/null || { echo "setsid is required" >&2; exit 2; }
pids=()
cleanup() {
  local leader pgid i alive
  trap - EXIT INT TERM
  for leader in "${pids[@]}"; do
    pgid="$(ps -o pgid= -p "$leader" 2>/dev/null | tr -d ' ' || true)"
    if [[ "$pgid" == "$leader" ]]; then
      kill -INT -- "-$leader" 2>/dev/null || true
    fi
  done
  # Give rosbag recorder and launch-managed nodes time for graceful shutdown.
  sleep 3
  for leader in "${pids[@]}"; do
    # The group may survive after its leader exits; group IDs remain reserved.
    kill -TERM -- "-$leader" 2>/dev/null || true
  done
  for i in 1 2 3 4 5; do
    alive=0
    for leader in "${pids[@]}"; do
      if kill -0 -- "-$leader" 2>/dev/null; then alive=1; fi
    done
    [[ "$alive" == 0 ]] && break
    sleep 1
  done
  for leader in "${pids[@]}"; do
    if kill -0 -- "-$leader" 2>/dev/null; then
      echo "Force-stopping remaining offline group $leader" >&2
      kill -KILL -- "-$leader" 2>/dev/null || true
    fi
  done
  for leader in "${pids[@]}"; do wait "$leader" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

# Verify group leadership: never signal a potentially shared process group.
register_group() {
  local leader="$1" pgid attempt
  # The asynchronous shell may not have exec'd setsid when $! is read.
  # Poll for the NEW group and never kill the original/shared shell group.
  for attempt in {1..60}; do
    pgid="$(ps -o pgid= -p "$leader" 2>/dev/null | tr -d ' ' || true)"
    if [[ "$pgid" == "$leader" ]]; then
      pids+=("$leader")
      return 0
    fi
    if [[ -z "$pgid" ]]; then
      echo "Offline launch process exited before session creation: pid=$leader" >&2
      exit 4
    fi
    sleep 0.05
  done
  echo "Offline child failed to acquire its own process group: pid=$leader pgid=$pgid" >&2
  # Kill only the direct child; NEVER signal a group that we did not own.
  kill -TERM "$leader" 2>/dev/null || true
  exit 4
}

setsid ros2 launch leroy_description description.launch.py \
  > "$run_dir/tf.log" 2>&1 &
register_group "$!"
sleep 3
if ! kill -0 "${pids[0]}" 2>/dev/null; then
  echo "Static TF launch exited; see $run_dir/tf.log" >&2
  exit 4
fi

# Capture tracking status and the fail-closed localization validity signal.
setsid ros2 bag record -o "$run_dir/status_bag" --topics \
  /xrslam/tracking_status /xrslam/localization_valid \
  > "$run_dir/status_record.log" 2>&1 &
register_group "$!"

setsid ros2 launch leroy_localization xrslam_visual_inertial.launch.py \
  camera_imu_offset_sec:=0.0 \
  imu_cov_g_diagonal:=6.850035236e-8 \
  imu_cov_a_diagonal:=1.084722832e-5 \
  imu_cov_bg_diagonal:="$bg" \
  imu_cov_ba_diagonal:="$ba" \
  > "$run_dir/xrslam.log" 2>&1 &
register_group "$!"
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
