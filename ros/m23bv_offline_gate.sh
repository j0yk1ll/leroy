#!/usr/bin/env bash
# M23BV strictly hardware-free initial acceptance; never a physical runner.
set -Eeuo pipefail
cd "$(dirname -- "${BASH_SOURCE[0]}")"
export PYTHONDONTWRITEBYTECODE=1
for script in m23bv_sensor_only_run.sh m23bv_offline_gate.sh m23bv_podman_build_offline.sh m23bv_synthetic_dds_preflight.sh \
          m23bv_synthetic_launch_tree_preflight.sh \
          m23bv_ready_candidate_build_offline.sh \
          m23bv_ready_candidate_runtime_preflight.sh \
          m23bv_sigint_v2_build_offline.sh m23bv_sigint_v2_runtime_preflight.sh \
          m23bv_v2_qos_order_diagnostic.sh \
          m23bv_reliable_v3_build_offline.sh \
          m23bv_reliable_v3_runtime_preflight.sh \
          m23bv_reliable_v3_bag_preflight.sh \
          m23bv_v3_geometry_preflight.sh; do
  bash -n "$script"
done
for py in m23bv_camera_only.launch.py m23bv_qos_guard.py \
          m23bv_bounded_capture.py m23bv_window.py \
          m23bv_derive_closed_bag.py m23bv_offline_tests.py m23bv_synthetic_dds_probe.py \
          m23bv_m23i_independent_preflight.py m23bv_negative_contract_tests.py \
          m23bv_synthetic_launch_tree_probe.py \
          m23bv_camera_ready_gate.py m23bv_camera_ready_candidate.launch.py \
          m23bv_ready_candidate_runtime_probe.py \
          m23bv_camera_ready_gate_sigint_v2.py m23bv_sigint_v2_runtime_probe.py \
          m23bv_reliable_rectifier_v3_candidate.launch.py \
          m23bv_reliable_v3_runtime_probe.py \
          m23bv_reliable_v3_bag_probe.py \
          m23bv_v3_geometry_probe.py m23bv_v3_geometry_oracle.py; do
  python3 - "$py" <<'PY'
import ast, sys
from pathlib import Path
ast.parse(Path(sys.argv[1]).read_text(), filename=sys.argv[1])
PY
done
python3 -u m23bv_offline_tests.py
python3 -u m23bv_negative_contract_tests.py
printf 'M23BV_HOST_OFFLINE_CONTRACT=PASS\n'
printf 'M23BV_ACTUAL_JAZZY_RUNTIME=UNTESTED\n'
printf 'M23BV_PHYSICAL_DEVICE_ACCESS=NONE\n'
