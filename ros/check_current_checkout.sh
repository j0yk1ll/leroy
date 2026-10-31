#!/usr/bin/env bash
# Offline validation that a fresh source checkout contains all ACTIVE
# (nonactuating) runtime build inputs. Does not open any robot device.
set -Eeuo pipefail
cd "$(dirname "$0")"
for file in Dockerfile Dockerfile.live-perception Dockerfile.live-costmap \
    Dockerfile.xrslam Dockerfile.live-stage4-imu \
    Dockerfile.live-stage4c-capture \
    src/leroy_bringup/scripts/live_stage3_build.sh src/leroy_bringup/scripts/live_stage3_start.sh \
    leroy.sh \
    src/leroy_bringup/scripts/live_stage3_monocular_sidecar.sh \
    src/leroy_bringup/scripts/live_boot_session.sh src/leroy_bringup/scripts/live_boot_yaw.sh \
    src/leroy_sensors/leroy_sensors/imu_service.py \
    src/leroy_sensors/leroy_sensors/imu_overlap.py \
    src/leroy_sensors/leroy_sensors/delivery_trace.py \
    src/leroy_sensors/leroy_sensors/spin_budget.py \
    src/leroy_localization/leroy_localization/heading_node.py \
    src/leroy_bringup/leroy_bringup/readiness.py \
    src/leroy_localization/leroy_localization/kinematics.py \
    src/leroy_localization/leroy_localization/yaw_core.py \
    src/leroy_localization/leroy_localization/auto_calibration_core.py \
    src/leroy_imu_observer/leroy_imu_observer/imu_source.py \
    src/leroy_imu_observer/leroy_imu_observer/imu_protocol.py \
    src/leroy_bridge/package.xml src/leroy_xrslam/package.xml \
    src/leroy_nav2_perception/package.xml \
    m23bv_v3_stage3_xrslam_monocular_diag.yaml \
    m23bv_v3_stage3_xrslam_euroc_template_UNCALIBRATED.yaml; do
    [[ -f "$file" && ! -L "$file" ]] || {
        echo "REPO_PORTABILITY=BLOCKED_MISSING_$file" >&2; exit 2;
    }
done
# Validate build contexts, static commands and no accidental host-device
# access in build scripts. No Podman build or ROS graph is started.
# Syntax-check every retained top-level shell entry point, including
# the optional signed-controller launcher (but NEVER execute it).
for script in ./*.sh src/leroy_bringup/scripts/*.sh; do
    bash -n "$script"
done
python3 - <<'PY'
from pathlib import Path
import re

root = Path(".")
# The current checked-in runtime has one maintained shell interface.
# Historical M16-M23 experiments and optional Nav2 fixture Dockerfiles are
# archived, not additional startup procedures.
expected_scripts = {"build_current_images.sh", "check_current_checkout.sh", "leroy.sh"}
found_scripts = {p.name for p in root.glob("*.sh")}
if found_scripts != expected_scripts:
    raise SystemExit("Unexpected root shell entry points: " + str(sorted(found_scripts ^ expected_scripts)))
required_bringup_scripts = {
    "exec.sh", "live_boot_readiness.sh", "live_boot_session.sh",
    "live_boot_yaw.sh", "live_boot_yaw_snapshot.sh",
    "live_stage3_build.sh", "live_stage3_monocular_sidecar.sh",
    "live_stage3_start.sh", "live_stage4_imu_sidecar.sh",
}
bringup_scripts = {p.name for p in (root/"src/leroy_bringup/scripts").glob("*.sh")}
if bringup_scripts != required_bringup_scripts:
    raise SystemExit("Bringup script inventory changed: " + str(sorted(bringup_scripts ^ required_bringup_scripts)))
dockerfiles = (
    "Dockerfile", "Dockerfile.live-perception",
    "Dockerfile.live-costmap", "Dockerfile.xrslam",
    "Dockerfile.live-stage4-imu", "Dockerfile.live-stage4c-capture",
)
for name in dockerfiles:
    # These image recipes deliberately use plain local COPY statements,
    # not external URLs as build contexts.
    source = (root/name).read_text()
    for line in source.splitlines():
        m = re.match(r"^COPY\s+(.*)", line)
        if not m:
            continue
        args = m.group(1).split()
        if args[0].startswith("--"):
            raise SystemExit("Unexpected COPY option: " + name)
        if len(args) < 2:
            raise SystemExit("Malformed Docker COPY in " + name)
        for src in args[:-1]:
            if not (root/src).exists():
                raise SystemExit("Missing Docker build input: " + src +
                                 " required by " + name)

builder = (root/"build_current_images.sh").read_text()
for tag in ("jazzy", "perception", "costmap", "xrslam",
            "stage4b-imu", "stage4c-capture-offline"):
    if tag not in builder and tag not in (root/"src/leroy_bringup/scripts/live_stage3_build.sh").read_text():
        raise SystemExit("Missing canonical image tag " + tag)
assert "/dev/video8" in (root/"src/leroy_bringup/scripts/live_stage3_start.sh").read_text()
assert "motor" not in (root/"src/leroy_bringup/scripts/live_stage3_build.sh").read_text().split(
    "LIVE_STAGE3_IMAGES=BUILT_NOT_LAUNCHED")[-1]
print("REPO_PORTABILITY=PASS_SOURCE_BUILD_INPUTS")
PY
PYTHONPATH=".:src/leroy_sensors:src/leroy_bringup:src/leroy_localization:src/leroy_capture:src/leroy_imu_observer${PYTHONPATH:+:$PYTHONPATH}" python3 -m unittest -v \
    tests.unit.boot_localization_core_test \
    tests.unit.boot_yaw_core_test \
    tests.unit.boot_auto_calibration_test \
    tests.unit.boot_yaw_tracking_test \
    tests.integration.live_boot_readiness_test \
    tests.integration.live_boot_service_test \
    tests.integration.current_live_runtime_test \
    tests.integration.repository_readme_test \
    tests.unit.test_imu_protocol \
    tests.safety.leroy_runtime_test \
    tests.safety.test_operator_stop \
    tests.integration.test_camera_perception_startup \
    tests.integration.test_camera_localization_sidecar \
    tests.safety.test_imu_source_safety \
    tests.unit.test_imu_delivery \
    tests.unit.test_capture_contract \
    tests.unit.test_capture_metadata \
    tests.integration.test_capture_metadata_ros \
    tests.integration.test_capture_ingress \
    tests.integration.test_capture_audit

python3 -m unittest discover -s src/leroy_nav2_perception/test \
  -p test_live_costmap_static_contract.py -v

grep -Fqx 'CMD ["/bin/false"]' Dockerfile.live-stage4c-capture
grep -Fqx 'LIVE_CAPTURE_COMMISSIONED = True' src/leroy_capture/leroy_capture/recorder.py
echo 'REPO_PORTABILITY=PASS_OFFLINE_CURRENT_RUNTIME'
