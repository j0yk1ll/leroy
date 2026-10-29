#!/usr/bin/env bash
# Offline validation that a fresh source checkout contains all ACTIVE
# (nonactuating) runtime build inputs. Does not open any robot device.
set -Eeuo pipefail
cd "$(dirname "$0")"
for file in Dockerfile Dockerfile.live-perception Dockerfile.live-costmap \
    Dockerfile.xrslam Dockerfile.live-stage4-imu \
    Dockerfile.live-stage4c-capture \
    live_stage3_build.sh live_stage3_start.sh \
    live_stage3_monocular_sidecar.sh \
    live_boot_session.sh live_boot_yaw.sh \
    live_boot_imu_service.py live_boot_yaw.py \
    boot_yaw_core.py boot_auto_calibration_core.py \
    m18_imu_stream_source.py m18_imu_stream_contract.py \
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
bash -n build_current_images.sh live_stage3_build.sh \
    live_stage3_start.sh live_stage3_monocular_sidecar.sh \
    live_boot_session.sh live_boot_yaw.sh
python3 - <<'PY'
from pathlib import Path
import re

root = Path(".")
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
    if tag not in builder and tag not in (root/"live_stage3_build.sh").read_text():
        raise SystemExit("Missing canonical image tag " + tag)
assert "/dev/video8" in (root/"live_stage3_start.sh").read_text()
assert "motor" not in (root/"live_stage3_build.sh").read_text().split(
    "LIVE_STAGE3_IMAGES=BUILT_NOT_LAUNCHED")[-1]
print("REPO_PORTABILITY=PASS_SOURCE_BUILD_INPUTS")
PY
python3 -m unittest -v \
    boot_localization_core_test \
    boot_yaw_core_test \
    boot_auto_calibration_test \
    boot_yaw_tracking_test \
    live_boot_readiness_test \
    live_boot_service_test \
    current_live_runtime_test \
    autonomous_motor_yaw_test
echo 'REPO_PORTABILITY=PASS_OFFLINE_CURRENT_RUNTIME'
