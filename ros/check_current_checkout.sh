#!/usr/bin/env bash
# Source-only production integrity and safety regressions. Opens no hardware.
set -Eeuo pipefail
cd "$(dirname "$0")"
bash -n ./leroy.sh ./build_current_images.sh ./check_current_checkout.sh
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
r=Path('.')
expected={'leroy_bridge','leroy_bringup','leroy_sensors','leroy_perception','leroy_localization','leroy_navigation','leroy_description','leroy_msgs'}
actual={p.name for p in (r/'src').iterdir() if (p/'package.xml').is_file()}
assert actual == expected, (actual,expected)
for name in actual:
    assert ET.parse(r/'src'/name/'package.xml').getroot().findtext('name') == name
assert {p.name for p in r.glob('*.sh')} == {'leroy.sh','build_current_images.sh','check_current_checkout.sh'}
assert not list(r.glob('*.py'))
assert not list(r.glob('Dockerfile.*'))
assert not list(r.glob('*.yaml'))
assert "BLOCKED_PRODUCTION_MOTION_NOT_COMMISSIONED" in (r/'leroy.sh').read_text()
assert "enabled: false" in (r/'src/leroy_navigation/config/hardware.disabled.yaml').read_text()
assert "bridge_command_output_enabled: false" in (r/'src/leroy_navigation/config/hardware.disabled.yaml').read_text()
assert (r/'src/leroy_bridge/leroy_bridge/command_ingress.py').is_file()
assert (r/'src/leroy_bridge/leroy_bridge/motor_cdc_protocol.py').is_file()
print('PRODUCTION_SOURCE_LAYOUT=PASS_FAIL_CLOSED')
PY
PYTHONPATH=".:src/leroy_sensors:src/leroy_localization:src/leroy_bringup:src/leroy_perception:src/leroy_bridge${PYTHONPATH:+:$PYTHONPATH}" python3 -m unittest -v \
  tests.unit.boot_localization_core_test \
  tests.unit.boot_yaw_core_test \
  tests.unit.boot_auto_calibration_test \
  tests.unit.boot_yaw_tracking_test \
  tests.unit.test_imu_protocol \
  tests.safety.test_production_inhibit
echo 'LEROY_PRODUCTION_CHECK=PASS_OFFLINE'
