#!/usr/bin/env bash
# Hardware-free YAML -> ROS Xacro wiring check in the pinned sensor image.
set -Eeuo pipefail
cd "$(dirname -- "$0")"
fail() { echo "STAGE3_SENSOR_MOUNTS=BLOCKED_$1" >&2; exit 2; }
[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] || fail WRONG_BRANCH
[[ -z "$(git status --porcelain)" ]] || fail DIRTY_WORKTREE
for file in src/leroy_description/config/sensor_mounts.yaml \
            src/leroy_description/urdf/leroy.urdf.xacro \
            src/leroy_description/CMakeLists.txt; do
  [[ -f "$file" && ! -L "$file" ]] || fail SOURCE_MISSING
  [[ "$(git hash-object "$file")" == "$(git rev-parse "HEAD:ros/$file")" ]] || fail SOURCE_DRIFT
done
[[ "$(id -u)" != 0 ]] || fail ROOT_NOT_ALLOWED
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] || fail ROOTLESS_REQUIRED
image=localhost/leroy-sensors:m23bv-reliable-rectifier-v3
expected=f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c
actual="$(podman image inspect "$image" --format '{{.Id}}')" || fail IMAGE_MISSING
actual="$(printf '%s' "$actual" | sed 's/^sha256://')"
[[ "$actual" == "$expected" ]] || fail IMAGE_DRIFT
echo "STAGE3_SENSOR_MOUNTS_PINNED_IMAGE_ID=$actual"
podman run --rm --pull=never --network none --ipc private --pid private \
  --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=64m \
  --mount type=bind,src="$PWD/src/leroy_description/config/sensor_mounts.yaml",dst=/tmp/sensor_mounts.yaml,readonly \
  --mount type=bind,src="$PWD/src/leroy_description/urdf/leroy.urdf.xacro",dst=/tmp/leroy.urdf.xacro,readonly \
  --entrypoint /bin/bash "$image" -c '
set -eo pipefail
test ! -e /dev/video2 && test ! -e /dev/ttyACM0
. /opt/ros/jazzy/setup.sh
command -v xacro >/dev/null || exit 2
xacro /tmp/leroy.urdf.xacro sensor_mounts_file:=/tmp/sensor_mounts.yaml \
    > /tmp/leroy-expanded.urdf
python3 -B - <<PY
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml

cfg = yaml.safe_load(Path("/tmp/sensor_mounts.yaml").read_text())
assert cfg["status"] == "provisional"
mounts = cfg["mounts"]
expected = {"base_to_imu": "imu",
            "base_to_front_camera": "front_camera",
            "base_to_workspace_camera": "workspace_camera"}
assert set(mounts) == set(expected.values())
root = ET.parse("/tmp/leroy-expanded.urdf").getroot()
joints = {j.attrib["name"]: j for j in root.findall("joint")}
assert root.attrib["name"] == "leroy"
assert len(joints) == 6
def origin(j):
    elem = joints[j].find("origin")
    assert elem is not None
    return {k: tuple(map(float, elem.attrib[k].split())) for k in ("xyz", "rpy")}
origins = {k: origin(k) for k in joints}
for joint, name in expected.items():
    fields = mounts[name]
    assert set(fields) == {"xyz", "rpy"}
    for k in ("xyz", "rpy"):
        vals = fields[k]
        assert isinstance(vals, list) and len(vals) == 3
        assert all(type(x) in (int, float) and math.isfinite(x) for x in vals)
        assert all(abs(a-b) < 1e-10 for a,b in zip(origins[joint][k], vals))
assert origins["base_to_imu"]["rpy"] == (0.,0.,0.)
assert origins["front_camera_to_optical"]["rpy"] == (-1.5707963267948966,0.,-1.5707963267948966)
assert origins["front_optical_to_legacy_camera_optical"]["rpy"] == (0.,0.,0.)
print("STAGE3_SENSOR_MOUNTS_XACRO_EXPANSION=PASS_YAML_MATCHES_URDF")
print("STAGE3_SENSOR_MOUNTS_IMU_FRAME=ROBOT_ALIGNED_NO_DOUBLE_ROTATION")

# Test a modified TEMPORARY YAML; never modify the checked-in values.
probe = yaml.safe_load(Path("/tmp/sensor_mounts.yaml").read_text())
probe["mounts"]["front_camera"]["xyz"][0] += 0.001
Path("/tmp/probe.yaml").write_text(yaml.safe_dump(probe))
with Path("/tmp/probe.urdf").open("w") as out:
    subprocess.run(["xacro", "/tmp/leroy.urdf.xacro",
                    "sensor_mounts_file:=/tmp/probe.yaml"], check=True, stdout=out)
changed = {j.attrib["name"]:j for j in ET.parse("/tmp/probe.urdf").getroot().findall("joint")}
new_x = float(changed["base_to_front_camera"].find("origin").attrib["xyz"].split()[0])
assert abs(new_x-origins["base_to_front_camera"]["xyz"][0]-0.001) < 1e-10
print("STAGE3_SENSOR_MOUNTS_YAML_OVERRIDE=PASS_CONFIG_ACTUALLY_DRIVES_XACRO")
PY
echo STAGE3_SENSOR_MOUNTS_CONTAINER_DEVICES=NONE
' || fail NO_DEVICE_YAML_XACRO_TEST
echo STAGE3_SENSOR_MOUNTS=PASS_CONFIG_DRIVEN_TF_NO_DEVICE
echo STAGE3_SENSOR_MOUNTS_RUNTIME_BUILD=NOT_PERFORMED
echo STAGE3_SENSOR_MOUNTS_CALIBRATION=PROVISIONAL
echo STAGE3_SENSOR_MOUNTS_MCU_MOTORS_RECORDING=NOT_AUTHORIZED
