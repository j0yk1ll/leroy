#!/usr/bin/env bash
# No-device YAML -> standard tf2_ros static-publisher verification.
# Source-only: no ROS processes or physical sensors.
set -Eeuo pipefail
cd "$(dirname -- "$0")"
fail() { echo "STAGE3_SENSOR_TF=BLOCKED_$1" >&2; exit 2; }
[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] ||
  fail WRONG_BRANCH
# Pin only test input files; do not disturb unrelated user worktree changes.
for file in src/leroy_description/config/sensor_mounts.yaml \
            src/leroy_description/launch/description.launch.py \
            src/leroy_description/CMakeLists.txt \
            src/leroy_description/package.xml; do
  [[ -f "$file" && ! -L "$file" ]] || fail MISSING_INPUT
  [[ "$(git hash-object "$file")" == "$(git rev-parse "HEAD:ros/$file")" ]] ||
    fail INPUT_SOURCE_DRIFT
done
[[ ! -e src/leroy_description/urdf/leroy.urdf.xacro ]] ||
  fail OLD_URDF_STILL_PRESENT
[[ "$(id -u)" != 0 ]] || fail ROOT_NOT_ALLOWED
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail ROOTLESS_REQUIRED
image="localhost/leroy-sensors:m23bv-reliable-rectifier-v3"
expected="f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c"
actual="$(podman image inspect "$image" --format '{{.Id}}')" || fail IMAGE_MISSING
actual="$(printf '%s' "$actual" | sed 's/^sha256://')"
[[ "$actual" == "$expected" ]] || fail IMAGE_ID_DRIFT
echo "STAGE3_SENSOR_TF_PINNED_SENSOR_IMAGE=$actual"

podman run --rm --interactive --pull=never --network none --ipc private --pid private \
  --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=64m \
  --env ROS_LOG_DIR=/tmp/leroy-ros-launch-logs \
  --env ROS_HOME=/tmp/leroy-ros-home \
  --mount type=bind,src="$PWD/src/leroy_description/config/sensor_mounts.yaml",dst=/tmp/sensor_mounts.yaml,readonly \
  --mount type=bind,src="$PWD/src/leroy_description/launch/description.launch.py",dst=/tmp/description.launch.py,readonly \
  --entrypoint /bin/bash "$image" -s <<'LEROY_SENSOR_TF_CONTAINER' || fail VALIDATION_FAILED
set -eo pipefail
test ! -e /dev/video2 && test ! -e /dev/ttyACM0
# ROS 2 Jazzy launch constructs a logger even for an unexecuted Node
# action. Constrain that harmless output to our existing ephemeral /tmp
# tmpfs; never write /root/.ros/log or create files on the host.
case "$ROS_LOG_DIR:$ROS_HOME" in
  /tmp/leroy-ros-launch-logs:/tmp/leroy-ros-home) ;;
  *) echo STAGE3_SENSOR_TF=BLOCKED_UNSAFE_LOG_DESTINATION >&2; exit 2 ;;
esac
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
test -w "$ROS_LOG_DIR" && test -w "$ROS_HOME"
echo STAGE3_SENSOR_TF_LAUNCH_LOG_DIR=EPHEMERAL_CONTAINER_TMPFS
. /opt/ros/jazzy/setup.sh
# The immutable V3 sensor image has ROS Jazzy, but leroy_description was
# deliberately never installed there. To test the *source* launch's normal
# ament lookup without rebuilding or modifying the pinned image, emulate
# ONLY the package index and installed YAML in this container's /tmp tmpfs.
# This ephemeral fixture never starts an executable or changes ROS packages.
test_prefix=/tmp/leroy-description-source-fixture
mkdir -p "$test_prefix/share/ament_index/resource_index/packages" \
         "$test_prefix/share/leroy_description/config"
: > "$test_prefix/share/ament_index/resource_index/packages/leroy_description"
cp /tmp/sensor_mounts.yaml "$test_prefix/share/leroy_description/config/sensor_mounts.yaml"
export AMENT_PREFIX_PATH="$test_prefix:$AMENT_PREFIX_PATH"
echo STAGE3_SENSOR_TF_AMENT_INDEX=EPHEMERAL_SOURCE_FIXTURE_NO_INSTALL
python3 -B - <<'LEROY_SENSOR_TF_PYTHON'
import importlib.util
import math
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.actions import Node

source_yaml = Path("/tmp/sensor_mounts.yaml")
package_share = Path(get_package_share_directory("leroy_description"))
assert package_share == Path("/tmp/leroy-description-source-fixture/share/leroy_description")
assert (package_share / "config/sensor_mounts.yaml").read_bytes() == source_yaml.read_bytes()
print("STAGE3_SENSOR_TF_PACKAGE_LOOKUP=PASS_EPHEMERAL_AMENT_INDEX_ONLY")

module_path = "/tmp/description.launch.py"
compile(Path(module_path).read_text(), module_path, "exec")
spec = importlib.util.spec_from_file_location("leroy_tf_under_test", module_path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
assert len(mod.FRAME_EDGES) == 6
f = "/tmp/sensor_mounts.yaml"
edges = mod.load_sensor_transforms(f)
assert len(edges) == 6
expected = [
    ("base_link", "imu_link", (0.095, -0.095, 0.), (0., 0., 0.)),
    ("base_link", "front_camera_link", (0., 0., 0.), (0., 0.5235987755982988, 0.)),
    ("front_camera_link", "front_camera_optical_frame", (0., 0., 0.),
        (-math.pi/2, 0., -math.pi/2)),
    ("front_camera_optical_frame", "camera_optical_frame", (0., 0., 0.), (0., 0., 0.)),
    ("base_link", "workspace_camera_link", (0., 0., 0.),
        (0., 1.0471975511965976, 0.)),
    ("workspace_camera_link", "workspace_camera_optical_frame", (0., 0., 0.),
        (-math.pi/2, 0., -math.pi/2)),
]
for got, want in zip(edges, expected):
    assert got[:2] == want[:2], (got[:2], want[:2])
    for a, b in zip(got[2:], want[2:]):
        assert len(a) == 3 and len(b) == 3
        assert all(abs(x-y) < 1e-12 for x, y in zip(a, b)), (got, want)
print("STAGE3_SENSOR_TF_FRAME_TREE=PASS_SIX_UNCHANGED_STATIC_EDGES")
print("STAGE3_SENSOR_TF_IMU=PASS_ALREADY_ROBOT_ALIGNED")
print("STAGE3_SENSOR_TF_OPTICAL_CONVENTIONS=PASS")

args = [mod.publisher_arguments(edge) for edge in edges]
assert all(len(a) == 16 for a in args)
assert all(a[-4:] == ["--frame-id", edge[0], "--child-frame-id", edge[1]]
           for a, edge in zip(args, edges))
assert all(a[:12:2] == ["--x", "--y", "--z", "--roll", "--pitch", "--yaw"]
           for a in args)
nodes = mod.make_publishers(f)
assert len(nodes) == 6 and all(isinstance(n, Node) for n in nodes)
ctx = LaunchContext()
ctx.launch_configurations["sensor_mounts_file"] = f
generated = mod._start_configured_publishers(ctx)
assert len(generated) == 6 and all(isinstance(n, Node) for n in generated)
assert len(mod.generate_launch_description().entities) == 2
print("STAGE3_SENSOR_TF_PUBLISHERS=PASS_SIX_STANDARD_TF2_ACTIONS_NOT_STARTED")

with tempfile.TemporaryDirectory(dir="/tmp") as temp:
    original = yaml.safe_load(Path(f).read_text())
    original["mounts"]["front_camera"]["xyz"][0] += 0.001
    trial = Path(temp) / "override.yaml"
    trial.write_text(yaml.safe_dump(original))
    updated = mod.load_sensor_transforms(trial)
    assert abs(updated[1][2][0] - 0.001) < 1e-12
    assert all(updated[i] == edges[i] for i in range(6) if i != 1)
    assert len(mod.make_publishers(trial)) == 6
    print("STAGE3_SENSOR_TF_YAML_OVERRIDE=PASS_DIRECTLY_CHANGES_PUBLISHER_ARGS")

    def reject(data, name):
        dest = Path(temp) / (name + ".yaml")
        dest.write_text(yaml.safe_dump(data))
        try:
            mod.load_sensor_transforms(dest)
        except RuntimeError:
            return
        raise AssertionError("unsafe YAML accepted: " + name)

    broken = yaml.safe_load(Path(f).read_text())
    broken["mounts"]["imu"]["rpy"][2] = math.pi
    reject(broken, "double_imu_rotation")
    broken = yaml.safe_load(Path(f).read_text())
    broken["fixed_frames"]["front_camera_optical"]["rpy"][0] = 0
    reject(broken, "optical_convention")
    broken = yaml.safe_load(Path(f).read_text())
    broken["mounts"]["unexpected_sensor"] = {"xyz":[0,0,0], "rpy":[0,0,0]}
    reject(broken, "unknown_frame")
    broken = yaml.safe_load(Path(f).read_text())
    broken["mounts"]["imu"]["xyz"][0] = float("nan")
    reject(broken, "nonfinite")
    print("STAGE3_SENSOR_TF_INVALID_CONFIG=PASS_FAIL_CLOSED")
LEROY_SENSOR_TF_PYTHON
echo STAGE3_SENSOR_TF_CONTAINER_DEVICES=NONE
LEROY_SENSOR_TF_CONTAINER

echo STAGE3_SENSOR_TF=PASS_YAML_STATIC_TF_NO_DEVICE
echo STAGE3_SENSOR_TF_REAL_NODES=NOT_LAUNCHED
echo STAGE3_SENSOR_TF_RUNNING_IMAGE=UNCHANGED
echo STAGE3_SENSOR_TF_GEOMETRY_CALIBRATION=PROVISIONAL
echo STAGE3_SENSOR_TF_MCU_RECORDING_MOTION=NOT_AUTHORIZED
