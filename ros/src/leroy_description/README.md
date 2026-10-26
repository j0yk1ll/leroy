# leroy_description — YAML-configured sensor TF

## Current validation result

**Operator PASS — source, isolated TF and full-runtime candidate gates:**
The source-only YAML validation reported `STAGE3_SENSOR_TF=PASS_YAML_STATIC_TF_NO_DEVICE`.
The subsequent isolated integration test built and installed the
current description package inside a temporary rootless container and
launched six actual static TF publishers. The independent ROS observer
verified all six `/tf_static` edges, translations, quaternions, six
concurrent DDS writers and clean shutdown:

```text
STAGE3_SENSOR_TF_INTEGRATION_TF_STATIC=PASS_SIX_REAL_EDGES
STAGE3_SENSOR_TF_INTEGRATION_TRANSLATIONS_AND_QUATERNIONS=PASS
STAGE3_SENSOR_TF_INTEGRATION_PUBLISHERS=PASS_SIX_DISTINCT_DDS_WRITERS
STAGE3_SENSOR_TF_INTEGRATION_SHUTDOWN=PASS
STAGE3_SENSOR_TF_INTEGRATION=PASS_REAL_TF_STATIC_NO_DEVICE
```

The first two tests used the pinned sensor image without devices or
host network access. A **third** test in the existing XRSLAM runtime
family also passed in a new, inert, separately tagged image candidate.
No accepted image tag or active robot runtime has been replaced;
geometry and sensor synchronisation remain provisional. Re-running
unchanged acceptance tests is unnecessary.


This package **does not use URDF or Xacro**. A small ROS 2 launch file
reads one YAML and starts only six standard
`tf2_ros/static_transform_publisher` processes to publish fixed
sensor transforms on `/tf_static`.

## Edit sensor mounting coordinates here

`ros/src/leroy_description/config/sensor_mounts.yaml`

- `mounts.imu`: `base_link -> imu_link`
- `mounts.front_camera`: `base_link -> front_camera_link`
- `mounts.workspace_camera`: `base_link -> workspace_camera_link`
- `fixed_frames.front_camera_optical`:
  `front_camera_link -> front_camera_optical_frame`
- `fixed_frames.camera_optical_alias`:
  `front_camera_optical_frame -> camera_optical_frame`
- `fixed_frames.workspace_camera_optical`:
  `workspace_camera_link -> workspace_camera_optical_frame`

YAML units: `xyz` **metres**; `rpy` **radians**, in roll/pitch/yaw
order. `base_link` is +X forward, +Y left, +Z up.
Optical frames follow ROS +X right, +Y down, +Z forward. All six
transform values, including the optical-frame conventions, are visible
in the YAML; frame IDs and topology are fixed by the launcher to avoid
accidental map/odom authority or duplicate children.

The values were migrated **without numeric changes** from the previous
Xacro: IMU (+95, −95, 0) mm with identity robot-frame orientation,
front camera provisional +30° pitch and zero translation, workspace
camera provisional +60° pitch and zero translation. This is a source
configuration migration, **not calibration acceptance**. The YAML
currently records `status: provisional` and the launch refuses other
status labels rather than pretending a setting confers permission.

`/imu/data_raw` is already robot-aligned by MCU firmware
(forward=−ICM X, left=−ICM Y, up=+ICM Z). **Keep
`mounts.imu.rpy=[0,0,0]`**; the loader rejects a second sensor-package
rotation. Changing a physical IMU chip mounting requires reviewing the
separate firmware profile (draft PR #155), not applying duplicate
rotations in ROS.

## Launch and ownership

The existing launch entry point is unchanged:

```bash
ros2 launch leroy_description description.launch.py
```

An optional configuration override uses:

```bash
ros2 launch leroy_description description.launch.py \
  sensor_mounts_file:=/absolute/path/to/sensor_mounts.yaml
```

The launch is included by Leroy bringup and by the Stage 3 XRSLAM
diagnostic. **Start exactly one publisher per child frame** and never
run the earlier `robot_state_publisher`/URDF TF owner simultaneously;
the two mechanisms would duplicate `/tf_static` authority.
No `odom -> base_link` or `map -> odom` is published here.

The source-only migration does NOT update already-built immutable
Podman sensor or XRSLAM images. Actual runtime adoption requires a
separately reviewed package build/install path. Do not alter the
accepted Stage 3 physical runner, flash the MCU, or enable pose,
Nav2 goals, recording or motor output as part of this conversion.

To verify the new source without accessing hardware, from either
directory run:

```bash
# Repository root:
bash ros/m23bv_v3_stage3_sensor_tf_yaml_no_device.sh

# Alternatively inside ros/:
bash ./m23bv_v3_stage3_sensor_tf_yaml_no_device.sh
```

This isolated test uses the pinned existing ROS Jazzy image to parse
YAML, import the launch without starting nodes, and compare the six
configured TF edges against their old values. It also validates a
temporary YAML override and deliberate invalid configurations.
A PASS does not establish physical sensor centers, camera–IMU phase,
XRSLAM tracking or Nav2 readiness.

## Live static TF integration test (separate gate)

The source-only YAML/action test is **operator PASS**, but did not
start `tf2_ros` executables. To validate actual ROS `/tf_static`
publication without robot devices, Stage 3 adds:

```bash
# From repository root:
bash -n ros/m23bv_v3_stage3_sensor_tf_integration_no_device.sh
python3 -B -c 'import ast; from pathlib import Path; ast.parse(Path("ros/m23bv_v3_stage3_sensor_tf_live_verify.py").read_text())'
bash ros/m23bv_v3_stage3_sensor_tf_integration_no_device.sh
```

This test installs the `leroy_description` source package **only**
into an isolated rootless Podman container's ephemeral `/tmp` workspace
with no network and no device mappings, launches all six standard
static publishers, and independently verifies the actual
transient-local `/tf_static` messages, parent/child frame IDs,
positions, rotations and concurrent DDS writer count. Finally it
checks bounded clean shutdown. **Operator integration PASS has been
observed**; see `ros/M23BV_V3_STAGE3_FULLSTACK_DIAGNOSTIC.md`
for the exact acceptance markers, original stdout and limitations.

No firmware, physical calibration, new robot image, MCU, camera,
Nav2 activation or motor operation is part of this test.

## Stage 3 actual Leroy runtime candidate (separate from accepted tests)

The accepted sensor-image source-only and live-`/tf_static` tests do
**not** automatically update the installed description in the existing
normal Leroy/XRSLAM runtime. A separate additive candidate is now
defined in `ros/Dockerfile.stage3-tf-runtime-candidate`.

It derives **only** from the exact pinned local
`localhost/leroy-ros:xrslam` full-runtime image, copies the four
validated description inputs and installs a new
`/opt/leroy_stage3_tf_ws/install` ament overlay. All older
`/leroy_ws/install` packages remain intact. Sourcing the
`/usr/local/bin/leroy-stage3-tf-runtime-env` helper selects the
overlay. The candidate's default command is inert and **does not
launch camera, bridge, XRSLAM, Nav2 or motors**.

The guarded host candidate builder/test is:

```bash
# From repository root:
git pull --ff-only
bash -n ros/m23bv_v3_stage3_tf_runtime_candidate_no_device.sh
bash -n ros/m23bv_v3_stage3_tf_runtime_env.sh
python3 -B -c 'import ast; from pathlib import Path; ast.parse(Path("ros/m23bv_v3_stage3_tf_runtime_candidate_verify.py").read_text())'
bash ros/m23bv_v3_stage3_tf_runtime_candidate_no_device.sh
```

The test checks inherited runtime packages, installed-description
overlay precedence, six **live** static TF DDS transforms and bounded
cleanup using the previously accepted observer. It builds a
separately tagged local image offline; it does **not** deploy or
activate the physical robot runtime or replace any accepted image.
**Operator PASS:** the offline candidate build produced image ID
`9985c05cfcd5d9774b64b790c6f5c1b39c29f989d7ace6dca4e341f5f511841d`.
The new YAML description overrode the old ament package while the
bringup, bridge, DepthART and XRSLAM packages were retained. The
no-device candidate test then launched six actual static TF publishers
and observed correct `/tf_static` frames, translations, quaternions,
six DDS writers and clean controlled shutdown:

```text
STAGE3_TF_RUNTIME_CANDIDATE_DEFAULT=PASS_INERT_NO_ROBOT_BRINGUP
STAGE3_TF_RUNTIME_OVERLAY=PASS_NEW_YAML_DESCRIPTION_RESOLVED
STAGE3_TF_RUNTIME_STACK=PASS_BRINGUP_BRIDGE_DEPTHART_XRSLAM_RETAINED
STAGE3_TF_RUNTIME_LIVE_TF=PASS_SIX_EDGES_IN_FULL_RUNTIME_IMAGE
STAGE3_TF_RUNTIME=PASS_CANDIDATE_REAL_TF_NO_DEVICE
STAGE3_TF_RUNTIME_ACTIVE_ROBOT=NOT_DEPLOYED
```

The accepted original runtime images were not overwritten. The new
candidate was **not deployed**, and normal robot bringup was **not
started**. There is no need to repeat these tests on unchanged code.
See `ros/M23BV_V3_STAGE3_FULLSTACK_DIAGNOSTIC.md` for scope
and remaining calibration/deployment gates.
