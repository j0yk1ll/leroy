# Archived component documentation and historical milestones

The **only active README** is at the repository root: `README.md`.
The previous component READMEs (firmware, electronics, utilities, ROS
packages, and experimental Nav2 packages) were preserved unchanged
in `archive/before-single-readme-2026-10-29` and removed from the
working tree. The root guide now contains the supported checkout,
build, ROS node startup/shutdown, STM32 firmware build and calibration
tool instructions. Archive component docs are historical references,
not current motor-authorizing instructions.

To view an original component document without restoring it:

```bash
git show archive/before-single-readme-2026-10-29:firmware/README.md
git show archive/before-single-readme-2026-10-29:ros/src/leroy_bridge/README.md
git show archive/before-single-readme-2026-10-29:utils/imu/README.md
```


The accepted current diagnostic runtime is documented in
[`../README.md`](../README.md),
[`CURRENT_LIVE_RUNTIME.md`](CURRENT_LIVE_RUNTIME.md) and
the canonical [`leroy.sh`](leroy.sh) runtime entry point.

To simplify a fresh checkout, the **historical milestone-only** root
`M16`–`M23` reports and selftest/experimental Dockerfiles were removed
from the active source tree. They remain fully available in the Git
history and the unmodified snapshot branch:

`archive/full-runtime-history-2026-10-29`

To recover an individual historical file into a working tree with a
complete Git repository:

```bash
git show archive/full-runtime-history-2026-10-29:ros/Dockerfile.m22h-selftest
git show archive/full-runtime-history-2026-10-29:ros/M22H_SIGNED_ROS_DIAGNOSTIC_BRIDGE.md
```

Do not treat the archived recipes as the current startup path.

**Historical shell runner cleanup:** 57 top-level `ros/m18_*.sh` through
`ros/m23*.sh` one-off commissioning, synthetic and experimental runners
were also removed from the active checkout. None is required by the
**operator-verified** combined camera/depth/XRSLAM and IMU/yaw
sensor runtime. Most target
the already-archived milestone Dockerfiles or superseded container
images, and some historical tests still refer to those old scripts.
Those tests are archival, not the supported `check_current_checkout.sh`
contract. The scripts remain in the full-runtime history branch:

```bash
git show archive/full-runtime-history-2026-10-29:ros/m23bv_v3_stage3_vio_host.sh
git show archive/full-runtime-history-2026-10-29:ros/m22h_no_device_run.sh
```

**Current shell and container recipe inventory:** The obsolete M16
sensor fixture launcher, its matching `m16_sensor_only_run.sh`, and
the detached `Dockerfile.nav2` / `Dockerfile.m16-sensors` experiments
are now archived. The active checkout has **14 shell scripts** and
**six Dockerfiles**, all in the canonical build/check script contract.

**Historical Python cleanup (2026-10-30):** 169 historical Python
modules, probes, tests and standalone M16–M23 launch files were
removed from the active source tree. These include the abandoned
synthetic Nav2 integration launchers, fake sensor publishers and
unneeded standalone evidence/rosbag programs. All originals are
available in `archive/pre-ros-python-cleanup`, pinned to the exact
pre-pruning `main` revision.

**Protected runtime dependency:** `m18_imu_stream_source.py` and
`m18_imu_stream_contract.py` still implement the live STM32 IMU
publisher and packet parser. They deliberately keep their physically
validated names. The IMU source still imports the passive observer
`leroy_imu_observer.m16_passive_observer`. That ROS package
therefore retains only its observer, `m16_passive_contract`,
`m15_sensor_contract`, `fault_contract` and Python package marker.
None of the old synthetic Nav2 behavior is built or launched. The
mecanum kinematics library and geometry tests also remain for the
numbered odometry TODOs.

`ros/check_current_checkout.sh` now checks the maintained shell,
Python source, build recipes and key no-device contracts. The archived
milestone experiments are **not** part of that active regression suite.

**Do not delete the only copy of the archive branch or its Git bundle**
until the historical development work is backed up elsewhere. A fresh
GitHub repo created from a source-only export will not include Git refs,
old pull requests or their discussions.

The entire **current** six-image family uses:
`Dockerfile`, `Dockerfile.live-perception`,
`Dockerfile.live-costmap`, `Dockerfile.xrslam`,
`Dockerfile.live-stage4-imu`, `Dockerfile.live-stage4c-capture`.
No milestone-only Nav2 image is a supported build target.

## Completed stage division

The former Stage 3 and Stage 4 were implementation milestones, not
separate operating modes. The current startup is `bash ./leroy.sh run`;
`bash ./leroy.sh build` builds the entire container family. Old image
tags and lower-level launchers remain as internal compatibility layers
to preserve physical evidence and regression tests, not as distinct
operator workflows.

The previous `LIVE_STAGE3_REPRODUCIBLE.md` is available in the
`archive/full-runtime-history-2026-10-29` snapshot:

```bash
git show archive/full-runtime-history-2026-10-29:ros/LIVE_STAGE3_REPRODUCIBLE.md
```

Legacy separate Stage 3 / 4 GitHub Actions workflows were folded into
the single maintained `.github/workflows/leroy-current-checkout.yml`
no-device integration regression workflow. The archived Git history
retains their original definitions.

## Final ROS dependency audit (2026-10-30)

The maintained interface is `ros/leroy.sh` (check/build/status/run). The
source-only CI and six-image build are the preservation boundaries; arbitrary
milestone filenames alone are not a reason to retain or remove a file.

- **Live runtime/build required:** `leroy_bridge`, `leroy_bringup`,
  `leroy_description`, `leroy_msgs`, `leroy_depthart`,
  `leroy_nav2_perception` and
  `leroy_xrslam`. The costmap fixture is explicitly built and checked in
  `Dockerfile.live-costmap`; it is not an orphan.
- **Sensor source dependency:** `leroy_imu_observer` remains because
  the live M18 IMU source imports its passive observer and the package
  retains the corresponding installed console entry point.
- **Inhibited/future work, not a live motion permission:**
  `leroy_nav2` keeps `hardware.disabled.yaml` and dry-run contracts;
  `leroy_nav2_odometry` retains the kinematics library and tests for
  future measured wheel odometry. Empty launch installation metadata was
  retired from its setup.py; no runtime nodes were added.
- **Current bounded diagnostics:** Stage 4B callback service and exact-stamp
  trace remain dependencies of `live_stage4_imu_overlap.py`. Stage 4C/4D
  capture, ingress, source metadata, bag audit, and paced synthetic DDS
  regression files remain, as do the corresponding no-device tests.
  The M18 source and two startup-mounted XRSLAM YAMLs are active despite
  milestone-derived names.
- **Retired in the consolidated cleanup:** uncommissioned powered-yaw
  controllers, standalone manual-yaw witness, historical Stage 4A text-log
  audit, unused rosbag round-trip utility, and obsolete QoS/smoke/VIO YAML
  fixtures. Their historical contents remain in Git.

The audit is based on the repository's tracked imports, COPY statements,
installed package interfaces and test manifests. It does **not** establish
physical acceptance, prove absence of external consumers, or replace the
container build and live hardware safety qualification.

**Perception package build migration:** The former standalone
`leroy_nav2_costmap_fixture` C++ target is now installed as
`leroy_nav2_perception costmap_fixture`. The perception package uses
`ament_cmake` plus `ament_cmake_python` so its C++ costmap fixture and
three Python entry points live in one ROS package. The passive IMU
observer package is now named `leroy_imu_observer`.
