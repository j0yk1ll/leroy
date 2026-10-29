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

**Retained exception:** `ros/m16_sensor_only_run.sh` is copied by
`Dockerfile.nav2`, so it remains in the active source alongside the
14 current build, diagnostic and runtime shell entry points,
including the new unified `leroy.sh`.

The current shell interface is enforced by
`ros/check_current_checkout.sh`: all 15 scripts are syntax-checked,
and changes to their inventory require an intentional update to that
source-only contract.

**Do not delete the only copy of the archive branch or its Git bundle**
until the historical development work is backed up elsewhere. A fresh
GitHub repo created from a source-only export will not include Git refs,
old pull requests or their discussions.

The current six-image family includes:
`Dockerfile`, `Dockerfile.live-perception`,
`Dockerfile.live-costmap`, `Dockerfile.xrslam`,
`Dockerfile.live-stage4-imu`, `Dockerfile.live-stage4c-capture`.
The additional `Dockerfile.nav2` and `Dockerfile.m16-sensors` are
retained as a pair for the independent Nav2 offline development build.

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
