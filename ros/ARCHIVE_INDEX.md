# Legacy milestones and build recipes

The accepted current diagnostic runtime is documented in
[`../README.md`](../README.md),
[`CURRENT_LIVE_RUNTIME.md`](CURRENT_LIVE_RUNTIME.md) and
[`LIVE_STAGE3_REPRODUCIBLE.md`](LIVE_STAGE3_REPRODUCIBLE.md).

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

Do not treat the archived recipes as the current startup path. Old
`mXX` experiment runners may refer to archived Dockerfiles, and are not
part of `build_current_images.sh` or supported current boot procedures.

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
