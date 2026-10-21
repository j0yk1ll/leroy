# M23AI — try pthreads instead of oneTBB while retaining composability

**Goal:** Preserve the stock ROS 2 Jazzy composable image_proc::RectifyNode,
but replace the OpenCV parallel backend (oneTBB) with pthreads and
test the same strict non-device 900/900 shutdown workload.

**This branch initially provides ONLY a dependency-audit gate. No
alternative OpenCV build or synthetic runtime is authorized yet.**

## Why the experiment is targeted

In the one accepted M23AF process, M23AG found the crash core's saved
RIP equaled si_addr and was unmapped at crash time. That instruction
address had earlier been backed by executable libtbb.so.12.11 code
at file offset 0x1af5b, in BOTH same-process snapshots. This is
strong evidence for a mapping lifetime mismatch; it is not proof
of a particular unload call or responsible TBB function.

Ubuntu 24.04 ships distro OpenCV 4.6.0+dfsg-13.1ubuntu1; its core
library package depends on oneTBB. OpenCV officially supports
WITH_TBB=OFF, WITH_OPENMP=OFF, WITH_PTHREADS_PF=ON.

Do not equate cv::setNumThreads(1) with removing TBB: limiting
thread count alone does not remove the shared-library dependency.

## Gate A: source-only and exact-image read-only baseline audit

From the accepted M23AH checkout:

~~~bash
cd /tmp/leroy-m23ah
git fetch origin feature/m23ai-opencv-pthreads-composable-ab
git worktree add --detach /tmp/leroy-m23ai \
  origin/feature/m23ai-opencv-pthreads-composable-ab
cd /tmp/leroy-m23ai
git diff --name-status 3f5e489c227514e82fbfb3ca1fb7a16329131bd0 HEAD
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ai_source_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ai_dependency_audit.py
~~~

Expect exactly three newly added files, seven synthetic offline tests,
and M23AI_STATUS=NO_LIBRARY_INSPECTION_BY_DEFAULT.

Then inspect the previous M23AF image as a baseline, without running
ROS, starting camera processing or mounting any devices:

~~~bash
(
set -euo pipefail
set -C
umask 077
test "$(id -u)" -ne 0
cd /tmp/leroy-m23ai
test -z "$(git status --porcelain)"
test "$(git rev-parse HEAD)" = \
  "$(git rev-parse origin/feature/m23ai-opencv-pthreads-composable-ab)"
IMAGE_ID=a56f89bd8d3a69264e846319f8915efcff1fcd04a2dd3bf54d8491b89a119777
test "$(cat "$HOME/.local/state/leroy-m23af/image-id-01.txt")" = "$IMAGE_ID"
test "$(podman image inspect localhost/leroy-sensors:m23af-shutdown-maps-v1 \
  --format '{{.Id}}')" = "$IMAGE_ID"
D="$HOME/.local/state/leroy-m23ai"
test ! -e "$D"
mkdir -m 700 -- "$D"
podman run --rm --pull=never --network=none \
  --ipc=private --pid=private --cap-drop=all \
  --security-opt=no-new-privileges --read-only \
  --pids-limit=32 --memory=512m \
  --tmpfs /tmp:rw,nosuid,nodev,size=16m \
  --mount "type=bind,source=$PWD/ros/m23ai_dependency_audit.py,destination=/opt/m23ai_dependency_audit.py,readonly" \
  --entrypoint /usr/bin/python3 "$IMAGE_ID" \
  -B /opt/m23ai_dependency_audit.py --audit >"$D/baseline-01.log" 2>&1
grep -E '^M23AI_' "$D/baseline-01.log"
)
~~~

This executes only Python and readelf within a disconnected rootless
no-device container. It does not launch ROS or process sensor data,
and the accepted image and earlier crash evidence remain unchanged.

Expected when OpenCV is the direct TBB consumer:
M23AI_BASELINE_VERDICT=OPENCV_CORE_LINKS_TBB.
Also record M23AI_BASELINE_JSON with the version of Ubuntu's OpenCV
core package, which OpenCV shared objects have direct TBB NEEDED
dependencies and which shared objects the stock ROS rectifier uses.
If TBB is NOT directly linked by core, STOP before planning the build.

## Gate B (planned, not yet implemented/authorized): actual replacement

1. Stage pinned, verified exact Ubuntu distro OpenCV 4.6.0 package
   sources and distribution patches; do NOT mix unpinned newer
   OpenCV SONAME/ABI with prebuilt ROS cv_bridge/image_proc.
2. Build a separate diagnostic OpenCV package set with
   WITH_TBB=OFF, WITH_OPENMP=OFF, WITH_PTHREADS_PF=ON and
   PARALLEL_ENABLE_PLUGINS=OFF. Do not install on Arch host
   or replace existing Ubuntu container system packages.
3. Verify the complete compatible OpenCV dependency closure.
   Fail if multiple incompatible OpenCV libraries coexist,
   if the rectifier cannot load, or if any TBB ELF map remains.
4. Rerun the previously accepted strict 900/900 synthetic
   nonzero-D ROS component control with independent child exit
   status and full graph/QoS/header stamp/drain checks. Use the
   SAME composable architecture, not the direct-linked executable.
5. A clean result with TBB truly absent supports a possible
   backend-specific mitigation; it does not prove TBB was
   inherently broken. A crash, missing frame, unknown loader,
   or any TBB mapping is BLOCKED.

No LD_PRELOAD, custom TBB shim, RTLD_NODELETE intervention or
cv::setNumThreads(1) substitute is accepted as TBB replacement.

No camera/serial/STM32/IMU, XRSLAM/DepthART, recording, Nav2,
motion, production or physical >250ms continuity claims.
