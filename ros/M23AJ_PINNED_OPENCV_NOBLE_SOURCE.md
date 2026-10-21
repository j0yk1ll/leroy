# M23AJ — pin exact Noble OpenCV source for TBB-free pthread experiment

## Accepted input and purpose

M23AI #128 source tests 7/7 and exact-image dependency audit
PASS after correcting Podman's tmpfs copy-up ENOSPC. Exact M23AF
immutable diagnostic image ID:
a56f89bd8d3a69264e846319f8915efcff1fcd04a2dd3bf54d8491b89a119777.

Audit saw libopencv_core.so.4.6.0 direct DT_NEEDED libtbb.so.12;
55 installed OpenCV DSOs inspected; none other had a *direct* TBB
NEEDED entry. Jazzy cv_bridge, image_geometry and librectify need
OpenCV ABI .so.406, including core, flann, imgproc, imgcodecs and
calib3d. This establishes a **plausible replacement target**, not
proof TBB was the only library with runtime TBB use, or that a
recompiled OpenCV will be ABI compatible with all ROS dependencies.

Do not switch to the standalone rectifier in this experiment.
The aim is to retain the same stock dynamically loaded composable
RectifyNode with its previous strict non-device 900/900 rectification
and child SIGINT/SIGSEGV evaluation while swapping OpenCV's TBB
parallelism to pthreads in a separate immutable diagnostic image.

## Verified canonical source provenance

Ubuntu Noble official source opencv 4.6.0+dfsg-13.1ubuntu1:
https://packages.ubuntu.com/source/noble/opencv

Four source files hosted by Ubuntu under:
https://archive.ubuntu.com/ubuntu/pool/universe/o/opencv/

1. opencv_4.6.0+dfsg-13.1ubuntu1.dsc
2. opencv_4.6.0+dfsg.orig.tar.xz
3. opencv_4.6.0+dfsg.orig-contrib.tar.xz
4. opencv_4.6.0+dfsg-13.1ubuntu1.debian.tar.xz

Published Ubuntu .dsc SHA256:
4194d944a274df50ebeb60d2fb511c464a392727afbe12403da13f53261d9f47

Published Ubuntu Debian-patch archive SHA256:
6b3fbe614353fc074eb121307ea39b0f802d69463aa86355cef1f5e9ae4dabcc

The exact .dsc contains authoritative SHA256 and byte sizes for the
other *two* original tarballs; M23AJ independently verifies those
before any extraction or build. No arbitrary GitHub tag, newer OpenCV,
host-package override, unpinned mirror or alternative Debian version
is an acceptable source substitute.

## Gate A — source-only (no network or file access by default)

~~~bash
cd /tmp/leroy-m23ai
git fetch origin feature/m23aj-pinned-opencv-pthreads-source
git worktree add --detach /tmp/leroy-m23aj \
  origin/feature/m23aj-pinned-opencv-pthreads-source
cd /tmp/leroy-m23aj
git diff --name-status 8fdc07f397c133352ddf107f8a018a4a7e7ab63a HEAD
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23aj_source_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23aj_source_gate.py
~~~

Note: use the **current head of M23AI branch**, not the older
8fdc07f above, to diff for exactly this change set. The source
test and no-read gate are independent of repo ancestry.

Expected source tests OK, and
M23AJ_STATUS=NO_SOURCE_READ_BY_DEFAULT.

## Gate B — optional host-side Ubuntu source acquisition (not runtime)

This is the **only** network use contemplated: fetching source
archives from Ubuntu's public HTTPS archive on the operator host.
ROS/Podman runtime, production image, physical robot and package
manager are not touched, and the later diagnostic build remains
disconnected. Download only with operator approval after Gate A.
It is not necessary if identical source archives are already
available in an owner-private folder.

~~~bash
(
set -euo pipefail
set -C
umask 077
test "$(id -u)" -ne 0
D="$HOME/.local/state/leroy-m23aj/sources"
test ! -e "$D"
mkdir -p -m 700 -- "$D"
BASE="https://archive.ubuntu.com/ubuntu/pool/universe/o/opencv"
for FILE in \
    "opencv_4.6.0+dfsg-13.1ubuntu1.dsc" \
    "opencv_4.6.0+dfsg.orig.tar.xz" \
    "opencv_4.6.0+dfsg.orig-contrib.tar.xz" \
    "opencv_4.6.0+dfsg-13.1ubuntu1.debian.tar.xz"; do
  OUT="$D/$FILE"
  test ! -e "$OUT"
  curl --fail --location --silent --show-error --retry 0 \
      "$BASE/\${FILE//+/%2B}" --output "$OUT"
  chmod 600 "$OUT"
done
PYTHONDONTWRITEBYTECODE=1 python3 -B \
  /tmp/leroy-m23aj/ros/m23aj_source_gate.py --verify-sources
)
~~~

STOP on download, hash/size, permissions or version mismatch.
Do not erase/retry a partial source directory; it is evidence of
failure. Expected M23AJ_SOURCE_PROVENANCE=VERIFIED_UBUNTU_NOBLE.
The source set remains private; no archive uploads needed.

## Gate C — planned separate build/compatibility gate, not yet authorized

From that source provenance, a future additive Dockerfile will
unpack the pinned Debian source and apply its Ubuntu patches,
then build OpenCV **4.6.0** with:

- WITH_TBB=OFF
- WITH_OPENMP=OFF
- WITH_PTHREADS_PF=ON
- PARALLEL_ENABLE_PLUGINS=OFF

It must preserve .so.406 SONAME/required module ABI and inspect
every required ROS dependency; no system OpenCV replacement on the
Arch host or accepted parent image. Keep all alternate OpenCV DSOs
in a separate test-only image/prefix and require actual
runtime mappings show **no libtbb.so.12** and no mixed compatible
OpenCV versions before interpreting a shutdown verdict.

Run the strict nonzero-D 900 raw + 900 rect + 900 CameraInfo,
QoS/graph/stamp/drain and actual ROS component child-exit checks.
The comparison is against the M23AC matched stock component
control, not a separate directly linked rectifier. Production and
all physical M23A >250ms continuity, cameras, serial/STM32/IMU,
DepthART, XRSLAM, recorder, Nav2 and motion remain BLOCKED.
