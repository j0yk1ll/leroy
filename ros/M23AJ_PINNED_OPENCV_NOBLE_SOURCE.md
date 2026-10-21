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
git diff --name-status 1ff3293698cb48fea878a5703f0582ffbc954fe5 HEAD
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23aj_source_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23aj_source_gate.py
~~~

Expected 4 newly added files, 8 source tests OK, and
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
      "$BASE/${FILE//+/%2B}" --output "$OUT"
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

## Gate C — disconnected diagnostic build (after Gate B source verification)

A conservative, *not yet operator-validated* Dockerfile is included:
ros/Dockerfile.m23aj-opencv-pthreads. It starts from the already
pinned M23AF image and installs nothing into its system prefix.
It unpacks the exact Ubuntu source using dpkg-source with archive
hashes independently verified, uses CMake with strict pthreads/no-TBB
flags, and builds the minimal ROS rectifier library family
core, flann, imgproc, imgcodecs, features2d and calib3d.
If a required preinstalled development dependency is missing,
the offline build stops; do NOT run apt-get or allow online
dependencies in the diagnostic image without separate review.
Only libopencv_core.so.406 SONAME and direct TBB NEEDED absence
are build-time gates; **runtime absence of TBB and complete OpenCV
ABI compatibility remain UNPROVEN**, and no ROS run is authorized
by a successful build alone.

To prepare a separate private one-shot build context without
leaving large source tarballs in the Git worktree:

~~~bash
(
set -euo pipefail
set -C
umask 077
test "$(id -u)" -ne 0
cd /tmp/leroy-m23aj
test -z "$(git status --porcelain)"
S="$HOME/.local/state/leroy-m23aj/sources"
PYTHONDONTWRITEBYTECODE=1 python3 -B \
  ros/m23aj_source_gate.py --verify-sources
PARENT=a56f89bd8d3a69264e846319f8915efcff1fcd04a2dd3bf54d8491b89a119777
test "$(podman image inspect localhost/leroy-sensors:m23af-shutdown-maps-v1 \
  --format '{{.Id}}')" = "$PARENT"
CTX="$HOME/.local/state/leroy-m23aj/build-context-01"
LOG="$HOME/.local/state/leroy-m23aj/pthreads-build-01.log"
test ! -e "$CTX" && test ! -e "$LOG"
mkdir -m 700 "$CTX"
mkdir -m 700 "$CTX/m23aj_sources"
cp -- ros/Dockerfile.m23aj-opencv-pthreads "$CTX/"
cp -- ros/m23aj_source_gate.py "$CTX/"
cp -- "$S/"opencv_* "$CTX/m23aj_sources/"
podman build --network=none --pull=never \
  -f "$CTX/Dockerfile.m23aj-opencv-pthreads" \
  -t localhost/leroy-sensors:m23aj-pthreads-opencv-v1 \
  "$CTX" >"$LOG" 2>&1
grep -Fq 'M23AJ_PTHREADS_BUILD_CONTRACT=PASS' "$LOG"
ID="$(podman image inspect \
  localhost/leroy-sensors:m23aj-pthreads-opencv-v1 \
  --format '{{.Id}}')"
test "$ID" != "$PARENT" && test -n "$ID"
echo M23AJ_PTHREADS_BUILD_GATE=PASS
echo "M23AJ_PTHREADS_NEW_IMAGE_ID=$ID"
)
~~~

If the build fails, STOP and inspect the private existing log.
No deleting, clobbering, reusing build-01 evidence, removing
packages, or rerunning the original crashing control.

## Stage C-02 correction after first configure failure

The first operator build on Oct 21 2026 passed the source contract
but failed CMake configuration *before compilation* because Ubuntu's
repackaged +dfsg tarball omits 3rdparty/protobuf, 3rdparty/quirc and
3rdparty/ittnotify. OpenCV's output nevertheless identified
"Parallel framework: pthreads".

The Dockerfile now explicitly adds
`-DWITH_PROTOBUF=OFF -DWITH_QUIRC=OFF -DWITH_ITT=OFF`.
These optional libraries are not required by the intended
rectification workload. A source-only unit test requires both the
CMake options and CMakeCache OFF assertions. This is **not** a
change to the original source archives, OpenCV threading selection,
ROS component code, system image, or existing runtime experiment.

The operator must preserve the immutable first failure:
`pthreads-build-01.log`, `build-context-01` and all original source
archives. Build **once** from the updated branch using fresh paths:
`build-context-02`, `pthreads-build-02.log`, image tag
`localhost/leroy-sensors:m23aj-pthreads-opencv-v2`, and
`pthreads-image-id-02.txt`. Fail if any new output already exists.
No automatic cleanup/retry and no tag overwrite.

A successful build must show the source contract, all six OpenCV
.so.406 SONAME/no-direct-TBB DT_NEEDED checks and the pthreads
build contract; runtime absence of TBB and ABI compatibility are
still separately blocked.

## Gate D — planned runtime comparison; NOT AUTHORIZED YET

Before testing shutdown, create a separate source-reviewed runner
that explicitly loads ONLY the newly built OpenCV ABI-compatible
libraries, requires all relevant loaded OpenCV ELF mappings
correspond to the new prefix, and verifies no oneTBB mapping in
the actual composable process at startup AND shutdown. A clean
subprocess with only a different DT_NEEDED is not enough.

Maintain exact M23AF stock factory and mapping observer plus
full M23AA 900/900 nonzero-D, QoS/graph, stamp and drain checks.
Do NOT silently accept image_process crash, nodelete, startup
failure or frame loss as a successful replacement.
Production and all physical tests remain BLOCKED.



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
