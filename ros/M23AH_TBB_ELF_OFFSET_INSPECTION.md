# M23AH — exact-image oneTBB executable-offset inspection

**Source-only diagnostic; no new ROS, Podman runtime, crash/core
export, GDB, hardware or production work.**

## Accepted evidence

M23AF #126: exact one no-device strict 900/900 component control,
child SIGSEGV -11, executable file maps after stock factory and
at ROS shutdown. M23AG #127: same-run host crash core PID 32869
verified with exact nonce a3510ea397df3225250b7f2080031307.
Kernel-saved fault RIP 0x00007f9e48f86f5b equaled si_addr and
was unmapped in the core. This exact address was earlier within
the executable map backed by the path
/usr/lib/x86_64-linux-gnu/libtbb.so.12.11 in BOTH snapshots, at
file offset 110427 decimal = 0x1af5b hexadecimal.

This establishes prior same-process DSO mapping, not what
unmapped it, which code called it, or a TBB product defect.
libtbb.so.12.11 differs from libtbbmalloc.so.2. The prior
M23Q cores were not independently correlated to libtbb.

The original M23AF immutable image:
a56f89bd8d3a69264e846319f8915efcff1fcd04a2dd3bf54d8491b89a119777.

## Stage A — exactly three additive files and 10 source tests

~~~bash
cd /tmp/leroy-m23ag
git fetch origin feature/m23ah-tbb-elf-offset-inspection
git worktree add --detach /tmp/leroy-m23ah \
  origin/feature/m23ah-tbb-elf-offset-inspection
cd /tmp/leroy-m23ah
git diff --name-status c5e7187b470f8005a450f7a5310495e321a465a0 HEAD
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ah_source_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ah_elf_offset.py
~~~

Expected: 3 added only, 10 tests OK,
M23AH_STATUS=NO_LIBRARY_READ_BY_DEFAULT.
No library files read by default, no ROS or Podman.

## Stage B — single exact-image stopped-container copy

After Stage A passes, extract only the file-backed TBB shared
object from the **same immutable M23AF image**. Here podman
create creates a STOPPED container; podman cp only copies bytes;
neither starts/runs the container. No physical devices or network.

~~~bash
(
set -euo pipefail
set -C
umask 077
test "$(id -u)" -ne 0
cd /tmp/leroy-m23ah
test "$(git rev-parse HEAD)" = \
  "$(git rev-parse origin/feature/m23ah-tbb-elf-offset-inspection)"
test -z "$(git status --porcelain)"
IMAGE_ID=a56f89bd8d3a69264e846319f8915efcff1fcd04a2dd3bf54d8491b89a119777
test "$(cat "$HOME/.local/state/leroy-m23af/image-id-01.txt")" = "$IMAGE_ID"
test "$(podman image inspect localhost/leroy-sensors:m23af-shutdown-maps-v1 \
  --format '{{.Id}}')" = "$IMAGE_ID"
D="$HOME/.local/state/leroy-m23ah"
mkdir -m 700 -- "$D"
test ! -e "$D/libtbb.so.12.11"
test ! -e "$D/image-library-sha256-01.txt"
CID=""
cleanup() {
  if [ -n "$CID" ]; then
    podman rm "$CID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
CID="$(podman create --pull=never --network=none --ipc=private \
  --pid=private --cap-drop=all --security-opt=no-new-privileges \
  --read-only --entrypoint /bin/true "$IMAGE_ID")"
podman cp "$CID:/usr/lib/x86_64-linux-gnu/libtbb.so.12.11" \
  "$D/libtbb.so.12.11"
chmod 600 "$D/libtbb.so.12.11"
test -f "$D/libtbb.so.12.11" && test ! -L "$D/libtbb.so.12.11"
sha256sum "$D/libtbb.so.12.11" | awk '{print $1}' \
  >"$D/image-library-sha256-01.txt"
chmod 600 "$D/image-library-sha256-01.txt"
echo M23AH_EXACT_IMAGE=VERIFIED
echo M23AH_LIBRARY_STAGED=PRIVATE_FROM_STOPPED_IMAGE
)
~~~

STOP on any failed command. Do not delete or reuse private evidence
on failure. No binary uploads. Image and source check must match
the exact accepted image and branch heads; no tag substitutions.

## Stage C — explicit read-only ELF metadata analysis

After Stage B:

~~~bash
PYTHONDONTWRITEBYTECODE=1 python3 -B \
  /tmp/leroy-m23ah/ros/m23ah_elf_offset.py --inspect
~~~

Requires private owner-only extraction directory/file, matching
saved SHA-256 and a valid x86_64 ELF64 ET_DYN file. The analyzer
translates known **file offset** 0x1af5b to an ELF virtual address
through a unique PF_X PT_LOAD program header, verifies a GNU build
ID and tries offline addr2line using the calculated ELF address.

If debug symbols are absent, it returns UNKNOWN, not an invented
function. If mapped file offset does not belong to a unique
executable PT_LOAD, reject. Do not blindly pass file offsets to
addr2line; ELF VMA may differ. A source-function symbol alone does
not establish why the mapping disappeared or identify causal
caller/finalizer.

Expected M23AH_ELF_RESULT_JSON followed by
M23AH_ELF_INSPECTION=PASS_METADATA_ONLY and continued
M23AH_PRODUCTION_ACCEPTANCE=BLOCKED. Never replace a real
root-cause attribution with nearest exported symbol guesses.

## Stop conditions

No repeat M23AF crash or M23AG core export, no ROS/GDB launch,
camera, STM32/IMU, DepthART/XRSLAM VIO, recording, Nav2, motor
or motion. Original physical M23A >250ms gate and production
acceptance remain BLOCKED. All PRs diagnostic drafts.
