# M23AG — exact-run retained core versus executable-map snapshots

**Source-only draft: no M23AG operator tests or core export yet. No new ROS,
Podman runtime, GDB, device access or production changes.**

## Why this is the next step

M23AF #126 accepted a single synthetic nonzero-D crashing control:
900/900 rectified, child SIGSEGV -11, same-child post_factory maps (216)
and on_shutdown maps (241), run nonce
a3510ea397df3225250b7f2080031307 in both phases. At the shutdown
snapshot, librectify.so still had an executable mapping. A new retained
host SIGSEGV core candidate for PID 32869 was discovered, and the
operator corrected the coredumpctl numeric UID/GID formatting parser
and verified all eight core identity fields, including nonce:

- Host PID 32869, container PID 34 (do NOT interchange).
- Timestamp 2026-10-21 12:45:08 CEST; UID/GID 1000.
- Executable /opt/m23af-cpp-install/lib/leroy_m23af_shutdown_maps/component_container.
- Same nonce in saved command line, SIGSEGV, core present.
- M23AF image ID a56f89bd8d3a69264e846319f8915efcff1fcd04a2dd3bf54d8491b89a119777.
- Owner-private prior log ~/.local/state/leroy-m23af/control-01.log.
- Owner-private immutable image ID record in same directory.

Unlike historical cores from M23Q, this is the SAME crashing process as
the captured maps. Never compare old M23Q absolute RIPs across ASLR.

## Question and limits

Read the already-retained core's ELF NT_SIGINFO, NT_FILE, PT_LOAD and
x86_64 NT_PRSTATUS metadata. Require unique saved crash-thread RIP
equal to kernel si_addr, SIGSEGV/SEGV_MAPERR, and the fault RIP
unmapped at core time. Then find whether RIP was inside exactly one
prior executable-file map in post_factory and/or on_shutdown.
Report prior mapped file path and offset only for unique matches.

A prior mapping match establishes former executable-map **presence
at snapshot time**, not code/function ownership, actual dlclose/unmap
timing, identity of the invalid callback or a production fix. If no
match, report NO_PRIOR_EXECUTABLE_MAP_MATCH, not proof of no DSO.

## Gate A — source-only verification and ten tests

~~~bash
cd /tmp/leroy-m23af
git fetch origin feature/m23ag-same-run-core-map-join
git worktree add --detach /tmp/leroy-m23ag \
  origin/feature/m23ag-same-run-core-map-join
cd /tmp/leroy-m23ag
git diff --name-status 567b5b9ee10c069db6479e2d7c0f5d1eac7ec709 HEAD
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ag_source_test.py
~~~

Expect exactly 3 *new* files and 10 passing offline tests.
No ROS/Podman/GDB invoked.

## Gate B — read-only metadata preflight ONLY

~~~bash
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ag_core_join.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ag_core_join.py --preflight
~~~

First command prints M23AG_STATUS=NO_CORE_ACCESS_BY_DEFAULT.
Second only reads existing M23AF full log and immutable image-ID record,
verifies full synthetic control, ordered maps, container PID and run
nonce, then runs coredumpctl --no-pager info 32869 to confirm exact
host PID/UID/GID/executable/timestamp/signal/nonce and present core.
Expected M23AG_PREFLIGHT=PASS and M23AG_CORE_EXPORT=NOT_STARTED.
No core exported, no executable/container run, no new files created.

STOP on any mismatch: do not relax the source/identity gates.

## Gate C — one-time historical core export + offline join

Separately approve only AFTER Gate A/B pass:

~~~bash
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ag_core_join.py --export-and-join
~~~

This rechecks everything, then creates a protected one-shot directory
~/.local/state/leroy-m23ag/retained-core-32869-v1. It uses
coredumpctl --no-pager --output=<corefile> dump 32869 to export the
already-retained core, with a private export log. It never starts ROS,
GDB, Podman, or opens hardware. Only bounded ELF notes are parsed; no
stack memory is read. Expected terminal result:
M23AG_VERDICT=CORE_RIP_JOIN_COMPLETED_NO_CAUSAL_PROOF, with compact
M23AG_JOIN_RESULT_JSON and production BLOCKED.

Do not upload binary core or full map arrays. If the output directory
exists or any phase fails, STOP: never remove existing evidence or
silently rerun/replace an experimental sample.

No production or physical M23A camera gate, STM32/IMU, recording,
DepthART/XRSLAM VIO, Nav2/motor motion. M23Z #120 remains paused.
