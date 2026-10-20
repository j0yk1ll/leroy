# M23Z — one bounded child executable-map snapshot before shutdown

**SOURCE PREPARATION ONLY.** This draft does not authorize image builds, ROS execution,
physical devices or production changes. Parent is M23Y PR #119 at
c41d6d38fb7432c032b16d8f69dd31802c5a4089. Preserve parent image,
all A1/A2/B1 logs, historical cores and source unchanged.

## Evidence and question

Two accepted M23Y stock baseline trials A1/A2 and one raw-only B1
delivered all 900 genuine nonzero-distortion rectified frames, with no
header gaps >250ms; all component children subsequently died SIGSEGV -11.
B1's raw-only image_rect publisher allowlist was verified live. Its
component-only M23R trace observed **zero** of the four optional
compressed/depth/Theora/Zstd transport DSO dlopen events, compared with
four in each baseline, but the same shutdown fault persisted.
Therefore those observed optional loads are not necessary for B1's fault.

Two saved cores independently report SEGV_MAPERR with si_addr == RIP
at an unmapped instruction target. The fault PC has the same page offset
and an adjacent candidate return PC +0x801a in both cores. Neither the
currently unmapped address nor the unreliable backtrace names the former
code owner. Do not claim that dlclose, Theora, Zstd, libc or a
particular destructor was responsible.

## Minimal diagnostic (not a fourth A/B run)

A new separate M23Z C tracer is an exact M23R source copy except for two
standard headers, a single once-only executable mapping snapshot function
and a call just after the first DLCLOSE_BEGIN and immediately before the
underlying dlclose. It keeps all M23R_TRACE_EVENT output compatible with
M23R's strict parser and the original tracer intact.

The snapshot uses dl_iterate_phdr to save bounded executable ELF PT_LOAD
virtual address ranges and module basenames of still-mapped shared
objects from within the component_container child. It accepts only the
existing component-only M23R_TRACE_ONLY gate and new M23Z_MAP_ONLY=1
gate. Snapshot cap 400; a missing/truncated/duplicate census blocks
analysis. Raw ASLR addresses are copied to the host's new owner-private
experiment log as M23Z_PRIVATE_EXEC_MAP records. **Never paste those
records publicly or into chat.**

A new raw-arm-only launch differs from the original M23X raw launch
in just the child tracer library path. The new wrapper temporarily
substitutes this launch, calls the exact M23Y/M23X/M23S/M23R/M23P
harness, checks original 900/900 matched image/CameraInfo delivery,
effective raw-only publisher allowlist, actual graph/QoS, source
calibration and independent child shutdown classification. It never
bypasses genuine rectification, records, SLAM or hardware guards.

### Step gates

- **A — source only:** check exact parent, six additive files and run
  the offline Python/C source contract tests, without ROS/containers.
  Docker image source tests copy two existing reference source files.
- **B — separately approved build:** verify saved immutable M23Y v2
  parent ID, rootless disconnected Podman build with --pull=never and
  --network=none, unique tag, bounded resources and protected log.
  The Dockerfile only builds the standalone C child tracer and tests.
- **C — read-only image audit:** new image ID distinct; parent and
  historical images unchanged; verify experimental raw-arm entrypoint,
  source contract and synthetic-only env.
- **D — optional one separately reviewed synthetic-only run:** exact
  new immutable image, no network/devices, no privilege elevation,
  private PID/IPC, bounded memory/tmpfs/pids/120s and new 0600 log.
  Require 900 published and observed raw/rect/Info, 900 matching
  stamps, zero unmatched rect, no rect interval >250ms, live raw-only
  effective parameter, QoS, nonzero-D, child-only original trace and
  complete executable-map snapshot. Child SIGSEGV -11 may reproduce;
  a clean exit under extra tracing is not proof of a fix. **Stop after
  exactly one run; B2 remains forbidden by the M23W stop rule.**
- **E — if D crashes and host retained its core:** select uniquely
  by event time, executable/node identity and signal; analyze existing
  crash privately and compare its actual fault RIP to the M23Z map
  ranges recorded from that **same process/run**, not another core
  with different ASLR. Verify candidate module with exact ELF build ID.
  A matching former mapped module is a lead, not causation proof.
  No core or address disclosure.

No modification of stock Jazzy image_proc, input CameraInfo.D, M23Y
drained publisher, M23X effective parameter inspector, production or
any previous image. Instrumentation may perturb timing.

**Additive files:**
ros/m23z_loader_map_trace.c;
ros/m23z_raw_rectifier.launch.py;
ros/m23z_snapshot_runner.py;
ros/m23z_source_contract_test.py;
ros/Dockerfile.m23z-child-exec-map;
ros/M23Z_CHILD_EXEC_MAP_DESIGN.md.

All PRs draft/unmerged. Root cause unresolved. M23A physical camera,
STM32/IMU, recording, VIO/Nav2, motors/motion and production BLOCKED.
