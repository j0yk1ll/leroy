# M23R — no-device image_proc component shutdown lifetime trace

**Draft-only, stacked on M23Q PR #111. Not executed on the operator host.**

## Why this exists

The two historical M23P cores, host PIDs 46312 and 48349, independently
report SIGSEGV/SEGV_MAPERR with `si_addr == RIP` and no surviving
`NT_FILE` or `PT_LOAD` mapping at the respective instruction pointers.
Their first frame's page offset is the same (`0xf5b`) and the separation
to the second **unverified** journal frame is `0x801a`. This supports a
repeatable unmapped-instruction target during shutdown. It **does not**
identify an unloaded module, corrupted callback, destructor or DDS bug.
Recovering 93 exact-image libraries did not repair the historical
unwind.

The latest M23P publisher-first synthetic delivery had 899 raw and
899 rectified images received, all matching header stamps, max rect
header gap **34.232ms**, real all-RELIABLE/VOLATILE graph audit,
and **still** a component_container exit **-11** on SIGINT.

M23R's purpose is to collect **prospective, bounded, device-free**
load/unload and launch lifecycle ordering around that known failure.
It is not a shutdown fix, a motion test, or authority to alter
M23A's original strict `>250ms` physical image-header-gap gate.

## Scope

**No modifications to existing M23P code or launch files, M23A, production
ROS, physical-camera controls, STM32 CDC, motion, IMU, XRSLAM, Nav2 or
recorder.** This is a separate overlay that adds only:

- `m23r_loader_trace.c` — opt-in `LD_PRELOAD` interposition on
  `dlopen` and `dlclose`, gated by
  `M23R_TRACE_ONLY=1` **and** the exact running executable name
  `component_container`. It logs bounded monotonic-nanosecond
  timestamp, pid/tid, module path, handle and return code to the
  component's existing standard error; a constructor and destructor
  marker bracket the observer lifetime. `DLCLOSE_BEGIN` is emitted
  **before** the actual call, `DLCLOSE_RETURN` after it.
- `m23r_rectifier_trace.launch.py` — the same sole
  `image_proc::RectifyNode` component and topic remaps as M23P;
  the ROS launch `SetEnvironmentVariable` scopes `LD_PRELOAD`
  to its **component child**, leaving M23P's fabricated publisher,
  C++ observer and Python runner uninstrumented. Logs launch process
  start, exit and shutdown-request event markers.
- `m23r_shutdown_trace_runner.py` — calls M23P's unchanged
  `m23p.run()` after replacing only the launch filename, thus reusing
  device-visibility refusal, profile allowlist, synthetic fake
  calibration, publisher-first QoS order, graph audit, counters,
  strict zero-gaps-over-250ms requirement and **component child exit**
  checks. A second fail-closed parser checks loader constructor,
  single PID, valid trace fields and plausible dlclose counts.
  It preserves a bounded last-80-event diagnostic window in the
  output; **does not infer causality** from the last event.
- `m23r_shutdown_trace_test.py` — 14 synthetic, no-device
  offline source/behavior contracts.
- `Dockerfile.m23r-shutdown-trace` — uses only the existing
  local M23P no-device image. Builds a small local interposer
  offline, without rebuilding upstream ROS or fetching packages.

**Diagnostic perturbation:** `LD_PRELOAD` changes dynamic-linker
timing and may suppress, induce or move a race. A clean instrumented
shutdown is NOT proof the underlying issue was fixed. Missing a
`dlclose` event does NOT prove there was no unloading. Even a
`DLCLOSE_BEGIN` immediately before a crash is temporal correlation,
not established causal ownership of the unmapped code.

## Gate A — source tests only; no ROS and no devices

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23r-no-device-shutdown-trace
git pull --ff-only origin feature/m23r-no-device-shutdown-trace
cd ros

PYTHONDONTWRITEBYTECODE=1 python3 -u m23r_shutdown_trace_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23p_no_device_rectifier_shutdown_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23q_core_fault_mapping_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23q_compare_prior_core_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23q_missing_core_libraries_test.py

```

Also verify all M23R Python files without importing ROS or writing bytecode:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 - <<'PY'
import ast
from pathlib import Path
for file in ("m23r_shutdown_trace_runner.py",
             "m23r_rectifier_trace.launch.py",
             "m23r_shutdown_trace_test.py"):
    ast.parse(Path(file).read_text(), filename=file)
print("M23R_PYTHON_SYNTAX=PASS")
PY
```

Require M23R **14/14** tests, M23P **14/14**, and unchanged M23Q
offline contracts **15/15 + 7/7 + 8/8** (58 total), with all respective
`*_SOURCE_CONTRACT=PASS` or `*_COMPARISON_CONTRACT=PASS` markers.
Do not continue on a failing test. The operator reported the initial
13/13 M23R source-contract result PASS; the 14th packaging regression
contract has not yet been run on the operator host.

## Offline build attempt and strict-compiler correction

Operator reported the initial disconnected build stopped at the C
compiler (exit **1**) before running the M23R Python image contracts:

```text
error: ignoring return value of 'write' declared with attribute
       'warn_unused_result' [-Werror=unused-result]
error: 'nonnull' argument 'handle' compared to NULL [-Werror=nonnull-compare]
```

The M23R interposer now captures the `write()` return value and
stops only that trace event on short/error writes (best-effort
diagnostics; no retries, no process intervention). The redundant
`handle != NULL` guard has been removed because the system
`dlclose` function declares its argument nonnull. Existing
loader-forwarding and device-free behavior are unchanged. A
new thirteenth offline source regression contract guards the
two fixes. **The C warnings were resolved on the next operator build**, but the
in-image Python contracts then failed because
`/opt/m23r/Dockerfile.m23r-shutdown-trace` was missing.

Before proceeding to any runtime test, rerun Gate A (14/14 M23R),
recheck the exact M23P base image ID and repeat Gate B. Only
`M23R_NO_DEVICE_TRACE_IMAGE_CONTRACT=PASS` and build exit 0
permit considering separate no-device idle/synthetic runs.

## Second build attempt and image-copy correction

The second offline build passed strict C compilation but the
in-image Python source tests exited **2**:

```text
FileNotFoundError: [Errno 2] No such file or directory:
  '/opt/m23r/Dockerfile.m23r-shutdown-trace'
```

The checkout's M23R tests passed because that Dockerfile exists in
the checkout, but the diagnostic image did not include its own
Dockerfile while the safety-contract test expected to read it.
The build now explicitly copies `Dockerfile.m23r-shutdown-trace`
into `/opt/m23r`. A 14th offline contract verifies **all six**
files needed by the in-image tests have corresponding Dockerfile
`COPY` directives.

This correction has not yet been operator-built. Require M23R
**14/14** source tests, pinned M23P base-image SHA verification,
`M23R_NO_DEVICE_TRACE_IMAGE_CONTRACT=PASS` and build exit 0
before any diagnostic ROS runtime test. No physical device or
motion operation is authorized.

## Gate B — disconnected local build (still no ROS execution)

The candidate historical M23P image was identified in M23Q by
immutable image ID
`6eeb8ca08525d451dc0f216f097c1df7e300444d8c8ae551fde576d64a1c48f1`.
Do not silently use a rebuilt or retagged image.

```bash
M23P_BASE=localhost/leroy-sensors:m23p-rectifier-shutdown
M23P_EXPECTED=6eeb8ca08525d451dc0f216f097c1df7e300444d8c8ae551fde576d64a1c48f1
M23P_ACTUAL="$(podman image inspect "$M23P_BASE" --format '{{.Id}}' 2>/dev/null)" || exit 2
test "${M23P_ACTUAL#sha256:}" = "$M23P_EXPECTED" || {
  echo "M23R_BLOCKED=M23P_IMAGE_ID_CHANGED"; exit 2;
}

podman build --network none --pull=never \
  -f Dockerfile.m23r-shutdown-trace \
  -t localhost/leroy-sensors:m23r-shutdown-trace .
```

Require build exit0 and
`M23R_NO_DEVICE_TRACE_IMAGE_CONTRACT=PASS`. This build runs only
offline source tests and a local compiler, never the component.

## Gate C — separate, operator-authorized no-device lifecycle probes

**Do not run this gate as part of merely opening or reviewing the PR.**
Require successful Gates A and B, human confirmation that this is
only a disposable no-device test, and continued **BLOCKED** status
for any physical camera, serial or robot-motion operation.

First execute idle alone, then synthetic in a **separate new
container**. No host device is mapped; no shared host IPC, host PID
namespace or network access. Device discovery is explicitly
forbidden by the inherited M23P preconditions.

```bash
umask 077
mkdir -p "$HOME/.local/state/leroy-m23r"
chmod 700 "$HOME/.local/state/leroy-m23r"

set +e
podman run --rm --pull=never \
  --network none --ipc private --pid private \
  --shm-size 512m \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  -e M23P_PROFILE=idle \
  localhost/leroy-sensors:m23r-shutdown-trace \
  2>&1 | tee "$HOME/.local/state/leroy-m23r/idle-trace.log"
echo "M23R_IDLE_EXIT_CODE=${PIPESTATUS[0]}"
```

Only after inspecting the idle results, the independently authorized
synthetic-only run would use exactly the same restrictions and
`-e M23P_PROFILE=synthetic`, with a **different host log filename**.
Do not combine both profiles into one launch or run the camera.

For each run retain:
- M23P's original `M23P_LIFECYCLE_JSON`,
  `M23P_LIFECYCLE_VERDICT`, `M23P_DEVICE_ACCESS=NONE`.
- M23R's `M23R_TRACE_JSON`, `M23R_LIFECYCLE_VERDICT`,
  `M23R_TRACE_CAPTURED` event window,
  `M23R_DEVICE_ACCESS=NONE`.
- Actual shell pipeline's `M23R_*_EXIT_CODE`, not merely
  `ros2 launch` exit status.
- Synthetic publisher counts and raw/rect header matching/gaps
  under the unmodified M23P criteria.

If the component SIGSEGV persists, record it as **BLOCKED** even
if synthetic frame delivery remains healthy. If it does not occur,
report **CLEAN OBSERVED UNDER INSTRUMENTATION ONLY**, never a
validated shutdown fix or production acceptance.

## Exit conditions / nonclaims

M23R can supply temporally ordered loader events that help
target later source-level or executor instrumentation. The trace
does not capture arbitrary `dlclose` internal unload operations,
full thread stack traces or definitive ownership of the missing
instruction address. No physical camera, STM32, VIO, recorder or
motion is authorized, regardless of outcome. M23A original bag
and `>250ms` blocking rule remain unchanged. Keep this and the
parent PRs draft/unmerged pending operator verification.
