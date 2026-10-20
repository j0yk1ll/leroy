# M23X — controlled publisher plugin fan-out diagnostic, synthetic only

**Draft / source prepared, NOT RUN.** Stacked on M23W source-only
protocol. Do **not** merge. Root cause of original `component_container`
SIGSEGV `-11` still **UNRESOLVED**. Strict M23A original physical
camera image-header gap >250 ms and production acceptance remain BLOCKED.

## Operator's prebaseline reproduction (separate historical image)

The operator ran the *previously pinned* M23S image
`01a0334afd19fea39cc4b8a1dc457e65452d6b0d73d1b959ce654904e29e78c2`
using nonzero synthetic distortion and reported:

```text
M23W_PREBASELINE_RUN_EXIT_CODE=2
M23W_PREBASELINE_CHILD_EXIT=-11
M23W_PREBASELINE_RAW_RECT_MATCHED=900
M23W_PREBASELINE=CRASH_REPRODUCED
M23W_DEVICE_ACCESS=NONE_BY_TEST_SCOPE
M23W_ROOT_CAUSE=UNRESOLVED
M23W_PRODUCTION_ACCEPTANCE=BLOCKED
```

The operator's command also checked immutable image identity,
rootless Podman, an explicit image tag and no hardware mapping.
This is **an operator report**, not a directly observed ROS test
by the reviewer. This is **NOT** M23X A1/A2 evidence, because
M23X instrumentation and launch source are a separate as-built
image and can change initialization/timing.

## Specific single-variable experiment

Nonzero-D stock `image_proc::RectifyNode` is left unchanged.
Both arms inherit the M23S publisher-first synthetic
`m23s.run → m23r.run → m23p.run` path, M23P QoS and
observer/camera-information checks, child-only M23R loader tracer,
and SIGINT shutdown classifier.

The only intended **component** difference is:

| Source | Effective output publisher plugin parameter |
|---|---|
| `m23x_baseline_rectifier.launch.py` | No explicit parameter; default plugin allowlist |
| `m23x_raw_rectifier.launch.py` | `parameters=[{"image_rect.enable_pub_plugins": ["image_transport/raw"]}]` |

Publisher plugin `image_transport/raw_pub` maps to the string-array
allowlist entry **`image_transport/raw`**, not `raw`.
The parameter is configured *at component construction*.
`/camera/image_rect` continues to be genuinely rectified
using the nonzero-D OpenCV path, not replaced with
`/camera/image_raw`.

### Critical parameter verification and fail-closed semantics

The original M23P `m23p_qos_graph.py` checks exact
RELIABLE/VOLATILE endpoints but does **not** observe
the effective ROS parameter. M23X replaces only its
`inspect_synthetic_graph` callback with an identically
invoked passive sidecar in both arms.

`m23x_graph_param.py` queries the `/camera/rectify_node`
GetParameters service while matching the M23P QoS graph
and *before the delayed synthetic frames begin*.
It requires ROS ParameterValue `PARAMETER_STRING_ARRAY`;
for baseline the effective default must contain
`image_transport/raw` and at least one additional declared
publisher plugin, and raw arm must equal
`["image_transport/raw"]` exactly. If the parameter,
node identity, graph, type, service or timing cannot be
verified, the wrapper raises `KeyboardInterrupt` *inside*
the inherited M23P preframe graph-check seam: the inherited
try/finally cleanup stops publisher, component and observer
and marks the run BLOCKED, without falsely treating the
failed audit as a clean experiment. Any event after frames
start also remains BLOCKED under M23P's timing guard.

This passive parameter inspector is an **extra process and
may perturb scheduling**; that is why a new M23X-image
baseline is mandatory. No test outcome demonstrates
the historical M23S reproducer is unchanged until the
new control actually reproduces it.

`m23x_allowlist_ab.py` reuses the M23S/M23R/M23P
runtime implementation and its source/D/profile checks,
then uses the existing private log parsers to require
900 Image and 900 CameraInfo published, 900 each
raw/rect/CameraInfo observed, 900 raw↔rect stamps matched,
zero unmatched rect stamps and no gap >250 ms. It
independently checks the M23R tracer and
`component_container` child exit. Raw-only clean
shutdown can be labeled **synthetic-only observation**,
never production fix.

## A1 operator result — diagnostic API spelling blocked the pre-frame audit

The first operator-executed M23X-image A1 baseline was
**inconclusive**. Operator markers:

```text
M23X_A1_PREFLIGHT=PASS
M23X_A1_RUN_EXIT_CODE=2
M23X_A1_PARAMETER_VERIFIED=FALSE
M23X_A1_RECT_MATCHED=None
M23X_A1_CHILD_EXIT=None
M23X_A1=BLOCKED_OR_NOT_REPRODUCED
M23X_A1_M23X_PARAMETER_AUDIT=BLOCKED_ValueError
M23X_A1_COMPONENT_EXIT_CLASS=CLEAN_COMPONENT_SHUTDOWN
M23X_A1_GRAPH_QOS_VERIFIED=FALSE
M23X_A1_M23P_FAULT_CATEGORIES=KeyboardInterrupt,SYNTHETIC_LOG_INVALID
M23X_A1_M23S_FAULT_CATEGORIES=PUBLISHER_NOT_VERIFIED
```

The pre-frame observer failure caused the intended
fail-closed cleanup before images were validated. Clean
component exit in that **aborted** attempt is neither a
successful raw-output change nor evidence against the
historic nonzero-D SIGSEGV. A1 does **not** count toward
either crashing control; **A2/B1/B2 stayed STOPPED**.

### Found offline source defect — Jazzy parameter client import

The initial `m23x_graph_param.py` imported and constructed
`AsyncParametersClient` (**plural**). ROS Jazzy
`rclpy/rclpy/parameter_client.py` at upstream blob
`512bfeed9afec96a32ef4ce7a230a72672a2a6f7`
actually defines **`AsyncParameterClient`** (singular).
The initial import therefore could not succeed against
that Jazzy API, and was outside the observer's
`try/except`. The M23X wrapper captured and discarded
the child observer's stderr and raised its own generic
`ValueError`, masking the import problem. This defect
is a sufficient explanation for the A1 pre-frame failure,
without a claim that no other issues can emerge after
the import correction.

The proposed source-only repair corrects the class name
and adds a **fifth offline AST contract** checking the exact
Jazzy API spelling plus bounded status markers for the
next observer failure category (`GRAPH_QOS`,
`PARAMETER_SERVICE`, `PARAMETER_RESPONSE`,
`ALLOWLIST_CONTENT`). The existing synthetic
publisher, stock rectifier, baseline/raw launch
configurations, tracer, QoS and shutdown code remain
unchanged. No verbose exception strings, raw loader
traces, PIDs or paths are surfaced in public logs.

**Rebuild requirement:** the first M23X image was
built from the faulty diagnostic source. Do **not**
retag/overwrite it and do **not** rerun A1 in that old
image. Reconfirm new Git head and offline Gate A,
then create a **new uniquely tagged** disconnected
v2 image using the same verified M23S immutable
parent, a new owner-private log and new image-ID
evidence. Verify the new image/defaults and source
provenance before any separately operator-controlled
synthetic A1 trial. The previous A1 result remains
inconclusive and its private log stays untouched.

## Pre-run source and isolation gates — no runtime authorization

The old M23S image must still exist by **immutable SHA256
identity**; all operator host prep must avoid mutating,
rebuilding or retagging it. The M23X Dockerfile inherits
its *existing local tag* solely to copy in new diagnostics
and run **offline Python source tests at image build**;
the reference tag must first be confirmed to resolve
to the expected immutable ID. No network pull is
permitted.

Only after source diff, syntax and contract tests
pass should an operator independently approve the
new disconnected **build**. Only after image digest,
ELF identity and as-built launch checks pass should
a new operator-executed no-device diagnostic run
be considered. **This source PR authorizes neither
a build nor a run merely by existing.**

No physical camera, `/dev/video*`, `/dev/ttyACM*`,
MCU, IMU, serial, VIO, Nav2, recorder, controller,
motors, real-data replay, production instance, or
hardware access. Only disposable rootless containers
with `--network none`, private PID/IPC, dropped
capabilities, `no-new-privileges`, read-only root,
bounded tmpfs, no `--device`, verified localhost
ROS discovery and no global `LD_PRELOAD`.

Save each trial in a protected, unique private log
with no overwrites; never share raw traces, cores,
host paths, live PIDs/TIDs, pointer values, or
private manifests on GitHub.

## Predefined finite pilot

**Maximum four separate runs, strictly sequential:**

1. A1: M23X final-image baseline, nonzero D and tracer only.
   Require full evidence, **child SIGSEGV -11** to continue.
2. A2: identically configured second baseline, separate
   fresh container and log. Require the same crash and
   full evidence.
3. B1: same immutable image, nonzero D, raw-only output
   publisher plugin allowlist, complete observation and
   component exit classification. Any SIGSEGV stops B2.
4. B2: only after operator reviews B1 and it is clean
   with full observations, same arm, fresh container.

If A1/A2 are clean, ambiguous, incomplete or unsafe, do
**not** run either intervention. Four observations are
not statistical verification of a fix.

Baseline crashes in A1/A2 are diagnostic evidence even
though the inherited runner returns **2/BLOCKED**.
A clean B1/B2 can only implicate the **publisher
plugin fan-out/lifetime change as a whole**. It does
not identify Theora, Zstd, TBB allocator, an event
callback, `dlclose` unmapping, or a specific UAF.
The historical cores' repeated `si_addr==RIP`
and unmapped fault target support invalid instruction
execution during teardown, not a particular library.

## Gate A — source-only review, not a runtime command

On the operator host, first `git fetch --no-tags origin`
and inspect the M23X commit and diff relative to M23W
without switching branches. Only these seven *new*
tracked files are permitted:

- `ros/m23x_baseline_rectifier.launch.py`
- `ros/m23x_raw_rectifier.launch.py`
- `ros/m23x_graph_param.py`
- `ros/m23x_allowlist_ab.py`
- `ros/m23x_allowlist_ab_test.py`
- `ros/Dockerfile.m23x-raw-publisher-controlled`
- `ros/M23X_RAW_PUBLISHER_CONTROLLED_DIAGNOSTIC.md`

A Gate A PASS must verify **exact source SHA**, only
these paths, no change to existing ROS/production sources
or M23W, parsed AST of all new Python files, **exactly
one extra `ComposableNode.parameters` keyword**
present only in the raw arm, and the exact allowlist
`["image_transport/raw"]`.

Offline Python unit tests require no ROS import, but
must be run by the operator as an independent source
contract gate. **No ROS launch, Podman, build, or device
is authorized by a source PASS.**

## Status

```text
M23X_SOURCE_READY_FOR_OPERATOR_REVIEW=TRUE
M23X_OPERATOR_SOURCE_TESTS=NOT_RUN
M23X_IMAGE_BUILD=NOT_RUN
M23X_RUNTIME_TESTS=NOT_RUN
M23X_ORIGINAL_SIGSEGV_ROOT_CAUSE=UNRESOLVED
M23X_DEVICE_ACCESS=NONE_BY_SOURCE_PREPARATION
M23X_PRODUCTION_ACCEPTANCE=BLOCKED
```

All stacked PRs remain draft/unmerged.
