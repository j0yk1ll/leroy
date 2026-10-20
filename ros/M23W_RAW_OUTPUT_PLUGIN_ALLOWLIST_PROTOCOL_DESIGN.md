# M23W — raw-output publisher allowlist: source-only protocol design

**Status:** DESIGN ONLY — no experiment authorization, ROS/Podman
execution, container/build, new test, installed-package change,
physical hardware, robot motion, or production acceptance. This
documentation-only review is stacked on M23V and explicitly does
**not** modify the stock `image_proc::RectifyNode`, any launch
script, runner, configuration, image, binary, plugin, or source test.

## Question, nonclaim and prerequisites

**Prospective hypothesis:** Does restricting **the stock
rectifier's *output* image_transport publisher** to its raw
publisher plugin change the reported nonzero-D component-container
SIGINT crash in a repeated, otherwise identical, device-free
synthetic comparison?

- H0: No demonstrable difference in SIGINT fault incidence
  between unchanged M23S-compatible nonzero-D baseline and
  raw-only publisher-allowlist intervention.
- H1: A demonstrable difference exists under otherwise equal
  image identity, child preload, synthetic timing, QoS, ROS
  distribution, installed libraries, and launch/shutdown
  protocol. Even if observed, it establishes an association
  with the allowlist intervention, **not** which DSO/callback
  caused the fault or whether any library was unmapped.
- **Precondition requiring separate operator authorization:**
  first re-establish reproducibility of the original
  nonzero-D **M23S-matched** traced crashing baseline,
  *without* the M23T OpenCV helper. The original M23S one-off
  SIGSEGV is not sufficient for a frequency claim.
- If this prereq fails or the baseline is clean, **do not
  proceed to the intervention**. A clean M23T `default`
  helper-preloaded run cannot substitute for this baseline.
- The exact sample size, repetition ordering, shutdown
  schedule, stopping rules, protected logging, approvals,
  and resource caps require a distinct written **no-device
  runtime safety gate**. **None are granted here.**

## Proven upstream source mechanism and precise parameter

Read and pin these specific reviewed upstream files (Git blob IDs
are *source identity*, not the installed ELF Build IDs):

| File | Source ref | Git blob SHA |
|---|---|---|
| `ros-perception/image_common/image_transport/src/publisher.cpp` | `jazzy` | `48f0257e4dacd959bfc6f64ef7245cd2059ce057` |
| `ros-perception/image_common/image_transport/default_plugins.xml` | `jazzy` | `70c86767a24632b0129964fa707c5c9d75efd9cb` |
| `ros-perception/image_common/image_transport/include/image_transport/raw_publisher.hpp` | `jazzy` | `65fed4a7d4fade823677ea0ce87a67aba6552e4a` |
| `ros-perception/image_pipeline/image_proc/src/rectify.cpp` | `jazzy` | `e7cbd9b1e1a0c42fff53241a0094d80988979623` |
| `ros2/launch_ros/launch_ros/descriptions/composable_node.py` | `jazzy` | `f5508daa85f7612b3e5629d36b6ec8b78bc0c23f` |
| `ros-perception/image_transport_plugins/theora_image_transport/theora_plugins.xml` | `jazzy` | `13f07c9e657adf1e116a82baa687c8bd57b1eca2` |
| `ros-perception/image_transport_plugins/zstd_image_transport/zstd_plugins.xml` | `jazzy` | `53737f64d0a5cd7fa407e0290056f515d16956a9` |

[Publisher implementation](https://github.com/ros-perception/image_common/blob/jazzy/image_transport/src/publisher.cpp)
derives `all_transport_names` by **stripping the `_pub`
suffix from the `pluginlib::ClassLoader::getDeclaredClasses()`
lookup name**. It uses that list as the default value of
`<output-topic>.enable_pub_plugins` and appends `_pub`
again for each `createUniqueInstance` call.

[Default raw plugin manifest](https://github.com/ros-perception/image_common/blob/jazzy/image_transport/default_plugins.xml)
declares **`name="image_transport/raw_pub"`**. Therefore
the exact list value corresponding to raw publisher is
**`image_transport/raw`**; `raw` alone is **NOT** the
correct allowlist entry for the reviewed source path.
The `RawPublisher::getTransportName()` function independently
returns `"raw"`, but is **a distinct transport-identifier
interface**, not the pluginlib allowlist class-name stem.

Stock `RectifyNode` calls
`image_transport::create_publisher(this, "image_rect", ...)`;
under namespace `/camera` its expanded output base topic is
`/camera/image_rect`. The reviewed `Publisher::Publisher`
uses the effective node namespace length to strip `/camera`
and replaces slashes with dots, yielding the **relative ROS
parameter name** `image_rect.enable_pub_plugins`.

**Illustrative future component parameter override; DO NOT
APPLY OR EXECUTE**:

```yaml
image_rect.enable_pub_plugins:
  - image_transport/raw
```

For a future **authorized** `ComposableNode(parameters=[...])`
override, this is the intended string-array value (not an
environment variable or `image_transport=raw` subscriber
transport hint). Its actual installation-specific parsing,
effective type and presence must be verified independently
before any run; reviewed upstream source does not imply
the effective value has been observed.

Theora manifest lists `image_transport/theora_pub` from
`theora_image_transport_component`, and Zstd lists
`image_transport/zstd_pub` from `zstd_image_transport`.
With default all-declared publisher transports, source explains
why such plugins could be instantiated even though the
M23S fixture publishes raw Image and CameraInfo. Their
`dlclose` trace entries are **close targets**, not identified
callers or demonstrated codec use.

## Existing M23P/R/S/T local source: no intervention present

Pinned from private repository branch `feature/m23v-transport-teardown-source-review`:

| Local file | Git blob SHA | Important fact |
|---|---|---|
| `ros/m23p_rectifier_only.launch.py` | `b49793f7c59ed471b6d0ba24020afc17c0b2d14b` | Stock `RectifyNode`, `/camera` namespace and fixed remappings; **no `parameters=`** |
| `ros/m23r_rectifier_trace.launch.py` | `cf94a395c608f1aca2411be9a5c674ce7f3875c2` | Same node options; child loader tracer preload |
| `ros/m23t_rectifier_thread_trace.launch.py` | `d035f5ec58622c8b1eb04335030d0ec09fa94f68` | Same node options; child loader tracer **plus** OpenCV helper |
| `ros/m23p_no_device_rectifier_shutdown.py` | `0bda340b9d751eebf0c5464fb1bf4c608cff5616` | Synthetic-only runner and observer checks, child SIGINT and exit classification |
| `ros/m23r_shutdown_trace_runner.py` | `c8388fec5091b3c85977228f739c9816cdebcf5b` | Loader tracing scoped through M23P launch substitution |
| `ros/m23s_rectification_ab.py` | `913e6abffff2bde21adb908a6a802070361cb4e1` | D=0 / D!=0 fixture and inherited M23R/M23P baseline |
| `ros/m23s_cpp/src/m23s_ab_publisher.cpp` | `bab0ee8adfa046614c43c7ad4b019df968f7958e` | Raw `rclcpp::create_publisher` fixture; synthetic 900-frame source |

Operator's **M23V Gates A–D PASS** included source discovery
(`enable_pub_plugins=0` tracked source hits), literal
`ComposableNode` ASTs with no explicit parameters and
the intended differing preload/runner dispatch. Gate D showed:
M23P no explicit preload, M23R tracer-only, M23T tracer
plus helper. Operator's **M23V final remote head check PASS**
verified exact parent head
`0926afc38961eb5a9046edc6762eb4b1e1199c95`,
doc-only changes, root cause unresolved and production blocked.

**Boundary:** zero tracked hits do not prove there was no
parameter override at runtime or in installed/container
configuration. No `ros2 param get` was performed.

## Candidate paired protocol — FOR REVIEW, NOT AUTHORIZED

The sole intended between-arm variation in a *future,
separately gated* run is the output publisher's
`image_rect.enable_pub_plugins` override.

| Dimension | Candidate baseline | Candidate intervention |
|---|---|---|
| Stock rectifier code | Unmodified `image_proc::RectifyNode` | Same |
| Synthetic frame content/CameraInfo | M23S nonzero D: `-0.08,0.01,0.001,-0.001,0`, 900 frames | Exactly same fixture |
| Publisher plugin allowlist | No explicit override (source default; requires runtime attestation) | Explicit `["image_transport/raw"]` |
| Child preload | M23R tracer only | Exactly same M23R tracer only; **NO M23T helper** |
| ROS domain/network/devices | Existing isolated, no-device profile | Same |
| QoS/graph/frame observer | Existing M23P/S checks | Same |
| SIGINT / child exit classification | Existing reproducibility and child code evidence | Same |
| Log retention and image ID | Unique immutable private artifacts for each run | Separate unique immutable private artifacts |

The **meaning of a positive result** is only that the
publisher-plugin **allowlist intervention** changed the
observed crash behavior in the particular verified runs.
The change may alter plugin construction, advertised DDS
endpoints, match callbacks, loading/refcounts, memory
layout and shutdown timing. It cannot discriminate Theora
from Zstd or isolate a faulty allocator, invalid callback
or unloaded page without independent evidence.

**Proposed gate boundaries to resolve in a later explicit
authorization (none assumed here):**

1. Current explicit operator approval, no-device scope,
   disposable isolated image/container, no host networking,
   physical device denial, no privileges/mounts, bounded
   runtime and log budget, fresh artifacts; archive runtime
   metadata and immutable image identifier.
2. Identical tracer-only baseline MUST reproducibly exit
   SIGSEGV `-11` after verified 900/900 synthetic
   delivery; failure to reproduce blocks intervention.
   Define repeatability rule **before** execution.
3. Separate override-value verification: effective
   `image_rect.enable_pub_plugins` must be a **string
   array** equal to `["image_transport/raw"]` only in
   intervention; verify the baseline effective value too.
   Otherwise the comparison is invalid.
4. Preserve safety guards and unchanged hardware exclusion,
   QoS/graph checks, tracer completeness and source
   instrumenter identity. No changes to OpenCV thread
   counts or global `LD_PRELOAD`.
5. On incomplete readiness, unverified plugin parameter,
   missing frame/observer counts, contaminated preloads,
   crash of a control, device exposure or ambiguous child
   exit: **FAIL CLOSED**, do not call clean launch parent
   exit a clean component exit.
6. Do not expand to real sensors, physical image-header-gap
   acceptance, STM32/IMU, VIO/Nav2/recorder or motion.

No repetition count, executable ROS command or automatic
test scheduling is specified: these would constitute an
independently authorized experiment, not source-only review.

## Decision following M23W Gate A — continue toward usable `/camera/image_rect`

**Operator intent (2026-10-20):** Continue work to make
`/camera/image_rect` reliable and lifecycle-safe for the future
XRSLAM/VIO image input. This is approval to **continue planning
and source-only analysis**, **not** permission to start another
ROS, Podman, synthetic, camera, or production test.

The camera-only history separates two distinct engineering
conditions:

1. **Delivery:** The M23N real-camera `raw_only_mjpeg`
   `reliable_100` subscriber received **1414** raw Images at
   ~29.821Hz, no header intervals >250ms and a clean
   component-free shutdown. In M23O's full rectifier
   `reliable_100` diagnostic, raw **1411** and rectified
   **1410** samples (~29.8Hz) had zero >250ms header
   intervals and **1410/1410** rect header stamps matched
   raw — but the component-container shutdown SIGSEGV
   still blocked acceptance. These are **receipts in those
   runs**, not independently authenticated camera-publisher
   counts or a general performance guarantee.
2. **Lifecycle:** M23S tracer-only synthetic zero-D arm was
   clean; the same-image nonzero-D arm delivered **900/900**
   matching rectified headers and then SIGSEGV **-11** on
   child SIGINT. Therefore, a `/camera/image_rect` topic
   existing or delivering frames does **not** mean the
   rectifier is safe to deploy.

### Definition of success for a future synthetic no-device diagnostic

The following is a **proposed acceptance specification**,
not an authorized test or a claim that it has passed:

- Exact stock `image_proc::RectifyNode` remains the
  pixel rectifier, exercising the **nonzero-D**
  `image_geometry`/OpenCV path, not substituting
  `/camera/image_raw`, changing `D` to zero,
  or redefining `/camera/image_rect`.
- Same 900-frame 640x480 synthetic RGB8 source with
  the original M23S fabricated nonzero-D CameraInfo,
  same header stamps, subscriber QoS and observed graph
  readiness. Retain M23P/M23R/M23S publisher-first synchronization
  and **original fail-closed validation**.
- Require complete publisher counts, correct synthetic
  distortion markers and valid image/CameraInfo association.
  Target **900/900 matched raw and rectified header
  stamps**, no unmatched rectified stamps, no >250ms
  rectified header-stamp interval, and complete graph/QoS
  verification. If counts miss the target, preserve
  the existing M23P minimum 810/900 as a **diagnostic
  completeness threshold only**, not an
  `image_rect` acceptance PASS.
- Require explicit **clean `component_container` child
  exit after SIGINT**, not just ROS launch parent exit 0.
  A SIGSEGV, other child failure, missing completion
  marker or timeout blocks the synthetic acceptance.
- Passing any synthetic result would validate **only
  that isolated tested condition**; it would not
  validate camera calibration, true optical rectification
  accuracy, physical image timing, camera/IMU phase,
  M23A bag continuity, XRSLAM/VIO or production.

### Proposed controlled two-arm decision tree — requires separate approval

**Arm A — original-behavior tracer-only control:** Same
M23S image/ELF identity, original M23R child-only loader
tracer, nonzero-D frames, default output plugin allowlist,
no M23T OpenCV helper. A new experimental harness
must first prove the *unmodified* baseline can still
reproduce the original SIGSEGV on SIGINT. **No skip
ahead** to raw-only if this does not reproduce. Review
must set a fixed, bounded repetition count **before**
any runtime execution; one historic crash is not
a repeatability estimate.

**Arm B — output raw-publisher-only intervention:**
Same controlled no-device image, source, publisher,
graph, QoS, preload, exit classifier and teardown
sequence; the **sole intended experimental variable**
is the stock `RectifyNode` output publisher parameter
`image_rect.enable_pub_plugins` as a string array
`["image_transport/raw"]`. Do not modify
`CameraInfo.D`, the upstream pixel processing,
the subscriber transport hint, `cv::getNumThreads`,
OpenCV threads or global `LD_PRELOAD`.

An approved runner would need to establish **effective
parameter parity** on both arms, not merely presence
of `parameters=` in source, and record the exact
loaded/declared publisher plugin set. Do not claim
Theora/Zstd loaded or unloaded based solely on their
close-target traces.

**Stop and interpretation:**

| Outcome | Bounded decision |
|---|---|
| Control does not reproduce fault | Stop; **no valid intervention comparison** |
| Control faults and intervention faults | Raw-only is **not a demonstrated fix**; investigate component/rectifier/OpenCV teardown and exact fault mapping under a separate scope |
| Control faults and intervention exits clean with complete rectified delivery | Output plugin fan-out/lifetime becomes a **candidate contributor**; repeatability and runtime plugin/parameter evidence still required; **no DSO attribution** |
| Any incomplete stream, endpoint, scope, safety or child exit evidence | BLOCKED regardless of clean parent exit |

A future source change should not be considered a
production remedy unless its mechanism is explained,
the original shutdown fault is shown absent under an
approved repeatable comparison, and real optical,
sensor/recording, safety and M23A gates are
subsequently completed. This review does **not**
authorize any of those subsequent steps.

### Pre-execution safety gate — NOT SATISFIED

A human operator must separately authorize a **written
bounded synthetic/no-device protocol** before any run,
including: approved exact image digest and source refs,
rootless disposable per-arm container, `--network none`,
private PID/IPC, `--cap-drop all`, no privilege elevation,
read-only filesystem with bounded tmpfs, **no `--device`**
mapping, no camera/serial/STM32/IMU/motor capabilities,
isolated ROS domain, protected immutable per-run evidence,
no global preload, explicit SIGINT/kill timeout policy,
fail-closed exit classifier, bounded run count, and
predefined stop rule. Host operator retains sole
execution authority. No new executable launch, test
script, container image, or experimental command
is created by this M23W document.

**Authorization: NO. Execution: NONE. Production: BLOCKED.**

## M23W Gate B — finite no-device protocol and explicit runtime approval (PENDING)

**Operator objective:** retain the real nonzero-D rectification
path and get `/camera/image_rect` to deliver correctly *and*
shut down cleanly. This specification is a **draft safety
decision**, not instructions to execute. **NO ROS, Podman,
image build, replay, physical sensor, or motor access is
authorized by this document or by "continue."**

### Already-existing guardrail sources, pinned locally

- `ros/m23p_no_device_rectifier_shutdown.py`,
  blob `0bda340b9d751eebf0c5464fb1bf4c608cff5616`:
  `M23P_SYNTHETIC_ONLY=1`, domain 170, localhost ROS
  discovery, checks no visible video/serial devices; verifies
  publisher graph-ready before attaching observer, then
  graph QoS before frame start, 900 frames, a two-second
  subscriber drain, independent component-child exit and
  `M23P` diagnostic verdict.
- `ros/m23p_qos_graph.py`,
  blob `907f663a50f12c801e2623ef1cfb3991d03b4c50`:
  checks exact `/camera/image_raw`, `/camera/camera_info`
  and `/camera/image_rect` publisher/subscriber identities,
  with `RELIABLE/VOLATILE` endpoints. **Does not capture
  `image_rect.enable_pub_plugins` effective value.**
- `ros/m23r_shutdown_trace_runner.py`,
  blob `c8388fec5091b3c85977228f739c9816cdebcf5b`:
  child-only tracer, refuses global `LD_PRELOAD`; requires
  a tracer constructor and coherent event parsing.
- `ros/m23s_rectification_ab.py`,
  blob `913e6abffff2bde21adb908a6a802070361cb4e1`:
  verifies exact fabricated D-vector/profile, delegates
  to unchanged M23R/M23P checks, records child `-11`
  separately if logged. A crash ordinarily makes its
  wrapper return nonzero; that is valid **fault evidence**
  when independently authenticated, **not a PASS**.

### Finite proposed pilot — maximum 2+2, zero authorized runs now

One *future* separately approved disposable image may
contain both launch variants, but they must run in separate,
fresh isolated containers with exactly the **same pinned
image digest** and installed ELF/library set. Since the new
image would differ from historical M23S, compare its
common installed ELF hashes with the historical record
and re-establish fault behavior in that new image. Record
the image ID without embedding it in GitHub. No dynamic
install, network pull, or runtime configuration mutation.

| Order | Trial | Rectifier output allowlist | Gate |
|---|---|---|---|
| 1 | `A1` | Original default (no override) | Nonzero D, unchanged M23R tracer-only child, verified 900 frames, expected child SIGSEGV -11 |
| 2 | `A2` | Original default (no override) | Same invariants, second independently verified SIGSEGV -11 |
| 3 | `B1` | `["image_transport/raw"]` | Only if A1/A2 both meet gates; require complete rectified delivery + explicit child exit |
| 4 | `B2` | `["image_transport/raw"]` | Only if B1's complete evidence has been separately inspected and no safety/quality blocker; maximum two interventions |

**Global cap: four trials, two baseline and two
intervention.** This is a bounded exploratory pilot only;
it is **not** a statistically powered study. Before execution,
the operator must approve actual image digests, protected
unique local evidence destinations, immutable artifact
retention, and the literal runtime invocation after a
separate review of the as-built launch/runner diff.

**Immediate STOP** if either baseline does not reproduce
child SIGSEGV `-11` with otherwise valid complete input,
if any input/graph/readiness/transport parameter attestation
is missing, if any hardware is visible, if any unplanned
process/network access appears, if resource caps are
exceeded, or if a trial exits ambiguously. On an
intervention `-11` crash, stop remaining intervention
runs; the raw-only intervention has not demonstrated a
solution. Never execute `B1` or `B2` by automatic
continuation from an earlier command: each new trial
requires the operator to inspect the previous summary.

### Single intended variable and necessary instrumentation

- Both arms: stock `image_proc::RectifyNode`, original
  M23S nonzero `D=[-0.08,0.01,0.001,-0.001,0]`,
  900 fabricated RGB8 640x480 frames and CameraInfo,
  unchanged publisher, graph QoS, subscriber,
  frame timestamps, child-only M23R tracer,
  output topic `/camera/image_rect`, two-second drain,
  SIGINT and fail-closed exit classification.
- **No M23T OpenCV `LD_PRELOAD` helper** on either arm.
  No new `cv::setNumThreads`, allocator flags, transport
  hint changes or zero-distortion substitution.
- **A arm:** no explicit output-publisher plugin
  allowlist parameter; the actual parameter value
  must be captured, including its string-array type.
- **B arm:** the sole intended override is the
  node-relative string-array parameter
  `image_rect.enable_pub_plugins =
  ["image_transport/raw"]`. Source review indicates
  this is the prefix of `image_transport/raw_pub`.
  The override must be set **before publisher creation**,
  not after the rectifier is loaded.
- Existing M23P graph inspector is **insufficient** for
  the parameter check. Any future diagnostic observer
  must capture the effective ROS parameter type
  and values from the **same `/camera/rectify_node`**
  before frames begin, with matching identical
  observation machinery in A and B. An unverified
  `ros2 param get` string, duplicate node names,
  or unchecked global override does not pass.
  Verify the output raw publisher remains available
  and the expected endpoints/QoS still match.
- The design must compare baseline/override launch
  source *structurally*: same `ComposableNode`
  plugin/name/namespace/remaps and instrumentation;
  only explicit publisher plugin allowlist differs.
  New instrumentation may perturb timing, so require
  a fresh baseline crash in the **final as-built**
  image and harness, not just in historical M23S.
- For each run capture local immutable summary and raw
  evidence *separately* under owner-only permissions;
  per-run log files must be new/not overwritten.
  Never post private log text, PIDs/TIDs, loader
  handles, core, host device enumeration or
  private paths to GitHub. Share only PASS/BLOCKED
  result markers, counts and nonidentifying verdicts.

### Interpretation and quality thresholds

- Source publisher must truthfully report **900 Image
  and 900 CameraInfo** messages, correct nonzero D
  and exact synthetic payload.
- Require graph snapshot before publishing, all
  expected `RELIABLE/VOLATILE` endpoints and
  consistent original ROS RMW in both arms.
- Existing harness accepts at least `810/900`
  raw and rect observations, `>=90%` rect header
  matching, zero unmatched rect stamps and
  no rect header intervals **>250 ms**. This
  threshold is an evidence-validity floor, **not**
  a final `/camera/image_rect` quality PASS.
- Target the historical full synthetic
  `900/900` matched delivery and explicitly
  **clean `component_container` child exit**
  on SIGINT for any provisional B-arm success.
  Never equate parent `ros2 launch` status
  with component-child status.
- A provisional clean result is **not** a verified
  root cause, production fix, or proof of camera
  calibration/optical accuracy. Output plugin fan-out,
  RMW endpoint count, memory mapping and callback
  lifetimes can change together. Do **not** blame
  Theora/Zstd/TBB from `dlclose` *target* names.

### Isolation and resource budget to approve separately

Require operator-reviewed exact `podman run` arguments
to establish **rootless**, `--network none`,
private IPC/PID, `--cap-drop all`,
`--security-opt no-new-privileges`,
`--read-only`, bounded `/tmp` tmpfs,
`--pull=never`, **no `--device`**, host
device mounts, privileged flags, host-network/PID,
serial, camera, IMU or motor access. Maintain
`ROS_DOMAIN_ID=170`, discovery `LOCALHOST`,
`LEROY_NAV2_DRY_RUN=1` and all historical
M23S isolation settings; separately attest the
production container remains untouched and
any production or motor-capable process remains
excluded. Operator approves per-trial timeout
bounds and max resource/log bytes before execution;
preexisting M23P waits are bounded but do not
replace a host-side total-run timeout or log cap.

**EXPLICIT APPROVAL REQUIRED FOR A FUTURE STEP:**
authorize *preparing executable no-device diagnostic
code* separately from authorizing *building or running*
it. Passing documentation-only Git/AST checks,
or a message such as "continue", permits **neither
build nor run**. Absent each scoped authorization:
`M23W_RUNTIME_AUTHORIZED=NO`,
`M23W_DEVICE_ACCESS=NONE`,
`M23W_PRODUCTION_ACCEPTANCE=BLOCKED`.

### Gate B offline read-only source-review check

Use **only host Git/Python**; no ROS or Podman:

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch --no-tags origin

PYTHONDONTWRITEBYTECODE=1 python3 - <<'PY'
import subprocess

def git(*parts):
    return subprocess.check_output(["git", *parts], text=True).strip()

head = "origin/feature/m23w-raw-publisher-allowlist-design"
base = "origin/feature/m23v-transport-teardown-source-review"
path = "ros/M23W_RAW_OUTPUT_PLUGIN_ALLOWLIST_PROTOCOL_DESIGN.md"
expected_file_change = "A\t" + path
if git("diff", "--name-status", base + "..." + head) != expected_file_change:
    raise SystemExit("M23W_GATE_B_BLOCKED=NON_DOC_CHANGE")

doc = git("show", head + ":" + path)
for required in (
    "## M23W Gate B",
    "A1", "A2", "B1", "B2",
    '["image_transport/raw"]',
    "before frames begin",
    "component_container",
    "STOP",
    "M23W_RUNTIME_AUTHORIZED=NO",
    "M23W_PRODUCTION_ACCEPTANCE=BLOCKED",
):
    if required not in doc:
        raise SystemExit("M23W_GATE_B_BLOCKED=MISSING_PROTOCOL_FIELD")
print("M23W_GATE_B_DOC_ONLY=PASS")
print("M23W_GATE_B_PREEXECUTION_PROTOCOL=DOCUMENTED")
print("M23W_RUNTIME_AUTHORIZED=NO")
print("M23W_DEVICE_ACCESS=NONE")
print("M23W_PRODUCTION_ACCEPTANCE=BLOCKED")
PY
```

## Gate A — host-only document and configuration-name inspection

Only fetch/check Git source and parse Python syntax. This
is a **read-only gate** and provides **no permission to run
ROS, Podman, builds, probes or hardware**.

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch --no-tags origin

PYTHONDONTWRITEBYTECODE=1 python3 - <<'PY'
import ast
import subprocess
from pathlib import Path

branch = "origin/feature/m23w-raw-publisher-allowlist-design"
base = "origin/feature/m23v-transport-teardown-source-review"
name = "ros/M23W_RAW_OUTPUT_PLUGIN_ALLOWLIST_PROTOCOL_DESIGN.md"
def git(*args):
    return subprocess.check_output(["git", *args], text=True).strip()

if git("diff", "--name-status", f"{base}...{branch}") != "A\t" + name:
    raise SystemExit("M23W_BLOCKED=UNEXPECTED_CHANGESET")

text = git("show", f"{branch}:{name}")
if len(text) > 70_000:
    raise SystemExit("M23W_BLOCKED=OVERSIZE_DOC")

for expected in (
    "image_rect.enable_pub_plugins",
    "image_transport/raw",
    'image_transport/raw_pub',
    "source-only",
    "M23T helper",
    "SIGSEGV",
    "BLOCKED",
    "UNRESOLVED",
):
    if expected not in text:
        raise SystemExit("M23W_BLOCKED=DOCUMENT_INCOMPLETE")

for filename in (
    "m23p_rectifier_only.launch.py",
    "m23r_rectifier_trace.launch.py",
    "m23t_rectifier_thread_trace.launch.py",
):
    src = git("show", f"{branch}:ros/{filename}")
    tree = ast.parse(src, filename=filename)
    calls = [
        n for n in ast.walk(tree)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id == "ComposableNode"
    ]
    if len(calls) != 1:
        raise SystemExit("M23W_BLOCKED=NODE_COUNT")
    keys = {k.arg: k.value for k in calls[0].keywords}
    if "parameters" in keys:
        raise SystemExit("M23W_BLOCKED=UNEXPECTED_OVERRIDE")

print("M23W_DOC_ONLY=PASS")
print("M23W_CONFIG_SOURCE_ONLY=PASS")
print("M23W_EXPERIMENT_AUTHORIZED=NO")
print("M23W_ROOT_CAUSE=UNRESOLVED")
print("M23W_DEVICE_ACCESS=NONE")
print("M23W_PRODUCTION_ACCEPTANCE=BLOCKED")
PY
```

**Gate A limitations:** confirms only source-controlled
documents/launch ASTs in Git, not private logs or ROS effective
parameters. The operator must **not** execute a test after PASS
unless a new specific runtime protocol is reviewed and explicitly
authorized.

## Final status

M23V source-only review **COMPLETE**.
M23W candidate intervention **SPECIFIED, NOT EXECUTED**.
Original SIGSEGV **ROOT CAUSE UNRESOLVED**.
Original M23A >250ms physical-camera image-header-gap acceptance,
physical camera, STM32/IMU, VIO, Nav2, recorder, robot motion and
all production acceptance **BLOCKED**.
Keep all stacked PRs **draft and unmerged**.
Raw logs, private SHA256 manifest and core dumps stay on the
operator host; never commit or post them.
