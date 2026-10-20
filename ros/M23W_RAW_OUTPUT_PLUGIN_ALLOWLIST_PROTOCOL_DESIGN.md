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
