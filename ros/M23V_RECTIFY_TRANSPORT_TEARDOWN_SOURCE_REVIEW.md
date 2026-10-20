# M23V — stock RectifyNode and image_transport/pluginlib teardown ownership

**Source-only / documentation-only review; NO ROS, Podman, device, build,
reproduction, or production execution.** This is a new stacked draft
diagnostic PR based on M23U. Parent image pairs, logs, historical
cores, and M23U private SHA256 manifest remain **operator-host-only**.

## Preconditions and already-reported evidence

- M23U operator reported `M23U_MANIFEST_INTEGRITY=PASS` with verified
  hashes for `m23s_elf`, `m23t_elf`, `m23s_crash_trace` and
  `m23t_clean_trace`; reported `M23U_PRODUCTION_ACCEPTANCE=BLOCKED`.
  This source review did **not** access those private files.
- M23S `D=0` original arm delivered 900/900 matching stamps and
  shut down cleanly. M23S nonzero `D=[-0.08,0.01,0.001,-0.001,0]`
  delivered 900/900 but `component_container` exited **SIGSEGV -11**
  on SIGINT. One paired result supports an **association**, not causation.
- Clean-default M23T nonzero-D arm included an additional child-scoped
  OpenCV thread-probe `LD_PRELOAD`; the API reported **16 -> 16**.
  The `single` arm was **not run** because the crash prerequisite
  did not reproduce under the default probe.
- Operator host-only comparison verified **13/13 common M23S/M23T
  resolved ELFs** byte-identical by SHA256 and Build ID; M23T added
  its OpenCV helper, linked to `libopencv_core.so.406`.
- M23S traced `libtbbmalloc.so.2` `dlclose` overlapped **four
  T1 close intervals**, including Theora and Zstd close **targets**
  and two short same-handle nested closes, and lasted **6.859 ms**.
  M23T's corresponding close lasted **0.015 ms** with no observed
  cross-thread close overlap. Every observed `dlclose` returned;
  M23S failed to emit the M23R *tracer* destructor; M23T emitted it.
  Nested same-handle `dlopen` filename recorded `<unnamed>`,
  which conflates null and empty arguments. No caller, unloaded
  mapping or faulting-PC module has been identified.
- **No root cause, fix, production or physical-camera acceptance.**
  Stop the M23U loader-name-only investigation.

## Source provenance — Git blobs, NOT deployed ELF Build IDs

The GitHub upstream branches and tag below were read for this
review. Their individual git blob identifiers pin reviewed **source**.
They do not establish the exact installed ROS Jazzy source revision.

| Upstream source | Ref | Blob |
|---|---|---|
| `ros-perception/image_pipeline/image_proc/src/rectify.cpp` | `jazzy` | `e7cbd9b1e1a0c42fff53241a0094d80988979623` |
| `image_proc/include/image_proc/rectify.hpp` | `jazzy` | `aec7e5d2ebdf90c1f22bee0710438a227e25fc5d` |
| `ros-perception/image_common/image_transport/src/publisher.cpp` | `jazzy` | `48f0257e4dacd959bfc6f64ef7245cd2059ce057` |
| `image_transport/include/image_transport/publisher.hpp` | `jazzy` | `4f529121796d57553d40d32d2df7c4f7636ad843` |
| `image_transport/src/image_transport.cpp` | `jazzy` | `dfddcb91b45e4a127c44f2ce3109f34834e6819b` |
| `image_transport/src/camera_subscriber.cpp` | `jazzy` | `3075564a1f23393a24d9a767c2cae5be43a3c49a` |
| `image_transport/include/image_transport/camera_subscriber.hpp` | `jazzy` | `5157111df4d63811244663a9f8399ebcf49ea9c5` |
| `image_transport/src/subscriber.cpp` | `jazzy` | `85c7927cbb0e952a597fd85d001d7ec74b2782da` |
| `image_transport/include/image_transport/subscriber_filter.hpp` | `jazzy` | `37b216ed9a76bf90aba54424a00eca3ba12a282f` |
| `image_transport/include/image_transport/publisher_plugin.hpp` | `jazzy` | `c545b4cad755dd956ab7ef51f03ec54adadb8d07` |
| `ros2/rclcpp/rclcpp_components/src/component_container.cpp` | `jazzy` | `b9e667b141fd052c858e434afeb1fad4ef9a49bb` |
| `rclcpp_components/src/component_manager.cpp` | `jazzy` | `08e28db9d6863d9bcaee7f128996432420dd142c` |
| `rclcpp_components/include/rclcpp_components/component_manager.hpp` | `jazzy` | `6238284278f9c93026c7bb82dbbbb80927d0d2cf` |
| `rclcpp/include/rclcpp/event_handler.hpp` | `jazzy` | `887c571d16e74a3e69f092e5563036ef984e8590` |
| `rclcpp/src/rclcpp/publisher_base.cpp` | `jazzy` | `111246ed96fbd85008666ec9750e5eb97a415ee2` |
| `rclcpp/src/rclcpp/executors/single_threaded_executor.cpp` | `jazzy` | `689bbae39813e5a74224171619f933334183e4e2` |
| `ros/pluginlib/pluginlib/include/pluginlib/class_loader_imp.hpp` | `ros2` | `149d1cef04cb5a4376fb98dd5961835581050300` |
| `ros/class_loader/src/class_loader.cpp` | `ros2` | `a9a8823c44ad68a132e21aac3d31f3777f2cc6bc` |
| `ros/class_loader/src/multi_library_class_loader.cpp` | `ros2` | `d7ba907880bfe8a5ca6b9b8147a13bd0d1a97d16` |
| `ros/class_loader/src/class_loader_core.cpp` | `ros2` | `632f8c294e385ff41f87abb4056a73f121d64a01` |
| `ros-perception/vision_opencv/image_geometry/src/pinhole_camera_model.cpp` | tag `4.1.0` | `b3c92214b9123ab5b6089399a99e20994097bd6d` |

The upstream `ros2` class_loader/pluginlib branch and the
`vision_opencv` 4.1.0 tag are **reviewed reference sources**,
NOT proven deployed source versions. M23U already recorded the
installed ELF IDs, separate from these git blob IDs.

## 1. RectifyNode callback and member teardown

Source: [rectify.cpp](https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/src/rectify.cpp),
[rectify.hpp](https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/include/image_proc/rectify.hpp).

- Constructor sets `pub_options.event_callbacks.matched_callback =
  [this](...)` and conditionally `sub_camera_.shutdown()` or
  `create_camera_subscription(...std::bind(&RectifyNode::imageCb,
  this,...))`. Both callbacks retain **raw `this`**, not an owning
  `shared_ptr` or self-expiring guard.
- Stock class declares, in order:
  `sub_camera_`, `queue_size_`, `interpolation_`,
  `image_topic_`, `pub_rect_`, `model_`.
  There is **no declared explicit `~RectifyNode`**. Under ordinary
  C++ reverse member destruction, `model_` destructs first,
  then `pub_rect_` and finally `sub_camera_` (ignoring trivial
  intermediate members). This is a **real source-level lifetime
  ordering** worth checking, not evidence a callback ran late.
- In nonzero D, `imageCb` updates `model_`, creates a shallow
  cv_bridge view of the callback-owned input, rectifies through
  image_geometry/OpenCV, copies the output image and publishes it.
  A cached `PinholeCameraModel` map is therefore alive until
  `model_` destruction, independently of plugin teardown.
- In zero D, callback publishes the input without the
  camera-model/OpenCV mapping path; nonetheless construction of
  `pub_rect_` and advertised publisher plugins occurs in *both*
  arms. Thus plugin availability/loading is **not itself exclusive
  to the crashing nonzero-D branch**.

**Not established:** late invocation of `matched_callback`,
`imageCb`, access to destroyed `model_`, or unsafe concurrent
mutations of `sub_camera_`. A single-threaded executor is a
source-level design constraint on callback execution in that
executor, not a proof about all DDS/loader/allocator threads.

## 2. Publisher owns instances; image_transport also owns loaders

Source: [publisher.cpp](https://github.com/ros-perception/image_common/blob/jazzy/image_transport/src/publisher.cpp),
[image_transport.cpp](https://github.com/ros-perception/image_common/blob/jazzy/image_transport/src/image_transport.cpp).

- In `Publisher::Publisher`, `loader->getDeclaredClasses()`
  supplies `all_transport_names`, used as the **default** for
  `<relative-output-topic>.enable_pub_plugins`. The resulting
  allowlist controls `loader->createUniqueInstance(lookup_name)`
  and `pub->advertise(...)` for each permitted publisher plugin.
  This explains why installed, declared Theora/Zstd publishers
  could be instantiated despite raw-only synthetic delivery;
  **actual effective parameters and successful advertising have
  not been collected**. Subscription-side
  `image_transport="raw"` does **not by itself** prove the
  **output publisher's** plugin list was restricted to raw.
- `Publisher::Impl` contains `PubLoaderPtr loader_` declared
  **before** `publishers_`, a vector of shared publisher-plugin
  instances. Its `shutdown()` calls `pub->shutdown()` on each
  before `publishers_.clear()`; its destructor calls `shutdown()`.
  The public `Publisher::shutdown()` calls this and resets its
  shared `Impl`. The ordering makes instance cleanup visible
  in source, but a plugin shutdown may have its own callback
  lifetimes. The source does **not** prove they have all drained.
- `image_transport.cpp` contains a namespace-scope
  `static Impl * kImpl = new Impl();` holding shared
  `PubLoader`/`SubLoader`, which are passed to publishers
  and subscribers. This pointer is **not deleted in the
  reviewed source**. Per-publisher plugin instance destruction
  must not be equated with immediate destruction of these
  shared class-loader objects.
- The publisher overload that accepts `UniquePtr` can retain
  additional temporary plugin `shared_ptr` references during
  publishing. There is no observation that such references
  outlive the ROS executor in the crashing run.

**Not established:** what exact operation invoked each logged
Theora/Zstd `dlclose`, whether either library actually unloaded,
or whether its code was executing in another thread at that time.
The M23R tracer labels the `dlclose` **target handle**, not the
invoking stack.

## 3. Camera subscriber and synchronized callback cancellation

Source: [camera_subscriber.cpp](https://github.com/ros-perception/image_common/blob/jazzy/image_transport/src/camera_subscriber.cpp),
[subscriber.cpp](https://github.com/ros-perception/image_common/blob/jazzy/image_transport/src/subscriber.cpp),
[subscriber_filter.hpp](https://github.com/ros-perception/image_common/blob/jazzy/image_transport/include/image_transport/subscriber_filter.hpp).

- `CameraSubscriber::Impl` owns `image_sub_`,
  `info_sub_`, `sync_` and a `check_synced_timer_`.
  It registers a `TimeSynchronizer` callback wrapping the
  caller's `imageCb`, increments counters through bound
  references/pointers, and registers
  `check_synced_timer_` using `std::bind(..., impl_.get())`.
- `Impl::~Impl()` calls `shutdown()`, which marks
  `unsubscribed_` and calls `image_sub_.unsubscribe()` and
  `info_sub_.unsubscribe()`. `SubscriberFilter::unsubscribe()`
  calls `Subscriber::shutdown()`; the latter calls the
  transport plugin's `shutdown()`.
- `Subscriber::Impl` holds `SubLoaderPtr loader_`
  before `shared_ptr<SubscriberPlugin> subscriber_`;
  its destructor first requests shutdown, and normal reverse
  member destruction releases the plugin before the loader
  reference.
- Destroying a timer handle and unsubscribing do not, solely
  by these definitions, document a **join/drain guarantee for
  callbacks already executing on other threads**. Those
  semantics require the actual executor/RMW/callback-group
  state and/or separate instrumentation; no UAF has been shown.

## 4. Component executor and plugin loader ordering

Source: [component_container.cpp](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_components/src/component_container.cpp),
[component_manager.cpp](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_components/src/component_manager.cpp),
[component_manager.hpp](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_components/include/rclcpp_components/component_manager.hpp).

- `component_container` declares its `ComponentManager`
  shared pointer **before** the `SingleThreadedExecutor`
  shared pointer, explicitly commenting that executor must
  destruct first to avoid releasing component entities after
  unloading their code. The main thread runs `exec->spin()`.
- `ComponentManager::~ComponentManager()` conditionally
  removes component nodes from its executor when it can lock
  the weak executor pointer. The header declares
  `loaders_` **before** `node_wrappers_`, so normal
  reverse member destruction releases node wrappers before
  component class loaders.
- `PublisherBase` registers `matched_callback` in an
  `EventHandler`, whose `execute` calls the stored event
  callback; `SingleThreadedExecutor::spin` dispatches work
  in its spin loop. These codepaths identify event execution
  and cancellation **mechanisms**, not a verified all-threads
  drained state at the failing SIGINT.
- Manager/component loader ordering does **not** itself govern
  every independent image_transport plugin object, C++ static
  destructor, RMW worker, or allocator thread.

## 5. class_loader unload is conditional, not proof of unmapping

Source: [class_loader.cpp](https://github.com/ros/class_loader/blob/ros2/src/class_loader.cpp),
[class_loader_core.cpp](https://github.com/ros/class_loader/blob/ros2/src/class_loader_core.cpp),
[multi_library_class_loader.cpp](https://github.com/ros/class_loader/blob/ros2/src/multi_library_class_loader.cpp),
[pluginlib class loader](https://github.com/ros/pluginlib/blob/ros2/pluginlib/include/pluginlib/class_loader_imp.hpp).

- The upstream `class_loader::ClassLoader` destructor invokes
  `unloadLibrary()`. The unload path **declines unloading**
  when objects created by that loader remain live, and tracks
  its own load reference count.
- `class_loader_core.cpp` conditionally retains a library
  when a non-pure plugin library was opened, or when
  other registered factory metadata still refers to that
  library. A `MultiLibraryClassLoader` destructor calls
  `shutdownAllClassLoaders()`; pluginlib's low-level
  multi-library loader is configured with on-demand
  loading/unloading **disabled** in the reviewed reference.
- Therefore, `DLCLOSE_BEGIN` and `DLCLOSE_RETURN` events
  do not prove that a specific instruction page was unmapped.
  Neither do they show which function originally called close.
  The earlier M23Q historical core's unmapped RIP still has
  no verified module attribution.

## 6. Conclusions and hypothesis ranking

| Evidence or hypothesis | Supported conclusion | Still missing |
|---|---|---|
| Stock output plugin fan-out | Default publisher allowlist can enable every declared plugin | Installed effective parameters and exact installed source |
| Callback lifetime | Raw-`this` captures exist, plus subscriber timer bind to implementation pointer | Executing-after-free or undrained callback evidence |
| Teardown member order | `model_` before output `pub_rect_` before `sub_camera_` | Whether active callbacks overlap this order |
| Component unload order | Manager's node wrappers declared after component loaders; executor destruction intentionally first | DDS/allocator/background thread finalization |
| Class-loader unload | May defer actual unload for live factories/objects | Mapped DSOs and faulting RIP at SIGSEGV |
| M23S/M23T comparison | 13 common recorded ELF hashes/Build IDs match; shutdown close interleavings differ | Causality, repeats, thread state and source-version provenance |

**Priority research questions, not root-cause verdicts:**
(1) publisher-plugin shutdown + subscription/callback cancellation
order; (2) nonzero-D cached OpenCV-map lifetime relative to
teardown; (3) independent class_loader/RMW/allocator thread or
finalizer behavior; (4) exact faulting executable mapping. The
earlier successful TBB and Theora/Zstd `dlclose` return markers
cannot rank a specific defective library.

## Gate A — operator host-only documentation integrity

The following uses only Git and Python source-text checks; **it
does not invoke ROS, Podman, device interfaces or local builds**.

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23v-transport-teardown-source-review
git pull --ff-only origin feature/m23v-transport-teardown-source-review
git diff --name-status origin/feature/m23u-rectify-source-abi-review...HEAD

PYTHONDONTWRITEBYTECODE=1 python3 - <<'PY'
from pathlib import Path
p = Path("ros/M23V_RECTIFY_TRANSPORT_TEARDOWN_SOURCE_REVIEW.md")
text = p.read_text()
required = [
    "M23U_MANIFEST_INTEGRITY=PASS",
    "Publisher::Impl",
    "enable_pub_plugins",
    "CameraSubscriber::Impl",
    "sub_camera_",
    "pub_rect_",
    "model_",
    "class_loader",
    "UNRESOLVED",
    "BLOCKED",
]
# Manifest marker is operator-reported: not claimed as this report's command.
for token in required:
    if token not in text:
        raise SystemExit("M23V_SOURCE_DOCUMENT_MISSING=" + token)
print("M23V_HOST_ONLY_SOURCE_DOCUMENT=PASS")
print("M23V_DEVICE_ACCESS=NONE")
print("M23V_PRODUCTION_ACCEPTANCE=BLOCKED")
PY
```

Expected diff: **one added markdown file**
`ros/M23V_RECTIFY_TRANSPORT_TEARDOWN_SOURCE_REVIEW.md` and no
production/source/launch/Dockerfile/test modifications.
`M23V_HOST_ONLY_SOURCE_DOCUMENT=PASS` is only a **document
completeness** assertion; it cannot independently verify all
upstream source claims or the operator private manifest.

## Operator M23V Gate A report — PASS (2026-10-20)

The operator supplied the following completed **host-only**
source-document and change-scope gate markers:

```text
M23V_ONLY_DOC_CHANGE=PASS
M23V_HOST_ONLY_SOURCE_DOCUMENT=PASS
M23V_DEVICE_ACCESS=NONE
M23V_PRODUCTION_ACCEPTANCE=BLOCKED
```

These are the operator's reported results. They establish that the
M23V checkout passed the supplied *document-only* and expected
commit/file-scope checks. They do **not** show that compiled runtime
objects match each upstream git blob, or that any callback was
executed after destruction. The PR remains draft/unmerged.

## Gate B — host-only runner/configuration source discovery, NOT a ROS experiment

**Goal:** inventory existing tracked `ros/` source or launch files
that mention output-publisher `enable_pub_plugins`, `image_rect`,
`image_transport`, `component_container` or related diagnostic
runner names. This is preparatory source research, not a license
to change configuration or run synthetic publishing.

Operator command (only Git and Python; no network, ROS, Podman,
devices, builds, instrumentation or filesystem writes):

```bash
cd "$(git rev-parse --show-toplevel)"
PYTHONDONTWRITEBYTECODE=1 python3 - <<'PY'
from collections import defaultdict
from pathlib import Path
import subprocess

tracked = subprocess.check_output([
    "git", "ls-files", "-z", "--", "ros/"
]).decode("utf-8", "surrogateescape").split("\0")

suffixes = {".py", ".sh", ".bash", ".yaml", ".yml",
            ".xml", ".cpp", ".hpp", ".c", ".h"}
needles = (
    "enable_pub_plugins",
    "image_rect",
    "image_transport",
    "component_container",
    "m23s",
    "m23t",
)
hits = defaultdict(list)
count = 0
for raw in tracked:
    if not raw:
        continue
    path = Path(raw)
    if path.suffix.lower() not in suffixes:
        continue
    if path.is_symlink() or not path.is_file():
        continue
    if path.stat().st_size > 2_000_000:
        continue
    content = path.read_text(encoding="utf-8", errors="replace")
    for lineno, line in enumerate(content.splitlines(), 1):
        lower = line.lower()
        for needle in needles:
            if needle in lower:
                hits[needle].append((path.as_posix(), lineno))
                count += 1

print("M23V_SOURCE_DISCOVERY_BEGIN")
for needle in needles:
    records = hits[needle]
    print(f"M23V_SOURCE_TOKEN={needle}|hits={len(records)}")
    for path, lineno in records[:24]:
        print(f"M23V_SOURCE_LOCATION={needle}|{path}|line={lineno}")
    if len(records) > 24:
        print(f"M23V_SOURCE_OMITTED={needle}|{len(records)-24}")
print(f"M23V_SOURCE_MATCH_TOTAL={count}")
print("M23V_DEVICE_ACCESS=NONE")
print("M23V_SOURCE_DISCOVERY=PASS")
print("M23V_PRODUCTION_ACCEPTANCE=BLOCKED")
print("M23V_SOURCE_DISCOVERY_END")
PY
```

**Required:** `M23V_SOURCE_DISCOVERY=PASS`,
`M23V_DEVICE_ACCESS=NONE` and
`M23V_PRODUCTION_ACCEPTANCE=BLOCKED`, plus the bounded
token/location summary. A zero-match source token is **not**
proof of a runtime default or loaded parameter value; it only
means this limited tracked-file scan found none.

**Stop condition:** No ROS/test/container invocation, package
installation or parameter changes. A future `enable_pub_plugins`
A/B must first have an independently verified crashing
non-probe baseline, an explicitly defined sole variable, immutable
image IDs, reserved private logs and separate operator authorization.
M23T's clean default cannot substitute as the crashing baseline.

## Operator M23V Gate B — tracked source discovery PASS

The operator's read-only tracked-`ros/` source scan reported:

```text
M23V_SOURCE_TOKEN=enable_pub_plugins|hits=0
M23V_SOURCE_TOKEN=image_rect|hits=54
M23V_SOURCE_TOKEN=image_transport|hits=3
M23V_SOURCE_TOKEN=component_container|hits=46
M23V_SOURCE_TOKEN=m23s|hits=111
M23V_SOURCE_TOKEN=m23t|hits=81
M23V_DEVICE_ACCESS=NONE
M23V_SOURCE_DISCOVERY=PASS
M23V_PRODUCTION_ACCEPTANCE=BLOCKED
```

The scan examined tracked files ending in
`.py`, `.sh`, `.bash`, `.yaml`, `.yml`, `.xml`,
`.cpp`, `.hpp`, `.c`, or `.h`; ignored missing, symlink
and >2MiB files. The `enable_pub_plugins` count **0** is a
bounded repository scan, not a claim that the stock upstream
library lacks this parameter or that its effective runtime value
was read. It did not inspect untracked/generated scripts,
container overlays, CLI/global ROS arguments or installed
parameter overrides. Notably, the single M23S synthetic
publisher uses ordinary `rclcpp::create_publisher` rather than
image_transport to emit `/camera/image_raw` and
`/camera/camera_info`.

### Exact local launch and runner path — static review

Additional committed local sources were read from this M23V
branch without execution:

| Repository source | Git blob | Configuration or harness fact |
|---|---|---|
| `ros/m23p_rectifier_only.launch.py` | `b49793f7c59ed471b6d0ba24020afc17c0b2d14b` | One `image_proc::RectifyNode` ComposableNode under `/camera`, `image→image_raw`, `camera_info→camera_info`, **no `parameters=` keyword** |
| `ros/m23r_rectifier_trace.launch.py` | `cf94a395c608f1aca2411be9a5c674ce7f3875c2` | Same component, namespace and remaps, no `parameters=`; child launch adds `/opt/m23r/libm23r_loader_trace.so` preload |
| `ros/m23t_rectifier_thread_trace.launch.py` | `d035f5ec58622c8b1eb04335030d0ec09fa94f68` | Same rectifier options, no `parameters=`; child preload is M23R tracer **plus** `/opt/m23t/libm23t_opencv_thread_probe.so` |
| `ros/m23p_no_device_rectifier_shutdown.py` | `0bda340b9d751eebf0c5464fb1bf4c608cff5616` | `LAUNCH` points to M23P launch; starts synthetic publisher before rectified observer, verifies graph/QoS, later signals shutdown and checks component-child result |
| `ros/m23r_shutdown_trace_runner.py` | `c8388fec5091b3c85977228f739c9816cdebcf5b` | Replaces M23P `LAUNCH` with traced M23R launch; parses component loader events |
| `ros/m23s_rectification_ab.py` | `913e6abffff2bde21adb908a6a802070361cb4e1` | Reuses M23R/M23P harness and swaps to M23S fixture; changes fabricated CameraInfo distortion profile |
| `ros/m23s_cpp/src/m23s_ab_publisher.cpp` | `bab0ee8adfa046614c43c7ad4b019df968f7958e` | Synthetic Image/CameraInfo emitted with ROS `create_publisher`; distortion profile controlled by `M23S_DISTORTION_PROFILE` |
| `ros/m23t_thread_probe_runner.py` | `0fb1ffca165d6f262f1b97b5f8193eaf411cc64e` | Runs M23S with alternate M23T traced launch; validates OpenCV constructor setup marker |

**Deduction, conditional on the installed Jazzy implementation
matching reviewed publisher source:** absent an explicitly
supplied `image_rect.enable_pub_plugins` override, the
`RectifyNode` output publisher defaults to its set of declared
image_transport publisher plugins. The source can therefore
account for Theora and Zstd being instantiated in both D=0
and nonzero-D configurations even though the synthetic publisher
only emits uncompressed Image and CameraInfo. This **does not
show** that a codec actually transmitted frames, that the
effective runtime default was recorded, or that a plugin
caused shutdown SIGSEGV.

**Baseline warning:** M23S produced a nonzero-D SIGSEGV in the
original traced pair, but M23T's `default` preload helper
perturbed the child and exited cleanly. A new `raw` allowlist
A/B cannot claim a controlled comparison unless its
**unmodified, identical-preload baseline reproduces** the crash
again in a separately authorized synthetic no-device protocol.
No such experimental authorization is part of this review.

## Gate C — offline AST invariants for all three launch files

This **host-only, source-text-only** command parses source ASTs
without importing `launch`, ROS or the underlying Python modules.
It is a *configuration review* gate only:

```bash
cd "$(git rev-parse --show-toplevel)"
test "$(git branch --show-current)" = \
  "feature/m23v-transport-teardown-source-review" || {
  echo "M23V_BLOCKED=WRONG_BRANCH"
  exit 2
}
PYTHONDONTWRITEBYTECODE=1 python3 - <<'PY'
import ast
from pathlib import Path

names = [
    "m23p_rectifier_only.launch.py",
    "m23r_rectifier_trace.launch.py",
    "m23t_rectifier_thread_trace.launch.py",
]
expected = {
    "plugin": "image_proc::RectifyNode",
    "namespace": "/camera",
}
print("M23V_LAUNCH_AST_AUDIT_BEGIN")
for name in names:
    path = Path("ros") / name
    if path.is_symlink() or not path.is_file():
        raise SystemExit("M23V_BLOCKED=SOURCE_MISSING:" + name)
    source = path.read_text(encoding="utf-8")
    if len(source) > 250_000:
        raise SystemExit("M23V_BLOCKED=SOURCE_OVERSIZE:" + name)
    tree = ast.parse(source, filename=name)
    nodes = [
        n for n in ast.walk(tree)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id == "ComposableNode"
    ]
    if len(nodes) != 1:
        raise SystemExit("M23V_BLOCKED=NODE_COUNT:" + name)
    node = nodes[0]
    keys = {k.arg: k.value for k in node.keywords}
    if any(key not in keys or
           not isinstance(keys[key], ast.Constant) or
           keys[key].value != value
           for key, value in expected.items()):
        raise SystemExit("M23V_BLOCKED=NODE_IDENTITY:" + name)
    if "parameters" in keys:
        raise SystemExit("M23V_BLOCKED=EXPLICIT_PARAMS:" + name)
    if "remappings" not in keys or not isinstance(keys["remappings"], ast.List):
        raise SystemExit("M23V_BLOCKED=REMAPPINGS:" + name)
    remaps = ast.literal_eval(keys["remappings"])
    if remaps != [("image", "image_raw"),
                  ("camera_info", "camera_info")]:
        raise SystemExit("M23V_BLOCKED=REMAP_VALUES:" + name)
    print(f"M23V_LAUNCH={name}|NODE=RectifyNode|EXPLICIT_PARAMS=NONE|REMAPS=PASS")

if not all((Path("ros") / n).is_file() for n in names):
    raise SystemExit("M23V_BLOCKED=SOURCE_DISAPPEARED")
print("M23V_LAUNCH_AST_AUDIT=PASS")
print("M23V_DEVICE_ACCESS=NONE")
print("M23V_PRODUCTION_ACCEPTANCE=BLOCKED")
print("M23V_LAUNCH_AST_AUDIT_END")
PY
```

On failure, **stop**. Do not edit the stock component,
change plugin parameters or initiate a ROS/Podman/container
process. PASS confirms only those literal launch-source AST
invariants in the checked-out repo, not runtime or ABI state.

## Operator M23V Gate C — three literal launch ASTs PASS

The operator executed the **host-only Python AST parser** supplied
by M23V and reported:

```text
M23V_LAUNCH=m23p_rectifier_only.launch.py|NODE=RectifyNode|EXPLICIT_PARAMS=NONE|REMAPS=PASS
M23V_LAUNCH=m23r_rectifier_trace.launch.py|NODE=RectifyNode|EXPLICIT_PARAMS=NONE|REMAPS=PASS
M23V_LAUNCH=m23t_rectifier_thread_trace.launch.py|NODE=RectifyNode|EXPLICIT_PARAMS=NONE|REMAPS=PASS
M23V_LAUNCH_AST_AUDIT=PASS
M23V_DEVICE_ACCESS=NONE
M23V_PRODUCTION_ACCEPTANCE=BLOCKED
```

This verifies that the three **checked-out source ASTs** contain one
literal `image_proc::RectifyNode` definition each, namespace
`/camera`, exactly the same `image → image_raw` and
`camera_info → camera_info` remaps and **no ComposableNode
`parameters=` keyword**. It does *not* prove the installed
Jazzy plugin configuration, runtime/CLI parameter overrides or
whether any shutdown callback was active.

### Gate D — source-only preload and wrapper dispatch audit

The exact launch source shows **no preload assignment in M23P**;
M23R preloads only its loader tracer in the launch process
environment; M23T uses **that tracer plus the OpenCV helper**.
M23S delegates to M23R which delegates to the underlying
M23P runner. M23T replaces the M23R trace launch path and
delegates to M23S. This makes M23T `default` a materially
different child initialization condition even with
`cv::getNumThreads()=16` before and after.

Operator **host-only**, no-import AST validation (the inspected
Python files are only parsed and never imported or executed):

```bash
cd "$(git rev-parse --show-toplevel)"
test "$(git branch --show-current)" = \
  "feature/m23v-transport-teardown-source-review" || {
  echo "M23V_BLOCKED=WRONG_BRANCH"
  exit 2
}
PYTHONDONTWRITEBYTECODE=1 python3 - <<'PY'
import ast
from pathlib import Path

launches = {
    "m23p_rectifier_only.launch.py": None,
    "m23r_rectifier_trace.launch.py":
        "/opt/m23r/libm23r_loader_trace.so",
    "m23t_rectifier_thread_trace.launch.py":
        "/opt/m23r/libm23r_loader_trace.so:"
        "/opt/m23t/libm23t_opencv_thread_probe.so",
}

def read_ast(name):
    path = Path("ros") / name
    if path.is_symlink() or not path.is_file():
        raise SystemExit("M23V_BLOCKED=MISSING_SOURCE:" + name)
    if path.stat().st_size > 256_000:
        raise SystemExit("M23V_BLOCKED=OVERSIZE_SOURCE:" + name)
    return ast.parse(path.read_text(encoding="utf-8"), filename=name)

def string_expr(expr, constants):
    if isinstance(expr, ast.Constant) and isinstance(expr.value, str):
        return expr.value
    if isinstance(expr, ast.Name) and expr.id in constants:
        return constants[expr.id]
    if isinstance(expr, ast.BinOp) and isinstance(expr.op, ast.Add):
        return string_expr(expr.left, constants) + string_expr(expr.right, constants)
    raise ValueError("non-literal environment expression")

def has_call(tree, owner, method):
    return any(
        isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute)
        and n.func.attr == method
        and isinstance(n.func.value, ast.Name)
        and n.func.value.id == owner
        for n in ast.walk(tree)
    )

def has_attr_assignment(tree, owner, field, value_name):
    return any(
        isinstance(n, ast.Assign)
        and isinstance(n.value, ast.Name)
        and n.value.id == value_name
        and any(
            isinstance(target, ast.Attribute)
            and isinstance(target.value, ast.Name)
            and target.value.id == owner
            and target.attr == field
            for target in n.targets
        )
        for n in ast.walk(tree)
    )

print("M23V_PRELOAD_DISPATCH_AUDIT_BEGIN")

for name, expected in launches.items():
    tree = read_ast(name)
    constants = {}
    for item in tree.body:
        if (isinstance(item, ast.Assign)
            and len(item.targets) == 1
            and isinstance(item.targets[0], ast.Name)
            and isinstance(item.value, ast.Constant)
            and isinstance(item.value.value, str)):
            constants[item.targets[0].id] = item.value.value

    calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "SetEnvironmentVariable"
    ]
    if expected is None:
        if calls:
            raise SystemExit("M23V_BLOCKED=UNEXPECTED_PRELOAD:" + name)
        label = "NONE"
    else:
        if len(calls) != 1:
            raise SystemExit("M23V_BLOCKED=PRELOAD_COUNT:" + name)
        keys = {kw.arg: kw.value for kw in calls[0].keywords}
        try:
            actual_name = string_expr(keys["name"], constants)
            actual_value = string_expr(keys["value"], constants)
        except (KeyError, ValueError) as exc:
            raise SystemExit("M23V_BLOCKED=PRELOAD_EXPRESSION:" + name) from exc
        if actual_name != "LD_PRELOAD" or actual_value != expected:
            raise SystemExit("M23V_BLOCKED=PRELOAD_VALUE:" + name)
        label = "TRACE_ONLY" if "m23r_" in name else "TRACE_PLUS_OPENCV"
    print(f"M23V_LAUNCH_PRELOAD={name}|{label}|PASS")

r = read_ast("m23r_shutdown_trace_runner.py")
s = read_ast("m23s_rectification_ab.py")
t = read_ast("m23t_thread_probe_runner.py")
checks = [
    has_call(r, "m23p", "run"),
    has_attr_assignment(r, "m23p", "LAUNCH", "TRACE_LAUNCH"),
    has_call(s, "m23r", "run"),
    has_attr_assignment(s, "m23p", "PUBLISHER", "PUBLISHER"),
    has_call(t, "m23s", "run"),
    has_attr_assignment(t, "m23r", "TRACE_LAUNCH", "THREAD_LAUNCH"),
]
if not all(checks):
    raise SystemExit("M23V_BLOCKED=RUNNER_DISPATCH")
print("M23V_RUNNER_DISPATCH=PASS")
print("M23V_CHILD_PRELOAD_PARITY=M23R_AND_M23T_DIFFER")
print("M23V_PRELOAD_DISPATCH_AUDIT=PASS")
print("M23V_DEVICE_ACCESS=NONE")
print("M23V_PRODUCTION_ACCEPTANCE=BLOCKED")
print("M23V_PRELOAD_DISPATCH_AUDIT_END")
PY
```

**Gate D interpretation:** PASS would validate limited
lexical/AST source invariants only, not Python launch execution
order, effective `LD_PRELOAD` values in the child, installed
libraries, actual OpenCV workers or a faulting instruction.

**No synthetic A/B, ROS process or container is authorized.**
A new plugin-allowlist A/B could alter which transport DSOs
are loaded and would require its own controlled, operator-run
protocol, reproducing original crash baseline and approval.

## Restrictions and next possible investigation

This is a **source-only** finding; all root-cause claims are
withheld. Do **not** change the installed rectifier, image-transport
allowlist, class_loader/unload behavior, allocator or OpenCV thread
configuration yet. A future diagnostic variable such as output
`enable_pub_plugins` would be a **new runtime A/B** that could
itself change loader activity, and requires a separately
authorized non-device test protocol with a reproducing baseline.
It is **not authorized by this review**.

Keep M23V and M23U and all stacked parents **draft/unmerged**.
The original M23A strict **>250 ms physical image-header-gap**
gate, camera, serial/STM32/IMU, VIO, recorder, Nav2,
motor/robot motion and every production acceptance remain
**BLOCKED**. Private logs, manifest and historical core dumps
stay with the operator; do not add them to GitHub.
