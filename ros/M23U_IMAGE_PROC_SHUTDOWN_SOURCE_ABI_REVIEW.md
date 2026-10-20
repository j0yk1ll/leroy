# M23U — image_proc shutdown: source ownership and ABI evidence ledger

**Documentation-only, read-only analysis. Draft, stacked on M23T #114.**
No ROS launch, container execution, source replacement, camera, serial,
IMU, VIO, recorder or motor operations were performed to author this.
No production changes, no new runtime instrumentation and no device
authorization are supplied by this PR.

## Executive decision — source review complete; installed ABI unverified

The only controlled M23S same-image synthetic pair supplied thus far:
- calibrated `CameraInfo.D = [0,0,0,0,0]`: 900/900 matching
  rectified image stamps, max header gap **33.562ms**, clean
  `component_container` exit;
- identical publisher, startup/QoS, same image, but nonzero
  `D=[-0.08,0.01,0.001,-0.001,0]`: 900/900 matching rectified
  stamps, max header gap **33.521ms**, **child SIGSEGV (-11)**.

M23S's paired analyzer classified
`DISTORTION_BRANCH_ASSOCIATION_SUPPORTED_NOT_CAUSAL`. This is **one
pair**; it does not prove determinism, the failing module or callback.

M23T's later additional child-only OpenCV preload *default* profile,
with **the same nonzero D**, exited **cleanly**, with
`cv::getNumThreads()==16` before/after and **no call to
`cv::setNumThreads(1)`**. The original SIGSEGV therefore did **not**
reproduce after altering OpenCV initialization/preload, and M23T's
single-thread arm was correctly **stopped without execution**.
No threading fix, TBB culpability or reliable root cause was proven.

The first engineering priority is to determine what the **actual
installed runtime ELF objects and teardown ownership** establish
without perturbing ROS startup. Do **not** substitute a new
instrumented run or a rebuilt `image_proc` library for that missing
evidence.

## Source provenance — exact references reviewed (not local ELF proofs)

| Project / release source | File / pinned blob SHA | Key implementation |
|---|---|---|
| `ros-perception/image_pipeline`, `jazzy` | `image_proc/src/rectify.cpp`; `e7cbd9b1e1a0c42fff53241a0094d80988979623` | `RectifyNode` matched callback, zero-D bypass and nonzero-D conversion |
| Same branch | `image_proc/include/image_proc/rectify.hpp`; `aec7e5d2ebdf90c1f22bee0710438a227e25fc5d` | member declaration and destruction order |
| Same branch | `image_proc/CMakeLists.txt`; `fd3bc3deb067318e9abf37b347c749f7fc561b53` | `rectify` shared library / factory registration, OpenCV dependencies |
| `ros-perception/vision_opencv`, release tag `4.1.0` | `image_geometry/src/pinhole_camera_model.cpp`; `b3c92214b9123ab5b6089399a99e20994097bd6d` | persistent `Cache` maps, `initUndistortRectifyMap`, `cv::remap` |
| Same tag | `image_geometry/include/image_geometry/pinhole_camera_model.hpp`; `2bee551320e6befe5aa7ddbd6b0004ff8e8917cb` | owning `std::shared_ptr<Cache> cache_` and camera data |
| Same tag | `cv_bridge/src/cv_bridge.cpp`; `dd11c78d2f1b6e7257736d7562e2e6335cfee84d` | borrowed input view and copied ROS output message |
| `ros2/rclcpp`, `jazzy` | `rclcpp_components/src/component_container.cpp`; `b9e667b141fd052c858e434afeb1fad4ef9a49bb` | intentional executor/manager destruction order |
| Same branch | `rclcpp_components/src/component_manager.cpp`; `08e28db9d6863d9bcaee7f128996432420dd142c` | plugin loader, wrapper storage, manager destructor |
| Same branch | `rclcpp_components/include/rclcpp_components/component_manager.hpp`; `6238284278f9c93026c7bb82dbbbb80927d0d2cf` | `loaders_` declared before `node_wrappers_` |
| Same branch | `rclcpp_components/include/rclcpp_components/node_factory_template.hpp`; `988b8036bb620553a5bbd0d724e92bf59512c376` | creates component `std::shared_ptr<NodeT>` and type-erased wrapper |
| Same branch | `rclcpp_components/include/rclcpp_components/node_instance_wrapper.hpp`; `2705e4e8b3fd740e1c64bb65cedebea57b9ecf6a` | wrapper retains node pointer and bound getter |

Sources:
- https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/src/rectify.cpp
- https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/include/image_proc/rectify.hpp
- https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/CMakeLists.txt
- https://github.com/ros-perception/vision_opencv/blob/4.1.0/image_geometry/src/pinhole_camera_model.cpp
- https://github.com/ros-perception/vision_opencv/blob/4.1.0/image_geometry/include/image_geometry/pinhole_camera_model.hpp
- https://github.com/ros-perception/vision_opencv/blob/4.1.0/cv_bridge/src/cv_bridge.cpp
- https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_components/src/component_container.cpp
- https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_components/src/component_manager.cpp
- https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_components/include/rclcpp_components/component_manager.hpp
- https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_components/include/rclcpp_components/node_factory_template.hpp

The **upstream source branch/tag is NOT established as a byte-for-byte
match with operator-installed binaries**. In particular,
`vision_opencv` has no `jazzy` branch under that repository; tag
`4.1.0` was used for source review, not asserted as installed
runtime provenance. `image_proc/package.xml` on the reviewed
`jazzy` ref reports version `5.0.13`, not an operator-reported
binary version. The upstream source blob SHAs are git blob identities,
**not** ELF build IDs, SONAMEs, package hashes or binary ABIs.

## Source-backed callback/ownership timeline

1. **Constructor (both distortion arms):** `RectifyNode` registers
   `pub_options.event_callbacks.matched_callback = [this](...)`.
   Depending on subscriber count, this closure calls
   `sub_camera_.shutdown()` or allocates the
   `image_transport::create_camera_subscription(...,`
   `std::bind(&RectifyNode::imageCb, this, ...), ...)`.
   Both callbacks capture a **raw `this`**; safety depends on
   rclcpp/image_transport callback disconnection and node destruction
   sequencing, not on the lambda retaining `shared_ptr<RectifyNode>`.
   **No dangling callback is demonstrated.**

2. **Valid K, all-zero D:** `pub_rect_.publish(image_msg)` then returns
   without `model_.fromCameraInfo`, `cv_bridge`,
   `model_.rectifyImage`, map creation, or new output conversion.
   Real `image_rect` delivery is still required and was observed.

3. **Valid K, nonzero D:** `model_.fromCameraInfo(info_msg)` changes
   `model_` persistent state. The `image_geometry` camera model
   owns a `std::shared_ptr<Cache>`. That cache holds `cv::Mat`
   full/reduced remapping matrices and a map-dirty state.
   The `CALIBRATED` branch of `rectifyImage` calls
   `initRectificationMaps()` (including OpenCV
   `cv::initUndistortRectifyMap(..., CV_16SC2, ...)` on first/dirty
   calibration) followed by `cv::remap` into `rect`.
   The maps can survive individual image callbacks and are released
   when the camera-model cache is destroyed. The OpenCV threading
   backend and worker count for these calls remain **unverified**.

4. **Input image ownership:** `cv_bridge::toCvShare(image_msg)->image`
   creates a shallow `cv::Mat` view onto the ROS message's data, but
   the `imageCb` argument itself is a
   `sensor_msgs::msg::Image::ConstSharedPtr` retained for the
   callback's duration. The temporary `CvImage` object may be
   destroyed before the end of the callback, but the original
   `image_msg` still owns its bytes while the synchronous remap
   executes. Source does not reveal an immediate input-buffer
   use-after-free in this specific expression.

5. **Output image ownership:** `cv_bridge::CvImage(..., rect)`
   converts via `toImageMsg()` which allocates
   `std::make_shared<sensor_msgs::msg::Image>()`,
   resizes the message's `data` and **copies** rows/bytes from
   `cv::Mat`. The published output's byte storage is independently
   owned by the new ROS message in the reviewed implementation;
   this is **not** a zero-copy pointer to a destroyed local
   `rect`. The nonzero branch nevertheless adds allocation,
   copy/deallocation and publication lifetime work absent in the
   zero-D branch.

6. **Member teardown:** in `rectify.hpp`, the declaration order is
   `sub_camera_`, `queue_size_`, `interpolation_`,
   `image_topic_`, `pub_rect_`, `model_`. C++ destroys members
   in reverse order, thus the camera model (and its cached maps)
   is destroyed **before** `pub_rect_` and `sub_camera_`.
   Source alone cannot establish whether pending matched events or
   callbacks execute during teardown in the deployed binary.

7. **Container teardown:** Jazzy
   `rclcpp_components/src/component_container.cpp` declares
   `ComponentManager` before `SingleThreadedExecutor` with
   an explicit comment to **destroy the executor first** so that
   it cannot release component entities after their libraries
   unload. In `component_manager.hpp` `loaders_` is declared
   **before** `node_wrappers_`, so wrappers normally destruct
   before class loaders under ordinary reverse-member destruction.
   The manager's own destructor removes component nodes if its weak
   executor pointer still locks; it does not in itself prove all
   outstanding DDS events, callback groups, loader registrations
   and background workers are drained. A simple unconditional
   loader-before-node bug is **not** supported by this ordering.

## Ranked hypotheses and direct falsifiers

These are **engineering inferences**, not demonstrated root causes.

| Priority | Candidate mechanism | Why it fits M23S and M23T | What would disprove/narrow it |
|---|---|---|---|
| 1 | OpenCV camera-model cache / allocated remap maps / deferred allocator teardown | Cache+maps exist only after nonzero-D branch; crash happens at shutdown, no delivery loss; additional M23T OpenCV preload changes initialization conditions | Installed ELF audit reveals no relevant OpenCV map/allocator path; or controlled teardown shows map release completes with no stale callback |
| 2 | Component-node/image_transport callback lifetime / publisher matched events | Constructor stores `[this]` callback, dynamic camera subscription `std::bind(..., this)`; shutdown event interactions not proven drained; processing changes timing/allocation | Source/trace proves all such callbacks stopped and registration handles destroyed before component lifetime ends |
| 3 | OpenCV runtime worker/allocator/library lifetime | `cv::remap` introduces a code path that may use parallel backends; M23R trace saw `libtbbmalloc.so.2` close | Installed `readelf` plus runtime mapping shows unrelated allocator/backend or clean worker lifetime; no asserted backend today |
| 4 | Generic rclcpp `component_container` library unload order | Unmapped SIGSEGV PC and loader closes superficially match a late callback into unloaded code | Source currently **protects** ordinary manager/loader order; require direct evidence of a retained callback or unloaded code mapping before promoting this |
| 5 | Version/ABI collision (mixed OpenCV core, `image_geometry` headers, component .so paths) | Upstream source/tag can differ from actual installed packages; extra M23T preload changes dependency initialization | Verify exact ELF build IDs, resolved SONAMEs, package versions, dependency chains and symbol versions inside the **original M23S runtime image** |

**Do not claim** `libtbbmalloc.so.2` caused the crash because it
was the last successfully traced close, or that the unmapped first PC
`0xf5b` was positively identified as a now-unloaded shared object.
The existing M23Q core findings cannot reconstruct the missing module
with the currently available mapping evidence.

## ABI ledger — operator evidence MISSING, no guessing

Source-level logic above does **not** supply the deployed binary's
ABI. The next defensible task is an explicitly approved **read-only
static-ELF inventory** from the original **M23S** image (the one
that reproduced SIGSEGV), using tools already present there;
secondarily compare the M23T clean-default overlay.

- M23S diagnostic image ID observed in operator report:
  `01a0334afd19fea39cc4b8a1dc457e65452d6b0d73d1b959ce654904e29e78c2`.
- M23T image ID observed in operator report:
  `b849d4336faf203d2c955197af6fa7d1e9b0d5764d9e1e1a3e9d22b6d25d9254`.

For each *actual resolved ELF* (not an invented path), capture
`sha256sum`, ELF Build ID from `readelf -n`, ELF class/machine,
SONAME/NEEDED/RUNPATH/RPATH from `readelf -d`, and versioned-symbol
requirements from `readelf -V`. Capture runtime package versions
through installed package metadata (where available), **not** just
public branch names.

Relevant targets:
- `component_container` executable, `librclcpp_components.so`,
  `librectify.so`;
- `libimage_geometry.so`, `libcv_bridge.so`,
  `libimage_transport.so`, `librclcpp.so`;
- actual resolved `libopencv_core`, `libopencv_imgproc`,
  TBB runtime and `libtbbmalloc` **if present**;
- original M23R `libm23r_loader_trace.so`; for M23T **also**
  `libm23t_opencv_thread_probe.so`.

Compare source/target build IDs and library dependencies between
the M23S and M23T images **before** attributing the change to
OpenCV initialization. A new extra preload/library is a *documented
difference*, not automatically an ABI bug.

At minimum the operator should check:
1. Does the original M23S `librectify.so` resolve the **same**
   `libimage_geometry` and OpenCV SONAME major versions as M23T?
2. Do both images contain the **same build IDs and SHA256s** for
   the `image_proc`, `image_geometry`, `cv_bridge`,
   `rclcpp_components`, OpenCV and TBB objects?
3. Is the M23T helper adding a new ELF `DT_NEEDED` dependency
   (`libopencv_core`) and thus influencing **earlier module loading**
   even for its `default` arm?
4. Are there duplicate OpenCV major versions or inconsistent
   RUNPATH/RPATHs in the same rootfs?
5. Is there evidence of an outstanding callback/worker at unload?
   **Static `readelf` cannot answer this**; it would require
   a separately authorized runtime or retained private core.

**No actual ABI ledger was collected in this PR.** Avoid pretending
that a C++ helper's successful compilation proved runtime ABI
consistency, worker lifetime or the faulting function.

## Operator result — M23S original image static ELF inspection PASS

The operator reports `M23U_STATIC_ELF_EXIT_CODE=0` and
`M23U_STATIC_ELF_INSPECTION=PASS` for the original M23S
SIGSEGV-producing diagnostic image. These are **installed ELF Build IDs**,
not source blob IDs. The operator supplied a bounded path/Build ID
excerpt rather than complete SHA256, dynamic dependency or package
metadata; do not assume those other fields have been verified.

| Reported installed ELF path | Build ID |
|---|---|
| `/opt/ros/jazzy/lib/rclcpp_components/component_container` | `6a4e4dd2da8de60a224da230df34f681cc74f32e` |
| `/opt/m23r/libm23r_loader_trace.so` | `1e6a49d2f5700b44feed8d7113e4a0cc0ec1f2db` |
| `/opt/ros/jazzy/lib/librclcpp.so` | `43ab5428180ef658f107081f8e587215bcbac94c` |
| `/opt/ros/jazzy/lib/libcv_bridge.so` | `8bd0a35752c5f8085aee5c570514c69469bc7a05` |
| `/opt/ros/jazzy/lib/librectify.so` | `48db591c6c6eda0a16ec95699e8a34a234c08486` |
| `/opt/ros/jazzy/lib/libimage_geometry.so` | `937627421ba46ad2904b602b80d9dca3063f1234` |
| `/opt/ros/jazzy/lib/libimage_transport.so` | `77b3b18786f84e343e941343f925cf124cd79674` |
| `/usr/lib/x86_64-linux-gnu/libopencv_core.so.4.6.0` | `674ef9a47fda170b288a4d05597250ce5b2192bb` |
| `/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.6.0` | `117ae1589074b59cdafa86ec7d99d21e38cdfd4b` |
| `/usr/lib/x86_64-linux-gnu/libtbb.so.12.11` | `91dd0c49c55c7dd6f455dc466ded540122adb4c3` |
| `/usr/lib/x86_64-linux-gnu/libtbbmalloc.so.2.11` | `282636301e99d2b15d58cc8d9929b3e0ede8c363` |

The operator also reported aliases for
`libopencv_core.so`/`.406`, `libopencv_imgproc.so`/`.406`,
`libtbb.so`/`.12`, `libtbbmalloc.so`/`.2`, and
`libimage_transport.so` under the alternate ROS lib directory, each
with the **same respective Build ID** as its canonical path. This
is consistent with normal versioned shared-library symlinks; the
reported aliases **do not** establish an ABI collision.
The excerpt contains no `librclcpp_components.so` entry; absence
from this bounded listing does not establish absence from the image.

**Supported:** OpenCV 4.6.0 binary objects and TBB 12 runtime /
TBB malloc are present in the original image, with concrete Build IDs.
The presence of `libtbbmalloc.so.2` agrees with the prior M23R
last-close library name, but **does not establish that TBB caused the
crash**. A static ELF Build ID by itself does not demonstrate that
a library was mapped at the faulting RIP, or that an ABI mismatch
exists.

**Next evidence required:** Obtain the **same limited ELF Build ID
inventory from the already-existing clean-default M23T diagnostic
image**, compare matching resolved library paths/Build IDs (including
the added M23T OpenCV helper), and inspect only targeted `DT_NEEDED`
dependencies before inferring an initialization difference. Do not
run ROS, rebuild either image or change the preload configuration
during this static comparison. No M23T ELF inventory is in the
operator report yet.

## Gate A — source/document integrity (no execution necessary)

This PR changes exactly **one Markdown file** and **zero runtime
sources, Dockerfiles, tests or calibration fixtures**.
Review the diff and pinned-source URLs only. Upstream source was
retrieved for analysis; no changes were applied to upstream repos.

## Gate B — optional operator-only static ELF evidence (NOT authorized by PR)

If the operator later authorizes read-only container execution,
inspect the *exact immutable original M23S image first*. Use
`readelf`/`sha256sum`/`dpkg-query` on locally installed files;
no `ldd` (which can invoke a loader), `dlopen`, ROS nodes,
USB mappings, network, debug cores, or source rebuilds. Do not
rebuild the M23S image or rely on mutable tags after obtaining the
immutable image ID. Redirect output only to a protected local
non-overwriting log under `$HOME/.local/state/leroy-m23u`.

Potential command template — **operator must independently approve
before running; this is not execution authorization**:

```bash
umask 077
DIR="$HOME/.local/state/leroy-m23u"
mkdir -p "$DIR"
chmod 700 "$DIR"
IMAGE=localhost/leroy-sensors:m23s-rectification-ab
EXPECTED=01a0334afd19fea39cc4b8a1dc457e65452d6b0d73d1b959ce654904e29e78c2
ID="$(podman image inspect "$IMAGE" --format '{{.Id}}')" || exit 2
test "${ID#sha256:}" = "$EXPECTED" || {
  echo "M23U_BLOCKED=M23S_IMAGE_ID_CHANGED"; exit 2;
}
LOG="$DIR/m23s-elf-01.log"
test ! -e "$LOG" || { echo "M23U_BLOCKED=EXISTING_LOG"; exit 2; }

# ONLY after explicit authorization: execute readelf/hash/package-query
# tools inside the immutable rootfs; DO NOT start ROS.
podman run --rm --pull=never \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --entrypoint /bin/bash \
  "$ID" -lc '
    set -euo pipefail
    echo M23U_DEVICE_ACCESS=NONE
    dpkg-query -W -f="\${Package}=\${Version}\n"       "ros-jazzy-image-proc" "ros-jazzy-image-geometry"       "ros-jazzy-cv-bridge" "ros-jazzy-rclcpp-components"       "libopencv-core*" "libopencv-imgproc*" "libtbb*" || true
    for dir in /opt/ros/jazzy/lib /usr/lib/x86_64-linux-gnu; do
      test -d "$dir" || continue
      find "$dir" -maxdepth 3 \( -type f -o -type l \) \
        \( -name "librectify.so*" -o -name "libimage_geometry.so*" \
        -o -name "libcv_bridge.so*" \
        -o -name "librclcpp_components.so*" \
        -o -name "libimage_transport.so*" \
        -o -name "libopencv_core.so*" \
        -o -name "libopencv_imgproc.so*" \
        -o -name "libtbb.so*" -o -name "libtbbmalloc.so*" \) \
        -print
    done | sort -u | while IFS= read -r path; do
      resolved="$(readlink -f "$path")"
      test -f "$resolved" || continue
      echo "M23U_ELF_PATH=$path"
      echo "M23U_ELF_RESOLVED=$resolved"
      sha256sum "$resolved"
      readelf -h "$resolved" | grep -E "Class:|Machine:" || true
      readelf -n "$resolved" | grep -m1 "Build ID:" || true
      readelf -d "$resolved" | grep -E "SONAME|NEEDED|RPATH|RUNPATH" || true
      readelf -V "$resolved" | grep -E "Name: (GLIBC|GLIBCXX|CXXABI|TBB)" | sort -u | head -30 || true
    done
  ' >"$LOG" 2>&1
```

If the installed `readelf` or package metadata is missing,
**record the missing prerequisite and STOP**; do not use a network
download or change production. This static listing does **not**
fully resolve ELF dependencies to the exact mapped runtime objects:
that requires loader path/config analysis or a separately
authorized mapping snapshot. Redact sensitive host paths, serials
and any runtime address evidence before posting a bounded
summary. No raw cores or full logs go into the GitHub PR.

## Operator M23S static comparison retry — successful (2026-10-20)

The operator confirmed the assistant-supplied **first follow-up comparison**
script failed before producing an ELF inventory:
```text
M23U_DEVICE_ACCESS=NONE
/bin/bash: line 41: $3: unbound variable
M23U_m23s_EXIT_CODE=1
```
The failure was a **shell-quoting bug** in nested double-quoted
`awk` expressions (unescaped `$3` under `set -u`), **not**
a ROS/component or ELF failure. The existing failed log is preserved.

With a corrected static-only command that removed the fragile nested
`awk` expressions, the operator then reported:
```text
M23U_m23s_RETRY_EXIT_CODE=0
M23U_DEVICE_ACCESS=NONE
M23U_STATIC_COMPARE_INSPECTION=PASS
```
This completes the **original-image ELF comparison-log collection**
to a private, non-overwritten `m23s-abi-compare-02.log`.
The complete SHA256/DT_NEEDED records were **not pasted**; only the
success markers were supplied. This does not establish the M23T
library match and is not a fix or additional crash reproducer.

**Next evidence gate:** separately authorized static-only inventory
of the **M23T clean-default image**, immutable ID
`b849d4336faf203d2c955197af6fa7d1e9b0d5764d9e1e1a3e9d22b6d25d9254`,
including its child-only OpenCV probe, and **local offline**
comparison to the completed `m23s-abi-compare-02.log`.
Do **not** rerun the failed M23S command, overwrite either private
log, run ROS, rebuild diagnostic images or use hardware.
An identical set of inherited Build IDs would rule out a difference
in the recorded binaries, **not** prove that loader initialization
or a particular cache/callback/allocator caused or prevented SIGSEGV.

## Initial M23S–M23T diff excerpt — corrected scope

The operator's first static-comparison report showed a unified
`diff -u` hunk `@@ -1,4 +1,5 @@`, indicating an added
M23T helper alongside four unchanged **context lines**. That
hunk was temporarily misread as a complete 4-versus-5 record
inventory. **The later host-only audit below supersedes that
interpretation:** there are 13 original M23S records and 14 M23T
records, including all required library families, and every one
of the **13 common ELF records** matches in Build ID and SHA256.

The sole added helper is
`/opt/m23t/libm23t_opencv_thread_probe.so`, ELF Build ID
`7bcab42364099c13c1c5be0ce44f6914a54828db`, SHA256
`774b852fd9af03496a9c3e2e7a45fc9079a3b5e6b0ee5dcac44b31d6e4c0b89a`,
with direct `DT_NEEDED` entries for
`libopencv_core.so.406` and `libc.so.6`.
This additional dependency does **not** independently prove
OpenCV initialization or loader ordering caused the clean exit.

## Operator complete host-only audit — 13/13 common ELF objects match

The operator subsequently ran a **host-only** inventory and captured
loader-trace comparison from four pre-existing protected logs. The
final summary printed `M23U_HOST_ONLY_AUDIT=PASS` and
`M23U_ELF_COVERAGE_BEGIN/END`, demonstrating why the prior
`@@ -1,4 +1,5 @@` diff had been misleading: that diff hunk
displayed only the **four context records immediately around a
single added file**, not the entire 13/14-line files. Do **not**
infer total record counts from unified diff hunk context.

The actual counts reported by the host-only parser were:
- `M23S_ELF_COUNT=13`, `M23S_MISSING=NONE`;
- `M23T_ELF_COUNT=14`, `M23T_MISSING=NONE`;
- `M23U_COMMON_ELF_COUNT=13`;
- `M23U_COMMON_HASH_MISMATCHES=NONE`.

**Corrected conclusion:** all **13 common resolved ELF paths**
in the two saved comparison inventories have matching **Build IDs
and SHA256s**, including `component_container`,
`librectify.so`, `librclcpp.so`, `libimage_transport.so`,
`libimage_geometry.so`, `libcv_bridge.so`,
`libopencv_core.so.4.6.0`, `libopencv_imgproc.so.4.6.0`,
`libtbb.so.12.11`, `libtbbmalloc.so.2.11`,
`libclass_loader.so`, the existing M23R loader tracer and
a second resolved `libimage_transport.so` location.
All required path families were present. The **single recorded
additional ELF** is the M23T OpenCV helper with direct
`DT_NEEDED libopencv_core.so.406` and `libc.so.6`.

This establishes byte-identity of the **recorded** objects across
M23S/M23T, not runtime equivalence of load order, constructor state,
thread scheduling, callbacks, untracked DSOs or an absence of races.

### Existing ROS child loader traces — no new runtime activity

| Event | M23S nonzero-D SIGSEGV | M23T nonzero-D clean |
|---|---:|---:|
| `CONSTRUCTOR` | 1 | 1 |
| `DESTRUCTOR` | **0** | **1** |
| `DLOPEN_RETURN` | 41 | 36 |
| `DLCLOSE_BEGIN` | 9 | 13 |
| `DLCLOSE_RETURN` | 9 | 13 |
| Unreturned observed close calls | 0 | 0 |
| Last traced stage | `DLCLOSE_RETURN` | `DLCLOSE_RETURN` |
| Last traced module | `libtbbmalloc.so.2` | `OTHER` |
| Captured trace lines | 60 | 64 |

The operator's last twelve captured events from both traces include
a `libtbbmalloc.so.2` close. In M23S it was the last **recorded**
loader event; in M23T it preceded additional traced close returns.
M23S exited with component `SIGSEGV -11` and did **not** emit
the tracer `DESTRUCTOR` marker, whereas M23T exited cleanly and
emitted the marker. These are **temporal correlations only**.
Even a `DLCLOSE_RETURN` does **not** establish that the named
library was fully unloaded or was responsible for the invalid RIP.
Likewise a missing tracer destructor simply establishes that the
tracer never reached/logged that phase, not the exact faulting caller.

**Remaining uncertainties:** whether a late matched callback,
`PinholeCameraModel` map-cache destructor, allocator/worker,
other library finalizer or race caused an invalid control transfer;
the historical M23Q core's unmapped target remains unsymbolized.
The M23T helper itself can change OpenCV dependency loading,
initialization and scheduling despite not calling
`cv::setNumThreads(1)` in the default arm.

**Next safe investigation:** compare individual *existing* close
events and thread ordering from the two private M23R captured
windows, with sanitized module names/TID labels and relative
timestamps; do **not** run new ROS profiles or containers. The
tracer's last 80 events may not contain every earlier `dlopen`:
only the captured windows can be compared directly. No fix or
production acceptance is established.

## Operator per-thread loader close timeline — 60 versus 64 verified events

The operator's **host-only** timeline parser verified the complete
available M23R captured-event windows and reported
`M23U_TEARDOWN_TIMELINE=PASS`, with **60** M23S captured events and
**64** M23T captured events. The parser identified two distinct
traced threads in each run. Thread identifiers below are
**per-log pseudonyms**, not matching OS TIDs across runs.

| Existing trace event | M23S nonzero-D SIGSEGV | M23T nonzero-D clean |
|---|---|---|
| `librectify.so` close | T1 begin 40654.994ms, return 40655.508ms | T1 begin 40635.074ms, return 40635.559ms |
| TBB allocator close | T2 begin **40662.362ms**, return **40669.221ms** | T2 begin **40751.026ms**, return **40751.041ms** |
| TBB close wall duration | **6.859ms** | **0.015ms** |
| Other thread's close calls during TBB close | **Observed on T1** | **Not observed** |
| M23R tracer's `DESTRUCTOR` marker | **Absent** | T1 at **40750.585ms**, **before** T2's TBB close |
| Last traced event | T2 TBB allocator `DLCLOSE_RETURN` | T1 other `DLCLOSE_RETURN` |

In the M23S crashing run, a T1 `OTHER` close began at
**40658.761ms** and did not return until **40668.983ms**; this
overlapped the T2 TBB allocator close for roughly **6.621ms**.
T1 also recorded further nested/overlapping close begin/return
events during this period. The T2 TBB close's successful return
at 40669.221ms was the final captured loader event; the tracer
did **not** emit its destructor marker, and the actual child
subsequently exited SIGSEGV (-11).

In the M23T clean run, the T2 TBB allocator close lasted
approximately **0.015ms**, started **0.441ms after** the T1
tracer-destructor marker, and did not overlap the shown T1
close calls. The clean process emitted further T1 close-return
events afterwards.

**Interpretation:** same inspected 13 ELF binaries, but materially
different interleaving and elapsed time in the captured thread-local
loader close calls. This elevates a **concurrent shutdown/loader-
lifecycle race** to a testable engineering hypothesis; it does not
prove `libtbbmalloc`, `dlclose`, OpenCV, or any particular
object destructor causes an invalid jump. Both traces include a
successful TBB allocator `dlclose` return. The `DESTRUCTOR`
marker is the **M23R loader-interposer's destructor**, not proof
that the ROS RectifyNode destructor ran at that time. The trace
records only instrumented `dlopen`/`dlclose` and does not track
the complete glibc loader state, dynamic references or all process
threads. It also cannot associate the old M23Q unmapped RIP with
a specific ELF object.

**Next permitted read-only step:** locally pair
`DLCLOSE_BEGIN`/`DLCLOSE_RETURN` by *same thread and handle*
from the already retained M23S and M23T logs, verify the overlap
duration and whether either observed call was unfinished.
Print only elapsed time and sanitized library-category names.
No ROS or container execution and no new preload/helper.

## Final disposition and exclusions

**Completed:** Source ownership and teardown-order review, pinned
source references and explicit ABI evidence requirements.

**Still UNKNOWN:** Runtime loader ordering and initialization,
untracked runtime modules or independent ABI effects, the module at
the unmapped crash PC, specific invalid caller, actual OpenCV worker/
backend lifetime, image_transport callback drain, component teardown
timing, and reproducibility across multiple no-device pairs. The 13
common inspected ELF files now match in SHA256 and Build ID; this
does not prove they all executed identical paths. Do not label any
library a root cause.

**Remains BLOCKED:** nonzero-D component lifecycle, M23A original
strict >250ms real-camera gap gate, physical camera, STM32/IMU,
VIO, Nav2, recorder, robot motion and all production acceptance.

Keep M23U and all stacked parents draft/unmerged.
