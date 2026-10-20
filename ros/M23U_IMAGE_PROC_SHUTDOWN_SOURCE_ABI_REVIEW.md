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

## Final disposition and exclusions

**Completed:** Source ownership and teardown-order review, pinned
source references and explicit ABI evidence requirements.

**Still UNKNOWN:** Actual installed ELF versions/build IDs,
module responsible for unmapped crash PC, specific invalid caller,
active OpenCV worker/backend, image_transport callback drain,
component unload timing, and reproducibility across multiple
no-device pairs. Do not label any one library a root cause.

**Remains BLOCKED:** nonzero-D component lifecycle, M23A original
strict >250ms real-camera gap gate, physical camera, STM32/IMU,
VIO, Nav2, recorder, robot motion and all production acceptance.

Keep M23U and all stacked parents draft/unmerged.
