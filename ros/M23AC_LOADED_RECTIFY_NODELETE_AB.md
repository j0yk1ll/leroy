# M23AC — same component manager, RTLD_NODELETE after stock load

**Source-only draft. No operator build or runtime run has occurred.**
Stacked on M23AB #122. No edits to any prior source, camera launch,
ROS/Jazzy installed library, fixture, accepted image or private log.

## Motivation

M23AB accepted untraced, nonzero-D 900/900 control SIGSEGV under
`component_container`; same pinned new image's **direct-linked**
RectifyNode delivered 900/900 and exited cleanly. This suggests a
class-loader/lifetime interaction but does not demonstrate causation.

## First, repeatability of direct-linked success in existing M23AB v2

Before any new M23AC runtime, use the existing immutable
`localhost/leroy-sensors:m23ab-direct-linked-v2` image ID to perform
**two** separate bounded `M23AB_ARM=direct` runs. Each must deliver
900/900 raw, rect, CameraInfo, matched timestamps, graph/QoS, and clean
rectifier child 0; use unique protected logs, unchanged parent and no
external environment perturbations. Stop at the first non-clean, incomplete
or timed-out trial. This is a timing/race screening gate, not a statistically
reliable proof. Historical M23AB single clean and single crashing component
are preserved; no repeated physical/VIO/production validation.

## Focused hypothesis: library permanence after normal factory loading

The new `component_container` executable is the stock Jazzy
`rclcpp_components::ComponentManager` lifecycle with ONE subclass
override of `create_component_factory()`. It first calls stock factory
creation. Once the installed `librectify.so` is already loaded, both
experimental arms execute exactly one `dlopen(path, RTLD_NOLOAD | RTLD_NOW
| optional flag)` followed by `dlclose(handle)` of the **extra**
diagnostic reference. The only flag difference:

- `M23AC_ARM=control`: no additional flag.
- `M23AC_ARM=nodelete`: add `RTLD_NODELETE`.

Same C++ binary, same custom component manager and same launch source.
The child emits exactly one factory hook marker, requiring successful
`RTLD_NOLOAD` and `dlclose`. Parent performs a pre-frame audit for that
exact policy marker, then delegates unchanged to M23AA/M23Y strict 900
synthetic image+CameraInfo publisher and headers/QoS/stamp/child exit
validator. `M23AC_ARM` passes only a child environment policy. No
`LD_PRELOAD`, source edits to ROS, alternate OpenCV thread profile,
raw-only transport change or physical camera.

**Critical limits**: We test glibc `RTLD_NODELETE` applied to the
`librectify.so` ELF, NOT all transitive dependencies. Success does not
prove which library's teardown was responsible. The loader flag may
affect more than unmapping, and both arms include diagnostic
`dlopen`/`dlclose` timing effects. Only an accepted *crashing control*
makes a cleaner `nodelete` run interpretable. One successful run cannot
justify a production fix.

## Gates (stop on first failure)

A: Exact seven additive files and offline source tests (6 cases).
B: Verify pre-existing immutable M23AB v2 parent image ID locally.
    Build once as a new uniquely tagged child
    `localhost/leroy-sensors:m23ac-loaded-rectify-v1` with
    `podman build --network=none --pull=never`, protected build log.
    C++ compilation + six offline checks only; do NOT run ROS at build time.
C: Verify child distinct image ID and empty default M23AC_ARM, no preload.
D: One `M23AC_ARM=control` no-device trial in new child image.
    REQUIRE `M23AC_RESULT_JSON.factory_hook_verified_before_frames=true`,
    the inherited actual graph/QoS verification, 900 published/raw/rect
    and CameraInfo with stamp match, no >250ms gap, correct publisher
    drain, and rectifier child SIGSEGV -11, M23AA verdict
    `SIGSEGV_STOCK_RECTIFIER_900_OF_900`.
    A clean control, absent/mismatched hook, incomplete delivery or
    environment problems **stops the experiment**.
E: Only after D accepted, one `M23AC_ARM=nodelete` run from the **same
    immutable child image ID**, with exactly identical limits and new log.
    Compare independent child exit and full fixture acceptance.
    If nodelete is clean, treat as strong lead for loaded library lifetime,
    not a root cause. If it crashes, this specific NODELETE flag is
    insufficient; do not retry blindly.

Rootless containers only, never map devices or use a host network, PID or
IPC namespace. Require finite 120s outer timeout, private logs with no
overwrite, private image IDs. Stop on all unsupported outputs. M23Z remains
paused. Physical camera/M23A image-gap, DepthART/XRSLAM, STM32, motors,
navigation, recording and production acceptance remain BLOCKED.
