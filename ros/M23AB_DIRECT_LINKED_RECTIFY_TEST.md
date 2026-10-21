# M23AB — direct-linked stock Jazzy rectifier lifetime diagnostic

**Status: source only; no operator build or runtime evidence.**
Draft branch stacked on M23AA #121 head
`4c44a9ace335709c7e648d99d243a2b57e6a88a7`.
M23AA untraced component and generated standalone both crashed SIGSEGV -11
after full 900/900 genuine nonzero-D rectification. M23Z #120 remains paused.

## Hypothesis and limitations

Stock Jazzy's `image_proc::RectifyNode` lives in `librectify.so`.
Both `component_container` and the generated `image_proc/rectify_node`
instantiate through `class_loader`. This new tiny C++ executable instead
directly calls `std::make_shared<image_proc::RectifyNode>` and
**links the exact installed** `librectify.so` via the linker.
It runs a single-threaded rclcpp executor with the same topic remappings.
No custom camera processing, no raw image changes and no copied image_proc
source. This removes explicit runtime factory/dlopen operations from the
main executable, **not** any transitive `class_loader`, pluginlib,
OpenCV, TBB or other dynamic dependencies of `librectify.so`.

M23AB inherits the exact M23AA 900-frame drained publisher, graph/QoS
audit, observer, header stamp matching, zero >250ms gap requirement,
real nonzero fabricated distortion, SIGINT and child -11 classification.
Both M23AB arms are **untraced**, and the new image adds the direct-linked
binary even when running the matched component control.

## Gates

A. Verify Git branch and **eight added paths only** against pinned M23AA
   parent; run `PYTHONDONTWRITEBYTECODE=1 python3 -B
   ros/m23ab_source_test.py` on the host, no ROS/Podman.
B. Verify immutable previously tested M23AA v1 parent image ID privately.
   Rootless disconnected Podman build with `--network=none --pull=never`
   to new uniquely named M23AB child image. Offline source tests and
   C++ compile only. Fail closed on missing image_proc header/lib,
   compiler errors, or direct NEEDED librectify check.
C. Inspect new immutable image provenance/defaults and executable;
   **no runtime until all build/image gates pass**.
D. One new isolated `M23AB_ARM=component` control with no tracer.
   Require original `SIGSEGV_STOCK_RECTIFIER_900_OF_900`,
   `child_exit=-11`, strict frame/stamp/QoS/drain checks.
   If the control is clean or any data incomplete, STOP and preserve.
E. Only after accepted D, one `M23AB_ARM=direct` run from the
   **same immutable child image ID**. The inherited `M23AA_RESULT_JSON`
   carries authoritative outcome, plus `M23AB_*` labels.

Use unique private 0600 log paths, no overwrites. Run with rootless
`--network=none --ipc=private --pid=private --read-only --cap-drop=all
--security-opt=no-new-privileges --tmpfs /tmp ... --memory=2g
--pids-limit=128`, outer timeout 120s, no devices or privilege mapping.
Do not post image IDs, PIDs, raw logs, core bytes, addresses or dumps.

If both component and direct crash, runtime class-loader creation is not
necessary for the fault. If component crashes and direct exits cleanly,
loader/factory involvement becomes a lead but single trial/timing changes
preclude declaring root cause or production fix. Neither result authorizes
real camera, DepthART, XRSLAM, IMU/STM32, recording, navigation/motion,
M23A physical acceptance, or production. All remain BLOCKED.

## Source delta scope

Eight new paths:
- `ros/m23ab_cpp/CMakeLists.txt`
- `ros/m23ab_cpp/package.xml`
- `ros/m23ab_cpp/src/direct_rectify.cpp`
- `ros/m23ab_direct_rectifier.launch.py`
- `ros/m23ab_compare.py`
- `ros/m23ab_source_test.py`
- `ros/Dockerfile.m23ab-direct-linked`
- `ros/M23AB_DIRECT_LINKED_RECTIFY_TEST.md`

Old M23AA/M23Y/M23S code, original images and logs unchanged.
No hardware, no production changes, root cause UNRESOLVED.
