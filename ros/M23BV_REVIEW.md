# M23BV engineering review and signoff boundary

**Review date:** 2026-10-23

## Existing code inspected

- M20 launch uses a `ComposableNodeContainer` and the real STM32 observer; M20 starts the camera nine seconds before its IMU source and stops the IMU before the camera.
- M23H starts a `ros2 bag record` subprocess *before* starting M20, sends SIGTERM only to the M20 parent (not its process group), and sends SIGINT to the recorder process group for sqlite3 finalization.
- M23G recorder image QoS is still BEST_EFFORT, depth 20. Historical M23N/M23O diagnostics verified RELIABLE transport on camera raw and rectified paths.
- M23I PR #103 corrects the CameraInfo auditor's unbounded-cache false fault. The repository default branch M23A source inspected here lacks that correction.

## Patch decisions

1. Leave every original M20/M23H/M23G source file and all prior logs/recordings untouched. All scripts in this package have *new* names.
2. Base future derived runtime on the M23H **sensor** image and copy only alternative pthreads OpenCV libraries from the M23BT accepted image. Do not replace the M20 sensor dependency stack or modify `LD_LIBRARY_PATH` globally.
3. Use a standalone `image_proc` executable so the rectifier's library loader environment is truly process-scoped. Verify its availability and ELF dependencies on the actual derived image; the original M20 component container remains unmodified.
4. Fail closed on actual live publisher/subscriber QoS before the M20 process opens STM32 CDC. The graph inspector does not publish/subscribe payloads or interact with hardware.
5. Preserve the original M20 IMU observer, serial command allowlist and START/STOP/ACK behavior. Any inability to shut down cleanly blocks acceptance.
6. Separate candidate recording from *offline* derivation of a fully IMU-covered header-stamp window. Preserve original ROS CDR bytes and bag receipt timestamps; never pad, retimestamp or fabricate IMU measurements.
7. Preserve M23A image gap max 250 ms. The old known M23BU bag will still block on 64 internal camera gaps after any head/tail trim.

## Observed tests in this environment

- `bash -n`: PASS on additive shell scripts.
- Python AST parsing: PASS on additive Python sources.
- 26 synthetic offline unit tests: PASS.
- Guard negative paths: absent/best-effort image endpoints and unknown RMW BLOCKED.
- Window negative paths: missing IMU, missing CameraInfo, bad frame, nonmonotonic or large IMU/image gaps BLOCKED.
- Fake writer copy: serialized data and timestamps unchanged; source SHA-256 unchanged; output collision and traversal BLOCKED.
- Mock supervisor: recorder starts first, sensor parent SIGTERM occurs before recorder group SIGINT; sensor/recorder nonzero exit BLOCKED; absent operator flag BLOCKED.

## Unverified and mandatory remaining tests

No ROS Jazzy image, Podman runtime, raw device, real bag, or accepted M23BT-06 script was present in this execution sandbox. Consequently the following are NOT VERIFIED:

- Actual M23H sensor image identity, copied pthreads OpenCV loader/dependency maps, executable availability, and independence of cv_bridge system OpenCV.
- Jazzy image_proc lazy QoS behavior with the new standalone subprocess, live real endpoint QoS including the recorder endpoint, actual RELIABLE depth/history policy, and >=25 FPS synthetic DDS image transport.
- Real rosbag2_py writer/reader API, sqlite3 finalized metadata, exact real CDR byte copy and original file hashes under an offline ROS container.
- Clean process-tree shutdown when launching live, child crash/timeout behavior, and actual physical IMU START/STOP acknowledgments.
- M23I corrected M23A/M23F/M23J acceptance on a newly captured, trimmed recording.

**Release classification:** `M23BV_HARDWARE_FREE_SOURCE_DRAFT_TESTED`, not `M23BV_REAL_CAPTURE_READY`. No operator approval, new physical capture, motor controls, Nav2 obstacle outputs, metric odometry, or TF authority is included.

The operator must independently verify actuator power inhibition and robot immobilization and grant a **new** explicit authorization for any future physical test, only after the full no-device acceptance suite passes in the pinned runtime.