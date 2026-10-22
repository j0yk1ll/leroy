# M23BC — Disconnected observer build and synthetic host text persistence

**DRAFT: no camera, ROS graph, serial/MCU, recorder, IMU, motors, navigation or production authorization.**

M23BB operator source gate reported 32/32 PASS at commit 41b69407cf89451159b9bcb6626570f62d50aa4a. M23BA offline correlation source tests passed 28/28. Original M23AU physical trial is CLOSED and BLOCKED by a repeated raw subscriber header stamp; publisher/DDS origin remains unproven.

## Immutable foundation

- Parent: localhost/leroy-sensors:m23ay-acquisition-diagnostic-v1
- Exact parent image ID: 9ff2faf53109f88333e79c998b6946caa49f09fc807d2da422c1adcd47bda571
- M23AZ prior isolated component dlopen passed, no camera exposed.
- No original source, installed camera libraries, production launch, or pinned image may be changed.

## What the NEW source files do

- Dockerfile.m23bc-host-evidence-synthetic: creates an isolated *derived* image, builds only a copied M23K observer C++ package. Inert default CMD /bin/false.
- m23bc_offline_source_stage.py: inert default; under M23BC_OFFLINE_BUILD_ONLY=YES_NO_DEVICE creates a new copied M23K Python supervisor and observer source tree at /opt/m23bc-variant and /opt/m23bc-counter-source, preserving both exclusive-create evidence paths, original ROS process-group shutdown and camera QoS/gap gates. The copied supervisor is never executed now. Its two counter paths point to the new compiled observer in /opt/m23bc-counter-install.
- m23bc_binary_audit.py: checks copied C++ source per-sample flush, ELF64 x86-64 architecture, ldd -r dependencies and TBB exclusion, compiled C++ event strings. Never executes observer or ROS.
- m23bc_synthetic_persistence.py: inert default; writes only two very short SYNTHETIC text payloads to exact independent evidence paths from the M23BB host bind adapter. Files open in xb, are flushed and fsynced. Normal mode exits 0. Simulated abnormal mode deliberately terminates Python via os._exit(73) **after** the two fsync calls; no ROS, pixels, camera or DDS messages. Neither mode is part of this first operator source-only gate.
- m23bc_source_test.py: 19 host-only pure-Python regression tests on source transformations, path/approval guards, synthetic-only data, mocked crash preservation and strict binary metadata.

## Gate A — host-only source validation (current authorized action)

Create a NEW detached sparse checkout with exactly the commit SHA supplied in the operator reply. Select:
ros/Dockerfile.m23bc-host-evidence-synthetic
ros/m23bc_offline_source_stage.py
ros/m23bc_synthetic_persistence.py
ros/m23bc_binary_audit.py
ros/m23bc_source_test.py
ros/m23bb_host_evidence_adapter.py
ros/m23bb_evidence_source_contract.py
ros/m23k_camera_path_probe.py
ros/m23k_cpp/**

Run, without ROS, Podman or devices:

    PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bc_source_test.py
    PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bc_offline_source_stage.py
    PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bc_synthetic_persistence.py
    PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bc_binary_audit.py

Expect 19 tests PASS plus inert status markers. An unexpected failure blocks further work; do not run --stage manually on the host.

## Gate B — one network-disconnected image build (only after A PASS)

Check pinned parent ID (above), exact detached source SHA, no running rootless containers, clean checkout, unique target tag and new private immutable host log. Build only with podman build --network=none --pull=never --no-cache and Dockerfile.m23bc-host-evidence-synthetic. No --device or hardware mappings. The image build compiles the copied M23K C++ observer but NEVER instantiates a node, connects a DDS graph, or starts the camera.

Accept ONLY with build exit 0 and all markers:

    M23BC_STAGE_VERDICT=ISOLATED_SOURCE_COPY_READY
    M23BC_BINARY_AUDIT=PASS
    M23BC_OFFLINE_OBSERVER_BUILD=PASS

Record the full build log SHA256 and new exact image ID. On failure STOP; preserve the log, do not prune or rebuild the parent.

## Gate C — future isolated synthetic host-bind persistence (NOT YET RELEASED)

This future step uses two DIFFERENT owner-only 0700 host evidence directories, one for expected clean exit and one for expected deliberate exit 73. A 0600 literal sentinel file named .m23bb-host-bind-ready must be created in each with exact bytes M23BB_HOST_BIND_READY_V1 followed by a newline. The camera and counter log files must not exist.

Each case runs at most one no-network, no-device, cap-dropped, readonly, private PID/IPC guest with only its own directory bound to /evidence. Always use a /tmp tmpfs with notmpcopyup (inherited image contains large OpenCV build trees). The script verifies the M23BB exclusive mount contract, refuses V4L2, serial and USB buses, then writes only synthetic text using xb + flush + fsync. Require exact guest exit 0 or 73 by case, host files persisted even after Podman --rm, file contents equal the two synthetic reference payloads, valid owner/mode, private host hashes. If any deviation, BLOCK.

Do not execute this stage before separately reviewing the derived build and operator runtime commands. It does NOT validate ROS graph, subscriber callbacks, DDS delivery, camera frames, clean component shutdown, or M23AU root cause. Even a PASS leaves the original stamp monotonicity, >250ms continuity and production gates BLOCKED.

**Never execute ros/exec.sh or ros/robot.launch.py; both can bring up leroy_bridge and MCU serial/actuators.** No new real camera trial, recording, IMU, DepthART, XRSLAM, Nav2, motor/manipulator or production authorization.

M23BC_CAMERA_RETEST_AUTHORIZED=FALSE
M23BC_PRODUCTION_ACCEPTANCE=BLOCKED
