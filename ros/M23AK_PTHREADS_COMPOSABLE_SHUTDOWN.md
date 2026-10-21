# M23AK — pthreads OpenCV with the STOCK composable rectifier

## Provenance

M23AJ exact Ubuntu OpenCV 4.6.0+dfsg-13.1ubuntu1 source and patches passed SHA-256 verification. Network-disconnected corrected build v2 succeeded, all six OpenCV 4.6 shared objects passed .so.406 SONAME and no direct TBB DT_NEEDED checks.

Accepted M23AJ image ID:
334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025.

Accepted unchanged M23AF baseline image ID:
a56f89bd8d3a69264e846319f8915efcff1fcd04a2dd3bf54d8491b89a119777.

The build has NOT yet established runtime loader selection, TBB absence in a ROS component process, or clean shutdown.

## Synthetic experiment

M23AK reuses original M23AF custom stock component container, unchanged image_proc::RectifyNode, control-only balanced RTLD_NOLOAD factory check, original M23AA+M23Y full-delivery 900/900 fabricated nonzero-distortion images, reliable QoS and graph audit, raw/rectified stamps, two-second drain and independent child exit. M23AF already captures same-process executable file maps just after factory load and at rclcpp on_shutdown.

M23AK adds only two Python scripts: m23ak_maps_gate.py and m23ak_composable_compare.py. The former requires replacement OpenCV .so.4.6.0 executable maps from /opt/m23aj-opencv/lib, no foreign OpenCV mapping and no libtbb executable mapping in BOTH phases. The latter requires the original inherited stock-control factory hook, 900/900 data contract AND an actual ROS component child exit 0.

A clean run without TBB in those two snapshots would strongly support the OpenCV threading-backend workaround, but would not prove lifetime behavior, TBB absence for all instants in time or ABI correctness for all ROS packages.

## Operator steps

1. Fetch/check out the branch, verify a clean worktree and execute 9 source-only tests in ros/m23ak_source_test.py. Do not start ROS for source tests.
2. Run exactly ONE rootless container from the saved immutable M23AJ image *ID* with network=none, private IPC/PID, read-only root, no extra devices, tmpfs /tmp with notmpcopyup, pids/memory limits and two read-only binds placing M23AK Python scripts into /opt/m23af.
3. Source ROS Jazzy, workspace, M23AC and M23AF setup files in the container, then set LD_LIBRARY_PATH to have /opt/m23aj-opencv/lib as its very first directory. This does not modify the system OpenCV files or use LD_PRELOAD.
4. Run /opt/m23af/m23ak_composable_compare.py with explicit M23AK_ARM=pthreads, M23P_PROFILE=synthetic, M23Y_DRAIN_ONLY=1, M23S_DISTORTION_PROFILE=rectify, M23P_SYNTHETIC_ONLY=1, LEROY_NAV2_DRY_RUN=1, ROS_DOMAIN_ID=170, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST and M23R_TRACE_ONLY=0. All previous component arms must be empty.
5. Use a new protected non-overwriting host log m23ak/pthreads-composable-01.log. Report the wrapper process exit and M23AK_RESULT_JSON/M23AK_BLOCKED markers; do not upload entire map arrays or invoke coredump tools.
6. Do not rerun automatically; on a missing module, loader failure, child crash or incomplete frames, preserve the evidence and stop.

No real cameras, /dev/video, serial/IMU/STM32, VIO, recording, Nav2, motion, hardware tests or production acceptance. Physical >250ms continuity gate remains BLOCKED.
