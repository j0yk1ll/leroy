# M23AA — stock Jazzy standalone versus composable rectifier

This draft is stacked directly on M23Y PR #119, source head
`c41d6d38fb7432c032b16d8f69dd31802c5a4089`.
M23Z PR #120 remains paused and unchanged.

## Question

Does the unchanged stock Jazzy `rectify_node` executable avoid the
SIGSEGV that repeatedly occurs in `component_container` shutdown?
This compares process/manager lifetime, not rectification algorithms.
Both new launches use the same image_proc package, topic remappings,
node name, QoS environment, synthetic 900-frame M23Y drained publisher,
real nonzero-D rectification, M23P C++ observer and ROS graph verification.
Neither arm preloads the M23R tracer or M23T OpenCV helper. Existing
M23Y A1/A2 have already reproduced a 900/900 component SIGSEGV under
tracing; they are contextual historical evidence, not an untraced
matched control for this test.

## Gates

A: Review the exact six added files against the pinned M23Y parent;
execute `PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23aa_source_test.py`
from `ros/`, with no ROS, container, devices or hardware.

B: Verify the immutable M23Y v2 parent image locally. Build a NEW
uniquely tagged `localhost/leroy-sensors:m23aa-standalone-v1`
via `podman build --network=none --pull=never` using this new
Dockerfile and a clean pinned source tree. This only runs offline
tests and checks existing executables, no ROS. Do not replace or
rebuild M23Y or M23Z images. If build fails, STOP.

C: Inspect new image metadata/ID, unchanged parent, synthetic-only
defaults, existing `rectify_node` executable, and no LD_PRELOAD.
No ROS runtime yet.

D: Separately operator-approved device-free test: first one
`M23AA_ARM=component` control and inspect all source count,
graph/QoS, drain, child-exit metrics. Only if genuine 900/900
rectification and explicit component SIGSEGV -11 is observed,
consider one standalone arm from the SAME immutable M23AA image.
Use fresh owner-only logs and distinct disposable containers.
No device mapping, network, host IPC/PID namespace, privileged
container, IMU, STM32, recorder, VIO/Nav2 or motor nodes.
Exact standalone exit0 with complete 900/900 evidence is
`CLEAN_STOCK_RECTIFIER_900_OF_900`; SIGSEGV exit2 with
complete evidence is `SIGSEGV_STOCK_RECTIFIER_900_OF_900`.
Any other outcome BLOCKED. One clean arm alone does not prove
reliable production shutdown. Further repetitions require separate
review and defined stop rules.

The inner container launcher uses SIGINT and the inherited M23P
bounded subprocess teardown. The outer command must use a finite
120-second timeout, resource limits and private log. No raw core,
PID, memory address or device data should be posted publicly.
Original physical M23A image-gap, DepthART/XRSLAM, production and
robot motion acceptance remain BLOCKED.
