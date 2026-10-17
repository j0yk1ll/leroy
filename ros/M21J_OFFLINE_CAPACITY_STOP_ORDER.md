# M21J — Offline recorder capacity and STOP-before-persistence review

## Existing accepted evidence

M20 observed actual concurrent camera/IMU ROS messages, not calibrated
physical sensor synchronization. M21I subsequently passed 7/7 tests on
Arch for an entirely SYNTHETIC 120-second-scale metadata dataset:
2082 rectified image records, 3578 CameraInfo records, 6001 MCU IMU records,
11661 events total, 2,383,760 bytes, a deliberate 1.351 s camera gap,
119.886 s simulated receipt overlap, no sequence faults, exit code 0.
PR #74 is verified merged at ace421367f53550438f04af2929a22c90537fe17.

## Capacity issue discovered

M21I intentionally used a max_events=12000 budget, leaving only 339 slots
above its synthetic 11661 events. That is NOT sufficient for all plausible
120-second sensor delivery cadences. At intended 30 Hz camera image,
30 Hz CameraInfo and 50 Hz MCU IMU, including a possible sample at each
window endpoint, we budget 3601+3601+6001=13203 events. With a 20% safety
reserve, ceil(13203*1.2)=15844 slots are needed. This fits the existing
M21D recorder hard cap of 20000 slots, with 4156 slots reserved above
that planned requirement. Any live capture with different measured rates
must repeat admission planning with appropriate conservative rates.

Admission also uses a heuristic of 480 bytes/event plus 512 manifest bytes
against M21D's existing 8 MiB hard file-size limit. This is NOT a proven
worst-case output bound. The existing M21D.finish still enforces the ACTUAL
serialized output-byte limit and can independently reject oversize files.

## STOP-before-file-finalization

m21j_offline_capacity_stop.finalize_after_stop takes a caller-supplied
stop_request function, calls it FIRST, and only THEN calls capture.finish.
The function does not open a serial port, know any device, start ROS or
send commands by itself. In tests the callback is a fake returning
True/False or raising a synthetic exception.

An unacknowledged STOP or STOP exception is never marked as accepted.
The helper still attempts to persist otherwise valid metadata for
diagnostics, using M21D's exclusive no-overwrite file creation. The
reported lifecycle verdict is UNVERIFIED_STOP_OR_METADATA_CAPTURE, even
when its saved file separately passes the M21C format check. A data
format verdict must never imply actual MCU STOP or physical motor safety.

On the physical M20 runner, STOP is already attempted within finally.
This PR does NOT integrate the helper into the physical runner and
does NOT change any serial command, source node, graph, topic or camera.
Before that integration, the actual owning serial process must protect
STOP/ACK cleanup under all tested paths and perform file finalization
only afterward. A killed process or lost USB port defeats software
cleanup guarantees. No new physical experiment is authorized here.

## Hardware-free Arch host command

Run from the Leroy Git working tree, in an interactive Bash shell:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m21j-offline-capacity-stop-order
  git pull --ff-only origin feature/m21j-offline-capacity-stop-order
  cd ros
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
    -s . -p m21j_offline_capacity_stop_test.py -v
  PYTHONDONTWRITEBYTECODE=1 python3 m21j_offline_capacity_stop.py --demo
) 2>&1 | tee m21j_offline_capacity_stop.log
M21J_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M21J_OFFLINE_EXIT_CODE=$M21J_OFFLINE_EXIT"
```

Expected tests: Ran 10 tests, OK. Demo emits M21J_CAPACITY_REPORT_JSON
with capacity_ready=true, required_event_capacity=15844,
configured_event_capacity=20000, remaining_reserved_events=4156;
M21J_FINALIZATION_REPORT_JSON with SYNTHETIC provenance,
stop_acknowledged=true, metadata_saved=true and
verdict=SIMULATED_POST_STOP_FINALIZATION_READY;
M21J_VERDICT=OFFLINE_CAPACITY_AND_STOP_ORDER_READY;
M21J_OFFLINE_EXIT_CODE=0.

## Real hardware boundary

No test in M21J touches camera /dev/video8 or MCU /dev/ttyACM0;
no devices, ROS, Podman, controller, Nav2, XRSLAM, odom/TF, DepthART or
motors are used. The production leroy-ros must remain exited.
Before ANY future separately approved real MCU serial access,
both WHEEL and MANIPULATOR motor-power supplies must be physically
inhibited, hardware E-stop verified, robot immobilized, hardware
identities and exclusive ownership checked, isolated least-privilege
device-limited container established, and approved MCU START/STOP
ACK requirements enforced. A missing STOP ACK means the MCU is in
an unknown state and motor power must remain isolated.
DepthART CPU optimization remains deferred.
