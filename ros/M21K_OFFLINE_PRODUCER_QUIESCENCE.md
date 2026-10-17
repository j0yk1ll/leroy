# M21K — Offline producer quiescence before bounded recorder persistence

## Prerequisites

M21I full-window synthetic replay passed (11,661 records, exit 0).
M21J offline admission/finalization tests passed 10/10 on Arch with
M21J_OFFLINE_EXIT_CODE=0. Its capacity plan admits 120-second
30/30/50 Hz camera/image-info/IMU recording to a 20,000-event hard cap
after a 20% reserve. The estimated byte footprint is only a HEURISTIC.

GitHub currently contains M21J on feature/m21j-offline-capacity-stop-order
but it is not merged into main. M21K was created FROM the M21J feature
branch. It includes M21J as a prerequisite rather than assuming main
already contains that code.

## Identified integration risk

MetadataCapture.finish() serializes and closes the bounded capture.
A late ROS image callback, CameraInfo callback, or M18 IMU hook would
attempt to append after closure. M21J checks that an injected STOP
callback is attempted before file persistence, but did not verify
that concurrent producers are quiescent.

M21K adds finish_after_quiescence(...):

1. Validate only synthetic, injected callbacks and registered threads
   without touching hardware
2. Call the supplied STOP callback FIRST; record actual bool ACK or error
3. Call the supplied producer-shutdown callback even if STOP fails
4. Join all REGISTERED worker threads with a shared bounded deadline
5. If any worker is still alive or never started, do NOT write a file
6. If producers have stopped, attempt exclusive no-overwrite M21C JSONL
   serialization using the existing MetadataCapture.finish()
7. Only accept when STOP ACK was bool True, shutdown callback did not
   error, all registered threads stopped, and metadata file saved

The STOP callback here is always FAKE. The module cannot open serial,
start ROS or Podman, and does not create camera/IMU hardware producers.
Even when STOP ACK is unverified it may write a *diagnostic* file once
writers have stopped, but it NEVER accepts the lifecycle as successful.

IMPORTANT: The registered-thread check cannot prove that unregistered
threads, ROS executors, or background callbacks are drained. A later
real integration must enumerate and shut down ALL producers and block
any restart before finalization. Forced process termination, USB loss
or unverified STM32 STOP ACK cannot be solved by this helper.

## Pure-Python regressions

Nine tests cover: correctly ordered STOP/request/join/save, missing
STOP ACK with forensic capture and rejection, transport failure followed
by shutdown request, stalled producer timeout refusing file write,
shutdown callback failure, a never-started worker, destination
no-overwrite failure, invalid callbacks or worker registrations rejected
before any STOP callback, and closed-recorder rejection.

## Run in existing Arch repository checkout — no hardware

The user's checkout previously showed two local file deletions:
ros/CAMERA_TIMESTAMP_FIX.md and utils/src/utils/__init__.py.
Do not restore, stage, reset, or discard those local changes on account
of these tests. The commands print git status and switch branches only;
Git may refuse a switch if local changes would be overwritten.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m21k-offline-producer-quiescence
  git pull --ff-only origin feature/m21k-offline-producer-quiescence
  cd ros
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
    -s . -p m21k_offline_quiescence_test.py -v
  PYTHONDONTWRITEBYTECODE=1 python3 m21k_offline_quiescence.py
) 2>&1 | tee m21k_offline_quiescence.log
M21K_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M21K_OFFLINE_EXIT_CODE=$M21K_OFFLINE_EXIT"
```

Expected: 9 tests, OK; M21K_LIFECYCLE_REPORT_JSON with
evidence_type SYNTHETIC and saved_event_count 9;
M21K_VERDICT=OFFLINE_QUIESCENCE_CONTRACT_READY;
M21K_OFFLINE_EXIT_CODE=0.

## Scope and safety

No physical sensor capture is authorized. The production leroy-ros
container remains exited, and no motor bridge, /cmd_vel, odometry,
TF, XRSLAM, Nav2 or DepthART is started. Before ANY later operator-
approved MCU serial access, isolate power to BOTH wheel and
manipulator motors, verify hardware E-stop, immobilize robot,
confirm exact current MCU/camera identities and exclusive ownership,
and use only an isolated least-privilege device-limited container.
Missing physical STOP ACK means MCU state unknown and motor power
must remain physically inhibited. Physical camera–IMU offset and
spatial calibration remain unvalidated.
