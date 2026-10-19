# M23I — Read-only forensics for first physical ROS 2 recording

**Status: draft implementation. No hardware access; operator testing pending.**

## Evidence from capture001 (operator's first full-frame bag)

- M20 START/STOP IMU ACK: PASS, combined stream PASS
- sqlite3 bag finalized; M23H recorder/sensor exit codes both zero
- Image 729, CameraInfo 3906, IMU 6001, overlap 119.997594 s
- Max header gap: image **2951.988 ms**, info 52.074 ms,
  IMU 20.005 ms; >250ms image gate MUST remain blocked
- Sampled image variation observed, CameraInfo projection unchanged
- SHA stable: database 676675584 bytes:
  e9055fb76e4056023424bad44fdfd4b011bd1b46614e0f2d5535d10d641adb61
- SHA stable: metadata.yaml 2819 bytes:
  6ac930a2fdfdab649c0cb1961841609a8cad5abd354e8563760fd3e60687389f
- First audit correctly returned BLOCKED_CANDIDATE_STORAGE_OR_CDR.

### Correct a separate audit software issue

M23A kept every distinct CameraInfo stamp in memory and raised
BLOCKED_UNBOUNDED_CAMERA_INFO once it saw >2048 CameraInfo messages.
This legitimate 3906-message real stream exceeds that cap even if
every Image matches perfectly. M23I switches to a 2048-item FIFO
cache of old CameraInfo stamps, **without discarding unmatched
Image obligations or changing the 250ms gap limit**. Missing image
matches, bad timestamps, incorrect dimensions, invalid CDR and
IMU coverage failures continue to block.

M23D now retains its exact nested m23a_preflight report. M23F
reports m23a_faults, m23a_verdict, matched Image/CameraInfo pairs,
CameraInfo cache evictions, and gaps_over_250ms. The original
blocked verdict stays blocked.
The 2,951.988ms image timestamp gap should appear explicitly as
`BLOCKED_IMAGE_STAMP_GAP` after the improved diagnostic propagation.

## 1. Offline regression suite — 124 tests, NO devices

- M23I: 10 new in-memory long-bag regression tests
- M23H: 20; M23G: 19; M23F: 18; M23E: 19; M23D: 17; M23A: 21

~~~bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m23i-real-bag-diagnostics
  git pull --ff-only origin feature/m23i-real-bag-diagnostics
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  python3 -u m23i_long_bag_diagnostics_test.py
  python3 -u m23h_bounded_capture_test.py
  python3 -u m23g_live_recording_preflight_test.py
  python3 -u m23f_existing_recording_inventory_test.py
  python3 -u m23e_offline_evidence_snapshot_test.py
  python3 -u m23d_recorded_bag_readiness_test.py
  python3 -u m23a_xrslam_rosbag_preflight_test.py
) 2>&1 | tee m23i_offline_regressions.log
M23I_REGRESSION_EXIT=${PIPESTATUS[0]}
echo "M23I_OFFLINE_REGRESSION_EXIT_CODE=$M23I_REGRESSION_EXIT"
~~~

Require 124/124 PASS, M23I_LONG_BAG_FORENSICS_CONTRACT=PASS,
M23I_CAMERA_GAP_GATE_UNCHANGED=PASS, exit 0.

## 2. Build local disconnected forensics image

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m23f-existing-recording-inventory
  podman build --network none --no-cache \
    -f Dockerfile.m23i-forensics \
    -t leroy-sensors:m23i-real-bag-forensics .
) 2>&1 | tee m23i_no_device_build.log
M23I_BUILD_EXIT=${PIPESTATUS[0]}
echo "M23I_BUILD_EXIT_CODE=$M23I_BUILD_EXIT"
~~~

Require M23I_JAZZY_FORENSICS_IMPORT=PASS and exit code 0.

## 3. No-device synthetic long-bag acceptance

~~~bash
set +e
set +o pipefail
podman run --rm --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  localhost/leroy-sensors:m23i-real-bag-forensics \
  2>&1 | tee m23i_synthetic.log
M23I_SYNTH_EXIT=${PIPESTATUS[0]}
echo "M23I_SYNTH_EXIT_CODE=$M23I_SYNTH_EXIT"
~~~

Require M23I_SYNTHETIC_LONG_BAG_SELFTEST=PASS, exit 0.

## 4. Re-audit ORIGINAL existing real bag READ-ONLY

No new capture and no /dev/ttyACM* or /dev/video* device mapping.
Set the actual root of the previously captured bag and DO NOT modify
it. If the original M23H_HOST_RECORDINGS variable is available:

~~~bash
BAG_DIR="$M23H_HOST_RECORDINGS/m23g_live_capture001"
test -f "$BAG_DIR/metadata.yaml"

set +e
set +o pipefail
podman run --rm --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --volume "$BAG_DIR":/input/recordings:ro \
  -e M23I_DATA_ROOT=/input/recordings \
  localhost/leroy-sensors:m23i-real-bag-forensics \
  2>&1 | tee m23i_first_real_bag_forensics.log
M23I_REAL_BAG_EXIT=${PIPESTATUS[0]}
echo "M23I_REAL_BAG_AUDIT_EXIT_CODE=$M23I_REAL_BAG_EXIT"
~~~

**Expected result: still BLOCKED, exit 2.** The known
2,951.988ms image gap is disqualifying. This time the diagnostic
M23F_INVENTORY_JSON should include explicit m23a_faults and
gaps_over_250ms, identifying any additional independent failures.
Preserve original file hashes.

## 5. Compare upstream observer with recorded image count

M20 passive subscriber statistics are in M23H's closed capture log.
They are independent of the final rosbag recorder counts and are
useful for localizing loss. Parse on the host, no device access:

~~~bash
CAPTURE_LOG="$M23H_HOST_RECORDINGS/m23h_capture_capture001.log"
python3 - "$CAPTURE_LOG" <<'PY'
import json, sys
from pathlib import Path
lines = Path(sys.argv[1]).read_text(errors="replace").splitlines()
rows = [s for s in lines if s.startswith("M16_PASSIVE_REPORT_JSON=")]
if not rows:
    raise SystemExit("M23I_M20_OBSERVER_REPORT=MISSING")
topics = json.loads(rows[-1].split("=", 1)[1])["topics"]
for kind in ("camera_image", "camera_info", "imu"):
    print("M23I_M20_OBSERVED_" + kind.upper() + "=" +
          str(topics[kind]["accepted"]))
print("M23I_ROSBAG_IMAGE_COUNT=729")
PY
~~~

If M20 received markedly more than 729 images, investigate recorder
subscription DDS and SQLite backpressure. If M20 and rosbag both
observed ~729, investigate USB-camera MJPEG decode, image_proc
rectification, CPU scheduling and the camera publisher. Independent
subscribers can drop different messages; this comparison is a clue,
not proof of a specific loss mechanism.

Do not relax the 250ms gate to get a PASS. This first real bag is
valuable diagnostic evidence, not yet real XRSLAM-ready input.
