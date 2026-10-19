# M23M — Isolated 921,600-byte ROS Image DDS subscriber QoS matrix

**Draft and unexecuted:** additive to M23L PR #106, stacked on M23K #105.
NO physical camera or STM32, NO recorder, bridge, motor, motion or XRSLAM.

## Source evidence motivating this test

Operator's accepted M23L no-device synthetic run on 2026-10-19:
- Publisher 900/900 fabricated 640x480 RGB8 images, 900 CameraInfo.
- Per-image payload 921600 bytes; timer span **29.9667s**;
  publisher max callback **1.075ms** (API time, NOT end-to-end delivery).
- Independent C++ subscriber got only **299 Images (33.22%)**
  and **887 CameraInfo (98.56%)**.
- Subscriber image gaps >250ms: **26**, max **800.004ms**.
- M23L_VERDICT=BLOCKED_FULL_IMAGE_DDS_SUBSCRIBER_DELIVERY, exit 2.
- No malformed logs or hardware access. Real camera rate unverified.

This demonstrates a full-size Image DDS/subscriber delivery bottleneck
in one isolated graph, independent of the USB camera, JPEG decoding,
rectification, actual IMU, and physical rosbag recorder. It does NOT
identify the specific middleware mechanism or prove it is the only
cause of real camera drops. Original VIO bag stays BLOCKED.

## One-variable-at-a-time comparison

| mode | reliability | receiver depth | role |
|---|---|---:|---|
| `best_effort_5` | Best effort | 5 | M23L receiver baseline |
| `best_effort_100` | Best effort | 100 | Isolate queue depth |
| `reliable_100` | Reliable | 100 | Isolate reliability |

Publisher is **unchanged**, reliable history depth100, nominal 30Hz,
900 synthetic 640x480 RGB8 images (921600B each), 900 CameraInfo.
Both topic subscriptions use the selected receiver QoS profile;
raw-only mode skips the absent rectifier. C++ code logs its actual
selected mode `M23M_COUNTER_QOS_PROFILE=`; the Python supervisor
rejects any discrepancy from requested `M23M_SUB_QOS`.

Each run permits **2 seconds of subscriber drain** after successful
publisher completion, avoiding immediate teardown of in-flight
samples. Accordingly the earlier 299/900 M23L baseline (no drain)
is valid diagnostic evidence but must NOT be treated as an
identical-control run: rerun `best_effort_5` here as baseline.

Report counts, delivery fractions, actual publisher timespan, max
publish callback, actual RMW_IMPLEMENTATION if explicitly selected
(otherwise 'unspecified_default'), subscriber header gaps, verdict,
and zero-device checks. M23L's original 90% synthetic-only
classification threshold is retained; it has NOTHING to do with
M23A's strict 250ms real-camera VIO input gate.

## Phase A — Offline Python contract regressions (25 total)

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23m-dds-qos-matrix
git pull --ff-only origin feature/m23m-dds-qos-matrix
cd ros
PYTHONDONTWRITEBYTECODE=1 python3 -u m23l_synthetic_dds_baseline_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23m_dds_qos_matrix_test.py
```

Require previous 16/16 M23L and new **9/9 M23M** tests PASS.
Expected `M23M_OFFLINE_RECEIVER_QOS_MATRIX=PASS` and
`M23M_DEVICE_ACCESS=NONE`.

## Phase B — Disconnected image build

```bash
set +e
set +o pipefail
podman image exists localhost/leroy-sensors:m23l-synthetic-image-dds
podman build --network none --no-cache \
  -f Dockerfile.m23m-qos \
  -t leroy-sensors:m23m-synthetic-qos . \
  2>&1 | tee m23m_build.log
echo "M23M_BUILD_EXIT_CODE=${PIPESTATUS[0]}"
```

Require `M23M_JAZZY_QOS_CPP_BUILD=PASS`, build exit 0.
No package download, physical sensor mappings or robot launch.

## Phase C — Three independent zero-device synthetic runs

```bash
set +e
set +o pipefail
for qos in best_effort_5 best_effort_100 reliable_100; do
  echo "M23M_SELECTED_RECEIVER_QOS=$qos"
  podman run --rm --network none --ipc private --pid private \
    --shm-size 512m \
    --cap-drop all --security-opt no-new-privileges \
    --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    -e M23M_SUB_QOS="$qos" \
    localhost/leroy-sensors:m23m-synthetic-qos \
    2>&1 | tee "m23m_${qos}.log"
  echo "M23M_${qos}_EXIT_CODE=${PIPESTATUS[0]}"
done
```

These three tests are sequential and require no hardware approval.
**Exit 2 is expected if a profile still fails full-Image DDS
diagnostic readiness**. Do not mistake exit 2 for a crash; examine
`M23M_QOS_JSON`, `M23M_QOS_VERDICT`, and the source `faults`.
Invalid profile values are refused, and no rosbag is written.

## How to interpret

- `best_effort_5` weak but `best_effort_100` strong: receiver
  history depth materially affects receipt under this workload.
- Both best-effort modes weak but `reliable_100` strong: reliability
  (retransmission/order/backpressure) likely matters for large frames
  within these local RMW and CPU conditions; check publisher span
  to ensure it still holds ~30Hz.
- All modes ~33% (or poor): depth/reliability alone did not correct
  this DDS path. Next inspect actual active RMW implementation,
  DDS fragmentation/shared-memory/transport configuration, load
  and scheduling, with an explicit alternate-RMW experiment only
  if installed locally. Do not assert which middleware is active.
- Reliable receiver impairs publisher span: exchanging drops
  for backpressure does NOT establish camera-ready throughput.

Even a 900/900 synthetic DDS PASS does not validate live camera
Image content, CameraInfo fidelity, IMU coverage, extrinsics,
XRSLAM tracking, metric pose or robot motion. The original M23H
rosbag remains immutable, and 250ms image-gap rejection stands.
