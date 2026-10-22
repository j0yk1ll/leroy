# M23BE — Source-only REAL trace completeness contract (no live run)

**DRAFT / offline source-review only. No physical camera retest, no MCU,
serial, robotics, ROS execution, recording, or production acceptance.**

M23BD has now completed an offline read-only synthetic evidence
postflight, with operator-reported PASS, exact immutable log SHA256
`84986ce2126edbbc2ae3fa16fd6099f6b6a75b4b37d24bfb8f6d8c4a480d2b07`.
The preserved M23BC normal and deliberate process-exit logs and
payloads remain unchanged. The original M23AU first camera-only
trial remains CLOSED/BLOCKED due to one repeated raw-subscriber
stamp; whether that originates from `usb_cam` publisher timestamp
reuse, DDS behaviour, callback scheduling or another stage is NOT
yet attributed.

## Found gap

- Existing M23AY/M23AX camera diagnostic reports per-attempt
  `M23AX_PUBLISH pub_index=... fresh=... status=...
  dequeue_count=... v4l2_seq=... v4l2_timestamp_us=...
  ros_stamp_ns=...`, before
  `m_image_publisher->publish(...)`. The statement does **not**
  acknowledge DDS delivery.
- Existing M23K subscriber reports each
  `M23K_SAMPLE raw|rect|info ROS_STAMP STEADY_RECEIPT` and a
  final `M23K_COUNTER_COUNTS=RAW,RECT,INFO`. This can establish
  internal counter-log closure on a clean observer exit.
- Existing ROS launch log may show a clean `usb_cam_node_exe`
  and `component_container` child exit, but it has no explicit
  count of camera publisher attempts **after the last preserved
  M23AX pre-send line**, nor how many publish function calls returned.

Therefore **preserved bytes + apparently contiguous indexes
are NOT sufficient evidence of a complete camera publisher trace**.
Even a publisher final count would not prove that every attempted
Image was delivered by DDS or observed by a subscriber.

## What M23BE defines

`ros/m23be_trace_completion_contract.py` is read-only, inert by
default and standard-library-only except importing the earlier
`m23ba_event_correlation.py` pure Python parser. It defines a
**future, currently UNIMPLEMENTED** camera-source epilogue:

    M23BE_PUBLISHER_FINAL attempts=N publish_returns=N shutdown=clean

The epilogue is a future production-of-evidence contract, NOT an
observed current source marker. A separate source patch would need
to count returns *after* `m_image_publisher->publish(...)` and
emit a unique final summary under a completed clean shutdown
before any future physical camera protocol review. It must not
fabricate such a line from the last pub_index, avoid changing
camera publish semantics, and must fail to emit `shutdown=clean`
if abnormal termination or incomplete write occurs.

The validator requires:
1. Exactly one syntactically correct marker, positive bounded
   `attempts` and `publish_returns == attempts`, matching
   the occurrence-level parsed M23AX pre-send event count,
   with no M23AX pre-send events **after** the summary.
2. No M23AX metadata cap, ROS child-crash or segmentation-fault
   markers, and exactly one ROS **actual clean child-exit**
   marker for each `usb_cam_node_exe` and
   `component_container`. `ros2 launch` parent exit zero
   alone is insufficient.
3. Full `reliable_100` C++ subscriber coverage with exactly
   one final summary matching occurrence counts in raw,
   rect and info streams, as independently parsed by M23BA.
4. Separate regular non-symlink UTF-8 text files bounded
   at 4 MB each. No image pixels, ROS bags or frame capture.
5. Structural completion output only:
   `OFFLINE_STRUCTURAL_TRACE_CONTRACT_ONLY`;
   header stamp repeats remain **reported anomalies**, never
   silently converted into accepted monotonicity.

The returned JSON intentionally says
`publisher_completion_marker_implemented_in_current_image=false`,
`publication_return_is_not_dds_delivery_ack=true`,
`observer_and_publisher_capture_epochs_proven_equal=false`,
`data_continuity_or_stamp_monotonicity_accepted=false`,
`production_acceptance=BLOCKED`.
The current accepted image has **no final summary marker**,
so unmodified images cannot pass this future trace closure gate.

`ros/m23be_trace_completion_contract_test.py` defines 24
pure synthetic tests, including a fabricated proposed completion
marker, missing/duplicated/malformed publisher marker, return count
mismatch, ROS child crashes, missing clean markers, partial
subscriber counts, wrong QoS, symlinks, oversize inputs and
a RAW stamp-repeat case that must remain diagnostic-only.

## Operator Gate A — only source tests, NO ROS or devices

Use a fresh detached sparse checkout of the pinned M23BE PR commit.
Fetch:
- `ros/m23be_trace_completion_contract.py`
- `ros/m23be_trace_completion_contract_test.py`
- `ros/m23ba_event_correlation.py`
- `ros/m23bc_synthetic_persistence.py`
- `ros/m23bb_host_evidence_adapter.py` (import of the synthetic upstream)

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23be_trace_completion_contract_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23be_trace_completion_contract.py
```

Expected (NOT YET OBSERVED):
`Ran 24 tests`, `OK`,
`M23BE_STATUS=INERT_NO_ROS_NO_DEVICE`.

A PASS validates only the synthetic parser/contract; no actual
real camera file currently satisfies the new publisher epilogue.

## Separate future hold points

Before any future physical camera approval, independently review:
- A separately built, pinned binary that can emit the actual
  `M23BE_PUBLISHER_FINAL` on clean shutdown, with complete
  acquisition and publisher return counts (no synthetic spoof).
- No-ROS isolated source/ELF/loader and output-flush checks.
- Bounded size/fault handling for both text logs and unambiguous
  host persistent evidence / process exit manifest.
- A captured observation epoch definition and matched publisher/
  subscriber event IDs, if genuinely required for causal attribution.
- Renewed electrical motor and manipulator power isolation and
  explicit human approval for one camera-only experiment.

Do not execute `ros/exec.sh` or `ros/robot.launch.py`.
No camera, serial/MCU/IMU, recording, XRSLAM/DepthART, Nav2,
motor or production use is authorized by this PR.

M23BE_CAMERA_RETEST_AUTHORIZED=FALSE
M23BE_PRODUCTION_ACCEPTANCE=BLOCKED
