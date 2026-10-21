# M23AN — one device-free pthreads rectifier timing characterization

## What has passed

M23AK and M23AL: six independent **stock composable image_proc
RectifyNode** synthetic 900/900 clean SIGINT shutdowns using alternate
OpenCV 4.6 pthreads with no TBB executable mapping in either
post-factory or on-shutdown snapshot.

M23AM: six camera-pipeline ELFs (Leroy patched usb_cam overlay
executable/library/component, cv_bridge, image_geometry and
librectify) resolved dynamic relocations with all six OpenCV
libraries under `/opt/m23aj-opencv/lib/`, without TBB.

Immutable diagnostic image:
`334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025`.

Physical camera and production are NOT authorized.

## Question and measurement limitations

Characterize actual **ROS subscription delivery** at the existing
fabricated calibration fixture: 900 synthetic RGB8 640 x 480 raw,
rectified and CameraInfo messages, at approximately 30 Hz.
The reused observer emits three separate monotonic steady-clock
receipt timestamps per header stamp:

- `M23K_SAMPLE raw <header_ns> <receipt_ns>`
- `M23K_SAMPLE rect <header_ns> <receipt_ns>`
- `M23K_SAMPLE info <header_ns> <receipt_ns>`

Only after the original M23AK 900/900/QoS/graph/stamp/drain/full
real child-exit clean gate succeeds does the M23AN Python wrapper
open the observer and publisher logs and calculate:
- 900 unique and matching stamps for all streams;
- rectified subscriber observed FPS between first and last
  receipt, p50/p99/max of the successive rect callback intervals
  and count of measured receipt gaps >250ms;
- signed p50/p95/p99 of rect-minus-raw callback receipt skew,
  p99 absolute skew and negative-skew count.

Skew of receipt timestamps observed in one ROS subscriber process
is a composite DDS/executor/scheduling/delivery measurement; it
is NOT actual camera capture-to-pixel latency or OpenCV internal
rectify execution time. Negative skew is valid and reported, not
silently clamped. This stage does not yet measure rectifier CPU
utilization or peak RSS. Do not call it full performance
acceptance until CPU/memory and application requirements are
checked separately.

## Gate A — offline source checks

From accepted M23AM worktree:

~~~bash
cd /tmp/leroy-m23am
git fetch origin feature/m23an-pthreads-synthetic-timing
git worktree add --detach /tmp/leroy-m23an   origin/feature/m23an-pthreads-synthetic-timing
cd /tmp/leroy-m23an
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23an_source_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23an_synthetic_timing.py
~~~

Expect 11 offline unit tests and
`M23AN_STATUS=NO_ROS_OR_DEVICE_BY_DEFAULT`. No hardware,
container runtime or signal tests in this stage.

## Gate B — one independent isolated synthetic observation

Host preflight must verify saved M23AJ image ID, original M23AF
parent image ID, clean pinned source checkout and zero pre-existing
M23AN evidence. One owner-private non-overwriting log
`$HOME/.local/state/leroy-m23an/timing-01.log`.

Run a rootless `podman run` from the immutable M23AJ image ID,
`--network=none --ipc=private --pid=private --cap-drop=all`,
`--security-opt=no-new-privileges --read-only`, a new private
`/tmp` with `notmpcopyup`, time/PID/memory caps and **NO**
physical-device mappings. Mount read-only the three Python scripts
M23AK comparator, M23AK ELF map gate and M23AN timing reporter into
`/opt/m23af/`; ensure the alternate OpenCV prefix leads the
container's LD_LIBRARY_PATH. Reuse all required M23AK explicit
synthetic environment flags, and add `M23AN_ARM=timing`.

If the original inherited M23AK component test fails, this trial
blocks without a timing success marker. If observer logs are
incomplete, headers mismatch, synthetic payload is not 640x480
RGB8 or any rect callback receipt gap is over 250ms, fail and
preserve the log; do not retry.

Expected terminal marker only if complete:
`M23AN_VERDICT=SYNTHETIC_OBSERVER_TIMING_RECORDED`.

Report the summary JSON only, not the full raw maps, footage or
host cores. Run is device-free and provides no physical-camera
authorization. Remaining for first physical-camera gate:
rectifier process CPU/peak-memory budget characterization under
realistic load and explicitly reviewed isolated patched-usb_cam
camera launch/stop procedure; full VIO/STM32/IMU/motors/production
remain separately blocked.
