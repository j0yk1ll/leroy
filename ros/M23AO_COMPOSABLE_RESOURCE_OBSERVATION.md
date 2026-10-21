# M23AO — device-free composable rectifier CPU and peak resident memory

## Prior accepted measurements

- M23AK + M23AL: 6/6 clean stock composable pthreads OpenCV
  synthetic SIGINT shutdowns, 900/900 messages, OpenCV target-prefix
  mapping and no oneTBB executable mappings at captured phases.
- M23AM: ELF load/relocation compatibility for patched usb_cam,
  cv_bridge, image_geometry and stock librectify, no detected TBB.
- M23AN: independent 640×480 RGB8 900/900 synthetic observer timing
  at **30.032 received FPS**, rect callback gap max **56.172ms**,
  p99 signed raw-to-rect callback receipt skew **15.044ms** and no
  observed callback gaps over 250ms.

None of these tests authorized physical camera access.

## Objective

Measure the **actual ROS component child process** resource use during
the same M23AN synthetic run, with no C++ rebuild and no additional
ROS nodes.

The M23AO Python parent starts an in-process Python observer thread
before delegating to *unchanged* M23AN (itself requiring full M23AK
900/900, actual child exit 0, correct OpenCV mapping and timing
verdict). Monitor reads M23AF's existing post_factory marker in the
private launch log and obtains the exact child container PID and
32-character run nonce. Before each sample it verifies that the live
`/proc/PID/cmdline` contains the expected `m23af_run_id:=nonce`,
checks stable /proc/PID/stat starttime, and collects:

- `/proc/PID/stat` user+system CPU ticks, compared across samples,
  divided by system CLK_TCK and wall monotonic span;
- `/proc/PID/status` VmHWM (kernel high-water RSS in KiB), highest
  sample converted to MiB.

Sampling aims for 200ms, with at least 50 samples spanning at least
20 seconds of the approximately 30-second synthetic image stream.
A PID/nonce/starttime mismatch, empty data, unexpected early exit,
absent marker, source-run failure or insufficient coverage rejects
the measurement.

CPU percentage is expressed **as a percentage of one logical core**,
and can exceed 100% for a multi-threaded process. The measured
CPU time spans post_factory to the last captured sample, not the
entire child lifetime. VmHWM reflects only intervals sampled while
the process exists; no guarantee that post-final-sample peak was seen.
No camera capture-to-output latency or direct OpenCV kernel time is
measured. This stage is **resource characterization**, not acceptance
against a yet-unspecified hardware CPU/RAM budget.

## Source-only gate

From the accepted M23AN worktree, fetch this branch, create a new
detached clean worktree /tmp/leroy-m23ao, pin its expected source head,
and run:

~~~bash
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ao_source_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ao_component_resources.py
~~~

The first runs 12 no-device mock/parsing tests. The second must emit
`M23AO_STATUS=NO_ROS_OR_DEVICE_BY_DEFAULT`, not start ROS.

## One explicit no-device runtime

Use exact previously saved immutable M23AJ image ID
`334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025`
under rootless, network-disconnected, no-added-devices Podman, with
private PID/IPC, capabilities dropped, read-only root, a dedicated
`/tmp` tmpfs using `notmpcopyup`, and tight memory/PID/time caps.

Mount read-only the exact M23AK two Python files, M23AN timing file,
and M23AO monitor into `/opt/m23af`. Configure the same synthetic
ROS environment as accepted M23AN and additional
`M23AO_ARM=resources`; all legacy arms empty. Launch
`python3 -B -u /opt/m23af/m23ao_component_resources.py --run`
after sourcing ROS Jazzy/workspace/M23AC/M23AF and setting the new
OpenCV library directory first in LD_LIBRARY_PATH.

Write one new private non-overwriting host log
`~/.local/state/leroy-m23ao/resources-01.log`. If any stage fails,
stop: preserve existing evidence, do not retry automatically or run
the old crashing control.

Success marker:
`M23AO_VERDICT=SYNTHETIC_COMPONENT_RESOURCE_MEASURED`
with JSON reporting measured component CPU seconds, mean percent
of one logical CPU during sampled interval and high-water RSS MiB.
The verdict means only **observations complete**, never automatic
CPU/memory budget acceptance or permission to connect hardware.

## Camera-only authorization remains separate

After M23AO, compare measured CPU/RSS with the actual host capacity,
camera pipeline workload and explicitly specified budget. Prepare a
separately reviewed camera-only runbook with the patched usb_cam driver,
real CameraInfo calibration, explicit permitted device mapping,
no motion-capable services, motors physically disabled, rollback,
bounded duration, clean stop behavior and the >250ms continuity gate.

No real /dev/video, STM32/IMU/serial, DepthART, XRSLAM,
recording, Nav2, motors, motion or production in M23AO.
