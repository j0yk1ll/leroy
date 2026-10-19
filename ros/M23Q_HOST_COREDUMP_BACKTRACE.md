# M23Q — host-only saved-core inventory and backtrace triage

**Draft, not yet executed on the operator host.** Stacked on M23P PR #110,
which remains BLOCKED by a reproducible device-free `image_proc`
`component_container` shutdown SIGSEGV, not by continuous image delivery.

## Observations motivating M23Q

In M23P's most recent synthetic, device-free run:
- Publisher: **900** fabricated images and calibrated-shaped CameraInfo.
- Independent subscriber: **899 raw**, **899 rect**, **899/899** rect
  header stamps matched observed raw, max rect header gap **34.232ms**.
- Runtime RMW **rmw_fastrtps_cpp**, all required inspected publishers,
  rectifier subscriptions and C++ subscriptions **RELIABLE/VOLATILE**.
- Child `component_container` died **SIGSEGV (-11)** after launch
  shutdown `SIGINT`, while the parent `ros2 launch` exited 0.
- `M23P_DEVICE_ACCESS=NONE`, synthetic calibration only, overall verdict
  `BLOCKED_NO_DEVICE_RECTIFIER_LIFECYCLE`, exit 2.

A separate idle image_proc component previously shut down cleanly.
The exact crashing function, stack and faulting module remain unverified.

Operator's host preflight on Arch Linux confirmed:
- Host `gdb`: `/usr/bin/gdb`.
- Host `coredumpctl`: `/usr/bin/coredumpctl`.
- The old no-device Jazzy image has no gdb or eu-stack; it has readelf.
- `/proc/sys/kernel/core_pattern` pipes to host `systemd-coredump`.

**A coredump *handler* does not prove a core was saved or retained.**
systemd may report journal-only, missing, truncated, or inaccessible dumps.
Also, the number `pid 32` printed by ROS launch was a container PID,
**not necessarily the host PID** in the journal. Never guess or use
that number without confirming the matching host crash row.

## Phase A — read-only host journal inspection (do this first)

No camera or other device needs to be present. No Podman container,
ROS process, debugger attachment, sysctl modification or privilege
escalation is involved.

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23q-host-coredump-backtrace
git pull --ff-only origin feature/m23q-host-coredump-backtrace
cd ros

bash -n m23q_host_coredump_inventory.sh
bash ./m23q_host_coredump_inventory.sh
```

The script queries `coredumpctl --since "2026-10-19 00:00:00" list`,
showing **only candidate component_container rows** (Linux `comm`
may be truncated to `component_conta`) and preserves original list
metadata in a local directory under
`~/.local/state/leroy-m23q/inventory.*`, private to the operator.
It does **not** extract an actual core dump.

For an identified candidate, note the **host PID** in that candidate
row (and cross-check timestamp against the M23P run), then:

```bash
bash ./m23q_host_coredump_inventory.sh HOST_PID_FROM_CANDIDATE_ROW
```

Replace `HOST_PID_FROM_CANDIDATE_ROW` with the actual numeric host PID.
The script refuses to print a selected process's info unless the
result contains the `component_container` identity.

Provide the candidate rows and selected info excerpt, **not a binary
core dump**. Do not share journal entries for unrelated host programs
or private process environment variables.

Interpret `COREFILE` carefully:
- `present` or `journal`: a dump may be retrievable; subsequently
  perform *separately reviewed* host GDB analysis of that matched core.
- `missing`, `none`, `error`, `truncated`: availability or integrity
  is not proven. Do not claim a complete backtrace.
- No candidate: no retained journal *match was visible to this user*;
  do not say the segmentation fault never occurred.

## Phase B — if a retained suitable core exists

**Do not do this phase until inspecting Phase A's host PID,
COREFILE status and process metadata.** Prefer a symbolized backtrace
from the prior core before another process run. Host Arch GDB may
be unable to resolve Linux/ROS symbols from the Ubuntu/Jazzy container
once its `--rm` rootfs is gone. If so, extract *matching exact*
executable, shared libraries and debug symbols from the **local
offline image** into a private sysroot; set GDB `sysroot` accordingly.
Do not accidentally attribute symbols from Arch host libraries to
Jazzy component frames.

A systemd `coredumpctl debug` command can invoke the installed host
GDB for a selected core, but only after matching the correct host PID
and core status. The resulting GDB `thread apply all bt` / faulting
thread should be reported with module build IDs and symbol-resolution
limits, not just `SIGSEGV`.

No GDB capability is added to the production or M23P container.
No `--privileged`, host networking, mapped camera/STM32, or interactive
live process attach is authorized by M23Q.

## Strict invariant

M23Q is a **postmortem-only investigation**. No motion, Nav2, XRSLAM,
recorder, STM32 serial, real USB camera, fresh data capture or
production modifications. M23A's strictly-greater-than-250ms
Image-header-gap blocking rule and the first physical rosbag are
unchanged. A stack trace can locate a crash but cannot by itself
certify calibration, VIO or motion readiness.
