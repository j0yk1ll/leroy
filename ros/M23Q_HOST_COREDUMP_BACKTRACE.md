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

## Phase A evidence — operator verified historical crash

The operator's read-only `coredumpctl info 48349` confirmed:
- Host PID **48349**, process executable
  `/opt/ros/jazzy/lib/rclcpp_components/component_container`.
- Command line names `__node:=m23p_image_proc_container`.
- **SIGSEGV**, `si_code=SEGV_MAPERR`.
- October 19, 2026 at **16:24:57 CEST**, matching latest M23P
  899/900 synthetic rectified-image experiment.
- `Storage` is a **present** retained systemd .zst core; size on disk
  1.9 MB (not a claim about decompressed size).
- Host journal's crashing stack thread 89 has unsymbolized frames at
  `0x00007f56a4912f5b` and `0x00007f56a491af75`, plus libc
  frames; the faulting module remains unknown.

Other historical core host PIDs were 19636, 21771, 39260, 41267,
46312; this step chooses only **48349**. `SEGV_MAPERR` is an
unmapped-memory access classification, not proof of use-after-free
or an image_proc destructor bug.

## Phase B — guarded first-pass GDB analysis of retained core

**Phase A's process-identity and COREFILE checks are now satisfied.**
M23Q provides a guarded host-only script for a **postmortem**
first pass, not a fully symbolized diagnosis.

**First operator execution BLOCKED safely at image identity.**
The initial script pinned the **older** corrected-fixture M23P
image `8c75cdd760b9...`, but the user reported the newer local image
ID `6eeb8ca08525...` after the publisher-first M23P rebuild.
The old image ID was documented before the 16:24:57 publisher-first
crash and therefore was the wrong assumption for this crash. No
core extraction, image staging or GDB execution occurred.
A matching tag alone is *not* proof that `6eeb...` made the core.

The revised script pins immutable candidate image
`6eeb8ca08525d451dc0f216f097c1df7e300444d8c8ae551fde576d64a1c48f1`
and **fails closed** unless its Podman-created timestamp is **after**
the earlier 16:11:25 M23P SIGSEGV and **no later** than the selected
16:24:57 core. It stages the binary and exact Python runner and
read-only graph probe via a *stopped* container, compares the two
embedded Python sources against M23P Git commit
`6c300889a2f472c4f8b1bf568334a1254b6d7a03`,
and **only then** exports the retained core to private storage for
preliminary host GDB analysis. Source and chronology matching
cannot by themselves authenticate loaded module build IDs; later
symbol verification must still be explicit.

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23q-host-coredump-backtrace
git pull --ff-only origin feature/m23q-host-coredump-backtrace
cd ros

bash -n m23q_host_gdb_firstpass.sh
bash ./m23q_host_gdb_firstpass.sh
```

The script verifies exact host crash identity, `SIGSEGV`,
`Storage=(present)`, immutable candidate image ID and experiment-time
chronology, plus exact embedded Python files. Any mismatch is
BLOCKED rather than permitting accidental wrong-binary symbolization.
It calls `coredumpctl dump 48349 --output=...` **only after those
image/source checks**, extracting the existing core to a private
mode-0700 directory (core mode-0600) under
`~/.local/state/leroy-m23q/gdb-firstpass.*`.

The script calls `podman create` but **never `podman start/run`**.
The created container is never executed; it exists solely to copy
the exact Jazzy `component_container` binary and `librectify.so`
from the already-built local image into a private partial sysroot,
then is removed. It runs host GDB **in batch on the saved core**,
without live attachment, disables debuginfod and GDB auto-load,
uses `set sysroot` to avoid falsely loading Arch shared libraries,
and writes private first-pass `info threads`, `bt 30`,
`info symbol $pc`, `info sharedlibrary` and `info proc mappings`
to a log.

**The first-pass result is intentionally `PRELIMINARY_SYMBOLS_UNVERIFIED`.**
Only two files are initially staged. Missing shared libraries and
debug symbols may still leave frames as unknown. Examine the GDB
fault address and mapping path first, then stage **only exact mapped
libraries** from that same image for further backtrace analysis.
A faulting-thread GDB stack still requires matching build IDs and
symbol resolution before naming a responsible package.

The extracted core may contain private memory. **Do not upload it,
paste its contents, or copy it to a public directory.** Provide only
sanitized GDB backtrace/module mapping text and status markers. The
script never opens a device and never enables networking or debugger
attachment.

No GDB capability is added to the production or M23P runtime image.
No `--privileged`, host networking, mapped camera/STM32, or
new ROS process is authorized by M23Q.

## Strict invariant

M23Q is a **postmortem-only investigation**. No motion, Nav2, XRSLAM,
recorder, STM32 serial, real USB camera, fresh data capture or
production modifications. M23A's strictly-greater-than-250ms
Image-header-gap blocking rule and the first physical rosbag are
unchanged. A stack trace can locate a crash but cannot by itself
certify calibration, VIO or motion readiness.
