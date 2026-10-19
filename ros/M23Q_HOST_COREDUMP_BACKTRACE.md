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

**Second operator execution also blocked safely before core extraction.**
The pinned candidate image was accessible, but Podman reported its
creation time as `2026-10-19 14:23:21.51374453 +0000 UTC`.
GNU `date -d` rejects the redundant combination `+0000 UTC`,
leading to `M23Q_BLOCKED=IMAGE_CREATED_TIME_INVALID`.
This is a timestamp parsing problem, **not evidence that the image
falls outside the experiment window**. Interpreted using the explicit
`+0000` offset, this is **16:23:21 CEST**, later than 16:11:25
and earlier than 16:24:57. The script now strips only the redundant
literal ` UTC` suffix when ` +0000 UTC` appears, retains the
numeric offset, and still requires the original chronology and
source comparisons. No core was extracted or GDB run on this attempt.

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

## Phase C — inspect the existing core's mapped module, without GDB symbol guesses

**Operator first-pass result (2026-10-19):** historical core `48349` was
exported (110,616,576 bytes), immutable image ID and embedded M23P
sources were verified, and GDB completed with exit0. However, GDB's
faulting LWP **89** unwound mostly `??`, with PC
`0x00007f56a4912f5b`, nearby apparent frame
`0x00007f56a491af75`, and `info symbol $pc` returned no symbol.
The previous text-log regex found no mapping encompassing the PC.
Therefore no library or culprit has yet been established. The
unsymbolized stack is not a trustworthy complete call chain.

A small **read-only** stdlib inspector
`ros/m23q_core_fault_mapping.py` reads the ELF64 `PT_NOTE`
`NT_FILE` mapping table and `PT_LOAD` virtual memory ranges
directly from the **already extracted** core. It outputs **only**
the entries covering those two suspect addresses. It does not scan
or print core memory contents. Its CLI refuses every path except
a regular `core.48349` file in a private M23Q `gdb-firstpass.*`
directory.

From the M23Q branch's `ros` directory:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -u m23q_core_fault_mapping_test.py

CORE="$HOME/.local/state/leroy-m23q/gdb-firstpass.sXKTfoQL/core.48349"
test -f "$CORE" || exit 2

PYTHONDONTWRITEBYTECODE=1 python3 -u \
  m23q_core_fault_mapping.py "$CORE"
```

Expected offline test marker:
`M23Q_ELF_MAPPING_SOURCE_CONTRACT=PASS` (5 unit tests).
**The tests have not yet been run by the operator.**

Interpretation:
- `M23Q_FAULT_FILE_MATCHES=1` and a `M23Q_FAULT_FILE_MAP=...`
  record name the *mapped-file pathname in the core*, **not** a
  proven faulting function or genuine same-build local library.
  The emitted `pc_file_offset` can guide later ELF build-ID and
  symbol checks against the **pinned immutable M23P image**.
- `M23Q_FAULT_FILE_MATCHES=0` with a `PT_LOAD` match can mean
  anonymous/code mappings, missing `NT_FILE` metadata, or another
  memory mapping state. Do not force an unrelated library name.
- No match in either table means the required mapping isn't
  recoverable from this core's parsed metadata; inspect the ELF
  program headers/notes separately before concluding anything.
- The `NEARBY` address is from an **unverified stack frame**; treat
  its mapping as a cross-check, not independent reliable backtrace.

The script deliberately leaves
`M23Q_MODULE_BUILD_ID_VERIFIED=FALSE` and
`M23Q_FAULTING_FUNCTION_IDENTIFIED=FALSE`. If a file mapping exists,
the next narrowly scoped step is to copy that one module from the
**same pinned image**, compare build IDs and symbolize PC-relative
offsets. **No new ROS or hardware run** is required. Do not upload
the binary core or post unrelated file paths from its notes.

## Phase D — distinguish unmapped instruction fetch from invalid data access

**Operator's Phase C report:** `M23Q_NT_FILE_NOTES=1`, but
`M23Q_FAULT_FILE_MATCHES=0`, `M23Q_FAULT_LOAD_MATCHES=0`,
`M23Q_NEARBY_FILE_MATCHES=0`, and
`M23Q_NEARBY_LOAD_MATCHES=0`. The ELF core's mapping tables do
**not** associate the faulting PC `0x00007f56a4912f5b` with
any file or `PT_LOAD` range. This suggests a possible jump into
unmapped code during teardown, but **does not prove** that a library
was unloaded, that `image_proc` is at fault, or that the fault was
an instruction-fetch rather than invalid data access.

The existing read-only `m23q_core_fault_mapping.py` has been extended
to decode Linux x86_64 ELF `NT_SIGINFO` metadata:
- `si_signo` and `si_code` (kernel-reported signal and subtype).
- `si_addr` (kernel-reported bad memory address) at byte offset 16
  of the x86_64 `siginfo_t` descriptor.
- Whether `si_addr == 0x00007f56a4912f5b`, and whether that
  address has a core file/LOAD mapping. This is an inference only:
  even an equal unmapped PC and `SEGV_MAPERR` cannot prove the
  specific lifetime or destructor bug without a fuller stack.

**Source patch is not yet operator-tested**; no new core is created.
From the same M23Q `ros` branch and existing private core path:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -u m23q_core_fault_mapping_test.py

CORE="$HOME/.local/state/leroy-m23q/gdb-firstpass.sXKTfoQL/core.48349"
test -f "$CORE" || exit 2

PYTHONDONTWRITEBYTECODE=1 python3 -u \
  m23q_core_fault_mapping.py "$CORE"
```

The offline suite has **11 synthetic fixture tests** (up from five).
Relevant outputs:
`M23Q_NT_SIGINFO_NOTES`,
`M23Q_SIGINFO_SIGNAL`, `M23Q_SIGINFO_CODE`,
`M23Q_SIGINFO_FAULT_ADDRESS`, `M23Q_SIGINFO_EQUALS_FAULT_PC`,
and `M23Q_UNMAPPED_EXECUTION_HYPOTHESIS`.

- If SIGSEGV/SEGV_MAPERR `si_addr` equals unmapped PC, mark
  `SUPPORTED`: this is **evidence consistent with** executing
  unmapped code, not fault attribution.
- If `si_addr` differs, mark `NOT_ESTABLISHED`: seek the memory
  access target and registers rather than claiming unloaded code.
- If `NT_SIGINFO` is missing, mark `UNKNOWN`. Multiple signal
  notes or malformed notes fail closed.

Do not share the whole core; report only these bounded metadata
markers. Remain device-free, and preserve all prior blocked gates.

## Phase E — bound the unmapped-PC gap using surviving mappings

**Operator's Phase D result:** The already saved M23Q core has exactly
one `NT_SIGINFO` note, signal 11/SEGV_MAPERR, with kernel-recorded
`si_addr=0x00007f56a4912f5b` equal to the faulting PC. That
address has **no NT_FILE mapping and no PT_LOAD range** in the core.
`M23Q_UNMAPPED_EXECUTION_HYPOTHESIS=SUPPORTED`.

This is affirmative evidence for an unmapped instruction target at
the moment of crash. **It does not establish that code was unloaded
or that a specific library owned the address previously.** The
unmapped call target may result from a stale function pointer,
virtual dispatch, torn lifetime, or corruption.

The read-only inspector now also reports the single nearest
*surviving* file mapping and executable PT_LOAD below and above the
fault PC, with their exact distances in bytes. It outputs only a
basename and coarse `JAZZY/OTHER` label for a neighboring file, not
its full path, to reduce accidental disclosure of host private paths.
It explicitly prints `M23Q_NEAREST_MAPPING_PROVES_UNLOAD=FALSE`.
The bounding files are not automatically suspects.

**This Phase E source patch is not yet operator-tested.** Run only
synthetic offline tests and reread the same existing private core:

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23q-host-coredump-backtrace
git pull --ff-only origin feature/m23q-host-coredump-backtrace
cd ros

PYTHONDONTWRITEBYTECODE=1 python3 -u m23q_core_fault_mapping_test.py

CORE="$HOME/.local/state/leroy-m23q/gdb-firstpass.sXKTfoQL/core.48349"
test -f "$CORE" || exit 2

PYTHONDONTWRITEBYTECODE=1 python3 -u \
  m23q_core_fault_mapping.py "$CORE"
```

Expect **15** synthetic mapping tests and
`M23Q_ELF_MAPPING_SOURCE_CONTRACT=PASS`. Please report only
`M23Q_FAULT_NEAREST_FILE_*`,
`M23Q_FAULT_NEAREST_EXEC_LOAD_*`, and
`M23Q_NEAREST_MAPPING_PROVES_UNLOAD=FALSE`.
An absence of any neighboring mapping is still a valid observation
and **not** evidence of a specific unloaded library.

For actual causal attribution a future no-device-only controlled
experiment may need executable-module lifetime instrumentation,
address registration history or a symbolized valid return path;
do not authorize new experiments or alter production in Phase E.

## Phase F — bounded host GDB reinspection of thread 89 (saved core only)

**Operator Phase E observation:** The closest surviving file-backed
region below the unmapped PC ends at `0x00007f56a40a9000`
(`libsensor_msgs__rosidl_generator_c.so`, 8,822,619 bytes away),
and the nearest above begins at `0x00007f56a5116000`
(`libz.so.1.3`, 8,401,061 bytes away). The nearest executable
`PT_LOAD` boundaries are similarly distant, around 8.9 and
8.4 MB. **Neither neighboring library is evidence of ownership of
the unmapped address or of a specific library unload.** The
`NT_SIGINFO` observation remains that `SEGV_MAPERR si_addr == RIP`.

The next useful postmortem step is a **targeted host GDB reinspection**
of previously extracted core 48349, using the same pinned Jazzy
binary from the already-verified private sysroot. The new
`ros/m23q_host_gdb_thread89.sh` operates only on the existing
`~/.local/state/leroy-m23q/gdb-firstpass.sXKTfoQL` directory.
It neither starts a ROS/container process nor attaches to a live
process, accesses hardware, writes to the original core, downloads
symbols or prints raw stack memory.

From the M23Q branch:

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23q-host-coredump-backtrace
git pull --ff-only origin feature/m23q-host-coredump-backtrace
cd ros

bash -n m23q_host_gdb_thread89.sh
bash ./m23q_host_gdb_thread89.sh
```

The script verifies a private directory and existing regular
core/executable, then runs **host GDB in batch** with auto-load and
debuginfod disabled. The private log contains bounded diagnostics
for `info threads`, `thread 1` (historically LWP 89),
`info registers rip rsp rbp`, `info frame`, `bt 18`,
`p $_siginfo` and `info sharedlibrary`.

**Status: source written to draft PR, not yet operator-tested.**
A clean GDB batch exit and confirmed LWP 89 constitute a
`PRELIMINARY_FAULT_THREAD_CONFIRMED` result only. They do not
resolve the faulting module or validate an unwind through missing
symbols. If the output remains `??`, a later step may require
copying *only* the exact matching mapped dependencies and
checking build IDs; do not substitute host Arch libraries.

The log remains private. Share only the script's status markers
and a short reviewed excerpt around `info frame`, `rip/rsp/rbp`
and the first few `bt` frames; do not post the full core or any
unrelated private mappings. An existing output log causes the
script to block rather than overwrite it.

## Phase G — stage exact Jazzy dependency files for saved-core GDB

**Latest operator excerpt:** frame-zero details were absent from the
bounded extraction, and the private GDB log warned that many
`/opt/ros/jazzy/lib/*.so` and
`/usr/lib/x86_64-linux-gnu/*.so*` files could not be opened
while processing file-backed mappings. The specific ephemeral
`/dev/shm/fastrtps_...` warning is expected when examining a core
after the original process has exited; **do not try to recreate it**
or attribute the SIGSEGV to Fast DDS based on that warning.

The first-pass debugger sysroot contains only the pinned-image
`component_container` executable and `librectify.so`. **Missing
matching shared objects can prevent accurate unwinding and source-level
symbolization**, but restoring them does not guarantee that the
unmapped PC or its originating call can be identified.

This phase adds:
- `m23q_missing_core_libraries.py`: a strict, read-only parser for
  `warning: Can't open file ... during file-backed mapping note
  processing` lines in the **existing private first-pass GDB log**.
  Only ROS Jazzy and Ubuntu shared-object paths are allowed.
  Names with traversal, arbitrary host paths and `/dev/shm` are
  rejected, and the number of libraries is capped at 400.
- `m23q_missing_core_libraries_test.py`: **8 synthetic offline
  source tests**, no hardware or historical-core access.
- `m23q_host_gdb_exact_dependencies.sh`: checks the immutable
  candidate image ID and M23P embedded-source provenance again;
  calls `podman create` only, **never starts the container**;
  copies only the missing shared-library files into the existing
  private sysroot, rejects symlinks to avoid accidentally following
  an absolute link to the Arch host filesystem, and reruns host
  batch GDB on the already-extracted saved core.

The phase is **not operator-tested yet**. From the M23Q branch:

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23q-host-coredump-backtrace
git pull --ff-only origin feature/m23q-host-coredump-backtrace
cd ros

PYTHONDONTWRITEBYTECODE=1 python3 -u m23q_missing_core_libraries_test.py
bash -n m23q_host_gdb_exact_dependencies.sh

# Only after both offline checks succeed:
bash ./m23q_host_gdb_exact_dependencies.sh
```

Expected test marker:
`M23Q_MISSING_LIBRARY_ALLOWLIST_CONTRACT=PASS` with **8/8 tests**.

The guarded dependency stage reports
`M23Q_DEPS_WARNED_LIBRARIES`,
`M23Q_DEPS_COPIED`, `M23Q_DEPS_ALREADY_PRESENT`,
`M23Q_DEPS_UNRESOLVED`, and
`M23Q_DEPS_STAGE_STATUS`.
`PARTIAL` means some requested libraries could not be staged;
it is **not** a successful symbolization. Some shared-object names
may be symlinks in the original image, which this initial stage
deliberately refuses rather than risking host-library substitution.
The private rerun GDB log is `gdb-with-image-libs.log` in the
preexisting `gdb-firstpass.sXKTfoQL` directory. The shell script
will not overwrite any existing log or manifest.

Report only the status markers, first few new backtrace frames and
any remaining missing-library warning **count**. Do not upload
core memory, the manifest of full host paths, or unrelated log
content. A new, resolved caller would still require validated
source-module build IDs and logical evidence before claiming a
production shutdown fix.

This phase uses **no** new ROS process, no physical sensors, no
motor connection, no device mapping, no network and no live GDB
attachment.

## Strict invariant

M23Q is a **postmortem-only investigation**. No motion, Nav2, XRSLAM,
recorder, STM32 serial, real USB camera, fresh data capture or
production modifications. M23A's strictly-greater-than-250ms
Image-header-gap blocking rule and the first physical rosbag are
unchanged. A stack trace can locate a crash but cannot by itself
certify calibration, VIO or motion readiness.
