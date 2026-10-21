# M23AE — historical-core CPU-register attestation (offline only)

**Source-only diagnostic; no M23AE runtime, core inspection on operator host,
GDB, Podman, or ROS has been run.**

## Why this is the next discriminant

M23AD #124 finished the predetermined ABBA sequence with four accepted trials:
both control runs had child SIGSEGV -11 and both RTLD_NODELETE runs exited 0,
every time after full 900/900 real nonzero-D rectification and graph/QoS
verification. Together with M23AC #123: controls 3/3 crashing and NODELETE
3/3 clean in the component-factory path, all in the same immutable image.

Prior M23Q #111 already has **two** independently retained historical
no-device core exports and kernel NT_SIGINFO/NT_FILE/PT_LOAD verification:
host PID 46312, crash PC 0x00007fa7214bdf5b;
host PID 48349, crash PC 0x00007f56a4912f5b.
Both had SIGSEGV/SEGV_MAPERR with fault address equal to those addresses,
no mapping at the fault PC and a repeated page offset 0xf5b. GDB still had
an untrustworthy unwind after staging 93 selected dependencies.

M23AE does NOT pretend that GDB could identify a faulting DSO. It adds
an independent kernel-saved register check on **those same core files**:
per-thread ELF NT_PRSTATUS -> x86_64 saved RIP, RSP and thread identifier.
It requires **exactly one** saved thread's RIP to equal the historical crash
PC and the kernel NT_SIGINFO address, and proves that PC absent from both
saved NT_FILE and PT_LOAD ranges. It also checks whether the saved stack
pointer belongs to a PT_LOAD region, **not** what the stack contains.

We only read ELF headers and PT_NOTE register metadata, with the existing
bounded M23Q NT_SIGINFO/NT_FILE/PT_LOAD parser reused **unchanged**.
No debugger attach, disassembly, register-target memory access, stack
memory or library symbolization, full core output, live process, or altered
image. Nothing is installed into ROS or run inside a container.

## Gates and commands

1. Verify **exactly three** added M23AE files relative to M23AD HEAD
   and 8 offline synthetic unit tests:

   ```bash
   cd /tmp/leroy-m23ad
   git fetch origin feature/m23ae-historical-core-register-attestation
   git worktree add --detach /tmp/leroy-m23ae \
     origin/feature/m23ae-historical-core-register-attestation
   cd /tmp/leroy-m23ae
   git diff --name-status ae92eecfe5ba4d7fd4148941aff316e43ced3d4b HEAD
   PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ae_source_test.py
   ```

2. Dry-run: no core files opened:

   ```bash
   PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ae_saved_registers.py
   ```

   Expected `M23AE_STATUS=NO_CORE_ACCESS_UNLESS_EXPLICIT`.

3. Only if the existing two M23Q exported cores remain available,
   inspect their metadata once, read-only:

   ```bash
   PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ae_saved_registers.py --verify-existing
   ```

   It only accepts:
   - `~/.local/state/leroy-m23q/prior-core-46312/core.46312`
   - exactly one `~/.local/state/leroy-m23q/gdb-firstpass.*/core.48349`

   Both must be regular non-symlink, owner-private files, under M23Q
   storage, with correct ELF machine/notes and historical PC/signal.

4. On any absent core, unknown ELF layout, multiple candidate paths or
   mismatch, output `M23AE_BLOCKED=...`; stop and do not re-extract or
   rerun ROS. If both pass, report two
   `M23AE_CORE_RESULT_JSON` summaries and
   `M23AE_CROSS_CORE_RIP_AND_SIGINFO=VERIFIED_UNMAPPED_TARGET_BOTH`.

## Interpretation

A PASS independently attests that actual kernel-saved **CPU RIP**, rather
than just GDB's unwind or journal text, was the unmapped `si_addr` in
both prior cores. The repeatable RTLD_NODELETE sensitivity remains strong
evidence for a component/shared-object lifetime interaction, but neither
result identifies which library (if any) unmapped, nor whether a dangling
callback, use-after-free, corrupted indirect call, or finalizer caused it.

NT_FILE lists only mappings remaining when the core was made: it **cannot**
reveal an already unloaded DSO. M23AE **does not** claim a specific
faulting library, safe production fix, or causal proof from ABBA.

Current physical camera and original M23A >250ms gap acceptance, XRSLAM,
DepthART, IMU/STM32, recording, Nav2, motion, production, and any physical
robot runtime remain BLOCKED. All prior PRs are diagnostic drafts; M23Z
#120 remains paused. No new runtime tests are needed at this stage.
