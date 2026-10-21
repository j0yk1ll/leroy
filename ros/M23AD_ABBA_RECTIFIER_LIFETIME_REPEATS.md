# M23AD — preregistered same-image ABBA replication

**Source-only diagnostic protocol; no M23AD runtime has been performed.**
Stacked on M23AC draft PR #123, and retains the accepted existing M23AC
image, unchanged ROS code, stock image_proc, M23AA strict 900-frame
publisher/observer, graph/QoS audit and real child exit classifier.

## Accepted evidence

M23AB directly linked RectifyNode was clean in three 900/900 trials.
M23AC's component control crashed (exit -11), while the component manager
with verified RTLD_NODELETE hook was clean (exit 0), both with 900/900 and
the same immutable image
`a3ca7cef00be950a52da0685407f54e12bc2d14b7a0a327e745c3e5adc9392f6`.
Original M23AC files `~/.local/state/leroy-m23ac/control-02.log`,
`nodelete-01.log`, and `matched-image-id-02.txt` are preserved.

## Question and fixed-size sequence

Will independent synthetic 900-frame shutdown trials consistently yield
SIGSEGV for component-container `control` and clean exit for `nodelete`,
even after reversing trial order?

Four **serial** trials, **without replacement or restart**:
1. control: require 900/900 + SIGSEGV child -11, parent status 2.
2. nodelete: require 900/900 + clean child 0, parent status 0.
3. nodelete: require 900/900 + clean child 0, parent status 0.
4. control: require 900/900 + SIGSEGV child -11, parent status 2.

Order **A B B A**, predeclared, no tuning in between. Stop on first
timeout, missing/mismatched hook, incomplete data, alternative exit,
unexpected environment or image difference. The resulting observations are
a limited replication screen, not an estimate of crash probability.

## Safety/provenance gates

A. Fetch M23AD, verify exact three additive files relative to M23AC parent
   and run six pure-host offline Python unit tests. They import the runner,
   test accept/reject contracts, and never spawn Podman or ROS.
B. Runner must be explicitly passed `--execute`. It checks nonroot user,
   absence of global LD_PRELOAD, exact pinned image ID, matching retained
   M23AC ID record and known parent tag, required synthetic image defaults,
   accepted `control-02.log` and `nodelete-01.log`.
C. It creates one atomic permanent directory lock and a 0600 plan in
   `~/.local/state/leroy-m23ad/`; refuses any preexisting plan/lock/
   trial log. It does not resume after interruption or failure. Trial logs
   `trial-01-control.log`, `trial-02-nodelete.log`,
   `trial-03-nodelete.log`, `trial-04-control.log` open exclusively.
D. Each run invokes rootless Podman as same pinned image ID with
   `--pull=never --network=none --ipc=private --pid=private
   --cap-drop=all --security-opt=no-new-privileges --read-only
   --tmpfs /tmp:rw,nosuid,nodev,size=128m --pids-limit=128
   --memory=2g`; outer timeout 120 seconds. Only `M23AC_ARM` changes.
   No device mappings, privileged mounts, host network or robot process.
E. Each result must provide exactly one M23AA and M23AC JSON marker,
   exact expected policy and factory hook, graph_verified, 900 raw/rect/info,
   inherited strict stamp/gap/drain validation, proper child/parent exits,
   unmodified stock rectifier and explicit production BLOCKED metadata.
   Any mismatch halts the sequence.

## Operator commands

```bash
# In the existing Leroy repository checkout:
git fetch origin feature/m23ad-abba-rectifier-lifetime-repeats
git worktree add --detach /tmp/leroy-m23ad \
  origin/feature/m23ad-abba-rectifier-lifetime-repeats
cd /tmp/leroy-m23ad
git diff --name-status 76354f49de490303808bf092d3a6a0561e6f1809 HEAD
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ad_source_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ad_repeat.py
```

The final non-executing command prints the fixed order and
`M23AD_STATUS=NO_RUNTIME_NOT_AUTHORIZED`. Only if source tests/paths and
image provenance are verified should operator explicitly run:

```bash
cd /tmp/leroy-m23ad
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ad_repeat.py --execute
```

Expected after all four accepted outcomes:
`M23AD_ABBA_VERDICT=FOUR_OF_FOUR_MATCHED`.
Each individual result line `M23AD_TRIAL_RESULT=...` includes its
policy, child exit, frame counts and full inherited verdict. On failure
`M23AD_STOP=...` and no subsequent trial starts. Preserve logs.

## Interpretation and exclusions

ABBA concordance supports repeatable sensitivity to the NODELETE
intervention, but does **not** prove the rectifier was actually unmapped
on a control crash or distinguish destructor, callback, class_loader,
library finalization or transitive dependency fault. Both arms execute
the existing after-factory dlopen/no-load/dlclose diagnostic hook.
No root cause or production repair is established by four trials.

No new image, no Docker build, no physical cameras, motor control,
DepthART/XRSLAM, STM32/IMU, recording, Nav2, or production acceptance.
M23Z #120 remains paused. Existing PRs remain draft/unmerged.
