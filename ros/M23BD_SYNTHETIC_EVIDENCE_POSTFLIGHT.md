# M23BD — read-only two-case synthetic evidence postflight and manifest

**SOURCE ONLY. No ROS, Podman, camera, MCU/serial, robot, recording or hardware use.**
Stacked on M23BC [draft PR #145](https://github.com/j0yk1ll/leroy/pull/145).

## Grounded upstream evidence (operator-reported)

M23BC parent image remains
`9f5c85981bf0879e27462ded52eda45beb74ba0e938714eaed5b794e4cc6cf90`.
Operator reports successfully completed Stage C1 and C2 with two different
private host-bound directories. Both files survived Podman disposable
guest removal, including process exit 73 after explicit flush/fsync.

Normal case:
- `~/.local/state/leroy-m23bc/persistence-clean-01.log`
  SHA256 `6c6febf6506437952cb0650984ccfe80ddb7fe0d2322dc7560d810104eadea95`;
- `persistence-clean-01/m23bb-camera-launch.log`
  SHA256 `2b3375ecb148ed23e3b051d97ac634f4bac3cd12c56d2777d355f632cdb9ed86`;
- `persistence-clean-01/m23bb-counter.log`
  SHA256 `0226d46be4834d2bd142d92b3abdef9caf20f6e567258c52f516148b403b4144`.

Abnormal case:
- `~/.local/state/leroy-m23bc/persistence-abnormal-01.log`
  SHA256 `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`;
- `persistence-abnormal-01/m23bb-camera-launch.log`
  SHA256 `2b3375ecb148ed23e3b051d97ac634f4bac3cd12c56d2777d355f632cdb9ed86`;
- `persistence-abnormal-01/m23bb-counter.log`
  SHA256 `0226d46be4834d2bd142d92b3abdef9caf20f6e567258c52f516148b403b4144`.

Operator observed `M23BC_CLEAN_EXIT=0`,
`M23BC_CLEAN_HOST_PERSISTENCE=PASS`,
`M23BC_ABNORMAL_EXIT=73`,
`M23BC_ABNORMAL_HOST_PERSISTENCE=PASS`.
The C2 process stdout/stderr is empty BY DESIGN due to `os._exit(73)`.
The empty file does NOT independently prove that exit status; the exit code
is an operator declaration based on the original foreground shell run.

## M23BD scope

`ros/m23bd_synthetic_evidence_postflight.py` is a stdlib-only
**read-only** checker, inert by default. After explicit file arguments
it validates:
- two distinct sibling evidence directories with exactly the
  expected case names, 0700 mode and current user ownership;
- precisely three entries per case: host-bind sentinel, camera
  synthetic text and subscriber synthetic text, no unexpected entries;
- regular, non-symlinked, singly linked 0600 files owned by the user;
- exact sentinel and two original M23BC synthetic byte payloads,
  max 4 KiB per file, stable observed size/mtime while reading;
- exact C1 normal process log and expected empty C2 process log,
  owner-only and non-symlink, in the expected parent directory;
- operator-DECLARED exit codes 0 and 73, respectively;
- equal synthetic hashes across both cases.

It emits a deterministic read-only JSON digest manifest with
`OFFLINE_SYNTHETIC_EVIDENCE_POSTFLIGHT_PASS`; the manifest clearly says
`exit_codes_independently_proven_by_files=false`,
`actual_camera_trace_completeness_checked=false`,
`physical_camera_retest_authorized=false`,
`production_acceptance=BLOCKED`.

`ros/m23bd_synthetic_evidence_postflight_test.py` contains 26
synthetic positive/negative unit tests including exact source
payload compatibility, missing/extra files, symlink/hardlink, mode
and content corruption, altered/empty logs and no-write behavior.

**This check never interprets M23AX/PUBLISH or M23K/SAMPLE camera
logs and cannot accept any real-world camera recording.** That
future work requires separate offline schema/completeness review,
origin/epoch correlation, bounded text storage, and later
independent physical authorization. The M23AU repeated observed
RAW timestamp remains unexplained. No source, original image or
capture hardware is changed by M23BD.

## Gate A — only 26 host source tests, no device

Use a fresh detached source checkout pinned to the PR SHA and
include these files:

- `ros/m23bd_synthetic_evidence_postflight.py`
- `ros/m23bd_synthetic_evidence_postflight_test.py`
- `ros/m23bc_synthetic_persistence.py` (payload reference only)
- `ros/m23bb_host_evidence_adapter.py` (imported by the M23BC reference)

Command after checkout:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bd_synthetic_evidence_postflight_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23bd_synthetic_evidence_postflight.py
```

Expected `Ran 26 tests`, `OK`, and
`M23BD_STATUS=INERT_READ_ONLY_NO_ROS_NO_DEVICE`.
Operator has **not yet run** this gate.

## Gate B — read existing M23BC host logs only after A passes

Run the same Python file with all six explicit arguments, pointing
only to the two preserved synthetic C1/C2 directories, their
process stdout/stderr logs, and operator-reported 0 and 73 exits.
Do not create replacement evidence or rerun Podman.
Capture stdout in a NEW owner-only manifest file with noclobber,
hash it, and preserve it alongside earlier immutable logs.
Require exactly one `M23BD_POSTFLIGHT_VERDICT=PASS_SYNTHETIC_ONLY`
and exit 0. Any missing/mismatched source evidence BLOCKS.

Even if PASS, M23BD proves only that the synthetic evidence files
**present now** have expected bytes, size and private permissions;
it does not verify historical process events independently or
prove a specific ROS frame was published/delivered. In particular
nothing about the original M23AU raw header monotonicity failure
or clean physical camera shutdown is waived.

M23BD_CAMERA_RETEST_AUTHORIZED=FALSE
M23BD_PRODUCTION_ACCEPTANCE=BLOCKED
