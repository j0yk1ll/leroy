#!/usr/bin/env bash
# M23Q: complete a partial private sysroot with *only* missing
# shared-library files reported by host GDB for saved core PID 48349.
# Never starts image_proc, a running container, or a device process.
set -euo pipefail
umask 077

EXPECTED_IMAGE_ID="6eeb8ca08525d451dc0f216f097c1df7e300444d8c8ae551fde576d64a1c48f1"
M23P_COMMIT="6c300889a2f472c4f8b1bf568334a1254b6d7a03"
DIR="${HOME}/.local/state/leroy-m23q/gdb-firstpass.sXKTfoQL"
LOG="$DIR/gdb-firstpass.log"
CORE="$DIR/core.48349"
SYSROOT="$DIR/sysroot"
EXE="$SYSROOT/opt/ros/jazzy/lib/rclcpp_components/component_container"
MANIFEST="$DIR/gdb-missing-library-manifest.txt"
RECHECK="$DIR/gdb-with-image-libs.log"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

for bin in podman gdb python3 git cmp stat wc; do
    command -v "$bin" >/dev/null || {
        echo "M23Q_DEPS_BLOCKED=MISSING_TOOL:$bin"
        exit 2
    }
done
[ -d "$DIR" ] && [ ! -L "$DIR" ] &&
[ -f "$CORE" ] && [ ! -L "$CORE" ] &&
[ -f "$LOG" ] && [ ! -L "$LOG" ] &&
[ -f "$EXE" ] && [ ! -L "$EXE" ] &&
[ -f "$DIR/m23p_runner_from_image.py" ] &&
[ -f "$DIR/m23p_graph_from_image.py" ] || {
    echo "M23Q_DEPS_BLOCKED=SAVED_PRIVATE_EVIDENCE_MISSING"
    exit 2
}
[ "$(stat -c '%a' "$DIR")" = 700 ] || {
    echo "M23Q_DEPS_BLOCKED=PRIVATE_DIRECTORY_MODE"
    exit 2
}
[ "$(stat -c '%u' "$CORE")" = "$(id -u)" ] || {
    echo "M23Q_DEPS_BLOCKED=CORE_NOT_OWNED"
    exit 2
}
[ ! -e "$MANIFEST" ] && [ ! -e "$RECHECK" ] || {
    echo "M23Q_DEPS_BLOCKED=REFUSING_TO_OVERWRITE_LOGS"
    exit 2
}
git show "$M23P_COMMIT:ros/m23p_no_device_rectifier_shutdown.py" |
    cmp -s - "$DIR/m23p_runner_from_image.py" || {
    echo "M23Q_DEPS_BLOCKED=M23P_RUNNER_PROVENANCE"
    exit 2
}
git show "$M23P_COMMIT:ros/m23p_qos_graph.py" |
    cmp -s - "$DIR/m23p_graph_from_image.py" || {
    echo "M23Q_DEPS_BLOCKED=M23P_GRAPH_PROVENANCE"
    exit 2
}
actual="$(podman image inspect "$EXPECTED_IMAGE_ID" --format '{{.Id}}' 2>/dev/null)" || {
    echo "M23Q_DEPS_BLOCKED=IMMUTABLE_IMAGE_NOT_AVAILABLE"
    exit 2
}
[ "${actual#sha256:}" = "$EXPECTED_IMAGE_ID" ] || {
    echo "M23Q_DEPS_BLOCKED=IMMUTABLE_IMAGE_MISMATCH"
    exit 2
}

# Generate bounded allowlist from saved GDB warning text only.
python3 -B "$SCRIPT_DIR/m23q_missing_core_libraries.py" "$LOG" >"$MANIFEST" || {
    rm -f "$MANIFEST"
    echo "M23Q_DEPS_BLOCKED=LIBRARY_ALLOWLIST_FAILED"
    exit 2
}
chmod 600 "$MANIFEST"
count="$(wc -l <"$MANIFEST")"
[ "$count" -gt 0 ] && [ "$count" -le 400 ] || {
    echo "M23Q_DEPS_BLOCKED=LIBRARY_COUNT_OUT_OF_RANGE"
    exit 2
}

CID=""
cleanup() {
    if [ -n "$CID" ]; then
        podman rm "$CID" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT

CID="$(podman create --pull=never --network none --ipc private --pid private \
      --cap-drop all --security-opt no-new-privileges \
      --entrypoint /bin/true "$EXPECTED_IMAGE_ID")" || {
    echo "M23Q_DEPS_BLOCKED=STOPPED_IMAGE_CREATE_FAILED"
    exit 2
}

copied=0
failed=0
skipped=0
while IFS= read -r mapped_file; do
    case "$mapped_file" in
        /opt/ros/jazzy/lib/*.so|/opt/ros/jazzy/lib/*.so.*|\
        /usr/lib/x86_64-linux-gnu/*.so|/usr/lib/x86_64-linux-gnu/*.so.*|\
        /lib/x86_64-linux-gnu/*.so|/lib/x86_64-linux-gnu/*.so.*)
            ;;
        *)
            echo "M23Q_DEPS_BLOCKED=UNEXPECTED_MANIFEST_PATH"
            exit 2 ;;
    esac
    target="$SYSROOT$mapped_file"
    # Do not accept symlinks here: an absolute link can resolve into
    # the Arch HOST filesystem and create misleading GDB attribution.
    if [ -L "$target" ]; then
        failed=$((failed + 1))
        continue
    fi
    if [ -f "$target" ]; then
        skipped=$((skipped + 1))
        continue
    fi
    mkdir -p -- "$(dirname -- "$target")"
    # Source is exact path inside a STOPPED container. Refuse any directory,
    # symlink lacking a staged target, or failed copy; do not traverse /dev.
    if podman cp "$CID:$mapped_file" "$target" >/dev/null 2>&1 &&
       [ -f "$target" ] && [ ! -L "$target" ]; then
        chmod u+rw -- "$target" || true
        copied=$((copied + 1))
    else
        failed=$((failed + 1))
    fi
done <"$MANIFEST"
cleanup
CID=""

echo "M23Q_DEPS_DEVICE_ACTION=NONE"
echo "M23Q_DEPS_PINNED_IMAGE_VERIFIED=TRUE"
echo "M23Q_DEPS_WARNED_LIBRARIES=$count"
echo "M23Q_DEPS_COPIED=$copied"
echo "M23Q_DEPS_ALREADY_PRESENT=$skipped"
echo "M23Q_DEPS_UNRESOLVED=$failed"
if [ "$failed" -eq 0 ]; then
    echo "M23Q_DEPS_STAGE_STATUS=COMPLETE_FOR_SELECTED_WARNINGS"
else
    echo "M23Q_DEPS_STAGE_STATUS=PARTIAL"
fi

# Read historical core again. Never start/continue/attach a process.
set +e
gdb --nx --nh --quiet --batch \
    -ex 'set confirm off' \
    -ex 'set pagination off' \
    -ex 'set auto-load off' \
    -ex 'set debuginfod enabled off' \
    -ex "set sysroot $SYSROOT" \
    -ex "file $EXE" \
    -ex "core-file $CORE" \
    -ex 'thread 1' \
    -ex 'info registers rip rsp rbp' \
    -ex 'info frame' \
    -ex 'bt 22' \
    -ex 'info sharedlibrary' \
    >"$RECHECK" 2>&1
gdb_status=$?
set -e
chmod 600 "$RECHECK"
echo "M23Q_DEPS_GDB_EXIT_CODE=$gdb_status"
echo "M23Q_DEPS_PRIVATE_GDB_LOG=$RECHECK"
echo "M23Q_DEPS_POSTMORTEM_ONLY=TRUE"
echo "M23Q_DEPS_FAULTING_LIBRARY_IDENTIFIED=FALSE"
echo "M23Q_DEPS_CAUSE_VERIFIED=FALSE"
[ "$gdb_status" -eq 0 ] || {
    echo "M23Q_DEPS_VERDICT=BLOCKED_GDB_ERROR"
    exit 2
}
echo "M23Q_DEPS_VERDICT=PRELIMINARY_ONLY"
