#!/usr/bin/env bash
# M23Q: host GDB FIRST PASS of a confirmed historical M23P saved core.
# Never starts ROS, image_proc, any device, or a running container.
set -euo pipefail
umask 077

HOST_PID="48349"
IMAGE="localhost/leroy-sensors:m23p-rectifier-shutdown"
EXPECTED_IMAGE_ID="8c75cdd760b9aacf65881e12317ec47b6189391356c6f4fa499e6d436fc6a0fa"

for bin in coredumpctl podman gdb; do
    command -v "$bin" >/dev/null || {
        echo "M23Q_BLOCKED=MISSING_DEPENDENCY:$bin" >&2
        exit 2
    }
done

BASE="${HOME}/.local/state/leroy-m23q"
mkdir -p -m 700 "$BASE"
chmod 700 "$BASE"
RUN_DIR="$(mktemp -d "$BASE/gdb-firstpass.XXXXXXXX")"
chmod 700 "$RUN_DIR"
echo "M23Q_POSTMORTEM_DIR=$RUN_DIR"
echo "M23Q_DEVICE_ACTION=NONE"
echo "M23Q_SELECTED_HOST_PID=$HOST_PID"

coredumpctl --no-pager info "$HOST_PID" >"$RUN_DIR/metadata.txt" 2>&1 || {
    echo "M23Q_BLOCKED=COREDUMP_METADATA_INACCESSIBLE"
    exit 2
}
if ! grep -Eq \
     'Command Line: .*/rclcpp_components/component_container.*__node:=m23p_image_proc_container' \
     "$RUN_DIR/metadata.txt"; then
    echo "M23Q_BLOCKED=CORE_PROCESS_IDENTITY_MISMATCH"
    exit 2
fi
if ! grep -Eq 'Signal: 11 \(SEGV\)' "$RUN_DIR/metadata.txt"; then
    echo "M23Q_BLOCKED=CORE_SIGNAL_NOT_SIGSEGV"
    exit 2
fi
if ! grep -Eq 'Storage: .* \(present\)' "$RUN_DIR/metadata.txt"; then
    echo "M23Q_BLOCKED=CORE_NOT_PRESENT"
    exit 2
fi

IMAGE_ID="$(podman image inspect "$IMAGE" --format '{{.Id}}' 2>/dev/null)" || {
    echo "M23Q_BLOCKED=EXACT_LOCAL_IMAGE_MISSING"
    exit 2
}
if [ "${IMAGE_ID#sha256:}" != "$EXPECTED_IMAGE_ID" ]; then
    echo "M23Q_BLOCKED=LOCAL_IMAGE_ID_CHANGED"
    echo "M23Q_IMAGE_ID=$IMAGE_ID"
    exit 2
fi
echo "M23Q_EXACT_IMAGE_ID_VERIFIED=TRUE"

CORE="$RUN_DIR/core.48349"
coredumpctl --no-pager --output="$CORE" dump "$HOST_PID" \
    >"$RUN_DIR/core-extraction.log" 2>&1 || {
    echo "M23Q_BLOCKED=CORE_EXTRACTION_FAILED"
    exit 2
}
if [ ! -s "$CORE" ]; then
    echo "M23Q_BLOCKED=EXTRACTED_CORE_EMPTY"
    exit 2
fi
chmod 600 "$CORE"
echo "M23Q_CORE_EXTRACTION=SUCCESS"
echo "M23Q_CORE_BYTES=$(wc -c <"$CORE")"

# Create only stopped container metadata, never start it.
CID=""
cleanup() {
    if [ -n "$CID" ]; then
        podman rm "$CID" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT
CID="$(podman create --pull=never --network none --ipc private --pid private \
      --cap-drop all --security-opt no-new-privileges \
      --entrypoint /bin/true "$IMAGE")" || {
    echo "M23Q_BLOCKED=STOPPED_IMAGE_STAGING_FAILED"
    exit 2
}

SYSROOT="$RUN_DIR/sysroot"
EXEPATH="/opt/ros/jazzy/lib/rclcpp_components/component_container"
mkdir -p "$SYSROOT/opt/ros/jazzy/lib/rclcpp_components"
podman cp "$CID:$EXEPATH" "$SYSROOT/$EXEPATH" || {
    echo "M23Q_BLOCKED=MATCHING_EXECUTABLE_COPY_FAILED"
    exit 2
}
# This is a minimal sysroot. Further exact loaded modules may be needed.
podman cp "$CID:/opt/ros/jazzy/lib/librectify.so" \
    "$SYSROOT/opt/ros/jazzy/lib/librectify.so" || {
    echo "M23Q_BLOCKED=MATCHING_RECTIFIER_COPY_FAILED"
    exit 2
}
cleanup
CID=""
echo "M23Q_STOPPED_IMAGE_FILES_STAGED=TRUE"

# Use host GDB for a postmortem read only; never attach to a live process.
# Avoid Arch host library misattribution: GDB's sysroot is private.
set +e
gdb --nx --quiet --batch \
    -ex 'set confirm off' \
    -ex 'set pagination off' \
    -ex 'set debuginfod enabled off' \
    -ex 'set auto-load off' \
    -ex "set sysroot $SYSROOT" \
    -ex "file $SYSROOT/$EXEPATH" \
    -ex "core-file $CORE" \
    -ex 'info threads' \
    -ex 'bt 30' \
    -ex 'info symbol $pc' \
    -ex 'info sharedlibrary' \
    -ex 'info proc mappings' \
    >"$RUN_DIR/gdb-firstpass.log" 2>&1
GDB_STATUS=$?
set -e

echo "M23Q_GDB_FIRSTPASS_EXIT_CODE=$GDB_STATUS"
echo "M23Q_GDB_FIRSTPASS_LOG=$RUN_DIR/gdb-firstpass.log"
echo "M23Q_POSTMORTEM_CLASSIFICATION=PRELIMINARY_SYMBOLS_UNVERIFIED"
echo "M23Q_NEXT=INSPECT_FAULT_ADDRESS_AND_SHARED_LIBRARY_MAPPINGS"
echo "M23Q_NO_CORE_UPLOAD=TRUE"
