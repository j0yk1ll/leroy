#!/usr/bin/env bash
# M23Q: offline host-GDB faulting-thread reinspection of previously
# extracted historical core 48349. Never attaches, starts ROS, or maps devices.
set -euo pipefail
umask 077

DIR="${HOME}/.local/state/leroy-m23q/gdb-firstpass.sXKTfoQL"
CORE="$DIR/core.48349"
SYSROOT="$DIR/sysroot"
EXE="$SYSROOT/opt/ros/jazzy/lib/rclcpp_components/component_container"
LOG="$DIR/gdb-thread89-reinspect.log"

for tool in gdb stat grep; do
    command -v "$tool" >/dev/null 2>&1 || {
        echo "M23Q_REINSPECT_BLOCKED=MISSING_TOOL:$tool"
        exit 2
    }
done

# Reuse only the previously verified private core/executable pair.
# Never substitute a mutable image tag, host Arch executable, or live PID.
[ -d "$DIR" ] && [ ! -L "$DIR" ] &&
[ -f "$CORE" ] && [ ! -L "$CORE" ] &&
[ -f "$EXE" ] && [ ! -L "$EXE" ] || {
    echo "M23Q_REINSPECT_BLOCKED=VERIFIED_CORE_OR_EXECUTABLE_MISSING"
    exit 2
}
case "$(stat -c '%a' "$DIR")" in
    700) ;;
    *) echo "M23Q_REINSPECT_BLOCKED=PRIVATE_DIRECTORY_MODE"; exit 2 ;;
esac
[ "$(stat -c '%u' "$CORE")" = "$(id -u)" ] || {
    echo "M23Q_REINSPECT_BLOCKED=CORE_NOT_OWNED_BY_OPERATOR"
    exit 2
}
[ ! -e "$LOG" ] || {
    echo "M23Q_REINSPECT_BLOCKED=LOG_ALREADY_EXISTS"
    exit 2
}

# Batch-only historical core; no run/continue/attach/network/memory dump.
set +e
gdb --nx --nh --quiet --batch \
    -ex 'set confirm off' \
    -ex 'set pagination off' \
    -ex 'set auto-load off' \
    -ex 'set debuginfod enabled off' \
    -ex 'set print elements 0' \
    -ex "set sysroot $SYSROOT" \
    -ex "file $EXE" \
    -ex "core-file $CORE" \
    -ex 'info threads' \
    -ex 'thread 1' \
    -ex 'info registers rip rsp rbp' \
    -ex 'info frame' \
    -ex 'bt 18' \
    -ex 'p $_siginfo' \
    -ex 'info sharedlibrary' \
    >"$LOG" 2>&1
status=$?
set -e
chmod 600 "$LOG"
echo "M23Q_REINSPECT_DEVICE_ACTION=NONE"
echo "M23Q_REINSPECT_CORE=HISTORICAL_HOST_PID_48349"
echo "M23Q_REINSPECT_GDB_EXIT_CODE=$status"
echo "M23Q_REINSPECT_PRIVATE_LOG=$LOG"
if [ "$status" -ne 0 ]; then
    echo "M23Q_REINSPECT_VERDICT=BLOCKED_GDB_COMMAND_FAILED"
    exit 2
fi
if ! grep -q 'Program terminated with signal SIGSEGV' "$LOG" ||
   ! grep -q 'LWP 89' "$LOG" ||
   ! grep -q '0x00007f56a4912f5b' "$LOG"; then
    echo "M23Q_REINSPECT_VERDICT=BLOCKED_FAULT_THREAD_NOT_CONFIRMED"
    exit 2
fi
echo "M23Q_REINSPECT_VERDICT=PRELIMINARY_FAULT_THREAD_CONFIRMED"
echo "M23Q_FAULTING_FUNCTION_IDENTIFIED=FALSE"
echo "M23Q_MODULE_BUILD_ID_VERIFIED=FALSE"
