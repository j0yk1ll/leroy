#!/usr/bin/env bash
# M23Q: host-only READ-ONLY inventory of already retained systemd core dumps.
# Never runs a container, debugger, ROS, hardware test or records any new core.
set -u
umask 077

if [ "$#" -gt 1 ]; then
    echo "Usage: bash m23q_host_coredump_inventory.sh [HOST_PID_FROM_LIST]" >&2
    exit 2
fi
if ! command -v coredumpctl >/dev/null 2>&1; then
    echo "M23Q_BLOCKED=HOST_COREDUMPCTL_MISSING" >&2
    exit 2
fi
SINCE="${M23Q_SINCE:-2026-10-19 00:00:00}"
ROOT="${HOME}/.local/state/leroy-m23q"
mkdir -p -m 700 "$ROOT" || exit 2
RUN_DIR="$(mktemp -d "$ROOT/inventory.XXXXXXXX")" || exit 2
chmod 700 "$RUN_DIR"
echo "M23Q_HOST_CREDUMP_LOG_DIR=$RUN_DIR"
echo "M23Q_CORE_SINCE=$SINCE"
echo "M23Q_NO_DEVICE_ACTION=TRUE"

# Journal metadata only. Treat no matched historical core as a valid
# inventory outcome, NOT evidence that there was no actual segfault.
coredumpctl --no-pager --since "$SINCE" list \
    >"$RUN_DIR/core-list.txt" 2>&1
LIST_CODE=$?
echo "M23Q_CORE_LIST_EXIT_CODE=$LIST_CODE"

# Linux task comm may truncate component_container to component_conta.
# The executable path can also appear as .../rclcpp_components/...
if grep -Ei 'component_conta|rclcpp_components' \
      "$RUN_DIR/core-list.txt" >"$RUN_DIR/rectifier-candidates.txt"; then
    echo "M23Q_CANDIDATES_BEGIN"
    cat "$RUN_DIR/rectifier-candidates.txt"
    echo "M23Q_CANDIDATES_END"
else
    echo "M23Q_CANDIDATES=NONE_VISIBLE"
    echo "M23Q_CORE_RETENTION=NOT_YET_VERIFIED"
fi
echo "M23Q_CORE_LIST_FILE=$RUN_DIR/core-list.txt"

if [ "$#" -eq 0 ]; then
    echo "M23Q_NEXT=SELECT_HOST_PID_FROM_CANDIDATES"
    exit 0
fi

HOST_PID="$1"
case "$HOST_PID" in
    ''|*[!0-9]*)
        echo "M23Q_BLOCKED=HOST_PID_NOT_NUMERIC" >&2
        exit 2 ;;
esac

coredumpctl --no-pager info "$HOST_PID" \
    >"$RUN_DIR/core-info.txt" 2>&1
INFO_CODE=$?
echo "M23Q_INFO_EXIT_CODE=$INFO_CODE"
if [ "$INFO_CODE" -ne 0 ]; then
    echo "M23Q_BLOCKED=CORE_METADATA_NOT_ACCESSIBLE"
    echo "M23Q_CORE_INFO_FILE=$RUN_DIR/core-info.txt"
    exit 2
fi

# Never display an unrelated process's metadata if a wrong PID is chosen.
if ! grep -Eq 'rclcpp_components/component_container|component_container' \
       "$RUN_DIR/core-info.txt"; then
    echo "M23Q_BLOCKED=CORE_PROCESS_IDENTITY_NOT_VERIFIED"
    exit 2
fi

echo "M23Q_SELECTED_HOST_PID=$HOST_PID"
echo "M23Q_CORE_INFO_FILE=$RUN_DIR/core-info.txt"
echo "M23Q_INFO_RELEVANT_BEGIN"
grep -E '^( *Timestamp:| *PID:| *Signal:| *Executable:| *Command Line:| *Storage:| *Size on Disk:| *Stack trace:| *[0-9]+[[:space:]])' \
    "$RUN_DIR/core-info.txt" | head -n 100 || true
echo "M23Q_INFO_RELEVANT_END"
echo "M23Q_NEXT=CLASSIFY_CORE_PRESENT_JOURNAL_MISSING_OR_TRUNCATED"
