#!/usr/bin/env bash
# M23Q: export ONLY already-retained historical PID 46312 core.
# No ROS, Podman, GDB, physical devices, network or elevated privileges.
set -euo pipefail
umask 077

BASE="${HOME}/.local/state/leroy-m23q"
DIR="$BASE/prior-core-46312"
CORE="$DIR/core.46312"
META="$DIR/core-info.txt"
PID=46312
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

for cmd in coredumpctl grep python3; do
    command -v "$cmd" >/dev/null || {
        echo "M23Q_PRIOR_BLOCKED=MISSING_DEPENDENCY:$cmd"
        exit 2
    }
done
[ -d "$BASE" ] && [ ! -L "$BASE" ] || {
    echo "M23Q_PRIOR_BLOCKED=PRIVATE_BASE_MISSING"
    exit 2
}
# Require new private output directory and refuse overwrites.
if [ -e "$DIR" ] || [ -L "$DIR" ]; then
    echo "M23Q_PRIOR_BLOCKED=EXISTING_OUTPUT_DIRECTORY"
    exit 2
fi
mkdir -m 700 -- "$DIR"
echo "M23Q_PRIOR_DEVICE_ACTION=NONE"
echo "M23Q_PRIOR_SELECTED_HOST_PID=46312"

coredumpctl --no-pager info "$PID" >"$META" 2>&1 || {
    echo "M23Q_PRIOR_BLOCKED=JOURNAL_INFO_UNAVAILABLE"
    exit 2
}
if ! grep -Eq '^[[:space:]]*PID: 46312 \(component_conta\)' "$META" ||
   ! grep -Eq '^[[:space:]]*Signal: 11 \(SEGV\) si_code: SEGV_MAPERR' "$META" ||
   ! grep -Eq '^[[:space:]]*Timestamp: .*2026-10-19 16:11:2[45] CEST' "$META" ||
   ! grep -Eq '^[[:space:]]*Command Line: /opt/ros/jazzy/lib/rclcpp_components/component_container .*__node:=m23p_image_proc_container' "$META" ||
   ! grep -Eq '^[[:space:]]*Storage: .* \(present\)' "$META"; then
    echo "M23Q_PRIOR_BLOCKED=CORE_IDENTITY_OR_STATUS_MISMATCH"
    exit 2
fi
echo "M23Q_PRIOR_JOURNAL_IDENTITY_VERIFIED=TRUE"

# Export the saved core to private storage; never attach or launch a process.
coredumpctl --no-pager --output="$CORE" dump "$PID" \
    >"$DIR/core-extraction.log" 2>&1 || {
    echo "M23Q_PRIOR_BLOCKED=HISTORICAL_CORE_EXPORT_FAILED"
    exit 2
}
[ -s "$CORE" ] && [ ! -L "$CORE" ] || {
    echo "M23Q_PRIOR_BLOCKED=EMPTY_OR_SYMLINK_CORE"
    exit 2
}
chmod 600 -- "$CORE"
echo "M23Q_PRIOR_CORE_EXTRACTION=SUCCESS"
echo "M23Q_PRIOR_CORE_BYTES=$(wc -c <"$CORE")"

PYTHONDONTWRITEBYTECODE=1 python3 -u \
    "$SCRIPT_DIR/m23q_compare_prior_core.py" "$CORE"
