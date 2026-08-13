#!/usr/bin/env bash
# ============================================================================
# Copy PX4's flight logs out of Pegasus's temp rootfs before Isaac exits.
#
#   ./sim/save-ulog.sh [run-name]
#
# Pegasus launches PX4 with cwd=tempfile.TemporaryDirectory()
# (px4_launch_tool.py:44,63), a directory destroyed the moment Isaac's process
# exits. Nothing else in this repo ever copies the .ulg files out, so every
# flight before this script existed left no flight recorder data at all
# (ADR-0003). This MUST run while Isaac Sim is still up.
#
# `run-name` lets the ulog land in the same ./logs/<run-name>/ directory
# joystick-server.py prints at startup for its CSV, so the two files describing
# one flight sit side by side. Omit it and a fresh timestamp is used.
# ============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

die() { echo "save-ulog: $*" >&2; exit 1; }

PX4_BIN_MATCH="build/px4_sitl_default/bin/px4"
PX4_PID="$(pgrep -f "$PX4_BIN_MATCH" | head -n1 || true)"
[[ -n "$PX4_PID" ]] || die "no PX4 process found (looked for '$PX4_BIN_MATCH').
  This must run while Isaac Sim -- and the PX4 it launched -- are still up;
  Pegasus's rootfs is deleted the moment the process exits."

PX4_CWD="$(readlink -f "/proc/$PX4_PID/cwd" 2>/dev/null || true)"
[[ -n "$PX4_CWD" && -d "$PX4_CWD" ]] || die "could not resolve /proc/$PX4_PID/cwd for PX4 pid $PX4_PID."

RUN_NAME="${1:-$(date +%Y%m%d-%H%M%S)}"
DEST_DIR="$REPO_DIR/logs/$RUN_NAME"

mapfile -t ULOGS < <(find "$PX4_CWD/log" -type f -name '*.ulg' 2>/dev/null | sort)
[[ ${#ULOGS[@]} -gt 0 ]] || die "no .ulg files under $PX4_CWD/log -- is SDLOG_MODE set? \
(phase 0 sets it under --vision; a plain --no-vision run never enables logging here)"

mkdir -p "$DEST_DIR"
for f in "${ULOGS[@]}"; do
    cp "$f" "$DEST_DIR/"
    echo "save-ulog: copied $(basename "$f") ($(du -h "$f" | cut -f1)) -> $DEST_DIR/"
done

echo "save-ulog: PX4 pid $PX4_PID, rootfs $PX4_CWD"
echo "save-ulog: ${#ULOGS[@]} file(s) in $DEST_DIR"
