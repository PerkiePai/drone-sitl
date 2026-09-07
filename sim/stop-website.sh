#!/usr/bin/env bash
# ============================================================================
# stop-website.sh -- shut down everything run-website.sh started.
#
#   ./sim/stop-website.sh
#
# Stops, in order:
#   agent_runner.py     any uploaded-script child
#   joystick-server.py  the web page on :8090 -- and joystick-server-descriptive.py
#                       (--descriptive, :8091) if that one was started instead
#   Isaac Sim + PX4     ONLY the instance running this repo's sim/bootstrap.py
#
# It matches this repo's own process signatures, so an unrelated Isaac / SITL
# on the box (e.g. a `screen -S friend` session) is left running.
#
# SIGTERM first, then SIGKILL after ~10s for anything that ignores it.
# ============================================================================
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

say() { echo "stop-website: $*"; }

# <label> <pgrep -f pattern>
stop_pat() {
    local label="$1" pat="$2" pids
    pids="$(pgrep -f "$pat" 2>/dev/null || true)"
    if [[ -z "$pids" ]]; then
        say "$label: not running"
        return 0
    fi
    say "$label: SIGTERM -> $(echo "$pids" | tr '\n' ' ')"
    kill $pids 2>/dev/null || true
    for _ in $(seq 1 10); do
        pgrep -f "$pat" >/dev/null 2>&1 || { say "$label: stopped"; return 0; }
        sleep 1
    done
    pids="$(pgrep -f "$pat" 2>/dev/null || true)"
    if [[ -n "$pids" ]]; then
        say "$label: still up, SIGKILL -> $(echo "$pids" | tr '\n' ' ')"
        kill -9 $pids 2>/dev/null || true
        sleep 1
    fi
    if pgrep -f "$pat" >/dev/null 2>&1; then
        say "$label: WARNING still running ($(pgrep -f "$pat" | tr '\n' ' '))"
    else
        say "$label: stopped"
    fi
}

stop_pat "agent_runner"    "$REPO_DIR/agent_runner\.py"
stop_pat "joystick-server" "python .*joystick-server(-descriptive)?\.py"
stop_pat "isaac sim + px4" "$REPO_DIR/sim/bootstrap\.py"
stop_pat "launch-sitl"     "$REPO_DIR/sim/launch-sitl\.sh"

# PX4 SITL runs as a child of the kit process and exits with it -- nothing to
# sweep here as long as "isaac sim + px4" above actually stopped.

say "done"
