#!/usr/bin/env bash
# ============================================================================
# run-website.sh -- one command from nothing to a flyable drone web page.
#
#   ./sim/run-website.sh
#
# Idempotent. Looks at what is already running and starts ONLY what is missing:
#
#   Isaac Sim + PX4 SITL   sim/launch-sitl.sh   -> logs/launch-sitl.console
#   joystick-server.py     the web page :8090   -> logs/joystick-server.console
#
# Run it twice and a component that is already live is left untouched. This is
# the script behind "run website" -- the full operator handbook is RUN-WEBSITE.md.
#
# Env:
#   CONDA_ENV       conda env for joystick-server.py           (default: drone)
#   ISAAC_WAIT_S    how long to wait for the sim to come up     (default: 360)
#   SKIP_SIM=1      only (re)start the web server, never Isaac
#   Anything sim/launch-sitl.sh honours (SITE=, HEADING_DEG=, ...) is passed
#   straight through.
# ============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p logs

CONDA_ENV="${CONDA_ENV:-drone}"
ISAAC_WAIT_S="${ISAAC_WAIT_S:-360}"

say() { echo "run-website: $*"; }

# The MJPEG camera server is the last thing drone_setup_px4_cesium.py starts, so
# a 200 here means the whole Isaac -> Pegasus -> PX4 chain is live.
isaac_up()      { curl -sf -o /dev/null --max-time 3 http://127.0.0.1:8080/detect; }
# Match OUR bootstrap specifically -- this box sometimes runs a second, unrelated
# SITL launcher, and "any kit process" would make us wait on theirs forever.
isaac_running() { pgrep -f "$REPO_DIR/sim/bootstrap.py" >/dev/null 2>&1; }
web_up()        { curl -sf -o /dev/null --max-time 3 http://127.0.0.1:8090/; }

# --- Isaac Sim + PX4 -------------------------------------------------------
if [[ "${SKIP_SIM:-0}" == "1" ]]; then
    say "SKIP_SIM=1 -- not touching Isaac Sim"
elif isaac_up; then
    say "Isaac Sim + PX4 already up (camera on :8080 answering)"
elif isaac_running; then
    say "Isaac Sim process is already running -- waiting for it to finish booting"
else
    say "starting Isaac Sim  (sim/launch-sitl.sh)"
    nohup ./sim/launch-sitl.sh "$@" > logs/launch-sitl.console 2>&1 &
    disown || true
    say "  pid $!   log: logs/launch-sitl.console"
fi

if [[ "${SKIP_SIM:-0}" != "1" ]] && ! isaac_up; then
    say "  waiting up to ${ISAAC_WAIT_S}s for the camera server on :8080 (first boot is 2-5 min)"
    deadline=$(( SECONDS + ISAAC_WAIT_S ))
    while (( SECONDS < deadline )); do
        if isaac_up; then break; fi
        if ! isaac_running; then
            say "  WARNING Isaac Sim exited before coming up -- see logs/launch-sitl.console" >&2
            break
        fi
        sleep 3
    done
    if isaac_up; then
        say "Isaac Sim + PX4 up"
    else
        say "WARNING sim not confirmed. The page will still load; 'link' stays red" >&2
        say "  until PX4 answers. Common cause: another process starving Isaac's" >&2
        say "  lockstep -- ps -eo pcpu,cmd --sort=-pcpu | head   (RUN-WEBSITE.md 9.4)" >&2
    fi
fi

# --- joystick server ----------------------------------------------------
if web_up; then
    say "joystick server already up on :8090"
else
    say "starting joystick-server.py"
    nohup conda run -n "$CONDA_ENV" python joystick-server.py \
        > logs/joystick-server.console 2>&1 &
    disown || true
    say "  pid $!   log: logs/joystick-server.console"
    for _ in $(seq 1 20); do web_up && break; sleep 1; done
    web_up && say "joystick server up" \
           || say "WARNING joystick server not answering -- see logs/joystick-server.console" >&2
fi

IP="$(hostname -I | awk '{print $1}')"
echo
say "ready  ->  http://${IP}:8090/"
say "  video    http://${IP}:8080/detect"
say "  fly      ARM -> TAKEOFF -> OFFBOARD -> pad, or click the map -> FLY"
