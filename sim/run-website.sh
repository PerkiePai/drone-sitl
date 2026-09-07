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
# Flags (consumed here; everything else is passed through to launch-sitl.sh):
#   --descriptive   run joystick-server-descriptive.py on :8091 instead --
#                   labelled camera/map panels + a sim-truth marker on the map.
#                   The two servers can run side by side (8090 and 8091); Isaac
#                   is shared. See memory descriptive-web-ui-fork.md.
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

# --- which web server: default, or the --descriptive fork ------------------
SERVER_SCRIPT="joystick-server.py"
WEB_PORT=8090
SERVER_LOG="logs/joystick-server.console"
SIM_ARGS=()
for arg in "$@"; do
    if [[ "$arg" == "--descriptive" ]]; then
        SERVER_SCRIPT="joystick-server-descriptive.py"
        WEB_PORT=8091
        SERVER_LOG="logs/joystick-server-descriptive.console"
    else
        SIM_ARGS+=("$arg")
    fi
done

# The MJPEG camera server is the last thing drone_setup_px4_cesium.py starts, so
# an answer here means the whole Isaac -> Pegasus -> PX4 chain is live. Hit the
# index, NOT a feed: /detect is an unbounded multipart stream that never EOFs,
# so `curl --max-time` always exits non-zero on it even when the server is up.
isaac_up()      { curl -sf -o /dev/null --max-time 3 http://127.0.0.1:8080/; }
# Match OUR bootstrap specifically -- this box sometimes runs a second, unrelated
# SITL launcher, and "any kit process" would make us wait on theirs forever.
isaac_running() { pgrep -f "$REPO_DIR/sim/bootstrap.py" >/dev/null 2>&1; }
web_up()        { curl -sf -o /dev/null --max-time 3 "http://127.0.0.1:${WEB_PORT}/"; }

# --- Isaac Sim + PX4 -------------------------------------------------------
if [[ "${SKIP_SIM:-0}" == "1" ]]; then
    say "SKIP_SIM=1 -- not touching Isaac Sim"
elif isaac_up; then
    say "Isaac Sim + PX4 already up (camera on :8080 answering)"
elif isaac_running; then
    say "Isaac Sim process is already running -- waiting for it to finish booting"
else
    say "starting Isaac Sim  (sim/launch-sitl.sh)"
    nohup ./sim/launch-sitl.sh "${SIM_ARGS[@]}" > logs/launch-sitl.console 2>&1 &
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
    say "joystick server already up on :${WEB_PORT}"
else
    say "starting ${SERVER_SCRIPT}"
    # --no-capture-output + -u: without them `conda run` buffers the child's
    # stdout and SERVER_LOG stays empty, so the RUN-WEBSITE.md ">>> params:"
    # link check can never pass.
    nohup conda run --no-capture-output -n "$CONDA_ENV" python -u "$SERVER_SCRIPT" \
        > "$SERVER_LOG" 2>&1 &
    disown || true
    say "  pid $!   log: ${SERVER_LOG}"
    for _ in $(seq 1 20); do web_up && break; sleep 1; done
    web_up && say "joystick server up" \
           || say "WARNING joystick server not answering -- see ${SERVER_LOG}" >&2
fi

IP="$(hostname -I | awk '{print $1}')"
echo
say "ready  ->  http://${IP}:${WEB_PORT}/"
say "  video    http://${IP}:8080/detect"
say "  fly      ARM -> TAKEOFF -> OFFBOARD -> pad, or click the map -> FLY"
