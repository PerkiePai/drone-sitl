#!/usr/bin/env bash
# ============================================================================
# One command to get from nothing to a drone hovering-ready on the Cesium map.
#
#   ./sim/launch-sitl.sh
#
# Wraps ~/isaac-sim6/isaac-sim.streaming.sh rather than editing it: that file
# belongs to the Isaac install and gets overwritten on upgrade. Everything this
# adds is command-line flags plus an --exec bootstrap, so a plain
# isaac-sim.streaming.sh run still behaves exactly as before.
#
# What it removes from the manual routine:
#   - loading Cesium and typing the origin       -> sim/sites.py
#   - adding + orienting the map model           -> sim/sites.py (globe anchor)
#   - adding, placing and hiding the ground plane-> sim/sites.py
#   - enabling Pegasus and Cesium in the UI      -> --ext-folder / --enable
#   - pasting the setup script, pressing Play    -> sim/bootstrap.py
#
# There is no stage file. The stage is authored fresh and Z-up on every run, so
# Isaac's unconditional set_stage_up_axis("z") is a no-op rather than a
# 90-degree rotation of the world. QGroundControl still gets opened and armed
# by you, on purpose.
#
# Per-flight overrides, no file editing needed:
#   SITE=bangkok-survey-040 ./sim/launch-sitl.sh         # which site (this is the default)
#   SPAWN_XYZ='[12.0, -4.0, -26.5]' ./sim/launch-sitl.sh # takeoff point (ABSOLUTE z)
#   HEADING_DEG=90 ./sim/launch-sitl.sh                  # compass heading
#   AUTOPLAY=0 ./sim/launch-sitl.sh                      # spawn but stay stopped
#
# SPAWN_XYZ's z is absolute, so an override has to account for the ground plane
# (ground_z = -26.99 at this site). Prefer editing spawn_agl_m in sim/sites.py,
# which is measured from the ground and cannot drift away from it.
# ============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ISAAC_DIR="${ISAAC_DIR:-$HOME/isaac-sim6}"
SITE="${SITE:-bangkok-survey-040}"
SETUP_SCRIPT="${SETUP_SCRIPT:-$REPO_DIR/drone_setup_px4_cesium.py}"
PEGASUS_EXTS="${PEGASUS_EXTS:-$HOME/PegasusSimulator/extensions}"
CESIUM_EXTS="${CESIUM_EXTS:-$HOME/cesium-omniverse-6.0-build/exts}"

ISAAC_SH="$ISAAC_DIR/isaac-sim.streaming.sh"

die() { echo "launch-sitl: $*" >&2; exit 1; }

[[ -x "$ISAAC_SH"        ]] || die "Isaac Sim launcher not found at $ISAAC_SH (set ISAAC_DIR=)"
[[ -f "$SETUP_SCRIPT"    ]] || die "setup script not found at $SETUP_SCRIPT (set SETUP_SCRIPT=)"
[[ -d "$PEGASUS_EXTS"    ]] || die "Pegasus extensions not found at $PEGASUS_EXTS (set PEGASUS_EXTS=)"
[[ -d "$CESIUM_EXTS"     ]] || die "Cesium extensions not found at $CESIUM_EXTS (set CESIUM_EXTS=)"
[[ -n "${CESIUM_ION_TOKEN:-}" ]] || die "CESIUM_ION_TOKEN is not set.
  Cesium cannot stream tiles without it. Export it once in your shell profile:
    export CESIUM_ION_TOKEN='<your ion token>'
  Get one at https://ion.cesium.com/tokens"

# --- consumed by sim/bootstrap.py -------------------------------------------
export SITL_SITE="$SITE"
export SITL_SETUP_SCRIPT="$SETUP_SCRIPT"
export SITL_AUTOPLAY="${AUTOPLAY:-1}"
export SITL_TILE_SETTLE_FRAMES="${TILE_SETTLE_FRAMES:-240}"

# --- consumed by drone_setup_px4_cesium.py's env-override block -------------
# Only exported when the operator actually set them; otherwise the site config
# supplies the spawn pose (bootstrap.py uses os.environ.setdefault). Written as
# `if` blocks rather than `[[ ... ]] && export ...` because under `set -e` a
# false && list is a footgun that depends on where it sits in the file.
#
# Anything else in that script's tunables block can be set the same way, e.g.
#   DRONE_SETUP_ADD_WIND=True DRONE_SETUP_STREAM_PORT=8081 ./sim/launch-sitl.sh
if [[ -n "${SPAWN_XYZ:-}" ]]; then
    export DRONE_SETUP_SPAWN_XYZ="$SPAWN_XYZ"
fi
if [[ -n "${HEADING_DEG:-}" ]]; then
    export DRONE_SETUP_HEADING_DEG="$HEADING_DEG"
fi

echo "launch-sitl: site       $SITE"
echo "launch-sitl: setup      $SETUP_SCRIPT"
echo "launch-sitl: spawn      ${SPAWN_XYZ:-from sim/sites.py}  heading ${HEADING_DEG:-from sim/sites.py}"

exec "$ISAAC_SH" \
    --ext-folder "$PEGASUS_EXTS" \
    --ext-folder "$CESIUM_EXTS" \
    --enable pegasus.simulator \
    --enable cesium.omniverse \
    --exec "$REPO_DIR/sim/bootstrap.py" \
    "$@"
