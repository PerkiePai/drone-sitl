# ============================================================================
# Headless bring-up for the PX4 + Cesium SITL stage.
#
# Kit runs this once at startup (`--exec sim/bootstrap.py`, wired up by
# sim/launch-sitl.sh). It replaces the click-path you would otherwise repeat
# every session:
#
#   build the stage from sim/sites.py  ->  let Cesium stream tiles
#   ->  run drone_setup_px4_cesium.py  ->  press Play.
#
# There is deliberately no .usd on disk. The stage is authored Z-up every run,
# so Isaac's unconditional set_stage_up_axis("z") is a no-op instead of a
# 90-degree rotation of the whole world. See
# docs/superpowers/specs/2026-07-31-sitl-stage-from-config-design.md
#
#   SITL_SITE                  key into sim/sites.py SITES (required)
#   SITL_SETUP_SCRIPT          drone setup script to exec (required)
#   CESIUM_ION_TOKEN           Cesium ion access token (required)
#   SITL_TILE_SETTLE_FRAMES    frames to let Cesium stream tiles (240)
#   SITL_AUTOPLAY              "0" leaves the sim stopped after spawning
#
# Nothing here is fatal: on error it prints a banner and leaves the app up, so
# you can still attach the WebRTC stream and inspect the stage by hand.
# ============================================================================
import asyncio
import os
import sys
import traceback

import omni.kit.app
import omni.timeline

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)

SITE_NAME     = os.environ.get("SITL_SITE", "")
SETUP_SCRIPT  = os.environ.get("SITL_SETUP_SCRIPT", "")
ION_TOKEN     = os.environ.get("CESIUM_ION_TOKEN", "")
SETTLE_FRAMES = int(os.environ.get("SITL_TILE_SETTLE_FRAMES", "240"))
AUTOPLAY      = os.environ.get("SITL_AUTOPLAY", "1") != "0"

# Extensions the builder and setup script import from. They live outside the
# Isaac install, so the launcher adds them with --ext-folder/--enable and we
# just wait for them to finish coming up.
REQUIRED_EXTS = ["pegasus.simulator", "cesium.omniverse"]


def _banner(*lines):
    print("*" * 78)
    for line in lines:
        print(f"*** {line}")
    print("*" * 78)


async def _frames(n):
    """Pump n app updates -- the only way to let Kit make progress from async code."""
    app = omni.kit.app.get_app()
    for _ in range(n):
        await app.next_update_async()


async def _wait_for_extensions():
    """Block until the out-of-tree extensions report enabled, or give up."""
    mgr = omni.kit.app.get_app().get_extension_manager()
    for _ in range(600):
        if all(mgr.is_extension_enabled(e) for e in REQUIRED_EXTS):
            return True
        await _frames(1)
    missing = [e for e in REQUIRED_EXTS if not mgr.is_extension_enabled(e)]
    _banner(f"extensions never enabled: {', '.join(missing)}",
            "Check the --ext-folder paths in sim/launch-sitl.sh.")
    return False


async def _run_setup_script(site):
    """Exec the setup script and wait for the coroutine it kicks off.

    drone_setup_px4_cesium.py ends with asyncio.ensure_future(...) and never
    hands the future back, so the spawn task is picked out by diffing the loop's
    task set around the exec. That keeps this launcher working with the script
    unmodified -- it is still the same file you paste into the Script Editor.

    Spawn pose comes from the site unless the operator already overrode it, so
    SPAWN_XYZ=... ./sim/launch-sitl.sh still wins.
    """
    os.environ.setdefault("DRONE_SETUP_SPAWN_XYZ", repr(list(site.spawn_xyz)))
    os.environ.setdefault("DRONE_SETUP_HEADING_DEG", repr(site.heading_deg))

    with open(SETUP_SCRIPT, encoding="utf-8") as f:
        source = f.read()

    ns = {"__name__": "__main__", "__file__": SETUP_SCRIPT}
    loop = asyncio.get_event_loop()

    before = set(asyncio.all_tasks(loop))
    print(f">>> Running setup script: {SETUP_SCRIPT}")
    exec(compile(source, SETUP_SCRIPT, "exec"), ns)
    spawned = set(asyncio.all_tasks(loop)) - before

    if spawned:
        await asyncio.gather(*spawned)
    else:
        # Fully synchronous setup script -- still give it frames to land.
        await _frames(60)


def _advisory(label, fn):
    """Run a diagnostic without ever letting it stop the bring-up.

    Every check below this point is ADVISORY. An earlier version refused to
    press Play when the georeference looked wrong, which was a mistake: Pegasus
    only launches PX4 and starts streaming sensor data on the timeline's play
    event, so withholding Play leaves no vehicle at all. The operator then meets
    the problem four layers downstream as "the web UI won't let me arm" -- with
    the actual banner scrolled off in the Isaac console. A loud warning plus a
    running sim beats a silent, correct refusal.
    """
    try:
        fn()
    except Exception as exc:
        _banner(f"{label} failed (continuing anyway): {exc.__class__.__name__}: {exc}")
        traceback.print_exc()


def _check_georeference(site):
    """Manual step 7 -- 'drone position is the same for both QGC and Isaac'.

    drone_setup_px4_cesium.py derives the PX4 GPS origin from the georeference
    prim, so matching coordinates here means the origin it used is the origin we
    authored. Advisory: a mismatch means QGC and Isaac disagree on where the
    drone is, which is worth shouting about but is still a flyable sim.
    """
    import stage_builder

    lat, lon, height = stage_builder.read_georeference()
    ok = (abs(lat - site.latitude) < 1e-6
          and abs(lon - site.longitude) < 1e-6
          and abs(height - site.height) < 1e-3)
    if ok:
        print(f">>> georeference verified: {lat}, {lon}, {height}")
    else:
        _banner("georeference read-back does not match the site config",
                f"authored: {site.latitude}, {site.longitude}, {site.height}",
                f"stage:    {lat}, {lon}, {height}",
                "QGC and Isaac will disagree on position. Playing anyway.")
    return ok


def _start_recorder_control():
    """Serve the RECORD button's control endpoints from inside Kit.

    The recorder needs Pegasus sensor callbacks and the replicator annotators,
    which exist only in this process -- the web server, in the `drone` conda
    env, cannot reach them. See
    docs/superpowers/specs/2026-08-07-web-recorder-design.md
    """
    import recorder_control

    recorder_control.start_control_server()


async def _bring_up():
    if not SITE_NAME or not SETUP_SCRIPT:
        _banner("SITL_SITE / SITL_SETUP_SCRIPT not set.",
                "Launch through sim/launch-sitl.sh, not kit directly.")
        return
    if not ION_TOKEN:
        _banner("CESIUM_ION_TOKEN is not set -- tiles cannot stream.")
        return

    # Let the app finish its own startup before touching the stage.
    await _frames(10)
    if not await _wait_for_extensions():
        return

    import sites
    import stage_builder

    site = sites.get_site(SITE_NAME)
    await stage_builder.build_stage(site, ION_TOKEN)

    print(f">>> Settling {SETTLE_FRAMES} frames for Cesium tiles.")
    await _frames(SETTLE_FRAMES)

    # Cesium writes each model's transform from its globe anchor on its own
    # update tick, which lands after build_stage returns. Re-assert the
    # orientation and local Z now that those ticks have happened, or the mesh
    # sits at the anchor's ellipsoid-derived height instead of where we want it.
    _advisory("model transform override", lambda: stage_builder.apply_model_overrides(site))

    # Also advisory: a setup script that dies partway still leaves prims on the
    # stage, and Play is what reveals how far it got. Swallowing the traceback
    # here would hide the single most useful diagnostic there is.
    try:
        await _run_setup_script(site)
    except Exception as exc:
        _banner(f"setup script failed (continuing to Play anyway): "
                f"{exc.__class__.__name__}: {exc}",
                "The drone may be missing or half-built -- this traceback is the",
                "reason PX4 never launches and the web UI cannot arm.")
        traceback.print_exc()
    await _frames(30)

    # Everything from here is advisory -- see _advisory(). Play must happen
    # regardless, because Pegasus launches PX4 and starts streaming sensor data
    # from the timeline's play event and nowhere else. No Play means no vehicle,
    # which is a much worse outcome than a warned-about stage.
    _advisory("model transform override (post-spawn)",
              lambda: stage_builder.apply_model_overrides(site))
    _advisory("up-axis check",
              lambda: stage_builder.assert_z_up("after drone_setup_px4_cesium.py"))
    _advisory("georeference check", lambda: _check_georeference(site))

    # Before Play on purpose: the operator reaches for the RECORD button as soon
    # as the page comes alive, and PX4 takes a while to appear after Play. The
    # server should already be answering by then.
    _advisory("recorder control server", _start_recorder_control)

    if AUTOPLAY:
        omni.timeline.get_timeline_interface().play()
        print(">>> Play pressed. PX4 SITL is starting -- connect QGroundControl now.")
        print("    If PX4 never appears (`pgrep px4` empty), Pegasus's timeline")
        print("    callback did not fire and the vehicle is not simulating.")
    else:
        print(">>> SITL_AUTOPLAY=0 -- sim left stopped. Press Play when ready.")


async def _guarded():
    try:
        await _bring_up()
    except Exception:
        _banner("bootstrap failed -- the app is still up, stage may be incomplete")
        traceback.print_exc()


asyncio.ensure_future(_guarded())
