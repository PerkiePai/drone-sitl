# Plan: build the SITL stage from config

**Spec:** `docs/superpowers/specs/2026-07-31-sitl-stage-from-config-design.md`
**Date:** 2026-07-31
**Branch:** `feat/map-waypoints` (current)

## Goal

`./sim/launch-sitl.sh` brings up Cesium at `13.66156872, 100.298235`, the survey
mesh, an invisible ground plane, and a PX4 drone on the pad — with no `.usd`
file anywhere in the loop, and with the stage authored Z-up so Isaac's
unconditional `set_stage_up_axis("z")` is a no-op.

## Architecture summary

```
sim/launch-sitl.sh          validate env, export, exec isaac-sim.streaming.sh
  └── kit --exec sim/bootstrap.py
        ├── sim/sites.py            Site/Tileset/ModelAnchor dataclasses + SITES
        ├── sim/stage_builder.py    build_stage(site, token) -> authors the stage
        └── drone_setup_px4_cesium.py   exec'd unchanged, task-diff await
```

`sites.py` is pure data and pure path resolution — importable and testable
outside Kit. `stage_builder.py` is Kit-only stage authoring. `bootstrap.py` owns
sequencing. That split is what makes anything testable at all.

## Tech stack

Python 3.11 (repo tests, conda env `drone`) / 3.12 (Kit interpreter), pytest,
USD via `pxr`, `cesium.omniverse.usdUtils`, `isaacsim.core.experimental.utils.stage`,
`isaacsim.core.api.objects.GroundPlane`, bash.

## Global constraints

- **Never author content before setting the up-axis to Z.** A Cesium globe
  anchor resolves its local transform against the stage up-axis; that ordering
  is the whole point of this rewrite.
- **No secrets in tracked files.** The ion token arrives only via
  `CESIUM_ION_TOKEN`.
- `drone_setup_px4_cesium.py` stays runnable by paste-into-Script-Editor. The
  only change to it is one default flag.
- Verified API signatures — do not substitute from memory:
  - `stage_utils.set_stage_units(meters_per_unit=1.0)` is **keyword-only**
  - `stage_utils.set_stage_up_axis("Z")` takes `"Y"`/`"Z"`
  - `GroundPlane(prim_path=..., z_position=..., visible=...)` handles the Z axis internally
  - `usdUtils.add_tileset_ion(name, asset_id, token="")`
  - `usdUtils.add_globe_anchor_to_prim(path)` returns `CesiumGlobeAnchorAPI`

---

## Task 1 — `sim/sites.py` + tests

**Creates:** `sim/sites.py`, `sim/tests/test_sites.py`, `sim/tests/__init__.py`
**Produces:** `Site`, `Tileset`, `ModelAnchor`, `SITES`, `get_site(name)`
**Consumes:** nothing

- [ ] **1.1 — Write the failing test.** Create `sim/tests/__init__.py` (empty) and `sim/tests/test_sites.py`:

```python
"""Site config is the only part of the stage build that runs outside Kit,
so it is the only part that can carry real tests. They guard the things that
actually broke: a stale model path, a spawn altitude that ignores the ground
plane, and a secret pasted into tracked config."""
import dataclasses
import re
from pathlib import Path

import pytest

import sites


def test_bangkok_site_is_registered():
    site = sites.get_site("bangkok-survey-040")
    assert site.latitude == pytest.approx(13.66156872)
    assert site.longitude == pytest.approx(100.298235)
    assert site.height == 0.0


def test_unknown_site_names_the_known_ones():
    with pytest.raises(KeyError, match="bangkok-survey-040"):
        sites.get_site("nowhere")


def test_every_model_path_exists():
    for site in sites.SITES.values():
        for model in site.models:
            path = model.resolved_path()
            assert path.is_file(), f"{site.name}/{model.prim_name}: missing {path}"


def test_spawn_sits_just_above_the_ground_plane():
    for site in sites.SITES.values():
        assert 0.0 < site.spawn_agl_m <= 2.0
        assert site.spawn_z == pytest.approx(site.ground_z + site.spawn_agl_m)


def test_config_carries_no_secrets():
    text = Path(sites.__file__).read_text(encoding="utf-8")
    assert not re.search(r"eyJ[A-Za-z0-9_-]{20,}", text), "JWT-shaped literal in tracked config"


def test_site_is_frozen():
    site = sites.get_site("bangkok-survey-040")
    with pytest.raises(dataclasses.FrozenInstanceError):
        site.latitude = 0.0
```

  The test module imports `dataclasses`, `re`, `pathlib.Path`, `pytest` and
  `sites`. `tests/__init__.py` is what puts `sim/` on `sys.path` for the bare
  `import sites` — pytest inserts the first ancestor directory without an
  `__init__.py`, which is `sim/`.

- [ ] **1.2 — Verify red.** `cd sim && python -m pytest tests/test_sites.py -q`
  Expected: collection error, `ModuleNotFoundError: No module named 'sites'`.

- [ ] **1.3 — Implement.** Create `sim/sites.py`:

```python
"""Site definitions for the SITL stage builder.

A "site" is everything a hand-baked .usd used to hold: where on the globe the
sim is anchored, which Cesium tilesets cover it, which reconstructed models sit
on top, how high the ground is, and where the drone starts.

Deliberately free of secrets -- the Cesium ion token reaches the builder through
CESIUM_ION_TOKEN in the environment, never through this file.

Module name is `sites`, not `site`: `site` is a stdlib module Kit's own
interpreter imports, and shadowing it on sys.path is a trap.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class Tileset:
    """A Cesium ion tileset streamed over the site."""

    name: str
    ion_asset_id: int


@dataclass(frozen=True)
class ModelAnchor:
    """A reconstructed map model, placed by lat/lon rather than by dragging.

    Cesium's globe anchor derives the prim's local transform from geographic
    coordinates, so the placement carries no assumption about the stage's
    up-axis. That is what makes rebuilding the stage from scratch safe.
    """

    prim_name: str
    usd_path: str          # relative to REPO_ROOT, or absolute
    latitude: float
    longitude: float
    height: float

    def resolved_path(self) -> Path:
        path = Path(self.usd_path).expanduser()
        return path if path.is_absolute() else (REPO_ROOT / path).resolve()


@dataclass(frozen=True)
class Site:
    name: str
    latitude: float
    longitude: float
    height: float                      # georeference origin height
    tilesets: tuple[Tileset, ...]
    models: tuple[ModelAnchor, ...]
    ground_z: float                    # metres, Z-up, from the georeference origin
    spawn_xy: tuple[float, float]      # local metres (x=East, y=North)
    spawn_agl_m: float                 # clearance ABOVE ground_z, not above origin
    heading_deg: float                 # compass: 0=N, 90=E

    @property
    def spawn_z(self) -> float:
        """Absolute spawn height. Derived so it cannot drift from the ground plane."""
        return self.ground_z + self.spawn_agl_m

    @property
    def spawn_xyz(self) -> tuple[float, float, float]:
        return (self.spawn_xy[0], self.spawn_xy[1], self.spawn_z)


BANGKOK_SURVEY_040 = Site(
    name="bangkok-survey-040",
    latitude=13.66156872,
    longitude=100.298235,
    height=0.0,
    tilesets=(Tileset(name="Google_Photorealistic_3D_Tiles", ion_asset_id=2275207),),
    models=(
        ModelAnchor(
            prim_name="survey_mesh_obj",
            usd_path="../metashape/output/survey_040_max_converted/survey_mesh_obj.usd",
            latitude=13.662013797285455,
            longitude=100.29186024766956,
            height=-27.141119462205456,
        ),
    ),
    ground_z=-26.992977143901495,
    spawn_xy=(0.0, 0.0),
    spawn_agl_m=0.5,
    heading_deg=0.0,
)

SITES: dict[str, Site] = {BANGKOK_SURVEY_040.name: BANGKOK_SURVEY_040}


def get_site(name: str) -> Site:
    """Look up a site by name, raising with the valid options if it is unknown."""
    try:
        return SITES[name]
    except KeyError:
        raise KeyError(f"unknown site {name!r}; known sites: {sorted(SITES)}") from None
```

- [ ] **1.4 — Verify green.** `cd sim && python -m pytest tests/test_sites.py -q`
  Expected: `6 passed`.

- [ ] **1.5 — Commit.**
  `feat(sim): describe SITL sites as config instead of a baked stage`

---

## Task 2 — `sim/stage_builder.py`

**Creates:** `sim/stage_builder.py`
**Consumes:** `sites.Site`, `CESIUM_ION_TOKEN` value (passed as an argument)
**Produces:** `async def build_stage(site, ion_token) -> None`

No unit test: every call it makes needs a live Kit stage. Its correctness is
covered by the manual gate in Task 6.

- [ ] **2.1 — Implement.** Create `sim/stage_builder.py`:

```python
"""Assemble the SITL stage from a Site definition.

Pure stage authoring -- no launch sequencing, no Play. bootstrap.py owns the
ordering; this module owns the content.

The order in build_stage is load-bearing. The up-axis is set to Z before any
content is authored, because a Cesium globe anchor resolves its local transform
against the stage up-axis. Authoring anchors into a Y-up stage and letting
Isaac's set_stage_up_axis("z") flip it afterwards is the exact failure this
module exists to prevent -- see
docs/superpowers/specs/2026-07-31-sitl-stage-from-config-design.md
"""
from __future__ import annotations

import omni.usd
from pxr import Sdf, UsdGeom

import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.api.objects import GroundPlane

from cesium.omniverse.usdUtils import usdUtils
from cesium.usd.plugins.CesiumUsdSchemas import IonServer as CesiumIonServer

from sites import ModelAnchor, Site

WORLD_PATH = "/World"
GROUND_PLANE_PATH = "/World/GroundPlane"
ION_SERVER_PATH = "/CesiumServers/IonOfficial"


async def build_stage(site: Site, ion_token: str) -> None:
    """Author a complete, Z-up SITL stage for `site`. Replaces the open stage."""
    await stage_utils.create_new_stage_async()
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0)

    stage = omni.usd.get_context().get_stage()
    world = UsdGeom.Xform.Define(stage, WORLD_PATH)
    stage.SetDefaultPrim(world.GetPrim())

    _set_ion_token(ion_token)
    _set_georeference(site)

    for tileset in site.tilesets:
        path = usdUtils.add_tileset_ion(tileset.name, tileset.ion_asset_id)
        print(f">>> tileset {tileset.name} (ion {tileset.ion_asset_id}) at {path}")

    for model in site.models:
        _add_model(model)

    GroundPlane(
        prim_path=GROUND_PLANE_PATH,
        name="ground_plane",
        z_position=site.ground_z,
        visible=False,
    )
    print(f">>> ground plane at z={site.ground_z:.3f} (invisible)")

    assert_z_up("after build_stage")
    print(f">>> stage built for site {site.name!r} at {site.latitude}, {site.longitude}")


def assert_z_up(when: str) -> None:
    """Fail loudly on the one mistake that silently rotates the whole world."""
    stage = omni.usd.get_context().get_stage()
    axis = UsdGeom.GetStageUpAxis(stage)
    if axis != UsdGeom.Tokens.z:
        raise RuntimeError(
            f"stage up-axis is {axis!r}, expected 'Z' ({when}). Cesium content, "
            f"the survey mesh and the ground plane will all be rotated 90 degrees. "
            f"See docs/superpowers/specs/2026-07-31-sitl-stage-from-config-design.md"
        )


def read_georeference() -> tuple[float, float, float]:
    """Read back (lat, lon, height) as authored, for the bootstrap's check."""
    georef = usdUtils.get_or_create_cesium_georeference()
    return (
        float(georef.GetGeoreferenceOriginLatitudeAttr().Get()),
        float(georef.GetGeoreferenceOriginLongitudeAttr().Get()),
        float(georef.GetGeoreferenceOriginHeightAttr().Get()),
    )


def _set_ion_token(token: str) -> None:
    """Put the ion token on the server prim, defining it if Cesium has not yet.

    cesium.omniverse's extension.py:525 _setup_ion_server_prims() defines
    /CesiumServers/IonOfficial on stage events, but that is event-driven and its
    timing relative to a stage we just created is not guaranteed.
    """
    stage = omni.usd.get_context().get_stage()
    usdUtils.get_or_create_cesium_data()

    if not stage.GetPrimAtPath(ION_SERVER_PATH).IsValid():
        server = CesiumIonServer.Define(stage, ION_SERVER_PATH)
        server.GetDisplayNameAttr().Set("ion.cesium.com")
        server.GetIonServerUrlAttr().Set("https://ion.cesium.com/")
        server.GetIonServerApiUrlAttr().Set("https://api.cesium.com/")
        server.GetIonServerApplicationIdAttr().Set(413)
    else:
        server = CesiumIonServer.Get(stage, ION_SERVER_PATH)

    server.GetProjectDefaultIonAccessTokenAttr().Set(token)
    usdUtils.set_path_to_current_ion_server(ION_SERVER_PATH)
    print(">>> ion token applied to /CesiumServers/IonOfficial")


def _set_georeference(site: Site) -> None:
    """Set the georeference origin. drone_setup_px4_cesium.py's
    read_cesium_georeference() reads this back to set the PX4 GPS origin, which
    is what makes QGC and Isaac agree on where the drone is."""
    georef = usdUtils.get_or_create_cesium_georeference()
    georef.GetGeoreferenceOriginLatitudeAttr().Set(site.latitude)
    georef.GetGeoreferenceOriginLongitudeAttr().Set(site.longitude)
    georef.GetGeoreferenceOriginHeightAttr().Set(site.height)
    print(f">>> georeference origin {site.latitude}, {site.longitude}, {site.height}")


def _add_model(model: ModelAnchor) -> None:
    """Payload the model and place it by lat/lon via a Cesium globe anchor."""
    path = model.resolved_path()
    if not path.is_file():
        raise FileNotFoundError(
            f"map model {model.prim_name!r} not found at {path} "
            f"(configured as {model.usd_path!r} in sim/sites.py)"
        )

    stage = omni.usd.get_context().get_stage()
    prim_path = f"{WORLD_PATH}/{model.prim_name}"
    xform = UsdGeom.Xform.Define(stage, prim_path)
    xform.GetPrim().GetPayloads().AddPayload(Sdf.Payload(str(path)))

    anchor = usdUtils.add_globe_anchor_to_prim(prim_path)
    anchor.GetAnchorLatitudeAttr().Set(model.latitude)
    anchor.GetAnchorLongitudeAttr().Set(model.longitude)
    anchor.GetAnchorHeightAttr().Set(model.height)
    anchor.GetDetectTransformChangesAttr().Set(True)
    anchor.GetAdjustOrientationForGlobeWhenMovingAttr().Set(True)
    print(f">>> model {model.prim_name} anchored at {model.latitude}, {model.longitude}, {model.height}")
```

- [ ] **2.2 — Byte-compile check.** `python -m py_compile sim/stage_builder.py`
  Expected: no output. (It cannot be imported outside Kit; this only catches syntax.)

- [ ] **2.3 — Commit.**
  `feat(sim): author the Cesium SITL stage programmatically`

---

## Task 3 — rewrite `sim/bootstrap.py`

**Modifies:** `sim/bootstrap.py`
**Consumes:** `SITL_SITE`, `SITL_SETUP_SCRIPT`, `CESIUM_ION_TOKEN`, `SITL_TILE_SETTLE_FRAMES`, `SITL_AUTOPLAY`
**Produces:** a playing sim

- [ ] **3.1 — Replace the file** with the version below. Changes from the current file: drops `SITL_STAGE` and `_open_stage`, adds `sys.path` wiring, `build_stage`, the post-setup up-axis assert and the georeference read-back.

```python
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
        await _frames(60)


def _check_georeference(site):
    """Manual step 7 -- 'drone position is the same in QGC and Isaac' -- as an
    assertion. drone_setup_px4_cesium.py derives the PX4 GPS origin from the
    georeference prim, so matching coordinates here means the origin it used is
    the origin we authored."""
    import stage_builder

    lat, lon, height = stage_builder.read_georeference()
    ok = (abs(lat - site.latitude) < 1e-6
          and abs(lon - site.longitude) < 1e-6
          and abs(height - site.height) < 1e-3)
    if not ok:
        _banner("georeference read-back does not match the site config",
                f"authored: {site.latitude}, {site.longitude}, {site.height}",
                f"stage:    {lat}, {lon}, {height}",
                "NOT pressing Play -- QGC and Isaac would disagree on position.")
    return ok


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

    await _run_setup_script(site)
    await _frames(30)

    # The setup script is where the up-axis used to get flipped. Re-check after
    # it, not just after the build -- this is the regression guard.
    stage_builder.assert_z_up("after drone_setup_px4_cesium.py")

    if not _check_georeference(site):
        return

    if AUTOPLAY:
        omni.timeline.get_timeline_interface().play()
        print(">>> Play pressed. PX4 SITL is starting -- connect QGroundControl now.")
    else:
        print(">>> SITL_AUTOPLAY=0 -- sim left stopped. Press Play when ready.")


async def _guarded():
    try:
        await _bring_up()
    except Exception:
        _banner("bootstrap failed -- the app is still up, stage may be incomplete")
        traceback.print_exc()


asyncio.ensure_future(_guarded())
```

- [ ] **3.2 — Byte-compile check.** `python -m py_compile sim/bootstrap.py`
  Expected: no output.

- [ ] **3.3 — Commit.**
  `feat(sim): build the stage in bootstrap and assert the up-axis survives setup`

---

## Task 4 — `sim/launch-sitl.sh`

**Modifies:** `sim/launch-sitl.sh`

- [ ] **4.1 — Swap the stage block for a site block.** Replace the `STAGE=` default and its `[[ -f "$STAGE" ]]` check with:

```bash
SITE="${SITE:-bangkok-survey-040}"
```

and add, alongside the other `die` checks:

```bash
[[ -n "${CESIUM_ION_TOKEN:-}" ]] || die "CESIUM_ION_TOKEN is not set.
  Cesium cannot stream tiles without it. Export it once in your shell profile:
    export CESIUM_ION_TOKEN='<your ion token>'
  Get one at https://ion.cesium.com/tokens"
```

- [ ] **4.2 — Swap the exported env.** Replace `export SITL_STAGE="$STAGE"` with `export SITL_SITE="$SITE"`, and drop the two `DRONE_SETUP_*` exports whose defaults now come from the site — keeping the pass-through so an explicitly set `SPAWN_XYZ`/`HEADING_DEG` still wins:

```bash
export SITL_SITE="$SITE"
export SITL_SETUP_SCRIPT="$SETUP_SCRIPT"
export SITL_AUTOPLAY="${AUTOPLAY:-1}"
export SITL_TILE_SETTLE_FRAMES="${TILE_SETTLE_FRAMES:-240}"

# Only export when the operator set them; otherwise the site config supplies
# the spawn pose (bootstrap.py uses os.environ.setdefault).
if [[ -n "${SPAWN_XYZ:-}" ]]; then
    export DRONE_SETUP_SPAWN_XYZ="$SPAWN_XYZ"
fi
if [[ -n "${HEADING_DEG:-}" ]]; then
    export DRONE_SETUP_HEADING_DEG="$HEADING_DEG"
fi
```

  Written as `if` blocks rather than `[[ ... ]] && export ...` on purpose: under
  `set -e` a false `&&` list is a footgun that depends on where it sits in the
  file. An `if` has no such dependency.

- [ ] **4.3 — Update the header comment and the echo lines** to name the site
  rather than the stage, and delete the "Baking the stage is a ONE-TIME job"
  paragraph — it describes a workflow that no longer exists.

- [ ] **4.4 — Verify preflight.** `bash -n sim/launch-sitl.sh` → no output, then
  `env -u CESIUM_ION_TOKEN ./sim/launch-sitl.sh`
  Expected: exits 1 with the `CESIUM_ION_TOKEN is not set` message, without
  launching Isaac.

- [ ] **4.5 — Commit.**
  `feat(sim): launch by site name and require the ion token up front`

---

## Task 5 — retire the baked-stage remnants

**Modifies:** `drone_setup_px4_cesium.py`, `.gitignore`
**Deletes:** `sim/stages/`

- [ ] **5.1 — Turn off the ground-plane flip.** In `drone_setup_px4_cesium.py`, change `FIX_GROUND_FLIP = True` to `False` and replace the two comment lines with:

```python
FIX_GROUND_FLIP    = False             # the stage builder authors the plane flat in a Z-up
                                       # stage, so there is nothing to un-flip and running
                                       # fix_ground_plane() would BREAK a correct plane by
                                       # forcing rotateX=90. Set True only for the
                                       # paste-into-Script-Editor path on a hand-built
                                       # Y-up stage, which is what it was always for.
```

  Leave `fix_ground_plane()` itself in place — it is still correct for that path.

- [ ] **5.2 — Delete the stage and its ignore rule.** It was gitignored, so it
  was never tracked and there is nothing to `git rm`:
  `rm -rf sim/stages`
  Then remove these two lines from `.gitignore`:

```
# Baked Isaac Sim stages — large binaries, rebuilt per site (guide section 3.1)
sim/stages/
```

- [ ] **5.3 — Verify no code still references a stage file.**
  `grep -rn "SITL_STAGE\|sim/stages\|sitl-stage" --include='*.py' --include='*.sh' .`
  Expected: no output. Scoped to code on purpose — `docs/joystick-guide.md`
  still describes the bake procedure at this point and is not cleaned up until
  Task 6.1, and the spec keeps its references as a historical record.

- [ ] **5.4 — Commit.**
  `refactor(sim): drop the baked stage and the ground-plane flip it needed`

---

## Task 6 — docs + manual verification gate

**Modifies:** `docs/joystick-guide.md`

- [ ] **6.1 — Rewrite §3.** Replace §3.1 (the bake procedure) with token setup, and §3.2 with site config:

```markdown
### 3.1 One-time setup

Export your Cesium ion token — the stage build needs it to stream tiles:

```bash
export CESIUM_ION_TOKEN='<your token>'    # add to ~/.bashrc
```

There is no stage file to build. `sim/sites.py` holds the georeference origin,
the tileset, the map model's globe anchor, the ground height and the takeoff
pose; the stage is authored fresh, Z-up, on every launch.

### 3.2 Per-flight overrides

```bash
SITE=bangkok-survey-040 ./sim/launch-sitl.sh   # pick a site (this is the default)
SPAWN_XYZ='[12.0, -4.0, -26.5]' ./sim/launch-sitl.sh   # override the takeoff point
HEADING_DEG=90 ./sim/launch-sitl.sh                    # override the heading
AUTOPLAY=0 ./sim/launch-sitl.sh                        # spawn, leave it stopped
```

Spawn z is absolute, so an override must account for the ground plane
(`ground_z = -26.99` at this site). Prefer editing `spawn_agl_m` in
`sim/sites.py`, which is measured from the ground and cannot drift.
```

  Update the §3 intro to say the launcher builds the stage rather than opening
  one, and delete the "Baking the stage is a ONE-TIME job" sentence.

- [ ] **6.2 — Commit.** `docs: launch by site config, not a baked stage`

- [ ] **6.3 — Run the acceptance gate on the box.** This is the part no test can
  cover; items 3–5 are the ones that failed last time.

```bash
export CESIUM_ION_TOKEN='<token>'
./sim/launch-sitl.sh
```

  1. reaches `>>> Play pressed` with no `***` banner
  2. no `stage up-axis is 'Y'` error — the regression guard stayed quiet
  3. tiles and the survey mesh render **level**, not rotated 90°
  4. QGC shows the drone at `13.6615°N, 100.2982°E`
  5. the drone rests on the invisible plane; it does not fall ~27 m on Play
  6. `curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/detect` → `200`

- [ ] **6.4 — Record the result** in the spec's Status line (`implemented and
  flown` + date, matching the convention in `2026-07-31-map-waypoints-design.md`).

---

## Follow-ups, not in this plan

- Rotate the Cesium ion token — the old one was printed to a terminal during
  the investigation that produced this plan.
- Auto arm + takeoff (roadmap V2a).
- `sim/stages/sitl-stage.usd` is deleted by Task 5; if anything in it was not
  captured in `sites.py`, it is gone. The extracted values are in the spec.
