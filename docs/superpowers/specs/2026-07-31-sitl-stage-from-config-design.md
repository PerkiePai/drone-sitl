# Design: build the SITL stage from config, not from a saved .usd

**Status:** designed, not yet implemented
**Date:** 2026-07-31
**Supersedes:** the bake-a-stage procedure added to `joystick-guide.md` §3.1
earlier today, which is withdrawn — see "Why the first attempt failed"

## Goal

Turn this nine-step manual routine into one command:

1. load Cesium, go to `13.661568727276276, 100.29823565964742`, height 0
2. run `drone_setup_px4_cesium.py`
3. add the map model USD, orientation `0,0,0`
4. move the drone to the takeoff position
5. add a ground plane, align it to the model, make it invisible
6. open and connect QGroundControl
7. check the drone position matches in QGC and Isaac
8. arm + takeoff
9. plan a route and fly

Steps 1–5 and 7 are automated here. Steps 6, 8 and 9 stay manual **by
decision** — a human gate before the props spin. Success is:

```bash
./sim/launch-sitl.sh          # -> drone on the pad at the right lat/lon, sim playing
```

with QGC showing the drone at the same place Isaac does, and no stage file
anywhere in the loop.

## Why the first attempt failed

The first cut of this feature baked the stage: build it once by hand, save
`sim/stages/sitl-stage.usd`, open it on every launch. It broke on first run —
after the setup script ran, all Cesium content was rotated 90°.

Root cause, confirmed in `isaacsim/core/api/simulation_context/simulation_context.py:1487`:

```python
async def _initialize_stage_async(self, ...):
    if get_current_stage() is None:
        await create_new_stage_async()
    set_stage_up_axis("z")          # unconditional, not restorable
```

`drone_setup_px4_cesium.py` calls `pg._world.initialize_simulation_context_async()`,
which lands there. The baked stage was **Y-up**:

```
upAxis = Y
/World/GroundPlane        xformOp:translate = (0, -26.99, 0)      # height in Y
/World/survey_mesh_obj    xformOp:rotateX:unitsResolve = -90.0    # USD reconciling Y/Z
```

So the moment the script ran, the stage's up-axis was rewritten Y→Z and every
transform in it was reinterpreted against a different up vector.

Two conclusions follow, and they shape this design:

- **Fighting the flip is unwinnable.** PhysX gravity, Pegasus reading
  `state.position[2]` as altitude (`drone_setup_px4_cesium.py`'s `WindDrag.update`),
  the wind model's `WIND_MIN_AGL_M` gate and `SPAWN_XYZ`'s documented `z=Up` all
  assume Z-up. Z-up is correct; the stage was wrong.
- **`GROUND_FLIP_DEG = 90` was never about Cesium.** The comment at
  `drone_setup_px4_cesium.py:38` blames the georeference for tipping a fresh
  plane "into a wall". It doesn't — a Z-up-authored ground plane in a Y-up stage
  merely looks tipped. `fix_ground_plane()` has been compensating for the
  up-axis mismatch all along.

## Why config beats a baked stage

Baking bought nothing at runtime. A `.usd` stores **no Cesium tiles** (they
stream from the network every launch either way) and **no mesh data** (the map
model is a payload pointing at `metashape/output/…`). The 7.7 KB file held
about eight numbers, all recoverable from it:

```
georeference   lat 13.66156872  lon 100.298235  height 0.0
tileset        Google Photorealistic 3D Tiles, ionAssetId 2275207   (present twice — stray duplicate)
map model      payload ../../../metashape/.../survey_mesh_obj.usd  (relative to sim/stages/;
               resolves to /home/innovation/pai/metashape/output/survey_040_max_converted/)
               globe anchor: lat 13.662013797285455  lon 100.29186024766956  height -27.141119462205456
ground plane   visibility invisible, height -26.99
```

The decisive detail is in the model entry: it is **globe-anchored**
(`cesium:anchor:latitude/longitude/height`), not placed by dragging. Cesium
derives its local transform from geographic coordinates, so its placement is
expressed in a form with no opinion about up-axis. The manual "fix its
orientation to 0,0,0" step is a globe anchor doing its job.

Building the stage from config therefore:

- **makes the up-axis bug unreachable** — a fresh stage is authored Z-up, so
  `set_stage_up_axis("z")` is a no-op by construction. There is no stored
  artifact whose up-axis can disagree with Isaac.
- makes a site diffable and reviewable, instead of opaque binary USD.
- retires `fix_ground_plane()` from the automated path: the plane is authored
  flat in a Z-up world, so there is nothing to un-flip.
- makes a second site a new config file rather than a new bake session.

## Architecture

```
sim/launch-sitl.sh          shell: validate, export env, exec isaac-sim.streaming.sh
  └── kit --exec sim/bootstrap.py
        ├── sim/site.py     site config: dataclass + SITES dict (no secrets)
        ├── sim/stage_builder.py   build_stage(site) -> assembles the stage
        └── drone_setup_px4_cesium.py   (unchanged, exec'd as today)
```

`stage_builder.py` is a pure stage-authoring module with no launch or asyncio
concerns; `bootstrap.py` keeps the sequencing (frames, waits, Play). That split
is what makes the builder testable at all — see Testing.

### Build order (this order matters)

```
create_new_stage_async()              # fresh, empty
set_stage_up_axis("Z")                # BEFORE any content is authored
set_stage_units(meters_per_unit=1.0)
  -> cesium data prim + ion server token
  -> georeference (lat, lon, height)  # PX4 GPS origin is read back off this
  -> tileset(s) via add_tileset_ion
  -> map model payload + globe anchor
  -> ground plane (Z axis, z_position, invisible)
  == stage ready; hand off to drone_setup_px4_cesium.py
```

Up-axis is set before content because a globe anchor's local transform is
resolved against the stage's up-axis; authoring anchors into a Y-up stage and
flipping afterwards is precisely the bug being fixed.

## Components

### `sim/sites.py`

```python
@dataclass(frozen=True)
class Tileset:
    name: str
    ion_asset_id: int

@dataclass(frozen=True)
class ModelAnchor:
    usd_path: str          # absolute, or relative to the repo root
    latitude: float
    longitude: float
    height: float

@dataclass(frozen=True)
class Site:
    name: str
    latitude: float
    longitude: float
    height: float          # georeference origin height; 0.0 for this site
    tilesets: tuple[Tileset, ...]
    models: tuple[ModelAnchor, ...]
    ground_z: float        # metres, Z-up, relative to the georeference origin
    spawn_xy: tuple[float, float]
    spawn_agl_m: float     # takeoff clearance ABOVE ground_z, not above origin
    heading_deg: float

SITES: dict[str, Site]     # keyed by name; "bangkok-survey-040" is the first
```

The module is `sites.py`, not `site.py`: `site` is a stdlib module that Kit's
own interpreter imports, and shadowing it on `sys.path` is a trap.

Spawn altitude is expressed as clearance **above the ground plane**, and the
builder derives `spawn_z = ground_z + spawn_agl_m`. Storing an absolute z
invites exactly the mismatch the baked stage had: `SPAWN_XYZ = (0, 0, 0.5)`
against a ground plane at `z = -26.99` is a 27 m drop on Play.

No token field. The ion token is a live secret and reaches the build only via
`CESIUM_ION_TOKEN` in the environment.

`ground_z` is `-26.99`, carried over from the baked stage's Y-up translate
`(0, -26.99, 0)` — under Y-up the height component *is* Y, so the magnitude
transfers directly to Z-up with no sign change.

### `sim/stage_builder.py`

```python
async def build_stage(site: Site, ion_token: str) -> None
```

Verified API surface, all confirmed present in this install:

| Need | Call |
|---|---|
| fresh stage | `stage_utils.create_new_stage_async()` (`impl/stage.py:283`) |
| up-axis / units | `stage_utils.set_stage_up_axis("Z")` (`:648`), `stage_utils.set_stage_units(meters_per_unit=1.0)` (`:567`, keyword-only) |
| cesium data prim | `usdUtils.get_or_create_cesium_data()` |
| georeference | `usdUtils.get_or_create_cesium_georeference()` then `GetGeoreferenceOriginLatitudeAttr().Set(...)` (also `…LongitudeAttr`, `…HeightAttr`) |
| tileset | `usdUtils.add_tileset_ion(name, asset_id, token)` |
| globe anchor | `usdUtils.add_globe_anchor_to_prim(path)` then `GetAnchorLatitudeAttr().Set(...)` (also `…LongitudeAttr`, `…HeightAttr`) |
| ground plane | `isaacsim.core.api.objects.GroundPlane(prim_path, z_position=…, visible=False)` — already takes `"Z"` and a z offset internally |

The ion server prim is **not** created here. `cesium.omniverse`'s
`extension.py:525 _setup_ion_server_prims()` auto-defines `/CesiumServers/IonOfficial`
with its URLs and application id, and wires `selectedIonServer`. The builder
only sets `GetProjectDefaultIonAccessTokenAttr()` on it — creating it if absent,
since that auto-setup is stage-event-driven and its timing relative to our build
is not guaranteed.

### `sim/bootstrap.py` (rewrite)

Loses stage-opening, gains stage-building. Sequence:

```
wait for pegasus.simulator + cesium.omniverse enabled
build_stage(site, token)
settle frames for tile streaming
exec drone_setup_px4_cesium.py   (task-diff await, as today)
verify: georeference lat/lon read back == site lat/lon
play()
```

The verification step is step 7 of the manual routine — "check drone position
is the same for both QGC and Isaac" — reduced to an assertion. It is cheap
because `drone_setup_px4_cesium.py`'s `read_cesium_georeference()` already
derives the PX4 GPS origin from the georeference prim; the check confirms the
prim we authored is the prim it found.

### `sim/launch-sitl.sh` (amend)

- drop `STAGE`, add `SITE` (default `bangkok-survey-040`)
- fail fast with an actionable message if `CESIUM_ION_TOKEN` is unset
- keep the existing `--ext-folder`/`--enable`/`--exec` wiring and the
  `DRONE_SETUP_*` override pass-through, both of which work

### `drone_setup_px4_cesium.py` (one-line default change)

`FIX_GROUND_FLIP` flips to `False`. The builder authors the plane flat in a
Z-up stage; `fix_ground_plane()` would clear its xform ops and force
`rotateX=90`, breaking a correct plane. The function is **kept**, because the
paste-into-Script-Editor path still meets hand-built Y-up stages where it is
the right tool.

## Error handling

Every failure below is one a silent success would hide for a whole flight.

| Failure | Response |
|---|---|
| `CESIUM_ION_TOKEN` unset | shell exits before launching Isaac, naming the variable |
| unknown `SITE` | builder raises listing the valid `SITES` keys |
| model USD path missing | builder raises with the resolved absolute path — a stale `metashape/output/…` path is the likeliest breakage |
| stage up-axis not Z after build | builder raises; the guard that makes this bug class permanently loud |
| georeference read-back ≠ site coords | bootstrap logs an error banner and leaves the sim **stopped**, rather than flying from the wrong origin |
| extensions never enable | existing banner, unchanged |

The app is left running on error, as today, so the WebRTC stream can be
attached to inspect a half-built stage.

## Testing

The honest constraint: `bootstrap.py` and `stage_builder.py` cannot run outside
Kit, and this box's only GPU is the one Isaac needs. So the split is:

**Unit tests, `sim/tests/test_sites.py`** — real pytest, runs anywhere, follows
the existing `streaming/tests/` pattern:

- `SITES` contains the bangkok site with the coordinates in this doc
- every `ModelAnchor.usd_path` resolves to a file that exists
- no `Site` carries anything token-shaped (guards against a secret being pasted
  into config later)
- `Site` is frozen — config is not mutated at runtime
- `spawn_z` lands just above `ground_z`, never at an absolute altitude that
  ignores the ground plane
- `get_site` raises a `KeyError` naming the known sites

**Manual verification, once, on the box** — the parts only a real run can prove:

1. `./sim/launch-sitl.sh` reaches Play with no error banner
2. the up-axis assertion passes (it is the regression test for this bug)
3. tiles and the survey mesh render in the right place, not rotated
4. QGC shows the drone at `13.6615°N, 100.2982°E`
5. the drone rests on the invisible plane rather than falling through it
6. `curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8080/detect` → `200`

Steps 3–5 are the ones that failed last time; they are the acceptance gate.

## Out of scope

- auto arm + takeoff (deliberate human gate; roadmap V2a territory)
- launching QGroundControl from the script (asked for and declined)
- converting the existing Y-up `sim/stages/sitl-stage.usd` — it is deleted, its
  numbers already extracted into `site.py`
- multiple simultaneous vehicles

## Consequences

- `sim/stages/` and its `.gitignore` entry are removed.
- `joystick-guide.md` §3 loses the bake procedure and gains the token setup.
- The `metashape/output/…` model path becomes a load-bearing external
  dependency, previously hidden inside the USD. The unit test makes it loud.
- The Cesium ion token found in the baked stage was printed to a terminal
  during this investigation and should be rotated.
