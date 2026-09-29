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
    zero_orientation: bool = True
    """Force the prim's rotation to identity after anchoring.

    The Metashape export bakes a non-identity orient into the model USD's own
    root prim, which composes through the payload and lands the mesh tilted.
    Zeroing it is the automated form of the manual step "add the map model USD,
    fix its orientation to 0, 0, 0".
    """

    translate_z: float | None = None
    """Escape hatch: pin the prim's local stage Z, leaving X/Y alone.

    Not the primary mechanism. `height` drives placement, and over this site's
    ~690 m origin-to-model distance it tracks local stage Z to within 0.037 m,
    so `height = -25.0` lands the mesh at z ~= -25.04. Use this only if Cesium
    is observed overwriting that.
    """

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
    stage_usd: str | None = None
    """A pre-authored map stage (map_setup_tool output) layered in as a sublayer
    instead of building tilesets/models from this config. It brings its own
    georeference, terrain collider and physics scene, so `tilesets`/`models` are
    ignored and no ground plane is added. `latitude`/`longitude`/`height` must
    still match the stage's CesiumGeoreference -- bootstrap reads it back and
    warns on a mismatch. Relative to REPO_ROOT, or absolute."""

    """`ground_z` is the invisible collision plane the drone rests on. It must
    match the height of the map model the drone appears to be standing on --
    the manual step was "move the ground plane to the same ground as in the 3D
    model". Set it below anything and the drone falls; above, and it hovers on
    an invisible floor. `spawn_z` is derived from it so the two cannot drift.
    """

    @property
    def uses_cesium(self) -> bool:
        """False for a map-stage site: nothing streams, so the Cesium extension,
        ion token and tile cache are all unnecessary (and the tile fetching is
        what made the sim laggy)."""
        return self.stage_usd is None

    def resolved_stage_path(self) -> Path | None:
        if self.stage_usd is None:
            return None
        path = Path(self.stage_usd).expanduser()
        return path if path.is_absolute() else (REPO_ROOT / path).resolve()

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
            height=-25.0,
        ),
    ),
    # Matches the survey mesh's height above, so the drone rests on the model's
    # ground rather than 2 m under it. The old -26.99 came from the Y-up baked
    # stage and no longer corresponds to anything.
    ground_z=-25.0,
    spawn_xy=(0.0, 0.0),
    spawn_agl_m=0.5,               # -> spawn_z = -24.5
    heading_deg=0.0,
)

# Values from drone_map_report.json (map_setup_tool): the stage's georeference,
# and the auto-found spawn pad (ground_z 5.9826 -> spawn_enu z 6.4826).
NT_TESTGS = Site(
    name="nt-testgs",
    latitude=14.028519616,
    longitude=100.43642032,
    height=-35.002,
    tilesets=(),
    models=(),
    ground_z=5.9826,
    spawn_xy=(4.7762, -19.3996),
    spawn_agl_m=0.5,               # -> spawn_z = 6.4826
    heading_deg=0.0,
    stage_usd="/home/innovation/Tiger/map_setup_output_nt_testgs/drone_map.usda",
)

SITES: dict[str, Site] = {s.name: s for s in (BANGKOK_SURVEY_040, NT_TESTGS)}


def get_site(name: str) -> Site:
    """Look up a site by name, raising with the valid options if it is unknown."""
    try:
        return SITES[name]
    except KeyError:
        raise KeyError(f"unknown site {name!r}; known sites: {sorted(SITES)}") from None
