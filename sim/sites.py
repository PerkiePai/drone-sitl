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
    """Pin the prim's local stage Z, leaving X/Y as the globe anchor placed them.

    `height` above is metres above the WGS84 ellipsoid and does NOT map 1:1 onto
    stage Z: an anchor height of -25 resolved to a local translate of ~147. When
    the mesh needs to sit at a known stage height, set it here. None leaves
    whatever the anchor produced.
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
            translate_z=-25.0,
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
