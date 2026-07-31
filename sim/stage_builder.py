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
    print(f">>> model {model.prim_name} anchored at "
          f"{model.latitude}, {model.longitude}, {model.height}")
