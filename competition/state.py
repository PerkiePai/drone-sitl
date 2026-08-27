"""Value objects handed to every Agent callback.

State is a read-only snapshot of the aircraft, rebuilt each tick from the
server's telemetry dict. Arena is fixed for the run.

UNITS match docs/competition-api.md section 3.3: metres, m/s, degrees.
Velocity is +up. Heading 0 = north, positive clockwise.
"""
import math
from dataclasses import dataclass

_DEG_LAT_M = 111_320.0


@dataclass(frozen=True)
class RouteInfo:
    state: str            # "IDLE" | "RUNNING" | "PAUSED" | "DONE"
    index: int            # 0-based waypoint currently targeted
    count: int
    distance_m: float | None


@dataclass(frozen=True)
class State:
    lat: float | None
    lon: float | None
    alt_agl: float
    vx: float             # world north, m/s, +N
    vy: float             # world east,  m/s, +E
    vz: float             # +up, m/s
    ground_speed: float
    heading: float        # deg, 0 = N, +CW
    roll: float           # deg
    pitch: float          # deg
    camera: str
    time_elapsed: float   # s since on_start
    time_remaining: float | None
    route: RouteInfo

    @classmethod
    def from_telemetry(cls, telem, *, camera, time_elapsed, time_limit):
        m = telem.get("mission") or {}
        return cls(
            lat=telem.get("lat"),
            lon=telem.get("lon"),
            alt_agl=float(telem.get("alt_m", 0.0)),
            vx=float(telem.get("vn", 0.0)),
            vy=float(telem.get("ve", 0.0)),
            vz=float(telem.get("vz", 0.0)),
            ground_speed=float(telem.get("gs", 0.0)),
            heading=float(telem.get("heading_deg", 0.0)),
            roll=float(telem.get("roll_deg", 0.0)),
            pitch=float(telem.get("pitch_deg", 0.0)),
            camera=camera,
            time_elapsed=float(time_elapsed),
            time_remaining=(None if time_limit is None
                            else max(0.0, float(time_limit) - float(time_elapsed))),
            route=RouteInfo(
                state=m.get("state", "IDLE"),
                index=int(m.get("index", 0)),
                count=int(m.get("count", 0)),
                distance_m=m.get("dist_m"),
            ),
        )


@dataclass(frozen=True)
class Arena:
    bounds: tuple         # (lat_min, lon_min, lat_max, lon_max)
    time_limit: float | None

    @classmethod
    def around(cls, lat, lon, radius_m, time_limit):
        dlat = radius_m / _DEG_LAT_M
        dlon = radius_m / (_DEG_LAT_M * max(math.cos(math.radians(lat)), 1e-6))
        return cls(bounds=(lat - dlat, lon - dlon, lat + dlat, lon + dlon),
                   time_limit=time_limit)
