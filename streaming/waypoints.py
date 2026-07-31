"""Waypoint sequencing for autonomous missions.

Pure logic: no MAVLink, no I/O. The setpoint loop asks advance() for a target
every tick and sends whatever it gets back; everything about "which waypoint
are we on" lives here and nowhere else.

Distances are great-circle. Legs in this application are tens to hundreds of
metres, where a flat approximation would also work -- haversine is used because
it is the same handful of lines and has no latitude at which it quietly stops
being true.

Design: docs/superpowers/specs/2026-07-31-map-waypoints-design.md
"""
import math
import threading

EARTH_RADIUS_M = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance between two lat/lon points, in metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = p2 - p1
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2.0) ** 2)
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial great-circle bearing, in compass degrees (0 = N, 90 = E).

    "Initial" matters: on a long leg the bearing changes as you fly it. Legs
    here are short enough that it does not, but the loop recomputes every tick
    anyway, so the aircraft tracks the true course either way.
    """
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam) * math.cos(p2)
    x = (math.cos(p1) * math.sin(p2)
         - math.sin(p1) * math.cos(p2) * math.cos(dlam))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
