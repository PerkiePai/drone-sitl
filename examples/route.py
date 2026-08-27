"""Route: fly a ~60 m box around the launch point at 25 m, logging progress.

Route runs closed-loop on the simulator host. on_waypoint fires per corner,
on_route_complete at the end -- then this holds the last point.
"""
from competition import Agent, Command, Hold, Route


class BoxSurvey(Agent):
    def on_start(self, arena):
        lat_min, lon_min, lat_max, lon_max = arena.bounds
        clat = (lat_min + lat_max) / 2
        clon = (lon_min + lon_max) / 2
        d = 30 / 111_320.0
        box = [(clat + d, clon - d), (clat + d, clon + d),
               (clat - d, clon + d), (clat - d, clon - d)]
        return Command(flight=Route(waypoints=box, alt=25.0))

    def on_waypoint(self, index, state):
        print(f"reached waypoint {index}")

    def on_route_complete(self, state):
        print("route complete -- holding")
        return Command(flight=Hold())
