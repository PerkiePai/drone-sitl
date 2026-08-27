"""Goto: fly to a point ~40 m north of the launch position, then hold.

Goto is stateful -- issue it once and forget it; on_arrival fires when the
aircraft gets there. 40 m north is about 0.00036 degrees of latitude.
"""
from competition import Agent, Command, Goto, Hold


class GoNorth40(Agent):
    def on_start(self, arena):
        lat_min, lon_min, lat_max, lon_max = arena.bounds
        centre_lat = (lat_min + lat_max) / 2
        centre_lon = (lon_min + lon_max) / 2
        self.target = (centre_lat + 40 / 111_320.0, centre_lon)
        return Command(flight=Goto(self.target[0], self.target[1], alt=30.0))

    def on_arrival(self, state):
        print(f"arrived at {state.lat:.6f}, {state.lon:.6f}")
        return Command(flight=Hold())
