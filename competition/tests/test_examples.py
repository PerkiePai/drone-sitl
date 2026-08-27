"""Every examples/ file must load as exactly one Agent and its callbacks must
not raise on a plausible State."""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

import agent_runner  # noqa: E402
from competition.state import Arena, State  # noqa: E402

EXAMPLES = os.path.join(ROOT, "examples")
FILES = sorted(f for f in os.listdir(EXAMPLES) if f.endswith(".py"))

TELEM = {"lat": 13.6, "lon": 100.3, "alt_m": 30.0, "vn": 0.0, "ve": 0.0,
         "vz": 0.0, "gs": 0.0, "heading_deg": 0.0, "roll_deg": 0.0,
         "pitch_deg": 0.0,
         "mission": {"state": "IDLE", "index": 0, "count": 0, "dist_m": None}}


@pytest.mark.parametrize("fname", FILES)
def test_example_loads_as_one_agent(fname):
    cls = agent_runner.load_agent(os.path.join(EXAMPLES, fname))
    assert cls is not None


@pytest.mark.parametrize("fname", FILES)
def test_example_callbacks_do_not_raise(fname):
    cls = agent_runner.load_agent(os.path.join(EXAMPLES, fname))
    agent = cls()
    arena = Arena.around(13.6, 100.3, 500.0, None)
    st = State.from_telemetry(TELEM, camera="nadir", time_elapsed=1.0,
                              time_limit=None)
    agent.on_start(arena)
    agent.on_tick(st)
    agent.on_frame(None, st)
    agent.on_arrival(st)
    agent.on_waypoint(0, st)
    agent.on_route_complete(st)


def test_all_six_primitives_are_covered():
    assert set(FILES) == {"velocity.py", "velocity_world.py", "goto.py",
                          "route.py", "hold.py", "camera.py"}
