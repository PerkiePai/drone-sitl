# Competitor submission examples

**Status: these run against the `competition` package today.** `flight()`,
`Route`, `Command` and the `Agent` callbacks are all implemented — see
`competition/commands.py` and `competition/__init__.py`. `single_flight.py`
needs nothing beyond the package; `full_flight_agent.py` additionally needs
`detection.py`'s deps (`torch`, `ultralytics`) and your own `weights.pt`,
which is what the Dockerfile is for.

Still open: the *design* in
`docs/superpowers/specs/2026-09-07-competition-flight-primitive-design.md`
is **PAUSED** on the Route-vs-ATE conflict (see its ⚠ section), so `Route`'s
role may still change. `flight()` itself is settled. Full packaging
rationale is in
`docs/superpowers/specs/2026-09-08-competitor-submission-docker-design.md`.
This repo is a sandbox; the real competition runs a separate, custom FC —
see the project memory `uav-competition-vs-sitl-repo`.

## Files

| File | What it shows |
|---|---|
| `single_flight.py` | Smallest possible script: hover → fly to a waypoint → spin. `flight()` only, no camera, no detection. |
| `full_flight_agent.py` | Lawnmower sweep flown on `flight()`, `on_frame` runs a detector, confident hit → switch to the oblique camera and orbit, then resume the sweep. |
| `detection.py` | `Detector`/`Detection` — everything about the model, nothing about flying. Assumes an ultralytics-style `.pt` checkpoint; swap `_load()`/`detect()` for your own model format. |
| `Dockerfile` | Builds a submission image on top of an organizer-published `competition-base:latest`. |
| `requirements.txt` | `full_flight_agent.py`'s extra deps (`torch`, `ultralytics`) on top of what `competition-base` already provides. |

`weights.pt` is not included — supply your own trained checkpoint at
build time; the Dockerfile's `COPY` expects it next to `detection.py`.

## More than one weight file?

Not a special case. `Detector` is one model per instance — instantiate it
twice (or more) if you need more than one:

```python
self.car_detector = Detector("car_weights.pt")
self.person_detector = Detector("person_weights.pt")
```

`COPY` in the Dockerfile handles any number of files the same way; just
list them all.

## Build & run

```bash
docker build -t my-submission .
docker run --network host --gpus all my-submission
```

`--network host`: the container reaches mavlink (`udp:14540`) and the
camera server (`:8080`) over the host's loopback, same as this repo's
`agent_runner.py` subprocess does today — still one machine, no real
network hop, Docker only changes how the code is packaged and launched.

`--gpus all`: `detection.py` falls back to CPU automatically
(`torch.cuda.is_available()`) if no GPU is passed through, so the example
still runs without it — just untested against the 200 ms `on_frame`
budget in that mode.

## `flight()` cheat sheet

```
flight(left_x, left_y, right_x, right_y)   # each in [-1, 1]
  left_y  = thrust   (+1 = full climb)
  left_x  = yaw       (+1 = full right / clockwise)
  right_y = pitch      (+1 = full forward)
  right_x = roll        (+1 = full right / strafe)
```

Full deflection on a translation axis = 5 m/s. See the flight() spec's
Decisions 3–5 for the full rationale, including the world→body rotation
both example files use to steer at a lat/lon target.
