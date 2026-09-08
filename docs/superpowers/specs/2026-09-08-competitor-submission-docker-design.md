# Design: Docker submission format + worked flight()/detection examples

**Status:** approved (design settled interactively; not yet grilled, see caveat below)
**Date:** 2026-09-08

## Relationship to existing docs

Sibling to `2026-09-07-competition-flight-primitive-design.md` (**PAUSED**
— unresolved `Route` vs. ATE conflict, see its ⚠ section). This doc is
deliberately independent of that pause: it only depends on `flight()`'s
signature/units (Decisions 3–5 of that doc, which were fully settled
before the pause), and the two worked examples here use **`flight()`
only** — no `Route`, no `Inspect`. Nothing here should be read as
resolving that doc's open question; if the eventual grill changes
Decisions 3–5, this doc's examples need a pass too.

**Caveat carried into everything below:** `flight()` is not implemented
in this repo's `competition` package yet (`competition/commands.py` still
only has `Velocity`/`VelocityWorld`/`Goto`/`Route`/`Hold`, matching the
sandbox's current, unrelated vocabulary). The example scripts below are
reference material for the *target* design — illustrative, not runnable
against this sandbox today. This is consistent with
[[uav-competition-vs-sitl-repo]]: this repo is a sandbox; the real
competition rig is a separate, custom FC.

Extends Decision 1 of the flight() doc ("local subprocess, same
machine, no network hop") rather than replacing it — Docker changes the
*packaging/execution* mechanism, not the *topology*.

---

## Decision 1 — Submission unit: a Docker image, built on an organizer-published base

Competitors submit a Dockerfile (+ their code + weights), not a bare
`.py` file. The organizer publishes `competition-base:latest`, which
already has the `competition` package (`Agent`/`Command`/`State`/
`flight()`, once implemented), `pymavlink`, `numpy`, `Pillow` — everything
`agent_runner.py`'s subprocess path already assumes today. A competitor's
Dockerfile is just:

```dockerfile
FROM competition-base:latest
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY agent.py weights.pt .
ENTRYPOINT ["python", "agent.py"]
```

**Why a base image instead of every submission being fully
self-contained:** the real value Docker adds here is dependency
isolation between competitors — one team's `torch==2.1`/CUDA build
doesn't have to coexist with another's in one shared conda env, the way
`agent_runner.py`'s subprocess model would force today. A base image
means competitors only own the *delta* (their code, their weights, their
extra pip packages), not re-solving `pymavlink`/numpy/Pillow versions
themselves.

## Decision 2 — Networking and GPU: `--network host`, `--gpus all`

Organizer runs `docker run --network host --gpus all <image>`. Still one
machine (Decision 1 of the flight() doc, unchanged) — the container just
uses the host's loopback to reach mavlink (`udp:14540`) and the camera
server (`:8080`) exactly like the current bare-subprocess path does.
`--network host` was chosen over a bridge network + port mapping for
simplicity; that's the natural place to add isolation later if/when the
still-deferred sandboxing work happens. GPU passthrough is assumed
available for the detection example (organizer's box has NVIDIA Container
Toolkit) — not yet confirmed for the real competition machine, see Open
Items.

## Decision 3 — `agent.py` / `detection.py` split; one `Detector` per model

The `Agent` subclass (flight/orchestration) never contains model code.
A separate `detection.py` owns loading weights and running inference:

```python
@dataclass
class Detection:
    class_name: str
    confidence: float
    bbox: tuple            # (x1, y1, x2, y2) pixels in the source image

class Detector:
    def __init__(self, weights_path, device=None): ...
    def detect(self, image) -> list[Detection]: ...
```

`agent.py` calls `self.detector.detect(image)` inside `on_frame`; it
never touches model internals. This mirrors the existing boundary
between `Harness` (flight loop) and `MjpegFrames` (camera decoding) —
each side answerable independently: what does it do, how do you call it,
what does it depend on.

**Multiple weight files are not a special case.** `Detector` is
per-model, not a registry — a competitor wanting two models (e.g. a car
detector and a person detector) just instantiates `Detector` twice:

```python
self.car_detector = Detector("car_weights.pt")
self.person_detector = Detector("person_weights.pt")
```

Deliberately **not** building a `weights=[...]`/ensemble API into
`Detector` — YAGNI. Docker packaging doesn't care either: `COPY` handles
any number of files identically.

---

## File layout

```
examples/competition-submission/
├── README.md              # status caveat, build/run instructions, spec pointers
├── Dockerfile              # FROM competition-base:latest, per Decision 1
├── requirements.txt         # torch, ultralytics -- the detection example's extra deps
├── single_flight.py         # Agent: hover -> waypoint -> spin, flight() only, no camera
├── full_flight_agent.py     # Agent: lawnmower sweep + on_frame detection + camera switch
└── detection.py             # Detector/Detection, per Decision 3
```

`weights.pt` is intentionally not included — it's supplied by whoever
builds the image; the Dockerfile/README just reference where it goes.

## Worked examples

**`single_flight.py`** — the smallest possible script: hover while
acquiring position, fly to a fixed offset waypoint using the Decision-5
world→body rotation from the flight() spec, then spin 360° in place using
`flight()`'s yaw axis alone. No camera, no detection — proves the
minimum shape of a `flight()`-only script end to end.

**`full_flight_agent.py`** — a toy lawnmower sweep flown entirely on
`flight()` (deliberately not `Route`, per the caveat above), nadir camera
by default. `on_frame` runs `Detector.detect()`; on a confident hit it
switches to the oblique camera (`Command(camera="oblique")`) and orbits
briefly (slow yaw) before resuming the sweep where it left off. This is
the shape a real submission combining flight + perception would take.

---

## Open items

1. GPU passthrough (`--gpus all`) assumes the organizer's box has the
   NVIDIA Container Toolkit installed — not yet confirmed for the actual
   competition machine. CPU fallback is in `detection.py` (`torch.cuda.is_available()`
   check) so the example degrades gracefully, but inference speed within
   the 200 ms `on_frame` budget on CPU is unverified.
2. `competition-base:latest` itself doesn't exist yet — it's gated on
   `flight()` actually landing in `competition/commands.py`, which is
   gated on the paused flight()-spec's Route/ATE grill. This doc's
   examples are written against the target shape so that packaging and
   the detection pattern can be worked out now, in parallel.
3. Not grilled. Per the flight() spec's own precedent, this design
   hasn't been stress-tested with `superpowers:grilling` — reasonable
   given it's a smaller, more mechanical decision (packaging format) than
   the vocabulary question, but flagged for consistency.
