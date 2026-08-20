# Design: competitor-facing drone control API (UAV search-and-read competition)

**Status:** design under discussion — no implementation yet
**Date:** 2026-08-09 (revised same day: gimbal dropped, two fixed cameras adopted)

> **This describes the competition rig, not this repo.** The competition runs on
> Isaac Sim with a **custom flight controller (not PX4)** and a Jetson in the
> loop. This SITL repo is the R&D sandbox the ideas are drawn from; where a
> mechanism here already exists in the sandbox it is cited, because a working
> precedent is worth more than a fresh opinion.

## The task

A multirotor searches an arena for a target vehicle and must **read the text on
it** through onboard cameras. Competitors submit a control/perception script;
the organiser runs it unattended and returns a log and a score.

The challenge is deliberately **perception, not airmanship**. Nobody is scored on
manual flying skill, so nobody should be writing PID loops. The intended winning
behaviour is: sweep high and wide on the nadir camera → spot a candidate →
divert to a standoff and read it on the oblique camera → move on, with the best
algorithm doing this fastest and with fewest wasted detours.

## Architecture

```
     ┌──────────────────────────┐            ┌────────────────────────────────┐
     │ Jetson (edge AI box)     │            │ Isaac Sim host                 │
     │                          │            │                                │
     │  competitor script       │            │  ┌──────────────────────────┐  │
     │   on_frame(image, state) │◄── obs ────┼──┤ harness                  │  │
     │   on_tick(state)         │            │  │  owns the loop           │  │
     │        │                 ├── cmd ────►│  └──────────┬───────────────┘  │
     │        ▼                 │            │             ▼                  │
     │  returns Command         │            │  ┌──────────────────────────┐  │
     └──────────────────────────┘            │  │ our flight controller    │  │
                                             │  │  attitude/rate ~250Hz–1k │  │
       best-effort, no deadline              │  └──────────┬───────────────┘  │
       (slow script = sluggish drone)        │             ▼                  │
                                             │      Isaac Sim physics         │
                                             └────────────────────────────────┘
                                                    real-time, free-running
```

The single most important property: **the deadline-bearing loop and the
network hop are on opposite sides of the flight controller.** The inner
stabilisation loop is local to the Isaac host — no network, no Jetson — so its
timing is trivially met. The competitor's loop crosses the wire but has **no
deadline at all**.

---

## Decisions

### D1 — Jetson is an edge AI box, not the flight controller

The Jetson runs the competitor's perception/decision script. Stabilisation is
never theirs.

### D2 — The flight controller is co-located with Isaac Sim

Not on the Jetson. This means the FC↔physics path is local and the FC↔competitor
path is the only network hop.

### D3 — The simulation free-runs in real time. It is NOT lockstep

Considered and rejected. Lockstep (sim clock waits for the Jetson each tick)
guarantees no dropped command and gives perfect reproducibility, but it is
mutually exclusive with running in real time — a slow script would simply make
the sim take longer rather than fly worse.

Free-running was chosen because **it makes algorithm speed self-penalising with
no scoring rule at all.** A script that thinks for 300 ms is a drone that flew
blind for 300 ms: sluggish, drifty, bad at chasing a vehicle. Efficiency is
enforced by physics, not by a formula the organiser has to defend.

Consequence: runs are not bit-reproducible. Accepted.

Consequence: the original worry — "if latency exceeds 10 ms the HITL link drops
and the drone falls" — **does not apply**, because of D4.

### D4 — The API is setpoint-level. Competitors never touch attitude, rate, or motors

This is what makes D3 safe. A velocity setpoint stays physically valid until
countermanded, so a late command is not a missing command — the aircraft keeps
doing the last sensible thing. There is no starvation failure mode.

Exposing rate/attitude control would move the hard deadline onto the
competitor's side of the network, where it cannot be met, and would also make
the competition about writing controllers rather than about perception.

```
FC inner loop — ours, Isaac host, real-time ~250Hz–1kHz   ← hard deadline, local
      ▲
      │  velocity / waypoint setpoints, best-effort ~20Hz  ← no deadline, networked
      │
competitor script — Jetson, as slow as it likes
```

Precedent: this is exactly the shape of `joystick-server.py` in this repo — 20 Hz
OFFBOARD velocity setpoints driven by a human thumb pressing buttons whenever it
feels like it. The competitor script replaces the thumb.

### D5 — Inversion of control: the harness owns the loop

Competitors implement callbacks; they do not write `while True`.

```python
class MyAgent(CompetitionAgent):
    def on_frame(self, image, state) -> Command | None:   # ~2–5 Hz, heavy perception
        ...
    def on_tick(self, state) -> Command:                  # ~20 Hz, cheap steering
        ...
```

Rationale: the organiser controls tick timing, can wrap any single call in a
timeout, and a hung submission costs one tick rather than hanging the whole sim.
It also narrows the surface for tampering between ticks.

**Two entry points, not one.** A good competitor wants a heavy detector at 2 Hz
and steering at 20 Hz. A single `on_frame` would force both into one budget and
push competitors into threading it themselves. Splitting them gives the fast/slow
decomposition for free, and means the expensive image is only serialised when
actually wanted — which matters a great deal under D8.

Returning `None` means "carry on as before," so a slow detector never stalls the
aircraft.

### D6 — Two fixed cameras, no gimbal, no variable zoom

**Revised.** An earlier draft assumed a pan/tilt gimbal with variable zoom. The
rig is now **two rigidly-mounted cameras**, which is simpler to build and, as it
turns out, strictly better: two fixed lenses provide "zoom" by *camera
selection*, so variable optics can be deleted entirely. No moving parts.

| | **Nadir — SEARCH** | **Oblique — INSPECT** |
|---|---|---|
| Role | detect candidates during the sweep | read the text |
| Focal length | 2.2 mm (wide) | **~18 mm (tele)** |
| HFOV | 113° | ~20° |
| Mounting | straight down | ~30–35° depression (open, see below) |
| At 117 m cruise | **352 m swath** | — |
| At ~100 m slant range | — | 36 m footprint, ~1.9 cm/px |
| Reads 30 cm text? | **no — deliberately** | **yes, ~16 px, at the OCR floor** |

That "no" in the nadir column is load-bearing, not a shortcoming: it is what
forces the two-phase search-then-inspect strategy (see *Coverage* below).

**Consequence — gaze is now coupled to flight.** With no gimbal, the camera
points wherever the airframe points, so aiming became a *flight* command rather
than a camera one:

```diff
- set_gimbal(pan, tilt)
- LookAt(lat, lon, alt)
- LookAtPixel(u, v)
- set_zoom(factor)
+ set_camera("nadir" | "oblique")
+ inspect(lat, lon)     # harness solves standoff, altitude and heading
```

**Consequence — a multirotor tilts to accelerate.** Attitude *is* gaze now, and a
multirotor's attitude is dictated by its acceleration:

```
hovering        →  camera truly nadir
cruising 20 m/s →  ~15–30° nose-down tilt to balance drag
                →  a "nadir" camera actually looks ~55 m AHEAD at 117 m altitude
accelerating    →  the footprint swings
```

So the search swath is a moving trapezoid with GSD varying across the frame, and
**a stable true-nadir view exists only when the drone is not accelerating.**
Reading therefore requires slowing and letting attitude settle, which costs
seconds every time. This is treated as a *feature* — it makes inspecting a
candidate genuinely expensive, so detector precision matters — but it must be
documented, because it is invisible until it bites.

Related precedent: this repo's hard-mounted down camera already models a silicone
anti-vibration mount and a global shutter precisely because there is no gimbal to
isolate it (`drone_setup_px4_cesium.py:205`). Carry both over — rolling-shutter
jello on a rigid camera would wreck OCR.

**Yaw matters again**, and only for the oblique: with a rigid camera it is the
sole horizontal aiming authority. Hence `inspect(lat, lon)` — the harness solves
the standoff triangle so that competitors are not all writing the same
trigonometry.

### D7 — Command vocabulary: 4 flight DOF + camera selection

A multirotor has **four** independently controllable degrees of freedom, not six.
Roll and pitch are not free variables — they are *how* the aircraft accelerates
horizontally, so commanding them separately from translation is contradictory.
The original "6-axis joystick" idea resolves to 4 flight DOF; with the gimbal
gone, the other two channels no longer exist.

```python
Command(
    flight = Velocity(fwd, right, up, yaw_rate)      # body frame
           | VelocityNED(north, east, down, yaw_rate) # world frame
           | Goto(lat, lon, alt, speed)
           | Route([wp, ...], speed)
           | Inspect(lat, lon)                        # standoff solved by harness
           | Hold(),
    camera = "nadir" | "oblique",
)
```

**Continuous floats, not discrete directions.** WASD is a human affordance,
invented because fingers can only press or not-press. A script has no fingers;
discrete directions would force it to emulate continuous control by toggling,
which is strictly worse than stating the number it wants.

**Ship both frames, world as default.** This repo uses body frame ("forward is
where the nose points") because a human pilot thinks that way. A script does not
— it thinks in map coordinates, because that is where its detections live.

**Strafe is in scope.** `joystick-server.py` deliberately omits it (left/right
yaw instead), which suits a human but not an orbit-to-read manoeuvre.

**Waypoints vs joystick was a false choice — the task needs both, in different
phases.** The area sweep is a `Route` (hand-steering a 30-minute lawnmower
pattern tests patience, not perception); inspection is `Inspect`/`Goto`/
`Velocity`. `Route` running autonomously while `on_frame` keeps evaluating is
what lets a competitor sweep and detect at the same time. Mid-route takeover is
already solved in `streaming/waypoints.py`.

**Geometry helpers must be provided**, or every competitor rewrites them
identically:

```python
state.pixel_to_ground(u, v)     # ray-cast a pixel to lat/lon on terrain
state.ground_to_pixel(lat, lon)
```

Without `pixel_to_ground`, "I see a car at pixel (840, 320) — where is it?" is a
camera-intrinsics-plus-terrain-intersection problem every team must solve the
same way. That is a tax on geometry skill, not the perception skill being scored.

### D8 — Statefulness is hybrid, split on hazard

| Command | State | Rationale |
|---|---|---|
| `Route` | stateful, survives a silent script, explicit abort | a pre-declared intention; continuing unattended is correct |
| `Velocity` | stateless, watchdog decays to hover | a stuck velocity is a hazard |
| `Goto` / `Inspect` / `Hold` | stateful | safe by nature |

This is not a new decision — it is the rule this repo already arrived at and
wrote down (`docs/joystick-guide.md:422`):

> A closed browser does not stop a route. Mission state lives on the server... That is deliberate: the input watchdog exists because a stuck manual key is a hazard, whereas an autonomous route completing unattended is correct.

A 30-minute sweep re-issued at 20 Hz would be ~36,000 redundant messages for one
intention; the hybrid takes that saving where it is safe and keeps the dead-man's
switch where it is not.

### D9 — Image delivery: competitor selects one camera; two quality tiers

**The 10 ms concern is real, but it is a bandwidth problem, not a clock-sync
problem.**

```
1920 × 1200 × 3 bytes  =  6.9 MB per frame
                          ↓ over 1 GbE
                       ~55 ms   ← 5.5× the budget, transfer alone
                          ↓ at 30 Hz
                     ~1.66 Gbps ← does not fit on the wire at all
```

| Approach | Latency | Cost |
|---|---|---|
| Raw full frame, 1 GbE | ~55 ms | over budget |
| Raw full frame, 10 GbE / direct | ~6 ms | needs the NIC; still 1.66 Gbps at 30 Hz |
| JPEG q90 | ~4 ms wire + ~5 ms encode | artifacts on small text |
| H.264 NVENC→NVDEC | ~5–10 ms | both boxes are NVIDIA; still lossy |
| Full-res **lossless** ROI crop (640×480) | ~7 ms | only works once you know where to look |

The trap in the middle rows: **lossy compression destroys exactly the small
high-frequency detail OCR depends on.** Compress hard enough to hit 10 ms
globally and the text may become unreadable for everyone — making the codec, not
the algorithm, decide the winner.

Chosen design:

- **One camera at a time.** `set_camera()` selects which feed `on_frame`
  delivers. Streaming both would double the bandwidth that is already the binding
  constraint. It also makes camera choice a genuine strategic decision — a
  competitor who leaves it on oblique gets a 20° soda straw and covers ~5% of
  the arena. That is a good mistake to make available.
- **Search tier** — selected camera, H.264, 5–10 Hz. Detecting a ~24 px vehicle
  survives compression comfortably.
- **Inspect tier** — `request_roi(camera, u, v, w, h)` returns a **lossless
  full-resolution crop**. A competitor who has localised a candidate needs a
  640×480 patch, not 10 km² of pixels.
- **State** — pose, attitude, velocity, time remaining etc. ride every tick at
  20 Hz. A few hundred bytes; costs nothing.

Note `request_roi` names its camera explicitly, so a competitor can pull a
lossless oblique crop while still sweeping on the nadir feed.

### D10 — Control style is free above the stabilisation layer, and nowhere below it

The flight modes are not four features but four levels of abstraction over one
primitive. Competitors pick whichever suits their algorithm:

| Style | Calls | Suits |
|---|---|---|
| Reactive / RL-ish | `set_velocity()` every tick | learned policies, visual servoing |
| Point-to-point | `goto()` / `inspect()` per decision | classical planners |
| Declarative | `follow_route()` once | coverage planners |
| Mixed | route + takeover on detect | the intended strategy |

Modes may be **mixed freely within a run** with no switch ceremony — route, then
velocity on detection, then back — as `streaming/waypoints.py` already does
without leaving OFFBOARD. Style freedom that requires declaring your style up
front is a menu, not freedom.

The one style that **cannot** be offered is "bring your own stabiliser" (attitude
or rate control from the Jetson), for the reason in D4. State this in the rules
explicitly: *"a 20 ms hiccup would put it in the ground"* is a far stronger
answer than *"we didn't want to."* Velocity is the floor — do not descend to
acceleration or thrust-vector even as an expert mode, since a stale acceleration
setpoint diverges **quadratically** rather than linearly.

**⚠ The layers are not equally advantaged, and this must be documented.**

```
follow_route()   → runs entirely on OUR box, closed-loop at the FC's full rate
                   network jitter never touches it

set_velocity()   → crosses the wire every tick
                   control quality bounded by link jitter + script speed
```

A competitor sweeping via `follow_route` flies a cleaner, faster line than one
hand-rolling the same sweep with `set_velocity` — not because they are better,
but because their control loop does not cross a network. This is acceptable and
arguably correct (it rewards understanding the system), but it creates two
obligations:

1. **Document the asymmetry**, so competitors choose knowingly. An undocumented
   advantage found by one team on day 3 is a scandal; a documented one is
   strategy.
2. **Tune every mode to the same standard.** A polished `Goto` and a sloppy
   velocity path means a style has been secretly mandated while a choice is
   advertised. This is the ongoing cost of offering style freedom.

---

## Physics that constrains the arena and cameras

Derived during design; recorded because these numbers decide whether the intended
strategy is possible at all.

### Legibility

For a pinhole camera, ground sample distance is `GSD = range / fx`. Taking ~16 px
of character height as the floor for reliable OCR:

```
max readable range  ≈  character height × fx / 16
```

With 3.45 µm pixels (fx = focal_mm / 0.00345):

| Lens | fx (px) | Readable range for 30 cm text |
|---|---|---|
| 2.2 mm wide (nadir) | 638 | **12 m** — cannot read from cruise |
| 18 mm tele (oblique) | 5,217 | **~98 m** — reads from standoff |

Two consequences carried from the earlier single-camera analysis:

1. **A single wide lens makes "fly high, spot, read" impossible** — the detail was
   never captured, and digital zoom only magnifies pixels that never contained
   the letters. The dominant strategy would degenerate to hovering at 12 m over
   every vehicle: ~50 s of elevator ride per candidate. The tele lens is what
   turns that into a ~2 s read.
2. **Downsampled streams silently destroy the task.** The 640×400 stream in this
   repo throws away 2/3 of the resolution in each axis. Competitors must get the
   native feed (D9).

### Coverage

The two inequalities that must **both** hold for the two-phase strategy to be the
optimal one:

```
COVERAGE    swath_nadir × speed × time  ≥  arena area   ← wide lens CAN sweep it
LEGIBILITY  nadir cannot read the text at cruise        ← wide lens CANNOT read it
```

If coverage fails, the winner is whoever guessed the right corner. If legibility
accidentally succeeds at search altitude, nobody ever diverts and the oblique
camera is dead weight.

For a 2 × 5 km arena (10 km²) in 30 min: 10,000,000 m² ÷ 1800 s = **5,556 m²/s**
with zero overlap and zero turnaround. Real lawnmower patterns lose 20–40% to
turns, so budget ~7,000 m²/s.

| Cruise | Swath needed | Altitude (113° HFOV) | Vehicle (4.5 m) at native 1920 px |
|---|---|---|---|
| 15 m/s | 470 m | 156 m | ~18 px — marginal |
| **20 m/s** | **350 m** | **117 m** | **~24 px — workable** |
| 25 m/s | 280 m | 93 m | ~31 px — comfortable |

**The budget closes.** At 117 m the nadir swath is 352 m; at 20 m/s that is
7,040 m²/s, so the sweep takes **≈23.7 min**, leaving **~6 min** for inspections.
Because the tele reads at ~100 m slant range, an inspection is a short *diversion*
rather than a descent — seconds, not the ~50 s a single-camera rig would have
cost. With one camera the budget did not close; with two it does.

### Inspection geometry

```
        drone ●  60 m altitude
              │╲
              │ ╲  120 m slant range, ~1.9 cm/px on the 18 mm tele
         60 m │  ╲
              │   ╲  30° depression
              └────▼──────
                 104 m standoff        🚗
```

With a rigid camera the depression angle **is** the geometry: fixing the angle
fixes the relationship between inspection altitude and standoff distance.

---

## Open questions

Not yet decided; each changes the design above.

1. **Oblique depression angle.** 30° sees vehicle sides well but needs a long
   standoff; 45° is a compromise; 60° mostly sees roof. Recommended **30–35°**
   given side-mounted text.
2. **Text placement and size.** Recommended: **both sides, identical, 30–50 cm
   characters.** Identical on both sides matters — one-sided markings make
   legibility depend on approach direction, which is luck rather than skill.
   Roof-only would muddle the two cameras' roles; sides-only keeps them crisp
   (nadir *finds*, oblique *reads*, neither does the other's job).
3. **Exact sensor choice.** Resolution and pixel pitch are assumed at 1920×1200 /
   3.45 µm from this repo's ZED; the focal lengths above must be re-derived if
   the real sensor differs.
4. **Blind vs cued search.** Blind over 10 km² has very high variance and tests
   search-pattern geometry more than perception. A cue ("last seen near here")
   preserves the search problem and leaves time for the find→read loop.
   Alternative: shrink to ~1 × 1 km and keep it blind.
5. **Decoy count**, and whether decoys are visually similar vehicles with
   *different* text (forcing OCR to be the discriminator rather than "find the
   red car"). The ~6 min inspection budget caps how many candidates a competitor
   can afford to visit — with 15 decoys nobody finishes.
6. **Battery/endurance model.** 30 min at 20 m/s is 36 km — beyond any real
   multirotor. If endurance is simulated it becomes the binding constraint
   instead of time, and competitors will optimise for it.
7. **Answer submission.** How the competitor reports the text read: one shot,
   multiple guesses, confidence values, when the run ends.
8. **Geofence, collision, and crash handling.** What happens on a boundary breach
   or a ground impact.
9. **Motion blur.** A rigid camera at 20 m/s over low terrain will smear. Whether
   Isaac simulates it, and whether it is enabled, decides how hard competitors
   must slow down to read.
10. **Isaac Sim tick rate**, and the FC inner-loop rate it drives.
11. **Rendering throughput.** 10 km² of photorealistic content viewed from 117 m
    is a heavy LOD/streaming load. This repo's own experience is that when
    rendering falls behind, sim time slows (`docs/joystick-guide.md:501`) —
    across many submissions that is an organiser-side throughput problem.
