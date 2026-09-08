# Design: `flight()` — one dual-stick primitive replaces Velocity/VelocityWorld/Goto/Hold

**Status:** PAUSED — unresolved conflict found (see ⚠ below), to be
grilled (`superpowers:grilling`) before this doc is finalized or turned
into an implementation plan.
**Date:** 2026-09-07

## ⚠ Unresolved: `Route` vs. the ATE score — pick this up first

Late in the conversation that produced this doc, a scoring rule surfaced
that was not in either prior doc: the competition scores **ATE** (how
closely the physical flight path follows the planned track) alongside a
separate **detection score** — leaving the track pauses ATE and starts the
detection-scoring window.

This conflicts with Decision 2 below (which keeps `Route`) and with the
"ATE-pause = Route-pause" mechanism that decision leans on:

- `Route` is organizer-side closed-loop control — it flies identically well
  for *every* competitor regardless of their code. If it's available during
  the ATE-scored track, everyone gets ~max ATE for free just by calling it;
  ATE stops differentiating anyone.
- This also collides with the *original* design doc's founding premise
  ("the task is deliberately perception, not airmanship" — D5). A track-
  following ATE score **is** scoring airmanship, which that premise
  explicitly says nobody should be scored on. `Route` was resolving that
  tension in the wrong direction for what's actually wanted here.

**Leading candidate resolution (not yet locked in):** cut `Route` after
all — for a sharper reason than Decision 2's tick-rate argument: it's not
just smoother, it's *required* for ATE to mean anything. Keep `Inspect`
(inherently an off-track maneuver — solving a standoff triangle — so it
never competes with ATE the way `Route` does). The sweep/track itself is
flown with the competitor's own `flight()` calls, which is what makes ATE a
real measurement of their control code. The ATE on/off trigger would then
need to become the geofence/distance-from-planned-track check that was
originally offered as an alternative to "Route-pause" and passed over
*because* `Route`-pause was simpler — that option is back on the table
since `Route`-pause no longer exists if `Route` is cut.

This changes Decision 2's table and Decision 2.2 below. **Do not implement
against this doc until this is resolved** — grill it first.

## Relationship to existing docs

This revises `docs/competition-api.md` and
`docs/superpowers/specs/2026-08-09-ONLY-design-competition-control-api-design.md`
(the original competitor-facing control API design). It supersedes:

- **D1–D4** (Jetson-as-edge-box, network-hop architecture) — deployment model
  changed, see Decision 1.
- **Part of D7** (command vocabulary) — `Velocity`/`VelocityWorld`/`Goto`/`Hold`
  removed, `flight()` added; `Route`/`Inspect` retained, unchanged.
- **Extends D8** (statefulness) — `flight()` is stateless and watchdogged,
  the same shape `Velocity` already had.

Precedent: this repo's own `2026-09-07-joystick-analog-sticks-design.md`
(human web UI) already moved WASD/d-pad to a proportional dual-stick model.
This brings the *competitor* API to a comparable shape — deliberately in
cartesian, not polar, form (Decision 3).

Validated against live Isaac Sim + PX4 SITL with a throwaway spike script
before writing this doc — see **Validation**.

---

## Decision 1 — Deployment model: no Jetson, no network hop

Competitors submit a script; the organizer runs it as a local subprocess
against the local FC/sim — same machine, same pattern this repo's
`agent_runner.py` already uses for uploaded joystick-page scripts. This
replaces the original design's Jetson-in-the-loop / network-hop architecture
(D1–D4) entirely.

**Consequence:** the "network jitter destabilizes a hand-rolled P-controller"
risk — originally an argument for keeping more of the vocabulary
server-side — no longer applies. The reasons `Route`/`Inspect` survive
(Decision 2) are independent of latency.

**Explicitly out of scope for this design:** sandboxing/resource-limiting
untrusted competitor code running on the organizer's box. `agent_runner.py`
already has a `subprocess` + `timeout` pattern; whether it's sufficient for
this purpose is separate work, deliberately deferred.

---

## Decision 2 — Vocabulary: `flight()` replaces four commands; `Route`/`Inspect` stay

| Command | Disposition | Why |
|---|---|---|
| `Velocity`, `VelocityWorld` | **removed** → `flight()` | the one-uniform-primitive goal driving this redesign |
| `Goto(lat, lon, alt)` | **removed** | trivially reimplementable on `flight()` (proven by spike, ~15 lines); no server-side value beyond what a competitor can already do themselves |
| `Hold()` | **removed** | `flight(0, 0, 0, 0)` — a bare zero-command isn't worth a type |
| `Route([...], alt, speed)` | **kept, unchanged** | two independent reasons below |
| `Inspect(lat, lon)` | **kept, unchanged** | solves the standoff/depression-angle triangle — real trigonometry every team would otherwise re-derive slightly differently, same category as the existing `pixel_to_ground` geometry helper |

**Why `Route` earns its keep** (both independent of Decision 1's latency point):

1. **Tick-rate smoothness.** `Route` interpolates the whole polyline on the
   sim host at the FC's native rate (250 Hz–1 kHz). A hand-rolled `flight()`
   loop only updates as fast as `on_tick` fires (~20 Hz) — visibly choppier
   over a ~24-minute, 30-waypoint sweep. Demonstrated structurally in the
   worked example below (not yet measured — see Open Items).

2. **It's the scoring system's on-track/off-track signal.** The competition
   scores an ATE (trajectory-following accuracy) metric while on the planned
   sweep, and a separate detection score once a competitor diverts to inspect
   a candidate. `Route`'s existing pause/resume state — the same mechanism
   that already pauses an autonomous mission on manual joystick takeover
   (`CommandState.set_stick`'s `became_active` edge, `waypoints.Mission`) —
   *is* that on-track/off-track boundary, for free:
   - `Route` running → ATE counts.
   - A `flight()`/`Inspect` call while a `Route` is active pauses it → off
     track, detection scoring window begins.
   - `resume_route()` → back on track, ATE resumes.

   **Decided:** this pause edge is the trigger, not a separate geofence/
   distance check against the planned corridor. Simpler, reuses existing
   precedent, at the cost of being slightly less literal than "actual
   distance off the planned path." No new Agent-facing API is needed for
   this — the organizer's scoring harness reads `Mission.state` directly.

If `Route` had been cut too, this on-track/off-track signal would need to be
built from scratch as a new mechanism. See the waypoint-sweep worked example
for the manual state machine that would replace it.

---

## Decision 3 — `flight()` signature: cartesian, body-frame, normalized

```python
flight(left_x, left_y, right_x, right_y)   # each in [-1, 1]
```

**Cartesian, not polar** (angle 0–360° + push amount, as first proposed). A
script naturally produces a direction *vector* — from a detection, a bearing
calculation — and polar costs every competitor an extra `atan2`/`sin`/`cos`
round-trip to enter and leave it, for no benefit. (The first version of the
validation spike did this and it was pure overhead; rewritten to cartesian
requires no round-trip.)

**Body-frame (nose-relative), not screen-DOM-relative.** Unlike the human
joystick UI's `set_stick(x, y)`, which flips DOM y-down before use (an
artifact of being a mouse/touch widget), `flight()`'s `x`/`y` **are** the
flight-semantic sign directly — there is no screen to correct for:

| Stick | axis | meaning |
|---|---|---|
| left | `y` | thrust — `+1` = full climb |
| left | `x` | yaw — `+1` = full right / clockwise |
| right | `y` | pitch — `+1` = full forward |
| right | `x` | roll — `+1` = full right / strafe |

This is the same `(pitch, roll, yaw, thrust)` shape `CommandState` already
produces internally for the human UI — `flight()` hands it over directly,
skipping the DOM-offset detour.

---

## Decision 4 — Units: normalized, 5 m/s max

Full deflection (`|x|` or `|y|` = 1.0) on a translation axis (thrust, pitch,
roll) = **5 m/s**. The scale is documented once, in the same style as
`CommandState`'s existing `speed_fwd`/`speed_right`/`speed_up` constants.

⚙ **Yaw's max rate is not yet decided in this conversation** — defaulting to
the existing `DEFAULT_YAW_RATE_DPS = 45°/s` unless corrected.

---

## Decision 5 — No dedicated world-frame helper

A competitor going toward a world-frame target (the common case — "fly to
this lat/lon") reads `state.lat/lon/heading` (already delivered with every
callback — no polling call needed) and rotates into `flight()`'s body-frame
`x, y` themselves:

```python
h = math.radians(state.heading)
rx = north * math.sin(h) + east * math.cos(h)
ry = north * math.cos(h) - east * math.sin(h)
```

~4 lines — judged small enough not to warrant a dedicated
`state.world_to_body()` helper, unlike `pixel_to_ground` (genuinely
nontrivial camera-intrinsics-plus-terrain-intersection math).

⚙ **Not explicitly confirmed** — this fell out of the "get_position()"
exchange but was never given a direct yes/no. Flagged for review.

---

## Worked examples

**A → B, then spin in place** (state machine the competitor owns, since
`flight()` has no completion event):

```python
class MyAgent(Agent):
    def on_start(self, arena):
        self.target_lat, self.target_lon, self.target_alt = 13.6630, 100.2990, 15.0
        self.phase = "fly"                 # fly -> spin -> done
        self.spin_start = None

    def on_tick(self, state):
        if self.phase == "fly":
            north, east = self._bearing(state.lat, state.lon)
            dist = math.hypot(north, east)
            if dist < 2.0:
                self.phase = "spin"
                self.spin_start = state.time_elapsed
                return Command(flight=flight(0, 0, 0, 0))
            h = math.radians(state.heading)
            rx, ry = north*math.sin(h)+east*math.cos(h), north*math.cos(h)-east*math.sin(h)
            mag = math.hypot(rx, ry) or 1
            return Command(flight=flight(0, 0, rx/mag, ry/mag))

        if self.phase == "spin":
            if state.time_elapsed - self.spin_start > 4.0:
                self.phase = "done"
                return Command(flight=flight(0, 0, 0, 0))
            return Command(flight=flight(1.0, 0, 0, 0))

        return Command(flight=flight(0, 0, 0, 0))
```

**Same A → B via `Route`** — arrival is a free event, no phase variable
needed for that leg:

```python
def on_start(self, arena):
    return Command(flight=Route([(13.6630, 100.2990)], alt=15.0))

def on_arrival(self, state):
    self.spin_start = state.time_elapsed
    return Command(flight=flight(1.0, 0, 0, 0))

def on_tick(self, state):
    if hasattr(self, "spin_start") and state.time_elapsed - self.spin_start > 4.0:
        return Command(flight=flight(0, 0, 0, 0))
```

**Multi-waypoint sweep, hand-rolled on `flight()`** (illustrates the cost if
`Route` had been cut too):

```python
class MyAgent(Agent):
    def on_start(self, arena):
        self.waypoints = arena.lawnmower(spacing=300, alt=117)
        self.wp_index = 0

    def on_tick(self, state):
        if self.wp_index >= len(self.waypoints):
            return Command(flight=flight(0, 0, 0, 0))
        tgt_lat, tgt_lon = self.waypoints[self.wp_index]
        north, east = self._bearing(state.lat, state.lon, tgt_lat, tgt_lon)
        dist = math.hypot(north, east)
        if dist < 5.0:
            self.wp_index += 1
            return Command(flight=flight(0, 0, 0, 0))
        h = math.radians(state.heading)
        rx, ry = north*math.sin(h)+east*math.cos(h), north*math.cos(h)-east*math.sin(h)
        mag = math.hypot(rx, ry) or 1
        push = min(1.0, dist / 15.0)
        return Command(flight=flight(0, 0, rx/mag*push, ry/mag*push))
```

Same shape as A→B, but this loop now runs every ~50 ms for the entire
~24-minute sweep — every leg re-derived, every turn's deceleration re-tuned
by whatever gain the competitor picked, updated only as fast as `on_tick`
fires. This is the concrete cost behind Decision 2's "tick-rate smoothness"
argument for keeping `Route`.

**With `Route` kept**, the same sweep is:

```python
def on_start(self, arena):
    wpts = arena.lawnmower(spacing=300, alt=117)
    return Command(flight=Route(wpts, alt=117, speed=20))

def on_route_complete(self, state):
    return Command(flight=flight(0, 0, 0, 0))
```

---

## Validation

A throwaway spike (`flight_waypoint_test.py`, not part of the API, not
committed) was run against live Isaac Sim + PX4 SITL on this box before
writing this doc:

- Implemented `flight()` as a thin wrapper reusing
  `streaming/offboard.py`'s existing `axes_to_body_velocity`/
  `axes_to_yaw_rate` — no new control logic, just a coordinate shape change.
- Implemented a ~15-line proportional guidance loop: bearing/distance to a
  target, rotate into body frame, decelerate on approach, hold altitude.
- Result: flew a 47 m offset (40 m north, 25 m east) and arrived within 2 m,
  holding altitude within 0.1 m throughout, using **zero yaw** — confirming
  a world-frame target needs no heading change, only the body-frame
  rotation in Decision 5.
- One real bug found and fixed in the process: the first version drained
  only one MAVLink message per tick and starved on position updates under
  load — fixed by draining all pending messages per tick, matching
  `joystick-server.py`'s own `_drain_mavlink` pattern. Worth carrying this
  lesson into the real harness implementation.

This confirms `flight()` alone is *sufficient* as a floor primitive. It does
**not** by itself measure the tick-rate-smoothness claim for `Route` (only a
single short hop was tested) — see Open Items.

---

## Open items (⚙ not yet settled)

1. Yaw's max rate at full deflection — defaulting to the existing
   `DEFAULT_YAW_RATE_DPS = 45°/s` constant; not explicitly decided in this
   conversation.
2. Whether the ~4-line world-frame rotation (Decision 5) should at least be
   a documented snippet in `docs/competition-api.md`'s examples, even
   without being a callable helper.
3. Sandboxing untrusted competitor code — explicitly out of scope for this
   design (Decision 1), deferred to separate work.
4. Submission/answer-scoring mechanics beyond the ATE/detection pause
   trigger (Decision 2.2) remain ⚙ in the original doc; this design doesn't
   extend them further.
5. The tick-rate-smoothness claim for `Route` (Decision 2.1) is argued
   structurally, not yet measured — a follow-up spike chaining several
   waypoints and comparing dispersion/time against a hand-rolled `flight()`
   loop would give real evidence before implementation, if wanted.
