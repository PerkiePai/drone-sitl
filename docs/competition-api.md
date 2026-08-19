# Drone Control API — competitor reference

Everything your submission can call, and exactly what it does.

**Design rationale lives elsewhere** — see
`docs/superpowers/specs/2026-08-09-competition-control-api-design.md` for *why*
the API is shaped this way. This document is the surface only.

> **Status:** draft. Signatures are settled in shape but not frozen; the values
> marked ⚙ are still being tuned.

---

## 1. The shape of a submission

**You do not write a loop.** The harness owns the loop and calls your methods.
Every call must return promptly — a call that overruns its budget is abandoned
and the aircraft carries on with its previous command.

```python
from competition import Agent, Command, Velocity, Route, Goto, Inspect, Hold

class MyAgent(Agent):

    def on_start(self, arena):
        """Once, before the clock starts. Set up your models here."""

    def on_frame(self, image, state) -> Command | None:
        """~5 Hz. `image` is the currently selected camera. Heavy perception."""

    def on_tick(self, state) -> Command | None:
        """~20 Hz. No image — cheap steering only."""

    def on_arrival(self, state) -> Command | None:
        """A Goto or Inspect finished."""

    def on_waypoint(self, index, state) -> Command | None:
        """A Route waypoint was reached."""

    def on_route_complete(self, state) -> Command | None:
        """The last Route waypoint was reached."""
```

Returning `None` means **"keep doing what you were doing."** A slow detector
never stalls the aircraft — the previous command stays in force.

### Timing

| Callback | Rate | Budget ⚙ | If you overrun |
|---|---|---|---|
| `on_frame` | ~5 Hz | 200 ms | call abandoned, previous command holds |
| `on_tick` | ~20 Hz | 50 ms | call abandoned, previous command holds |
| events | as they occur | 200 ms | call abandoned |

**The simulation does not wait for you.** It runs in real time. A slow script is
a drone that flew blind while you were thinking — you are not penalised by a
rule, you are penalised by physics.

---

## 2. Commands

Return a `Command` from any callback:

```python
Command(flight=..., camera=...)
```

Both fields are optional. `Command(camera="oblique")` switches cameras without
touching the flight path.

### 2.1 Flight

```python
Velocity(forward=0.0, right=0.0, up=0.0, yaw_rate=0.0)        # body frame
VelocityWorld(north=0.0, east=0.0, up=0.0, yaw_rate=0.0)      # world frame
Goto(lat, lon, alt, speed=None)                               # fly to a point
Route(waypoints, alt, speed=None)                             # autonomous sweep
Inspect(lat, lon)                                             # position to view a point
Hold()                                                        # station-keep
```

#### `Velocity` — body frame

Continuous, in m/s and deg/s. Not buttons: state the number you want.

| Field | Unit | Positive means |
|---|---|---|
| `forward` | m/s | direction the nose points |
| `right` | m/s | strafe right (no yaw change) |
| `up` | m/s | climb |
| `yaw_rate` | deg/s | rotate clockwise seen from above |

```python
return Command(flight=Velocity(forward=15.0))            # 15 m/s straight ahead
return Command(flight=Velocity(forward=8.0, right=3.0))  # diagonal, nose unchanged
return Command(flight=Velocity(yaw_rate=30.0))           # spin in place
```

**`Velocity` is stateless and watchdogged.** If no new `Velocity` arrives within
⚙ 0.5 s it decays to a hover. A stuck velocity is a hazard, so it is not allowed
to persist through a silent script. Re-issue it every tick if you want it held.

#### `VelocityWorld` — world frame

Same units. `forward`/`right` become `north`/`east`; heading is irrelevant.

> **Note — positive `up`, not NED.** Aviation convention would use *down*
> positive. This API uses **up positive in both frames** because the sign flip is
> a reliable source of bugs. There is no `down` field anywhere in this API.

Usually the better choice for autonomous code: your detections are in map
coordinates, so your commands may as well be.

#### `Goto` — single point

```python
Goto(lat, lon, alt, speed=None)     # alt in metres AGL; speed m/s, default ⚙ 15
```

Returns immediately; you get `on_arrival` when it completes. Stateful — it
survives a silent script.

#### `Route` — autonomous sweep

```python
Route(waypoints, alt, speed=None)   # waypoints: [(lat, lon), ...]
```

Flies the list in order. One altitude for the whole route. You get `on_waypoint`
per waypoint and `on_route_complete` at the end.

**Runs entirely on the simulator host**, closed-loop at the flight controller's
full rate — network jitter never touches it. This makes it meaningfully smoother
and faster than hand-flying the same path with `Velocity`, which crosses the
network every tick. Use `Route` for the sweep.

**Interrupting a route:** just return a different flight command. The route
pauses and keeps its place; `resume_route()` continues from the waypoint it was
heading for, `clear_route()` abandons it. No mode switch, no ceremony.

```python
def on_frame(self, image, state):
    if candidate := self.detect(image):
        return Command(flight=Inspect(*candidate), camera="oblique")   # route pauses

def on_arrival(self, state):
    self.read_text(state)
    return self.resume_route()                                         # carry on
```

#### `Inspect` — position to view a point

```python
Inspect(lat, lon)
```

The single most useful command for this task. Solves the standoff geometry for
you — distance, altitude and heading — so the oblique camera ends up looking at
the point you named. Because the cameras are rigidly mounted, this is a *flight*
manoeuvre, and doing it by hand means writing the same triangle every other team
is writing.

You get `on_arrival` when the aircraft is settled and pointed.

#### `Hold`

Station-keeps at the current position. Stateful and safe; holds against wind.

### 2.2 Camera

```python
Command(camera="nadir")     # wide 113° — search
Command(camera="oblique")   # tele ~20°, ~30–35° depression ⚙ — read text
```

**One camera at a time.** The selected camera is what `on_frame` delivers.

| | `nadir` | `oblique` |
|---|---|---|
| Focal length ⚙ | 2.2 mm | ~18 mm |
| HFOV | 113° | ~20° |
| Aim | straight down | ~30–35° depression ⚙ |
| Swath at 117 m | 352 m | — |
| Reads 30 cm text | **no** | yes, to ~98 m slant range |

There is **no zoom and no gimbal.** The two lenses *are* the zoom: you change
magnification by changing cameras, and you aim by flying.

> **Leaving it on `oblique` is a mistake worth avoiding.** A 20° field of view
> covers roughly 5% of what the nadir camera does. You will not find anything.

**The cameras are bolted to the airframe, so attitude is aim.** A multirotor
tilts to accelerate — around 15–30° nose-down at 20 m/s — so at cruise the
"nadir" camera is actually looking tens of metres ahead, and the footprint swings
while accelerating. A stable true-nadir view exists only when you are not
accelerating. Slow down before you read.

---

## 3. Sensing

### 3.1 `image`

Delivered to `on_frame`. The **currently selected camera**, H.264-compressed,
~5 Hz. Good enough to detect a vehicle; **not** good enough to trust for OCR —
compression artifacts attack exactly the fine detail text recognition needs.

### 3.2 `request_roi` — lossless crop

```python
crop = self.request_roi(camera, u, v, w, h)   # numpy array, full sensor resolution
```

Returns an **uncompressed, full-resolution** crop. This is what you read text
from. Max ⚙ 640×480 per call, ⚙ 2 calls/sec.

`camera` is named explicitly, so you can pull a lossless oblique crop while still
sweeping on the nadir feed.

```python
def on_arrival(self, state):
    u, v = state.ground_to_pixel("oblique", self.target_lat, self.target_lon)
    crop = self.request_roi("oblique", u - 320, v - 240, 640, 480)
    return self.submit(my_ocr(crop))
```

### 3.3 `state`

Passed to every callback. Cheap; delivered at the full ~20 Hz.

| Field | Unit | Meaning |
|---|---|---|
| `state.lat`, `state.lon` | deg | position |
| `state.alt_agl` | m | height above ground |
| `state.alt_amsl` | m | height above sea level |
| `state.vx`, `state.vy`, `state.vz` | m/s | velocity, world frame, +up |
| `state.ground_speed` | m/s | horizontal speed |
| `state.heading` | deg | 0 = north, positive clockwise |
| `state.roll`, `state.pitch` | deg | attitude — **read this before trusting a frame** |
| `state.camera` | str | currently selected |
| `state.time_elapsed` | s | since the clock started |
| `state.time_remaining` | s | until the run ends |
| `state.route` | obj | `.state`, `.index`, `.count`, `.distance_m` |

`state.pitch` is worth watching: a large pitch means the camera is not pointing
where you assume, and a frame captured mid-acceleration may be unusable for
reading.

### 3.4 Geometry helpers

```python
state.pixel_to_ground(camera, u, v)          -> (lat, lon)
state.ground_to_pixel(camera, lat, lon)      -> (u, v) | None
state.is_visible(camera, lat, lon)           -> bool
```

Ray-casts against terrain using the live camera pose, so they account for the
aircraft's current attitude. `ground_to_pixel` returns `None` if the point is
outside the frame.

```python
def on_frame(self, image, state):
    for box in my_detector(image):
        lat, lon = state.pixel_to_ground(state.camera, *box.center)
        self.candidates.append((lat, lon))
```

### 3.5 `arena`

Passed to `on_start`.

```python
arena.bounds                      # (lat_min, lon_min, lat_max, lon_max)
arena.time_limit                  # s
arena.lawnmower(spacing, alt)     # -> [(lat, lon), ...] convenience sweep pattern
```

---

## 4. Submitting an answer

```python
self.submit(text)              # your reading of the vehicle's text
```

⚙ Submission rules — number of attempts, scoring, and whether a submission ends
the run — are not yet finalised.

---

## 5. Complete example

A full two-phase strategy. Note how little of it is flight control.

```python
from competition import Agent, Command, Route, Inspect

class MyAgent(Agent):

    def on_start(self, arena):
        self.detector = load_my_detector()
        self.ocr = load_my_ocr()
        self.candidates = []
        self.seen = set()
        self.target = None
        return Command(
            flight=Route(arena.lawnmower(spacing=300, alt=117), speed=20),
            camera="nadir",
        )

    def on_frame(self, image, state):
        if state.camera == "nadir" and self.target is None:
            for box in self.detector(image):
                pos = state.pixel_to_ground("nadir", *box.center)
                if self.is_new(pos):
                    self.candidates.append(pos)
            if self.candidates:
                self.target = self.candidates.pop(0)
                return Command(flight=Inspect(*self.target), camera="oblique")

    def on_arrival(self, state):
        px = state.ground_to_pixel("oblique", *self.target)
        if px:
            crop = self.request_roi("oblique", px[0] - 320, px[1] - 240, 640, 480)
            if text := self.ocr(crop):
                self.submit(text)
        self.target = None
        return Command(flight=self.resume_route(), camera="nadir")

    def is_new(self, pos):
        key = (round(pos[0], 4), round(pos[1], 4))
        if key in self.seen:
            return False
        self.seen.add(key)
        return True
```

---

## 6. Not available

| | Why |
|---|---|
| Attitude / rate / motor control | The stabilisation loop cannot cross a network. A 20 ms hiccup would put the aircraft in the ground. Velocity is the floor. |
| Acceleration setpoints | A stale acceleration diverges quadratically, not linearly. |
| Blocking calls (`goto_and_wait`) | The harness owns the loop; a blocking call would stall your own tick. Everything returns immediately, completion arrives as an event. |
| Gimbal / zoom | The cameras are rigidly mounted. Change magnification by changing cameras, aim by flying. |
| Direct simulator access | Out of bounds. |
| Discrete direction commands | You have no fingers. State the velocity you want. |

---

## 7. Common mistakes

1. **Reading text from `image`.** It is H.264-compressed. Use `request_roi`.
2. **Leaving the camera on `oblique`.** 20° FOV finds nothing. Sweep on `nadir`.
3. **Reading while accelerating.** The camera is rigid; the airframe tilts to
   accelerate. Check `state.pitch`, or use `Inspect`, which settles for you.
4. **Hand-flying the sweep with `Velocity`.** It crosses the network every tick.
   `Route` runs on the simulator host and flies a cleaner, faster line.
5. **Expecting `Velocity` to persist.** It decays to hover after ⚙ 0.5 s.
   Re-issue it, or use a stateful command.
6. **Assuming NED.** `up` is positive here, in both frames.
7. **Blocking in `on_frame`.** Over budget means the call is abandoned; the
   aircraft flies on with the last command.

---

## Quick reference

```python
# flight
Velocity(forward, right, up, yaw_rate)          # body,  m/s + deg/s, stateless
VelocityWorld(north, east, up, yaw_rate)        # world, m/s + deg/s, stateless
Goto(lat, lon, alt, speed)                      # stateful -> on_arrival
Route(waypoints, alt, speed)                    # stateful -> on_waypoint / on_route_complete
Inspect(lat, lon)                               # stateful -> on_arrival, solves standoff
Hold()                                          # stateful

# camera
Command(camera="nadir")      # 113° wide, search
Command(camera="oblique")    # ~20° tele, read

# sensing
self.request_roi(camera, u, v, w, h)            # lossless crop, ⚙ 640x480, ⚙ 2/s
state.pixel_to_ground(camera, u, v)             # -> (lat, lon)
state.ground_to_pixel(camera, lat, lon)         # -> (u, v) | None

# route control
self.resume_route()  ·  self.clear_route()

# answer
self.submit(text)
```

| | |
|---|---|
| Angles | degrees; heading 0 = north, positive clockwise |
| Altitude | metres AGL unless stated |
| Velocity | m/s, **positive up in both frames** |
| Return `None` | "keep doing what you were doing" |
| Simulation | real time — it does not wait for you |
