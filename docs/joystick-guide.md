# Joystick flight control — operator handbook

Fly the Isaac Sim / Pegasus drone from a web page — by hand with six commands
(forward, backward, turn left, turn right, ascend, descend), or autonomously by
clicking a route on a satellite map. Commands go through real PX4 flight
control, not by moving the drone model directly.

Manual and autonomous share **one** OFFBOARD setpoint stream, so touching the
pad mid-route takes over in about two-thirds of a second with no mode switch.

- **Design:** `docs/superpowers/specs/2026-07-30-joystick-offboard-design.md`
  and `docs/superpowers/specs/2026-07-31-map-waypoints-design.md`
- **Build plan:** `docs/superpowers/plans/2026-07-30-joystick-offboard.md`
  and `docs/superpowers/plans/2026-07-31-map-waypoints.md`

> **Status:** flown and confirmed working on 2026-07-31 — ARM, TAKEOFF,
> OFFBOARD, all six flight commands, and full waypoint missions (fly, take
> over, resume, complete, re-fly, clear) verified against PX4 in Isaac Sim.
> 88 automated tests pass. Section 9 covers what to do when something breaks.

---

## 1. What talks to what

```
                     satellite tiles (Esri, internet)
                                 │
                                 ▼
Browser  ──WebSocket──►  joystick-server.py  ──UDP 14540──►  PX4 SITL
(phone or laptop)          (port 8090)                          │
    ▲                                                           │ TCP 4560
    └────────── MJPEG, port 8080 ────────  Isaac Sim + Pegasus ──┘
```

Three ports matter:

| Port | Serves | Started by |
|---|---|---|
| 8080 | Camera video (MJPEG) | Isaac Sim script |
| 8090 | The web page, `/vendor` assets, and `/ws` | `joystick-server.py` |
| 14540 | MAVLink to PX4 | PX4 SITL (auto-launched by Pegasus) |

The satellite map needs **internet from the browser** (Esri World Imagery
tiles). Leaflet itself is vendored into `web/vendor/` and served from port
8090, so only the tiles are an external dependency — if the box is offline the
page still loads and flies, you just get a black map.

The map lines up with the Cesium terrain because
`drone_setup_px4_cesium.py:669` sets the PX4 GPS origin from the Cesium
georeference, so PX4's idea of where it is and the satellite imagery agree
without any extra alignment.

---

## 2. One-time setup

Already done on this machine — listed so you can rebuild elsewhere.

```bash
conda run -n drone pip install fastapi uvicorn websockets
conda run -n drone python -c "import fastapi, uvicorn, websockets, pymavlink; print('ok')"
```

`websockets` is **not optional**. Bare `uvicorn` ships without a WebSocket
implementation, and the joystick's entire control path is a WebSocket. Without
it the page still loads and the camera still works, but no telemetry arrives
and no button does anything — a failure that looks like a dead drone rather
than a missing library. The tell is repeated
`No supported WebSocket library detected` in the server console.

Confirm you're on the right branch:

```bash
cd ~/pai/drone-sitl
git branch --show-current      # feat/map-waypoints
```

### Open the firewall (only if flying from another machine)

`ufw` is active on this box and drops unlisted ports silently, which a browser
reports as a **timeout**, not a refusal. Skip this if you only ever browse from
the box itself.

```bash
sudo ufw allow from 192.168.20.0/24 to any port 8090 proto tcp   # joystick UI
sudo ufw allow from 192.168.20.0/24 to any port 8080 proto tcp   # camera video
sudo ufw status numbered                                          # confirm
```

**Both ports are required.** The page is served from 8090, but the video is a
separate request your browser makes straight to 8080. Open only 8090 and you
get a working joystick with a permanently blank video panel — which looks like
a camera fault rather than a firewall one.

Scoping the rules to the LAN (`from 192.168.20.0/24`) rather than opening them
outright is worth doing: there is no authentication on either port (see
section 11).

---

## 3. Start Isaac Sim

1. Open Isaac Sim and load your Cesium stage.
2. **Leave the simulation STOPPED.**
3. `Window > Script Editor`, paste the whole of `drone_setup_px4_cesium.py`,
   and press `Ctrl+Enter`.
4. **Press Play.**

The console should report the drone spawning and a line about the MJPEG server
on port 8080.

Check the video is alive before going further:

```bash
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/detect
```

`200` means good. Anything else — see 9.5.

Two cameras are published:

- `/detect` — pan/tilt gimbal, aimed forward and 30° down. **Use this for flying.**
- `/down` — straight down (nadir), for VIO work.

Open `http://<box-ip>:8080/` to see both side by side.

---

## 4. Start the joystick server

In a terminal (not redirected to a file — you want to see the messages live):

```bash
cd ~/pai/drone-sitl
conda run -n drone python joystick-server.py
```

Expected output:

```
>>> MAVLink offboard link: udpin:0.0.0.0:14540
>>> setpoint loop at 20 Hz (2.0 m/s fwd, 1.0 m/s climb, 45 deg/s turn)
>>> open http://<box-ip>:8090/
>>> params: COM_RCL_EXCEPT=4 (offboard exempt from RC-loss failsafe), MIS_TAKEOFF_ALT=5.0, MPC_XY_VEL_MAX=3.0
```

**The fourth line is the one that matters.** It only appears once PX4 has
actually answered. If the first three print but the fourth never does, the link
to PX4 is broken — stop and go to 9.1. Nothing downstream will work.

Find your box IP with `hostname -I | awk '{print $1}'`.

---

## 5. Open the page

Go to `http://<box-ip>:8090/` on your laptop or phone. Both work; the page
figures out the video address from whatever host you loaded it from.

You should see the camera feed **and a satellite map side by side** (stacked on
a phone), a telemetry row, a four-way move/turn pad, a separate two-button
altitude column, and five command buttons.

The map centres itself on the drone the first time a position arrives, then
leaves the view alone so you can pan and zoom freely. The drone is a green
triangle pointing at its current heading. Until PX4 sends a position the map
says *waiting for position* rather than centring on lat/lon 0,0.

The telemetry row:

| Field | Meaning |
|---|---|
| `link` | Green `up` = PX4 is answering. Red `down` = no connection. |
| `mode` | PX4's **actual** flight mode. Green only when `OFFBOARD`. |
| `armed` | Whether the motors are live. |
| `alt` | Height above the launch point, metres. |
| `hdg` | Compass heading, degrees. |
| `speed` | Measured ground speed / what was commanded, m/s. A persistent gap means PX4 has the setpoint but is not achieving it. |
| `sim` | Sim time vs wall clock. Green above 80%, red below 40%. **Read this before concluding anything is broken.** |

`mode` shows what PX4 is really doing, not what you asked for. If PX4 drops out
of OFFBOARD on its own, you see it here immediately and the joystick stops
having any effect — by design, so a dead stick is never silent.

---

## 6. Fly it

Press the buttons in this order. Each one needs the previous to have taken
effect — watch the telemetry between steps.

### Step 1 — ARM
Press **ARM**. `armed` turns to `yes`. Propellers spin up; the drone stays put.

### Step 2 — TAKEOFF
Press **TAKEOFF**. `mode` goes to `AUTO.TAKEOFF` and `alt` climbs to about
5 m, then settles. Wait for it to stop climbing.

### Step 3 — OFFBOARD
Press **OFFBOARD**. `mode` turns green and reads `OFFBOARD`. The joystick is
now live.

> The OFFBOARD button stays greyed out for the first second after startup. PX4
> refuses to enter OFFBOARD unless setpoints are already streaming, so the
> server waits before offering it. This is normal — wait a second.

### Step 4 — fly

| Control | Does |
|---|---|
| ▲ FWD | Fly forward at 2 m/s (whichever way the nose points) |
| ▼ BACK | Fly backward at 2 m/s |
| ↺ TURN L | Rotate left at 45°/s |
| ↻ TURN R | Rotate right at 45°/s |
| ▲ ASCEND | Climb at 1 m/s |
| ▼ DESCEND | Descend at 1 m/s |
| release | Stop and hover |

The left pad moves and turns; the separate two-button column on the right is
altitude only.

Hold to move, release to stop. Keyboard: **arrows** or **WASD** for the pad
(`W` forward, `S` back, `A` turn left, `D` turn right), and **Q / E** for
ascend / descend. Combinations work — hold FWD and TURN R together to fly a
curve, or FWD and ASCEND to climb while advancing.

"Forward" always means the direction the nose currently points, so turning
changes where forward goes. PX4 resolves this every tick, so a turn takes
effect immediately without needing to re-aim anything.

### Step 5 — LAND
Press **LAND**. `mode` goes to `AUTO.LAND`, the drone descends and disarms
itself. **DISARM** is there as a manual cut if you need it.

`LAND` and `DISARM` always work, even when everything else is greyed out.

---

## 7. Fly a route

Autonomous flight is a **second source feeding the same setpoint stream** as the
joystick. You never leave OFFBOARD, so taking over is instant.

### Step 1 — get into OFFBOARD first

ARM → TAKEOFF → OFFBOARD, exactly as in section 6. **FLY does not do this for
you.** PX4 accepts position setpoints in any mode and then silently discards
them unless it is actually in OFFBOARD, so a route flown in `AUTO.LOITER` would
look perfectly sent and do nothing.

### Step 2 — plan

Click the map to drop waypoints. Numbered amber markers appear, joined by a
dashed line. Click a marker again to delete it; the rest renumber.

Nothing is sent to the server until you press FLY, so **planning costs the
aircraft nothing** — you can hover, look around on the camera, and lay out a
route at your leisure.

**Draw the route starting near the drone.** Missions always begin at waypoint 1,
so a distant first point means flying there before the route proper starts.

### Step 3 — set the altitude

The `alt` box follows your current altitude until you type in it. So the natural
flow is: climb manually to a height that looks right on the camera, then plan,
and the route is already at that height with no typing.

One altitude applies to the whole route.

### Step 4 — FLY

The drone yaws onto the first leg and departs at 3 m/s. The status line under
the map reads `flying waypoint 2/4 · 87 m · 29 s`, the active marker turns
green, and the distance counts down.

The nose tracks each leg, so the camera looks where the drone is going.

> **If FLY is greyed out, hover it.** The tooltip names the exact unmet
> condition. Two of the five gates — `home_valid` and being in OFFBOARD — fail
> *silently* inside PX4, which is precisely why the button explains itself
> rather than letting you press a dead control.

### Step 5 — take over whenever you want

**Touch the pad, or press any flight key.** The mission pauses on that first
press, the route stays drawn on the map, and you are flying manually within
about a second. PX4 never leaves OFFBOARD.

Releasing the button does **not** resume — that would mean letting go silently
re-engages autonomy. The FLY button relabels itself to **RESUME**; press it to
continue from the waypoint it was heading for, or **CLEAR** to abandon the
route.

> Any keypress with the page focused counts: W/A/S/D, the arrows and Q/E are all
> flight controls. If a route keeps pausing on its own, that is what is doing
> it. Click the map or empty background, not a control, before letting a route
> run unattended.

### Step 6 — completion

At the last waypoint the status reads `route complete — holding waypoint 4` and
the drone **holds that position** rather than drifting; it keeps station on a
position setpoint, not a zero-velocity hover, which matters once wind is on.

Press **FLY** again to re-fly the same route from waypoint 1 — repeating a
survey is one press. **LAND** and **DISARM** work throughout.

### What the states mean

```
IDLE ──FLY──► RUNNING ──any axis press──► PAUSED ──RESUME──► RUNNING
                 │                           │
            last wp reached              CLEAR │
                 ▼                           ▼
               DONE ────────────────────►  IDLE
```

### Two behaviours worth knowing

**A closed browser does not stop a route.** Mission state lives on the server,
so if the page dies mid-mission the drone finishes the route and holds. That is
deliberate: the input watchdog exists because a stuck manual key is a hazard,
whereas an autonomous route completing unattended is correct. Reload the page
and you reconnect to a mission still in progress.

**Leaving OFFBOARD auto-pauses the mission.** Pressing LAND mid-route, or any
PX4 failsafe, pauses rather than leaving it `RUNNING` while its setpoints are
being thrown away.

---

## 8. Checking it actually works

Worth doing on the first flight:

**Distance check.** Hold FWD and watch `speed` climb toward 2.0 m/s, then stop
on release.

Distance depends on the `sim` figure. At 100% you get about **6 m** in 3
seconds (2 m/s × 3 s). At 35% the same three seconds of your time is barely one
second of the drone's, so expect roughly a third of that and a partial
ramp-up — hold for 10 seconds or so instead. If it moves the wrong way or
`speed` stays near zero while the commanded figure is 2.0, see 9.4.

**Altitude check.** Hold ASCEND for 3 seconds. `alt` should rise about 3 m and
hold.

**Turn check.** Hold TURN R for 2 seconds. `hdg` should increase by about 90°
(45°/s × 2 s) and stay there. Then hold FWD and confirm the drone now travels
in the new direction — that is the proof that forward tracks the nose.

**Watchdog check.** Hold FWD and close the browser tab mid-press. The drone must
stop within about half a second. If it keeps going, the safety watchdog isn't
working and you should stop and investigate — that's the mechanism that saves
you when wifi drops.

---

## 9. When it goes wrong

### 9.1 `>>> params:` never prints / `link` stays red

PX4 isn't answering. In order of likelihood:

1. **Play isn't pressed** in Isaac Sim. Pegasus only launches PX4 on Play.
2. **PX4 didn't start.** Look in the Isaac console for PX4 output. Check
   `PX4_AUTOLAUNCH = True` at `drone_setup_px4_cesium.py:56`.
3. **Something else already owns 14540:**
   ```bash
   ss -ulnp | grep 14540
   ```
4. **Multiple drones.** Instance 1 uses 14541, not 14540. Point the server at
   it: `--mavlink udpin:0.0.0.0:14541`.

### 9.2 OFFBOARD button stays greyed out

It needs both one second of setpoint streaming *and* `link up`. If `link` is
red, fix 8.1 first — the button is downstream of that.

### 9.3 Mode flips out of OFFBOARD on its own

PX4 rejected or abandoned it. Common causes:

- **You pressed OFFBOARD before takeoff finished.** Wait for `alt` to settle.
- **RC-loss failsafe.** Should be prevented by `COM_RCL_EXCEPT=4`, which the
  server sets at startup — but only if it saw a HEARTBEAT. If line four never
  printed, this param never got set. Check in QGroundControl.
- **The browser tab was closed** and the watchdog zeroed everything. Reload.

### 9.4 It tilts but barely moves

**Check the `sim` figure first.** This is the most common confusing symptom and
it is usually not a fault at all.

A multirotor tilts in order to accelerate horizontally, so tilting is the
*correct* response to a forward command. If it tilts and then creeps forward,
the drone is fine and the clock is the problem.

PX4 SITL runs in **lockstep** with Isaac Sim: when rendering falls behind, the
physics clock slows to match. At `sim 35%`, three seconds of your time is about
one second of the drone's, so it has barely begun accelerating by the time you
expected it to have travelled 6 m.

Confirm with the telemetry: if `speed` reads something like `0.2/2.0`, the
commanded value is right and the aircraft is simply still building up to it.
Hold the button for 8–10 seconds and watch the measured figure climb.

To make it responsive, reduce the rendering load — the biggest cost is usually
the camera streaming (two MJPEG render products competing with physics):

| Change | Effect |
|---|---|
| `STREAM_FPS = 5` (`:68`) | Keeps video, much cheaper. **Try this first.** |
| `STREAM_W, STREAM_H = 320, 200` (`:67`) | Cheaper encode, smaller picture |
| `STREAM_CAMERAS = False` (`:65`) | Fastest sim, no video panel |
| Lower Cesium tile detail, close spare viewports | Frees GPU |

Watch the `sim` figure while you change things — that is what it is there for.

### 9.4b Genuinely no movement, or the wrong direction

- **Nothing moves at all, mode says OFFBOARD:** PX4 is receiving setpoints but
  ignoring them. Check `armed` is `yes`.
- **Moves the wrong direction:** a frame or sign problem. Note exactly which
  button produced which motion and report it — the mapping is unit-tested, so
  this would point at PX4-side frame handling.
- **Drifts sideways while holding forward:** wind. It's off by default now
  (`ADD_WIND = False`, `drone_setup_px4_cesium.py:41`); if you turned it back
  on, that's your answer.

### 9.4c FLY is greyed out

Hover the button — the tooltip names the blocker. In the order they are checked:

| Tooltip | Fix |
|---|---|
| `no MAVLink link` | PX4 isn't answering — see 9.1 |
| `waiting for position fix` | PX4 is up but has no GPS/EKF solution yet. Wait; it takes a few sim-seconds after Play. |
| `PX4 home position not set yet` | Home arrives on a 0.5 Hz stream. Wait a couple of seconds. If it never comes, PX4 has no valid position estimate. |
| `not in OFFBOARD (mode is …)` | Press OFFBOARD first. FLY deliberately does not do it for you. |
| `no waypoints — click the map` | Click the map. |

The last two matter most: PX4 **silently discards** position setpoints without a
valid home altitude or outside OFFBOARD, so without this gate a mission would
look perfectly sent and simply not happen.

### 9.4d The route flies, but the drone barely moves

Same cause as 9.4 — check `sim` first. A route at 3 m/s and 30% sim rate covers
ground at what *looks* like 1 m/s. The proof it is fine: `speed` in the
telemetry row should read close to 3.0 and the map's distance counter should be
falling.

### 9.4e The mission keeps pausing on its own

You are pressing a flight key. W/A/S/D, arrows and Q/E are all flight controls
whenever the page has focus, and any press pauses the route by design. Click the
map or the page background rather than a control before letting a route run.

### 9.4f The map is black

Tiles come from Esri over the internet and are the only external dependency in
the page. Check the browser (not the box) can reach
`server.arcgisonline.com`. Leaflet itself is served locally from
`http://<box-ip>:8090/vendor/leaflet.js` — if *that* 404s, `web/vendor/` is
missing and needs re-fetching (see the plan's Task 0).

### 9.5 Video is black or missing

1. Check `STREAM_CAMERAS = True` at `drone_setup_px4_cesium.py:65`.
2. Test directly: `curl -I http://127.0.0.1:8080/detect`
3. Missing Pillow in Isaac's Python prints a warning in the Isaac console:
   ```bash
   <isaac>/python.sh -m pip install pillow
   ```
4. From a phone, confirm port 8080 isn't firewalled off the LAN.

### 9.6 Motion stutters while holding a button

The browser's keepalive isn't reaching the server, so the watchdog keeps cutting
in. Usually flaky wifi. As a diagnostic you can loosen it:

```bash
conda run -n drone python joystick-server.py --watchdog 2.0
```

That is a *diagnostic*, not a fix — a longer watchdog means the drone keeps
flying longer after you lose contact with it.

### 9.7 Page times out from another machine (works on the box)

Firewall. `ufw` drops unlisted ports silently, so the browser reports a timeout
rather than a refusal. See the firewall step in section 2.

Note that testing with `curl` **on the box** does not prove external
reachability — that traffic never crosses the network. Test from the machine
you actually want to fly from.

A quick way to tell how much is blocked, from the remote machine:

- `http://192.168.20.141:8080/` shows video → only 8090 is blocked.
- It also times out → both ports are blocked.

### 9.8 `Address already in use`

An old server is still running:

```bash
ps -eo pid,cmd | grep "[j]oystick-server.py"
kill <pid>
```

---

## 10. Tuning

All optional:

```bash
conda run -n drone python joystick-server.py \
  --speed-fwd 4.0 \        # faster forward/back (default 2.0 m/s)
  --speed-up 2.0 \         # faster climb/descend (default 1.0 m/s)
  --yaw-rate 90.0 \        # faster turn (default 45 deg/s)
  --takeoff-alt 10.0 \     # higher takeoff (default 5.0 m)
  --mission-speed 5.0 \    # faster waypoint cruise (default 3.0 m/s)
  --arrival-radius 4.0 \   # waypoint counts as reached inside this (default 2.0 m)
  --port 9000              # different web port (default 8090)
```

`--help` lists all twelve flags.

**Start slow.** 2 m/s is deliberately gentle so mistakes are recoverable and
easy to see. Turn it up once you trust the setup.

### Why `--mission-speed` exists

It sets PX4's `MPC_XY_VEL_MAX` at startup, and you can see it confirmed in the
`>>> params:` line. PX4's own default is **12 m/s** — six times the pad's 2 m/s.
Left alone, pressing FLY would send the aircraft off far faster than anything
you had seen it do, on one button press, toward a point possibly off-screen.
3 m/s keeps FLY feeling like the same aircraft you were just flying by hand.

Raising it above about 6 m/s makes the 2 m arrival radius tight — at speed the
drone can overshoot a waypoint between ticks. Raise `--arrival-radius` with it.

### Turning wind back on

Wind is currently disabled so the basic proof reads clearly. Once forward
reliably means forward, set `ADD_WIND = True` at
`drone_setup_px4_cesium.py:41`. Holding heading and speed through 5 m/s gusts
is a considerably stronger demo than flying in dead air.

### Recording video

Focus an Isaac viewport and press **R** to start/stop MP4 recording of both
cameras. Files land in `~/flight_recordings/`.

---

## 11. What this does not do

Deliberately out of scope: strafing sideways (the left/right buttons rotate
instead), per-waypoint altitude (one altitude applies to the whole route), RTL
or return-to-home, survey/grid pattern generation, saving and loading routes,
geofencing, terrain following, offline map tiles, multiple drones, and any login
or access control.

There is also no mission upload to PX4's own `AUTO.MISSION` — routes are flown
as OFFBOARD position setpoints instead, which is what makes instant manual
takeover possible. The trade-off is that QGC will not show the route.

**There is no authentication.** Anyone who can reach port 8090 on your network
can fly the drone. Fine on a trusted LAN, not fine on an open network.

---

## Quick reference

```bash
# Isaac Sim: load stage, run drone_setup_px4_cesium.py, press Play
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/detect   # 200
conda run -n drone python joystick-server.py                            # wait for line 4
hostname -I | awk '{print $1}'                                          # your IP
# browse to http://<ip>:8090/
#   by hand : ARM -> TAKEOFF -> OFFBOARD -> fly -> LAND
#   by route: ARM -> TAKEOFF -> OFFBOARD -> click map -> FLY
#             pad to take over · RESUME to continue · CLEAR to abandon
```

| | |
|---|---|
| Web UI | `http://<box-ip>:8090/` |
| Video | `http://<box-ip>:8080/detect` |
| Controls | pad: ▲ fwd ▼ back ↺↻ turn (WASD/arrows) · column: ascend/descend (Q/E) |
| Speeds | 2 m/s horizontal, 1 m/s vertical, 45°/s turn, **3 m/s on a route** |
| Takeoff | 5 m |
| Route | click map to add · click marker to delete · one `alt` for the whole route · starts at waypoint 1 |
| Takeover | any pad press pauses · RESUME continues · never leaves OFFBOARD |
| Tests | `conda run -n drone pytest streaming/tests/ -q` (88 pass) |
