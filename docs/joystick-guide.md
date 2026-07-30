# Joystick flight control — operator handbook

Fly the Isaac Sim / Pegasus drone from a web page using four commands: climb,
descend, forward, backward. Commands go through real PX4 flight control, not by
moving the drone model directly.

- **Design:** `docs/superpowers/specs/2026-07-30-joystick-offboard-design.md`
- **Build plan:** `docs/superpowers/plans/2026-07-30-joystick-offboard.md`

> **Status:** the software is built and 30 automated tests pass, but the
> end-to-end flight has not been flown yet. The first run through this guide
> *is* the acceptance test. Section 8 covers what to do when something breaks.

---

## 1. What talks to what

```
Browser  ──WebSocket──►  joystick-server.py  ──UDP 14540──►  PX4 SITL
(phone or laptop)          (port 8090)                          │
    ▲                                                           │ TCP 4560
    └────────── MJPEG, port 8080 ────────  Isaac Sim + Pegasus ──┘
```

Three ports matter:

| Port | Serves | Started by |
|---|---|---|
| 8080 | Camera video (MJPEG) | Isaac Sim script |
| 8090 | The joystick web page | `joystick-server.py` |
| 14540 | MAVLink to PX4 | PX4 SITL (auto-launched by Pegasus) |

---

## 2. One-time setup

Already done on this machine — listed so you can rebuild elsewhere.

```bash
conda run -n drone pip install fastapi uvicorn
conda run -n drone python -c "import fastapi, uvicorn, pymavlink; print('ok')"
```

Confirm you're on the right branch:

```bash
cd ~/pai/drone-sitl
git branch --show-current      # feat/joystick-offboard
```

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

`200` means good. Anything else — see 8.5.

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
>>> setpoint loop at 20 Hz (2.0 m/s fwd, 1.0 m/s climb)
>>> open http://<box-ip>:8090/
>>> params: COM_RCL_EXCEPT=4 (offboard exempt from RC-loss failsafe), MIS_TAKEOFF_ALT=5.0
```

**The fourth line is the one that matters.** It only appears once PX4 has
actually answered. If the first three print but the fourth never does, the link
to PX4 is broken — stop and go to 8.1. Nothing downstream will work.

Find your box IP with `hostname -I | awk '{print $1}'`.

---

## 5. Open the page

Go to `http://<box-ip>:8090/` on your laptop or phone. Both work; the page
figures out the video address from whatever host you loaded it from.

You should see the camera feed, a telemetry row, a four-way pad, and five
command buttons.

The telemetry row:

| Field | Meaning |
|---|---|
| `link` | Green `up` = PX4 is answering. Red `down` = no connection. |
| `mode` | PX4's **actual** flight mode. Green only when `OFFBOARD`. |
| `armed` | Whether the motors are live. |
| `alt` | Height above the launch point, metres. |
| `hdg` | Compass heading, degrees. |

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
| ▲ | Climb at 1 m/s |
| ▼ | Descend at 1 m/s |
| ▶ | Forward at 2 m/s (the direction the nose points) |
| ◀ | Backward at 2 m/s |
| release | Stop and hover |

Hold to move, release to stop. Keyboard works too: **arrow keys** or **WASD**
(`W` climb, `S` descend, `D` forward, `A` backward). Diagonals work — hold ▲
and ▶ together to climb while moving forward.

Heading never changes. "Forward" means the same physical direction for the
whole flight.

### Step 5 — LAND
Press **LAND**. `mode` goes to `AUTO.LAND`, the drone descends and disarms
itself. **DISARM** is there as a manual cut if you need it.

`LAND` and `DISARM` always work, even when everything else is greyed out.

---

## 7. Checking it actually works

Worth doing on the first flight:

**Distance check.** Hold ▶ for 3 seconds. The drone should travel about **6 m**
(2 m/s × 3 s) and then stop. If it moves the wrong way or barely moves, see 8.4.

**Altitude check.** Hold ▲ for 3 seconds. `alt` should rise about 3 m and hold.

**Watchdog check.** Hold ▶ and close the browser tab mid-press. The drone must
stop within about half a second. If it keeps going, the safety watchdog isn't
working and you should stop and investigate — that's the mechanism that saves
you when wifi drops.

---

## 8. When it goes wrong

### 8.1 `>>> params:` never prints / `link` stays red

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

### 8.2 OFFBOARD button stays greyed out

It needs both one second of setpoint streaming *and* `link up`. If `link` is
red, fix 8.1 first — the button is downstream of that.

### 8.3 Mode flips out of OFFBOARD on its own

PX4 rejected or abandoned it. Common causes:

- **You pressed OFFBOARD before takeoff finished.** Wait for `alt` to settle.
- **RC-loss failsafe.** Should be prevented by `COM_RCL_EXCEPT=4`, which the
  server sets at startup — but only if it saw a HEARTBEAT. If line four never
  printed, this param never got set. Check in QGroundControl.
- **The browser tab was closed** and the watchdog zeroed everything. Reload.

### 8.4 It moves the wrong way, or not at all

- **Nothing moves, mode says OFFBOARD:** PX4 is receiving setpoints but ignoring
  them. Check `armed` is `yes`.
- **Moves the wrong direction:** a frame or sign problem. Note exactly which
  button produced which motion and report it — the mapping is unit-tested, so
  this would point at PX4-side frame handling.
- **Drifts sideways while holding forward:** wind. It's off by default now
  (`ADD_WIND = False`, `drone_setup_px4_cesium.py:41`); if you turned it back
  on, that's your answer.

### 8.5 Video is black or missing

1. Check `STREAM_CAMERAS = True` at `drone_setup_px4_cesium.py:65`.
2. Test directly: `curl -I http://127.0.0.1:8080/detect`
3. Missing Pillow in Isaac's Python prints a warning in the Isaac console:
   ```bash
   <isaac>/python.sh -m pip install pillow
   ```
4. From a phone, confirm port 8080 isn't firewalled off the LAN.

### 8.6 Motion stutters while holding a button

The browser's keepalive isn't reaching the server, so the watchdog keeps cutting
in. Usually flaky wifi. As a diagnostic you can loosen it:

```bash
conda run -n drone python joystick-server.py --watchdog 2.0
```

That is a *diagnostic*, not a fix — a longer watchdog means the drone keeps
flying longer after you lose contact with it.

### 8.7 `Address already in use`

An old server is still running:

```bash
ps -eo pid,cmd | grep "[j]oystick-server.py"
kill <pid>
```

---

## 9. Tuning

All optional:

```bash
conda run -n drone python joystick-server.py \
  --speed-fwd 4.0 \        # faster forward/back (default 2.0 m/s)
  --speed-up 2.0 \         # faster climb/descend (default 1.0 m/s)
  --takeoff-alt 10.0 \     # higher takeoff (default 5.0 m)
  --port 9000              # different web port (default 8090)
```

`--help` lists all nine flags.

**Start slow.** 2 m/s is deliberately gentle so mistakes are recoverable and
easy to see. Turn it up once you trust the setup.

### Turning wind back on

Wind is currently disabled so the basic proof reads clearly. Once forward
reliably means forward, set `ADD_WIND = True` at
`drone_setup_px4_cesium.py:41`. Holding heading and speed through 5 m/s gusts
is a considerably stronger demo than flying in dead air.

### Recording video

Focus an Isaac viewport and press **R** to start/stop MP4 recording of both
cameras. Files land in `~/flight_recordings/`.

---

## 10. What this does not do

Deliberately out of scope: strafing left/right, yaw/rotation, waypoints or
autonomous flight, multiple drones, and any login or access control.

**There is no authentication.** Anyone who can reach port 8090 on your network
can fly the drone. Fine on a trusted LAN, not fine on an open network.

---

## Quick reference

```bash
# Isaac Sim: load stage, run drone_setup_px4_cesium.py, press Play
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/detect   # 200
conda run -n drone python joystick-server.py                            # wait for line 4
hostname -I | awk '{print $1}'                                          # your IP
# browse to http://<ip>:8090/  ->  ARM -> TAKEOFF -> OFFBOARD -> fly -> LAND
```

| | |
|---|---|
| Web UI | `http://<box-ip>:8090/` |
| Video | `http://<box-ip>:8080/detect` |
| Controls | ▲ climb ▼ descend ▶ forward ◀ backward (or WASD / arrows) |
| Speeds | 2 m/s horizontal, 1 m/s vertical |
| Takeoff | 5 m |
| Tests | `conda run -n drone pytest streaming/tests/ -q` |
