# Design: web joystick → PX4 OFFBOARD velocity control (PoC)

**Status:** approved, ready for implementation planning
**Date:** 2026-07-30

## Goal

Prove that a drone flying in Isaac Sim / Pegasus can be commanded from a web UI
with **four** directions — climb, descend, forward, backward — routed through
real PX4 flight control rather than by moving the prim directly. A live camera
feed from the sim is embedded in the same page.

This is a proof of concept. Success is: press ▶ on a phone, watch the drone
translate forward in the Isaac viewport and in QGC.

Relationship to the existing roadmap (`2026-07-14-roadmap.md`): this is a manual
sibling of **V2a — commanding**. It shares the transport (pymavlink), the port,
and the OFFBOARD entry sequence, but streams *velocity* setpoints from operator
input instead of *position* setpoints from a goto-latlon planner. `streaming/
offboard.py` is intended to be the thing V2a's `SetpointSender` grows out of.

## Non-goals (YAGNI)

Strafe (left/right translation), waypoints, multi-vehicle, auth, HTTPS, WebRTC,
recording control, and the Gamepad API are all out of scope.

> **Amended 2026-07-30 — yaw is now in scope.** After first use, the original
> mapping proved confusing: a D-pad's horizontal axis reads as turn or strafe,
> not as reverse/forward. Remapped to six commands — pad ▲▼ = forward/backward,
> pad ◀▶ = turn left/right, plus a separate two-button column for
> ascend/descend.
>
> This cost almost nothing. `type_mask` 1479 already leaves bit 11 clear, so
> `yaw_rate` was always being transmitted; holding heading was just a hardcoded
> `0.0`. PX4 applies `yawspeed` outside its coordinate-frame switch
> (`mavlink_receiver.cpp:1025`), so it works in `BODY_NED` unchanged — verified
> in source before implementing.
>
> Consequence: heading is no longer fixed, so "forward" tracks the nose. PX4
> resolves `vx` against current yaw every tick, so this needs no extra maths.
> The sections below that describe a fixed heading are superseded by this note.

## Architecture

```
Browser (laptop / phone on LAN)          Linux box
┌────────────────────────┐         ┌──────────────────────────────────────┐
│ web/index.html         │         │ joystick-server.py      (conda: drone)│
│  ▲▼ climb/descend      │─ WS ───►│  ├─ FastAPI + uvicorn  :8090         │
│  ◀▶ back/forward       │◄─telem──│  ├─ CommandState (lock + watchdog)   │
│  ARM TAKEOFF OFF LAND  │         │  └─ setpoint thread @20 Hz ──────────┼──► udpin:0.0.0.0:14540
│  <img> detect feed     │◄─MJPEG──┼──────────────────────────────────────┘        PX4 offboard link
└────────────────────────┘   :8080 │
                                   │ Isaac Sim + Pegasus ── TCP 4560 ──► PX4 SITL
                                   │ drone_setup_px4_cesium.py
                                   └──────────────────────────────────────┘
```

All MAVLink I/O happens on **one** thread — pymavlink connections are not
thread-safe. The web layer only mutates shared state; the setpoint thread is the
sole owner of the connection. One-shot commands (arm, takeoff, land, mode
change) are pushed onto a queue and drained by that same thread.

## Verified constants

Every value below was read out of this machine's PX4 tree, not recalled. Line
references are to `~/PX4-Autopilot`.

| Thing | Value | Source |
|---|---|---|
| Offboard UDP endpoint | `udpin:0.0.0.0:14540` | `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink:4-5,26` |
| Coordinate frame | `MAV_FRAME_BODY_NED` = 8 | `src/modules/mavlink/mavlink_receiver.cpp:970` |
| `type_mask` (velocity + yaw_rate) | `1479` | bit arithmetic, verified |
| `DO_SET_MODE` params | p1=base_mode, p2=main, p3=sub | `src/modules/commander/Commander.cpp:737-738` |
| `PX4_CUSTOM_MAIN_MODE_AUTO` | 4 | `src/modules/commander/px4_custom_mode.h:48` |
| `PX4_CUSTOM_MAIN_MODE_OFFBOARD` | 6 | `src/modules/commander/px4_custom_mode.h:50` |
| `PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF` | 2 | `src/modules/commander/px4_custom_mode.h:58` |
| `PX4_CUSTOM_SUB_MODE_AUTO_LAND` | 6 | `src/modules/commander/px4_custom_mode.h:62` |
| `COM_RCL_EXCEPT` offboard bit | bit 2, so value `4` | `src/modules/commander/commander_params.c:808-820` |

Three of these are traps worth calling out explicitly:

1. **`udpin`, not `udpout`.** PX4 binds 14580 and *sends to* 14540, so the
   companion must **bind** 14540. `2026-07-13-pipeline-streaming.md:550` says
   `udpout:127.0.0.1:14540`, which would silently never receive a heartbeat.
   That plan should be corrected when v1 is built.
2. **`MAV_FRAME_BODY_NED` rotates by yaw only** and passes `vz` through as
   world-down (`mavlink_receiver.cpp:989-991`). That is exactly the semantics
   wanted: "forward" tracks the nose, "up" stays up regardless of attitude. An
   unsupported frame logs a critical error (`:1017`) rather than failing
   silently, so a mistake here would be visible.
3. **`COM_RCL_EXCEPT` defaults to 0.** With no RC transmitter in SITL, PX4's
   default `NAV_RCL_ACT = 2` (Return mode) fires an RC-loss failsafe and takes
   the drone away mid-demo. The server must set bit 2.

## Components

### `streaming/offboard.py`

Importable and unit-testable; imports neither web nor Isaac modules. Follows the
existing `streaming/` convention (flat module, no `__init__.py`, tests reach it
via `sys.path.insert`).

- `axes_to_body_velocity(held, speed_fwd, speed_up) -> (vx, vy, vz)` — pure
  function. `fwd` → `vx = +speed_fwd`; `back` → `vx = -speed_fwd`; `up` →
  `vz = -speed_up`; `down` → `vz = +speed_up` (NED: down is positive). `vy` is
  always `0.0`. Opposing directions held together cancel to zero.
- `CommandState` — thread-safe set of held directions plus `last_input_ts`.
  `velocity(now)` returns zeros when input is stale.
- `px4_mode_args(main, sub=0)` plus the mode constants above.
- `OffboardLink(conn)` — thin pymavlink wrapper: `send_velocity`, `set_mode`,
  `arm`, `disarm`, `set_param`, `poll_telemetry`.

### `joystick-server.py`

Entry point at repo root, matching the flat convention established by
`pipeline.py` and `vio-recorder-pai.py`. Owns argparse, the 20 Hz setpoint
thread, the one-shot command queue, and the FastAPI app.

Arguments: `--mavlink` (default `udpin:0.0.0.0:14540`), `--port` (8090),
`--speed-fwd` (2.0 m/s), `--speed-up` (1.0 m/s), `--takeoff-alt` (5.0 m),
`--watchdog` (0.5 s), `--offboard-warmup` (1.0 s).

### `web/index.html`

Single file, no build step. Four-way pad, command buttons, telemetry strip,
connection indicator, and an `<img>` MJPEG feed. The video URL is derived from
`window.location.hostname` so the page works from any device on the LAN without
being edited.

`web/` is a new top-level directory, extending `2026-07-14-project-structure.md`
— static assets are neither entry points nor importable modules, so neither
existing rule covers them.

## Control mapping

| Button | Body-NED velocity | Meaning |
|---|---|---|
| ▲ | `vz = -1.0` | climb |
| ▼ | `vz = +1.0` | descend |
| ▶ | `vx = +2.0` | forward (nose direction) |
| ◀ | `vx = -2.0` | backward |
| none | all zero | hover |

`yaw_rate = 0` with the yaw bit ignored is what holds heading, so "forward"
means nose-forward for the entire flight.

## Data flow (hold-to-move)

1. `pointerdown ▶` → WS `{"type":"axis","dir":"fwd","pressed":true}`
2. Server → `CommandState.set("fwd", True)`, stamps `last_input_ts`
3. Next 20 Hz tick → `state.velocity(now)` → `(2.0, 0.0, 0.0)`
4. → `SET_POSITION_TARGET_LOCAL_NED(frame=BODY_NED, type_mask=1479, yaw_rate=0)`
5. PX4 velocity controller → Pegasus → Isaac
6. `pointerup` → zeros → hover

## Command sequence

| UI action | MAVLink |
|---|---|
| ARM | `MAV_CMD_COMPONENT_ARM_DISARM`, param1 = 1 |
| TAKEOFF | `DO_SET_MODE(1, 4, 2)` — AUTO.TAKEOFF, altitude from `MIS_TAKEOFF_ALT` |
| OFFBOARD | `DO_SET_MODE(1, 6, 0)` |
| LAND | `DO_SET_MODE(1, 4, 6)` — AUTO.LAND |
| DISARM | `MAV_CMD_COMPONENT_ARM_DISARM`, param1 = 0 |

At startup the server `PARAM_SET`s `COM_RCL_EXCEPT = 4` and
`MIS_TAKEOFF_ALT = --takeoff-alt`, then reads back `PARAM_VALUE` to confirm both
took effect.

Telemetry decodes `HEARTBEAT.custom_mode` as `main = (cm >> 16) & 0xFF`,
`sub = (cm >> 24) & 0xFF`.

## Error handling

- **Input watchdog** — no WS message for more than `--watchdog` seconds forces
  zero velocity. Covers browser crash, wifi drop, and backgrounded tabs.
- **Keepalive ping** — the watchdog measures time since the last message of *any*
  kind, and a held button produces exactly one message. So the page must send
  `{"type":"ping"}` every 150 ms while anything is held, refreshing the stamp via
  `CommandState.touch()`. Without it, holding a direction would cut out after
  `--watchdog` seconds and hold-to-move would be broken rather than safe.
- **WS disconnect** — clears all held directions immediately.
- **Setpoint continuity** — the loop never stops once started. Zeros are a valid
  hover setpoint; stopping the stream would drop OFFBOARD.
- **OFFBOARD entry precondition** — PX4 rejects the mode switch unless setpoints
  are already streaming above 2 Hz. The button stays disabled until the server
  reports it has been streaming for `--offboard-warmup` seconds.
- **Mode drift** — telemetry reports PX4's *actual* mode. If PX4 leaves OFFBOARD
  for any reason, the UI turns red and joystick input is visibly ignored rather
  than silently doing nothing.
- **No PX4 heartbeat** — the server still starts; the UI shows disconnected and
  disables command buttons.
- **LAND and DISARM are never gated.**

## Isaac Sim side

`drone_setup_px4_cesium.py` needs **no code changes** for this PoC — only one
config toggle (wind, below):

- `VEHICLE_ID = 0` and `px4_autolaunch = True` (`:54,56,681-683`) launch PX4 SITL
  instance 0, so the 14540 offboard link exists exactly as assumed.
- `STREAM_CAMERAS = True` is already the default (`:65`), so the MJPEG server on
  8080 comes up unattended.

Two facts about this file shape the design:

- **The video feed is `/detect`, not `/fpv`.** This script has no `fpv_cam`. Its
  cameras are `down` (nadir) and `detect`, a pan/tilt gimbal defaulting to
  `tilt = -60°` — forward with 30° of downward look (`:205`, streamed at `:327`).
  That is a better piloting view than a fixed FPV and can be re-aimed live from
  the Isaac keyboard.
- **Wind is disabled for bring-up.** `ADD_WIND` was `True` with
  `WIND_SPEED_MS = 5.0`, `WIND_GUST_FRAC = 0.5`, and `WIND_FROM_DEG = None` —
  a *random bearing every run*. The drone would visibly crab sideways while
  forward is held, muddying the single thing this PoC exists to show. Set to
  `False` (`:41`). Re-enable it afterwards as a second, stronger demo: holding
  heading and forward velocity through 5 m/s gusts.

Note: the previous `drone_setup_px4_cesium-pai.py` provided an `fpv_cam` prim
that `vio-recorder-pai.py:136,178-182` records as cam1. That script degrades
gracefully to mono (`fpv_cam not on stage -> recording cam0 (down) only`), so
VIO recording still works — but two-sensor recording is gone until the FPV block
is ported over. Irrelevant to this PoC; relevant to the v1 streaming plan.

## Testing

**Unit** — `streaming/tests/test_offboard.py`, pytest in the `drone` env with a
`MagicMock` connection. No PX4 or Isaac required. Covers: all four directions
individually, combined (forward + climb), the empty set, the NED sign convention
(up produces negative `vz`), opposing-key cancellation, watchdog zeroing on
stale input, mode-argument packing, and that `send_velocity` emits frame 8,
`type_mask` 1479, and `yaw_rate` 0.

**Loop** — `streaming/tests/test_offboard_loop.py`. A fake PX4 binds UDP 14580
and emits HEARTBEAT; the test asserts the server sustains at least 2 Hz of
setpoints with the correct payload. Still no PX4 or Isaac required.

**Manual acceptance** — with Isaac Sim running: arm → takeoff → offboard → hold
▶ for 3 seconds → the drone translates roughly 6 m forward in the viewport and
in QGC, then holds position on release.

## Dependencies

New into the `drone` conda env: `fastapi`, `uvicorn`. Already present:
`pymavlink`, `pytest`.
