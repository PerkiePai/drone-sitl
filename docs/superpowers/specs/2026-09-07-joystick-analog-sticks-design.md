# Design: WASD/d-pad → proportional dual-stick joystick control

**Status:** approved, not yet implemented
**Date:** 2026-09-07

## Goal

Replace the discrete on/off WASD/d-pad control in both web UIs
(`web/`, `web-v2/`) with proportional (analog) dual-stick control, matching
how the senior described it: roll, pitch, yaw, thrust — the angle of a
control stick, not a held button. This also brings strafe (roll → `vy`)
into scope, reversing the explicit non-goal in
`2026-07-30-joystick-offboard-design.md`.

Relationship to that prior design: this changes the *shape* of operator
input (continuous stick position instead of held boolean directions) and
adds one new axis. Everything else it established — `MAV_FRAME_BODY_NED`,
the watchdog, the setpoint thread, arm/takeoff/offboard/land/disarm, the
mission-pause-on-manual-input behavior — is unchanged.

## Non-goals

Physical/Bluetooth gamepad support (Gamepad API) — sticks are on-screen,
touch/mouse-dragged widgets only. Rate/attitude control — setpoints stay
body-frame *velocity*, just proportional now instead of on/off.

## Control layout (Mode 2, like an RC transmitter)

- **Left stick** — `x` = yaw rate, `y` = thrust (up/down)
- **Right stick** — `x` = roll (strafe), `y` = pitch (fwd/back)
- Both sticks spring back to `(0,0)` on release. This PoC's existing
  "release = hover" safety model carries over unchanged; a real
  transmitter's non-centering throttle stick doesn't apply here because
  this is velocity control, not raw motor thrust.
- Keyboard stays as a full-deflection (±1) fallback: **arrows** drive the
  left stick (Up/Down = thrust, Left/Right = yaw), **WASD** drives the
  right stick (W/S = pitch, A/D = roll). This retires Q/E (previously
  altitude) since arrows now own thrust, and retires A/D-as-yaw since
  roll is new and WASD reads more naturally as translate-in-place
  (forward/back/strafe) than arrows do.

## Wire protocol

`{"type": "stick", "stick": "left" | "right", "x": <float>, "y": <float>}`,
`x`/`y` normalized to `[-1, 1]` (client clamps to the widget's radius
before sending). Replaces `{"type": "axis", "dir": ..., "pressed": ...}`.
Sent on every pointer move while dragging, and once on release with
`x=0, y=0`. The existing `{"type": "ping"}` keepalive (150 ms while any
stick is off-center) and the server's silence watchdog are unchanged.

Sign convention: the client sends raw normalized DOM offsets (right = +x,
down = +y — ordinary screen coordinates). The server, not the client,
owns the flip from screen coordinates to flight semantics — matching how
`offboard.py` already owns every other physical sign convention (NED
down-positive, yaw-rate-clockwise-positive, etc.):

- `thrust = -y` (dragging the stick *up*, negative screen `y`, climbs)
- `pitch = -y` (dragging the stick *up* pitches forward)
- `roll = x` (dragging *right* strafes right, `vy` positive = right in
  body-NED)
- `yaw = x` (dragging *right* yaws right, matching the old `yaw_right`
  sign)

## `streaming/offboard.py` changes

- `axes_to_body_velocity(held, speed_fwd, speed_up)` (boolean-set) →
  `axes_to_body_velocity(pitch, roll, thrust, speed_fwd, speed_right, speed_up)`
  (continuous floats in `[-1, 1]`), pure function, no change to NED sign
  handling already established.
- `axes_to_yaw_rate(held, yaw_rate_rps)` (boolean-set) →
  `axes_to_yaw_rate(yaw, yaw_rate_rps)` (continuous float).
- `CommandState` drops the boolean `_held` set for four continuous fields
  (`_pitch, _roll, _yaw, _thrust`), each clamped to `[-1, 1]` and
  deadzoned (values with `abs(v) < 0.05` snap to `0.0`, absorbing touch
  jitter). `set(direction, pressed)` is replaced by
  `set_stick(stick, x, y, now=None)`, which applies the sign flips above,
  updates the pair of fields owned by that stick, and returns `True`
  exactly when this call transitions the whole command from "all four
  axes at rest" to "at least one axis off-center" — the same edge the old
  `set()` detected via `pressed=True`, now generalized from boolean press
  to deadzone-crossing. Callers use that return value to trigger
  mission-pause / agent-abort, replacing the old
  `if msg.get("pressed"):` check.
- `held()` (unused outside its own test) and `DIRECTIONS` (superseded by
  the two stick names) are removed. Constructor gains `speed_right`
  (m/s, strafe speed — new) and keeps `speed_fwd`, `speed_up`,
  `watchdog_s`, `yaw_rate_dps`, in that positional order, for minimal
  diff at call sites plus one new trailing arg.
- Module docstring's "Four commands only... strafe deliberately out of
  scope" is updated: strafe is now in scope, wire format changed to
  continuous sticks, and the six-direction table is replaced.

## Servers (`joystick-server.py`, `joystick-server-descriptive.py`)

Both share `offboard.CommandState` and have an identical WS handler
block. `if kind == "axis": state.set(msg["dir"], bool(msg.get("pressed")))`
becomes:

```python
elif kind == "stick":
    became_active = state.set_stick(msg["stick"], float(msg["x"]), float(msg["y"]))
    if became_active:
        loop_thread.submit("mission_pause")
        agent_run.stop("manual takeover")
```

Both gain `--speed-right` (default `1.5` m/s) alongside the existing
`--speed-fwd`/`--speed-up`, passed into `CommandState(...)`.

## UI (`web/`, `web-v2/`)

Both forks get the same change (they share no JS but have parallel file
layouts): the `#controls` block's d-pad + altitude-column buttons are
replaced by two round drag widgets (`#stick-left`, `#stick-right`, each
with a `.knob` div), styled in each fork's own `app.css` (the
`--ui-scale` variable in `web-v2/css/app.css` still applies; `web/`
stays unscaled, matching its existing pattern). `controls.js` in each is
rewritten: pointer drag on a widget computes a clamped-to-the-circle
`{x, y}`, paints the knob, and sends the `stick` message; release snaps
back to center. WASD/arrows drive the same two stick states at full
deflection per the layout above, deferring to an active pointer drag on
that stick. The exported `resetHeld()` (called from `main.js` on
WS reconnect) is renamed `resetSticks()` — updated at its one call site
in each fork's `main.js` — and now re-centers both knobs and clears both
drag/keyboard state instead of clearing a held-key set.

## Testing

- `streaming/tests/test_offboard.py`: rewrite the `axes_to_body_velocity`/
  `axes_to_yaw_rate`/`CommandState` sections for continuous input —
  proportional scaling (not just full-deflection), the new roll axis,
  sign conventions per stick, deadzone snapping, the rest→active edge
  return value, watchdog zeroing. `OffboardLink` and mode-decoding tests
  are untouched (no API change there).
- `streaming/tests/test_web_ui.py` / `test_web_ui_descriptive.py`: the
  three tests that send `{"type": "axis", "dir": "fwd", "pressed": ...}`
  to exercise mission-pause-on-manual-input are updated to send
  `{"type": "stick", "stick": "right", "x": 0, "y": -1}` (full-forward
  pitch) instead, asserting the same pause/no-pause behavior.
- `streaming/tests/test_offboard_loop.py`: no message-format dependency
  found (verified by grep); no change expected, re-run to confirm.
- Manual: start a server (`--descriptive` or default), load the page in a
  browser, drag both sticks, confirm the WS payloads and that keyboard
  fallback still drives the knobs — without requiring Isaac Sim/PX4 to be
  running, since neither is needed to see the control surface itself
  work. Full flight verification (drone actually translates/strafes/
  climbs/yaws proportionally) is a follow-up manual flight test, same as
  the original design's acceptance step.
