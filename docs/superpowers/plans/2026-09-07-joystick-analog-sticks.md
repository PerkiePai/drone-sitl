# Proportional Dual-Stick Joystick Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the discrete on/off WASD/d-pad drone control in both web UIs (`web/`, `web-v2/`) with proportional dual-stick control (roll, pitch, yaw, thrust), matching a real RC transmitter's Mode 2 layout.

**Architecture:** `streaming/offboard.py`'s `CommandState` moves from a boolean held-direction set to four continuous `[-1, 1]` axes (`pitch, roll, yaw, thrust`), written by a new `set_stick(stick, x, y)`. Both servers (`joystick-server.py`, `joystick-server-descriptive.py`) swap their `{"type":"axis"}` WS handler for `{"type":"stick"}`. Both UIs (`web/`, `web-v2/`) replace their d-pad markup with two draggable circular widgets and rewrite `controls.js` to compute and send normalized stick position, keeping keyboard as a full-deflection fallback.

**Tech Stack:** Python (FastAPI/uvicorn, pymavlink, pytest), vanilla JS (ES modules, no build step, no libraries), plain CSS.

**Spec:** `docs/superpowers/specs/2026-09-07-joystick-analog-sticks-design.md`

## Global Constraints

- Wire message: `{"type": "stick", "stick": "left"|"right", "x": <float>, "y": <float>}`, replacing `{"type":"axis","dir":...,"pressed":...}`. `x`/`y` are raw normalized DOM offsets (right/down positive) — the client never applies flight sign conventions.
- Mode 2 layout: left stick `x`→yaw, `y`→thrust; right stick `x`→roll, `y`→pitch.
- Server-side sign flips (screen → flight): `thrust = -y`, `pitch = -y`, `roll = x`, `yaw = x`.
- Deadzone `0.05` (post-clamp `abs(v) < 0.05` → `0.0`), clamp range `[-1, 1]`.
- `speed_right` (new, strafe m/s) defaults to `1.5`; existing `speed_fwd`/`speed_up`/`watchdog_s`/`yaw_rate_dps` behavior and defaults are unchanged.
- Mission-pause / agent-abort on manual takeover still fires on the same edge (rest → active), now via `CommandState.set_stick`'s return value instead of `pressed=True`.
- Both sticks spring back to `(0,0)` on release; keyboard (arrows = left stick, WASD = right stick) drives full deflection (±1) per axis, deferring to an active pointer drag on that stick.

---

### Task 1: `streaming/offboard.py` — proportional stick control

**Files:**
- Modify: `streaming/offboard.py`
- Test: `streaming/tests/test_offboard.py`

**Interfaces:**
- Produces: `axes_to_body_velocity(pitch, roll, thrust, speed_fwd, speed_right, speed_up) -> (vx, vy, vz)`; `axes_to_yaw_rate(yaw, yaw_rate_rps) -> float` — pure functions, no clamping/deadzone of their own. `CommandState(speed_fwd=2.0, speed_up=1.0, watchdog_s=0.5, yaw_rate_dps=45.0, speed_right=1.5)` with `.set_stick(stick: "left"|"right", x: float, y: float, now=None) -> bool` (True on the rest→active edge — see Global Constraints), `.touch(now=None)`, `.clear()`, `.command(now=None) -> (vx, vy, vz, yaw_rate)`. `held()` and the old `set(direction, pressed)` are removed (grep confirms no caller outside this file's own tests).

This task changes the pure functions and `CommandState` together in one pass — they're too tightly coupled to split (`CommandState.command()` calls both functions directly), so splitting them would leave a broken intermediate state where the signatures disagree.

- [ ] **Step 1: Replace `streaming/tests/test_offboard.py`'s `axes_to_body_velocity`, `axes_to_yaw_rate`, and `CommandState` sections**

Replace everything from the `# --- axes_to_body_velocity` banner through `test_out_of_scope_direction_is_rejected` (i.e. the two pure-function sections plus the whole `CommandState` section — leave `# --- decode_px4_mode` and everything after it untouched) with:

```python
# --- axes_to_body_velocity -------------------------------------------------

def test_full_forward_pitch_is_positive_vx():
    assert offboard.axes_to_body_velocity(1.0, 0.0, 0.0, 2.0, 1.5, 1.0) == (2.0, 0.0, 0.0)


def test_full_back_pitch_is_negative_vx():
    assert offboard.axes_to_body_velocity(-1.0, 0.0, 0.0, 2.0, 1.5, 1.0) == (-2.0, 0.0, 0.0)


def test_pitch_is_proportional_to_deflection():
    assert offboard.axes_to_body_velocity(0.5, 0.0, 0.0, 2.0, 1.5, 1.0) == (1.0, 0.0, 0.0)


def test_full_right_roll_is_positive_vy():
    assert offboard.axes_to_body_velocity(0.0, 1.0, 0.0, 2.0, 1.5, 1.0) == (0.0, 1.5, 0.0)


def test_full_left_roll_is_negative_vy():
    assert offboard.axes_to_body_velocity(0.0, -1.0, 0.0, 2.0, 1.5, 1.0) == (0.0, -1.5, 0.0)


def test_roll_is_proportional_to_deflection():
    assert offboard.axes_to_body_velocity(0.0, 0.5, 0.0, 2.0, 1.5, 1.0) == (0.0, 0.75, 0.0)


def test_full_up_thrust_is_negative_vz_because_ned_down_is_positive():
    assert offboard.axes_to_body_velocity(0.0, 0.0, 1.0, 2.0, 1.5, 1.0) == (0.0, 0.0, -1.0)


def test_full_down_thrust_is_positive_vz():
    assert offboard.axes_to_body_velocity(0.0, 0.0, -1.0, 2.0, 1.5, 1.0) == (0.0, 0.0, 1.0)


def test_zero_axes_is_hover():
    assert offboard.axes_to_body_velocity(0.0, 0.0, 0.0, 2.0, 1.5, 1.0) == (0.0, 0.0, 0.0)


def test_pitch_roll_and_thrust_combine():
    assert offboard.axes_to_body_velocity(1.0, 0.5, 1.0, 2.0, 1.5, 1.0) == (2.0, 0.75, -1.0)


def test_local_constants_match_pymavlink():
    """offboard.py hard-codes these for readability. If pymavlink's dialect
    ever disagrees, fail here rather than silently send a wrong command."""
    from pymavlink.dialects.v20 import common as m
    assert offboard.MAV_FRAME_BODY_NED == m.MAV_FRAME_BODY_NED
    assert offboard.MAV_CMD_DO_SET_MODE == m.MAV_CMD_DO_SET_MODE
    assert offboard.MAV_CMD_COMPONENT_ARM_DISARM == m.MAV_CMD_COMPONENT_ARM_DISARM
    assert offboard.MAV_PARAM_TYPE_INT32 == m.MAV_PARAM_TYPE_INT32
    assert offboard.MAV_PARAM_TYPE_REAL32 == m.MAV_PARAM_TYPE_REAL32
    assert (offboard.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            == m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)


# --- axes_to_yaw_rate -------------------------------------------------------

def test_full_right_yaw_is_positive_rate():
    """NED yaw is positive clockwise seen from above, so right turn > 0."""
    assert offboard.axes_to_yaw_rate(1.0, 0.5) == 0.5


def test_full_left_yaw_is_negative_rate():
    assert offboard.axes_to_yaw_rate(-1.0, 0.5) == -0.5


def test_yaw_is_proportional_to_deflection():
    assert offboard.axes_to_yaw_rate(0.5, 0.5) == 0.25


def test_zero_yaw_is_zero_rate():
    assert offboard.axes_to_yaw_rate(0.0, 0.5) == 0.0


# --- CommandState ----------------------------------------------------------

def test_state_reports_stick_velocity():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)   # up-drag = full forward pitch
    assert s.command(now=100.1) == (2.0, 0.0, 0.0, 0.0)


def test_state_scales_with_partial_deflection():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -0.5, now=100.0)
    assert s.command(now=100.1) == (1.0, 0.0, 0.0, 0.0)


def test_state_release_returns_to_hover():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)
    s.set_stick("right", 0.0, 0.0, now=100.2)
    assert s.command(now=100.3) == (0.0, 0.0, 0.0, 0.0)


def test_right_stick_x_is_roll():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 1.0, 0.0, now=100.0)
    assert s.command(now=100.1) == (0.0, 1.5, 0.0, 0.0)


def test_left_stick_drives_thrust_and_yaw():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, yaw_rate_dps=45.0, speed_right=1.5)
    s.set_stick("left", 1.0, -1.0, now=100.0)    # x=yaw right, up-drag=climb
    vx, vy, vz, yr = s.command(now=100.1)
    assert (vx, vy, vz) == (0.0, 0.0, -1.0)
    assert math.isclose(yr, math.radians(45.0))


def test_watchdog_zeroes_stale_input():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)
    assert s.command(now=100.4) == (2.0, 0.0, 0.0, 0.0)   # still fresh
    assert s.command(now=101.0) == (0.0, 0.0, 0.0, 0.0)   # stale -> hover


def test_watchdog_also_stops_the_turn():
    """A stale link must not leave the aircraft spinning."""
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, yaw_rate_dps=45.0, speed_right=1.5)
    s.set_stick("left", 1.0, 0.0, now=100.0)
    assert s.command(now=101.0) == (0.0, 0.0, 0.0, 0.0)


def test_touch_keeps_a_stick_alive():
    """The page pings every 150 ms while a stick is off-center; without
    that the watchdog would cut hold-to-move off after watchdog_s."""
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)
    s.touch(now=100.4)
    assert s.command(now=100.7) == (2.0, 0.0, 0.0, 0.0)


def test_clear_drops_everything():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)
    s.clear()
    assert s.command(now=100.1) == (0.0, 0.0, 0.0, 0.0)


def test_deadzone_snaps_small_deflection_to_zero():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 0.02, -0.03, now=100.0)
    assert s.command(now=100.1) == (0.0, 0.0, 0.0, 0.0)


def test_out_of_range_deflection_is_clamped():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, speed_right=1.5)
    s.set_stick("right", 2.0, -3.0, now=100.0)
    assert s.command(now=100.1) == (2.0, 1.5, 0.0, 0.0)


def test_unknown_stick_is_rejected():
    s = offboard.CommandState()
    with pytest.raises(ValueError):
        s.set_stick("middle", 0.0, 0.0)


def test_set_stick_returns_true_on_rest_to_active_edge():
    s = offboard.CommandState()
    assert s.set_stick("right", 0.0, -1.0, now=100.0) is True


def test_set_stick_returns_false_while_already_active():
    s = offboard.CommandState()
    assert s.set_stick("right", 0.0, -1.0, now=100.0) is True
    assert s.set_stick("right", 0.0, -0.5, now=100.1) is False


def test_set_stick_returns_true_again_after_returning_to_rest():
    s = offboard.CommandState()
    assert s.set_stick("right", 0.0, -1.0, now=100.0) is True
    assert s.set_stick("right", 0.0, 0.0, now=100.1) is False   # back to rest, not an edge
    assert s.set_stick("right", 0.0, -1.0, now=100.2) is True   # rest -> active again


def test_command_combines_forward_climb_and_turn():
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, yaw_rate_dps=45.0, speed_right=1.5)
    s.set_stick("right", 0.0, -1.0, now=100.0)   # pitch forward
    s.set_stick("left", 1.0, -1.0, now=100.0)    # yaw right + climb
    vx, vy, vz, yr = s.command(now=100.1)
    assert (vx, vy, vz) == (2.0, 0.0, -1.0)
    assert math.isclose(yr, math.radians(45.0))
```

Then delete a second, out-of-place block that sits *after* the `OffboardLink` tests (between `test_bind_target_adopts_ids_from_heartbeat` and `test_send_velocity_forwards_yaw_rate`) — these are stale duplicates of tests already rewritten above, using the old boolean-set API; left in place they'd shadow the new same-named functions (Python keeps only the last definition of a given name) and fail. Delete this whole block, from the `# --- yaw (left/right turn the aircraft; they do NOT strafe) ---` banner through the end of `test_watchdog_also_stops_the_turn`, leaving `test_send_velocity_forwards_yaw_rate` (and everything from there on) untouched:

```python
# --- yaw (left/right turn the aircraft; they do NOT strafe) -----------------

def test_yaw_right_is_positive_rate():
    """NED yaw is positive clockwise seen from above, so right turn > 0."""
    assert offboard.axes_to_yaw_rate({"yaw_right"}, 0.5) == 0.5


def test_yaw_left_is_negative_rate():
    assert offboard.axes_to_yaw_rate({"yaw_left"}, 0.5) == -0.5


def test_opposing_yaw_cancels():
    assert offboard.axes_to_yaw_rate({"yaw_left", "yaw_right"}, 0.5) == 0.0


def test_no_turn_held_is_zero_yaw_rate():
    assert offboard.axes_to_yaw_rate(set(), 0.5) == 0.0


def test_yaw_does_not_produce_any_translation():
    """Turning must not sneak in sideways motion -- the whole point of putting
    yaw on left/right instead of strafe."""
    assert offboard.axes_to_body_velocity({"yaw_left"}, 2.0, 1.0) == (0.0, 0.0, 0.0)
    assert offboard.axes_to_body_velocity({"yaw_right"}, 2.0, 1.0) == (0.0, 0.0, 0.0)


def test_command_combines_forward_climb_and_turn():
    import math
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, yaw_rate_dps=45.0)
    for d in ("fwd", "up", "yaw_right"):
        s.set(d, True, now=100.0)
    vx, vy, vz, yr = s.command(now=100.1)
    assert (vx, vy, vz) == (2.0, 0.0, -1.0)
    assert math.isclose(yr, math.radians(45.0))


def test_watchdog_also_stops_the_turn():
    """A stale link must not leave the aircraft spinning."""
    s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5, yaw_rate_dps=45.0)
    s.set("yaw_right", True, now=100.0)
    assert s.command(now=101.0) == (0.0, 0.0, 0.0, 0.0)
```

Delete that block entirely (do not replace it with anything — the equivalent new tests already live in the `CommandState`/`axes_to_yaw_rate` sections from the first replacement above).

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `cd streaming/tests && python -m pytest test_offboard.py -v`
Expected: FAIL — `TypeError` on every new pure-function test (old signatures take `held, speed_fwd, speed_up` / `held, yaw_rate_rps`) and `AttributeError: 'CommandState' object has no attribute 'set_stick'` on every new `CommandState` test. `decode_px4_mode`/`OffboardLink`/position tests below the replaced section still pass.

- [ ] **Step 3: Rewrite `streaming/offboard.py`**

Replace the module docstring (lines 1–10) — currently:

```python
"""PX4 OFFBOARD velocity control for the web-joystick PoC.

Four commands only: climb, descend, forward, backward. Strafe and yaw are
deliberately out of scope -- see
docs/superpowers/specs/2026-07-30-joystick-offboard-design.md.

Every MAVLink constant below was read out of ~/PX4-Autopilot rather than
recalled; the source file and line are on each one. test_offboard.py also
asserts they agree with pymavlink's dialect.
"""
```

with:

```python
"""PX4 OFFBOARD velocity control for the web-joystick PoC.

Proportional dual-stick control: pitch, roll, yaw, thrust, all continuous
in [-1, 1] -- see docs/superpowers/specs/2026-09-07-joystick-analog-sticks-design.md
(supersedes the four-command, strafe-out-of-scope version described in
docs/superpowers/specs/2026-07-30-joystick-offboard-design.md).

Every MAVLink constant below was read out of ~/PX4-Autopilot rather than
recalled; the source file and line are on each one. test_offboard.py also
asserts they agree with pymavlink's dialect.
"""
```

Delete the `DIRECTIONS = ("fwd", "back", "yaw_left", "yaw_right", "up", "down")` line — its only reader is the old `CommandState.set()`, removed below.

Replace `axes_to_body_velocity` and `axes_to_yaw_rate` (currently the two functions between `DEFAULT_YAW_RATE_DPS = 45.0` and `class CommandState:`) with:

```python
def axes_to_body_velocity(pitch, roll, thrust, speed_fwd, speed_right, speed_up):
    """Map proportional stick deflection to a body-NED velocity setpoint.

    pitch/roll/thrust are each in [-1, 1]: full-back to full-forward
    (pitch), full-left to full-right (roll), full-down to full-up
    (thrust). NED means +vx is nose-forward, +vy is right, and +vz is
    DOWN, so climbing (+thrust) is negative vz. Callers (CommandState)
    own clamping and the deadzone; this function only scales.
    """
    vx = pitch * speed_fwd
    vy = roll * speed_right
    vz = -thrust * speed_up
    return vx, vy, vz


def axes_to_yaw_rate(yaw, yaw_rate_rps):
    """yaw in [-1, 1] -> rad/s. Positive yaw turns right (NED yaw is
    positive clockwise viewed from above) -- CommandState.set_stick maps
    a rightward stick drag to positive yaw, so the signs already agree
    here with no flip needed."""
    return yaw * yaw_rate_rps
```

Replace the whole `CommandState` class with:

```python
DEADZONE = 0.05


def _clamp(v):
    return max(-1.0, min(1.0, v))


class CommandState:
    """Thread-safe proportional stick state with a staleness watchdog.

    The web thread writes; the setpoint thread reads. If the browser stops
    talking -- crash, wifi drop, backgrounded tab -- velocity decays to zero
    instead of latching the last command at full speed.
    """

    def __init__(self, speed_fwd=2.0, speed_up=1.0, watchdog_s=0.5,
                 yaw_rate_dps=DEFAULT_YAW_RATE_DPS, speed_right=1.5):
        self.speed_fwd = speed_fwd
        self.speed_up = speed_up
        self.speed_right = speed_right
        self.watchdog_s = watchdog_s
        self.yaw_rate_rps = math.radians(yaw_rate_dps)
        self._lock = threading.Lock()
        self._pitch = 0.0
        self._roll = 0.0
        self._yaw = 0.0
        self._thrust = 0.0
        self._last_input = 0.0

    def _is_active(self):
        return bool(self._pitch or self._roll or self._yaw or self._thrust)

    def set_stick(self, stick, x, y, now=None):
        """Update one stick's pair of axes from raw DOM-offset x/y in
        [-1, 1] (right/down positive) -- the client sends screen
        coordinates and never applies flight sign conventions itself.

        Left stick: x -> yaw, y -> thrust (up-drag climbs, so thrust is
        -y). Right stick: x -> roll, y -> pitch (up-drag pitches
        forward, so pitch is -y too). Roll needs no flip: dragging right
        (+x) strafes right, and body-NED +vy is right.

        Returns True exactly when this call moves the whole command from
        all-axes-at-rest to at least one axis off-center (past the
        deadzone) -- the edge callers use to trigger mission-pause /
        agent-abort on manual takeover.
        """
        x = _clamp(x)
        y = _clamp(y)
        if abs(x) < DEADZONE:
            x = 0.0
        if abs(y) < DEADZONE:
            y = 0.0
        with self._lock:
            was_active = self._is_active()
            if stick == "left":
                self._yaw = x
                self._thrust = -y
            elif stick == "right":
                self._roll = x
                self._pitch = -y
            else:
                raise ValueError(f"unknown stick {stick!r}; expected 'left' or 'right'")
            self._last_input = time.monotonic() if now is None else now
            is_active = self._is_active()
        return (not was_active) and is_active

    def touch(self, now=None):
        """Keepalive. The watchdog measures time since the last message of
        any kind, and dragging sends a message only on change, so the page
        must refresh the stamp periodically while a stick is off-center."""
        with self._lock:
            self._last_input = time.monotonic() if now is None else now

    def clear(self):
        with self._lock:
            self._pitch = self._roll = self._yaw = self._thrust = 0.0

    def command(self, now=None):
        """Everything one setpoint needs: (vx, vy, vz, yaw_rate).

        Single method rather than separate velocity/yaw getters so both are
        read under one lock against one staleness check -- otherwise a
        watchdog expiry between two calls could zero the velocity while
        leaving the aircraft still turning.
        """
        now = time.monotonic() if now is None else now
        with self._lock:
            pitch, roll, yaw, thrust = self._pitch, self._roll, self._yaw, self._thrust
            last = self._last_input
        if (pitch or roll or yaw or thrust) and (now - last) > self.watchdog_s:
            return 0.0, 0.0, 0.0, 0.0
        vx, vy, vz = axes_to_body_velocity(
            pitch, roll, thrust, self.speed_fwd, self.speed_right, self.speed_up)
        return vx, vy, vz, axes_to_yaw_rate(yaw, self.yaw_rate_rps)
```

- [ ] **Step 4: Run the full test file and confirm everything passes**

Run: `cd streaming/tests && python -m pytest test_offboard.py -v`
Expected: PASS, all tests.

- [ ] **Step 5: Commit**

```bash
git add streaming/offboard.py streaming/tests/test_offboard.py
git commit -m "$(cat <<'EOF'
feat(offboard): proportional dual-stick control replaces boolean held-set

axes_to_body_velocity/axes_to_yaw_rate take continuous [-1,1] deflection
instead of a held-direction set, and CommandState.set_stick replaces
set(direction, pressed). Adds roll (strafe), previously out of scope,
and a rest-to-active edge return value for mission-pause-on-takeover
callers.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01V6JatY7VqZX14WbuTyrMpC
EOF
)"
```

---

### Task 2: `joystick-server.py` and `joystick-server-descriptive.py` — WS handler + `--speed-right`

**Files:**
- Modify: `joystick-server.py`
- Modify: `joystick-server-descriptive.py`
- Test: `streaming/tests/test_web_ui.py`

**Interfaces:**
- Consumes: `CommandState.set_stick(stick, x, y) -> bool` from Task 1.

- [ ] **Step 1: Update the mission-pause tests to send stick messages**

In `streaming/tests/test_web_ui.py`, there are three `await ws.send(json.dumps({"type": "axis", "dir": "fwd", "pressed": True}))` / `pressed: False` calls (in `test_pressing_a_direction_pauses_a_running_mission`, `test_releasing_a_direction_does_not_pause`, and one earlier setup send). Replace each:

`{"type": "axis", "dir": "fwd", "pressed": True}` → `{"type": "stick", "stick": "right", "x": 0, "y": -1}` (full-forward pitch — an active deflection, equivalent to the old `pressed: True`)

`{"type": "axis", "dir": "fwd", "pressed": False}` → `{"type": "stick", "stick": "right", "x": 0, "y": 0}` (centered — equivalent to the old `pressed: False`)

Rename the two test functions to match: `test_pressing_a_direction_pauses_a_running_mission` → `test_deflecting_a_stick_pauses_a_running_mission`; `test_releasing_a_direction_does_not_pause` → `test_centering_a_stick_does_not_pause`. Update their docstrings/comments the same way (s/pressed/deflected/, s/direction/stick/, s/release/center/).

`streaming/tests/test_web_ui_descriptive.py` has no `axis`/`dir`/`pressed` references (confirmed by grep) — leave it unchanged.

- [ ] **Step 2: Run the web UI tests and confirm they fail**

Run: `cd streaming/tests && python -m pytest test_web_ui.py -k "stick or pause" -v`
Expected: FAIL — the server still only understands `kind == "axis"`, so the stick message is silently ignored and the mission never pauses.

- [ ] **Step 3: Update the WS handler in both servers**

In `joystick-server.py`, replace:

```python
                if kind == "axis":
                    state.set(msg["dir"], bool(msg.get("pressed")))
                    # An EDGE, not a level. Polling held() at 20 Hz would miss
                    # a press-release inside one tick -- the drone would twitch
                    # and the mission would keep flying. Only presses pause: if
                    # releases did too, the mission would re-pause forever and
                    # RESUME could never take.
                    if msg.get("pressed"):
                        loop_thread.submit("mission_pause")
                        # First manual input is a hard kill for a running
                        # script -- an agent is aborted, not paused.
                        agent_run.stop("manual takeover")
```

with:

```python
                if kind == "stick":
                    became_active = state.set_stick(
                        msg["stick"], float(msg["x"]), float(msg["y"]))
                    # An EDGE, not a level. Polling command() at 20 Hz would
                    # miss a deflect-and-release inside one tick -- the drone
                    # would twitch and the mission would keep flying. Only the
                    # rest-to-active edge pauses: if returning to center did
                    # too, the mission would re-pause forever and RESUME could
                    # never take.
                    if became_active:
                        loop_thread.submit("mission_pause")
                        # First manual input is a hard kill for a running
                        # script -- an agent is aborted, not paused.
                        agent_run.stop("manual takeover")
```

Apply the identical replacement in `joystick-server-descriptive.py` (same text, confirmed identical by earlier inspection).

- [ ] **Step 4: Add `--speed-right` and pass it into `CommandState` in both servers**

In `joystick-server.py`, replace:

```python
    ap.add_argument("--speed-fwd", type=float, default=2.0, help="m/s")
    ap.add_argument("--speed-up", type=float, default=1.0, help="m/s")
```

with:

```python
    ap.add_argument("--speed-fwd", type=float, default=2.0, help="m/s")
    ap.add_argument("--speed-right", type=float, default=1.5, help="m/s, strafe")
    ap.add_argument("--speed-up", type=float, default=1.0, help="m/s")
```

and replace:

```python
    state = offboard.CommandState(args.speed_fwd, args.speed_up, args.watchdog,
                                  args.yaw_rate)
```

with:

```python
    state = offboard.CommandState(args.speed_fwd, args.speed_up, args.watchdog,
                                  args.yaw_rate, args.speed_right)
```

Apply the identical two replacements in `joystick-server-descriptive.py`.

- [ ] **Step 5: Run the web UI tests and confirm they pass**

Run: `cd streaming/tests && python -m pytest test_web_ui.py test_web_ui_descriptive.py -v`
Expected: PASS.

- [ ] **Step 6: Run the full streaming test suite as a regression check**

Run: `cd streaming/tests && python -m pytest -v`
Expected: PASS (`test_offboard_loop.py` included — it has no message-format dependency, confirmed by grep).

- [ ] **Step 7: Commit**

```bash
git add joystick-server.py joystick-server-descriptive.py streaming/tests/test_web_ui.py
git commit -m "$(cat <<'EOF'
feat(servers): handle {type:stick} WS messages, add --speed-right

Both joystick-server.py and joystick-server-descriptive.py switch their
WS handler from boolean axis/pressed to CommandState.set_stick, using
its rest-to-active return value for mission-pause-on-takeover.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01V6JatY7VqZX14WbuTyrMpC
EOF
)"
```

---

### Task 3: `web-v2/` UI — dual-stick widgets

**Files:**
- Modify: `web-v2/index.html`
- Modify: `web-v2/css/app.css`
- Modify: `web-v2/js/controls.js`
- Modify: `web-v2/js/main.js`

**Interfaces:**
- Consumes: `send(o)` from `./ws.js` (unchanged); WS message shape from Task 2 (`{"type":"stick","stick":"left"|"right","x":<float>,"y":<float>}`).
- Produces: `resetSticks()` exported from `controls.js` (renamed from `resetHeld`), called by `main.js`.

This task has no server-side unit test — it's browser code. Verification is manual (Task 5). Implement directly; there is no failing-test step here because there's no test harness for this file in the repo (`test_web_ui*.py` test the server, not the DOM).

- [ ] **Step 1: Replace the `#controls` markup in `web-v2/index.html`**

Replace:

```html
  <div id="controls">
    <div>
      <div id="pad">
        <span></span>
        <button data-dir="fwd"><span class="g">&#9650;</span><span class="t">FWD</span></button>
        <span></span>
        <button data-dir="yaw_left"><span class="g">&#8634;</span><span class="t">TURN L</span></button>
        <span></span>
        <button data-dir="yaw_right"><span class="g">&#8635;</span><span class="t">TURN R</span></button>
        <span></span>
        <button data-dir="back"><span class="g">&#9660;</span><span class="t">BACK</span></button>
        <span></span>
      </div>
      <div class="cap">move &amp; turn</div>
    </div>
    <div>
      <div id="alt">
        <button data-dir="up"><span class="g">&#9650;</span><span class="t">ASCEND</span></button>
        <button data-dir="down"><span class="g">&#9660;</span><span class="t">DESCEND</span></button>
      </div>
      <div class="cap">altitude</div>
    </div>
  </div>
```

with:

```html
  <div id="controls">
    <div>
      <div id="stick-left" class="stick" data-stick="left"><div class="knob"></div></div>
      <div class="cap">yaw &amp; thrust</div>
    </div>
    <div>
      <div id="stick-right" class="stick" data-stick="right"><div class="knob"></div></div>
      <div class="cap">roll &amp; pitch</div>
    </div>
  </div>
```

- [ ] **Step 2: Replace the `#pad`/`#alt`/`#controls button` CSS in `web-v2/css/app.css`**

Replace:

```css
#controls { display:flex; gap:22px; align-items:center; }
#pad { display:grid; grid-template-columns:repeat(3,74px);
       grid-template-rows:repeat(3,74px); gap:6px; }
#alt { display:grid; grid-template-rows:repeat(2,74px); gap:6px; width:74px; }
#controls button { border-radius:10px; border:1px solid #444;
                   background:#222; color:#eee; display:flex;
                   flex-direction:column; align-items:center;
                   justify-content:center; gap:2px; line-height:1; }
#controls button .g { font-size:calc(24px * var(--ui-scale)); }
#controls button .t { font-size:calc(10px * var(--ui-scale));
                      letter-spacing:.5px; color:#9a9a9a; }
#controls button.on { background:#2a6; border-color:#6f6; }
#controls button.on .t { color:#dff; }
```

with:

```css
#controls { display:flex; gap:22px; align-items:center; }
.stick { width:calc(148px * var(--ui-scale)); height:calc(148px * var(--ui-scale));
         border-radius:50%; background:#222; border:1px solid #444;
         position:relative; touch-action:none; }
.stick .knob { width:calc(56px * var(--ui-scale)); height:calc(56px * var(--ui-scale));
               border-radius:50%; background:#2a6; border:1px solid #6f6;
               position:absolute; left:50%; top:50%;
               transform:translate(-50%,-50%); pointer-events:none; }
```

(`.cap` below it already exists and needs no change.)

- [ ] **Step 3: Rewrite `web-v2/js/controls.js`**

Replace the entire file with:

```js
// The two virtual sticks and keyboard fallback: Mode 2 layout --
// left stick x=yaw, y=thrust; right stick x=roll, y=pitch. Screen
// coordinates only (right/down positive); the server owns every flight
// sign convention.
import { send } from './ws.js';

const STICKS = ['left', 'right'];
const drag = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 } };
const keys = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 } };
const dragging = { left: false, right: false };

function current(name) {
  // A live pointer drag on this stick wins over the keyboard fallback.
  return dragging[name] ? drag[name] : keys[name];
}

function paint(name) {
  const el = document.querySelector(`#stick-${name} .knob`);
  const r = document.getElementById(`stick-${name}`).getBoundingClientRect();
  const { x, y } = current(name);
  el.style.transform =
    `translate(calc(-50% + ${x * r.width / 2}px), calc(-50% + ${y * r.height / 2}px))`;
}

function sendStick(name) {
  const { x, y } = current(name);
  send({ type: 'stick', stick: name, x, y });
  paint(name);
}

STICKS.forEach(name => {
  const el = document.getElementById(`stick-${name}`);
  const setFromEvent = e => {
    const r = el.getBoundingClientRect();
    const cx = r.left + r.width / 2, cy = r.top + r.height / 2;
    let x = (e.clientX - cx) / (r.width / 2);
    let y = (e.clientY - cy) / (r.height / 2);
    const mag = Math.hypot(x, y);
    if (mag > 1) { x /= mag; y /= mag; }
    drag[name] = { x, y };
    sendStick(name);
  };
  el.addEventListener('pointerdown', e => {
    e.preventDefault(); el.setPointerCapture(e.pointerId);
    dragging[name] = true; setFromEvent(e);
  });
  el.addEventListener('pointermove', e => { if (dragging[name]) setFromEvent(e); });
  const release = () => {
    dragging[name] = false;
    drag[name] = { x: 0, y: 0 };
    sendStick(name);
  };
  el.addEventListener('pointerup', release);
  el.addEventListener('pointercancel', release);
  el.addEventListener('contextmenu', e => e.preventDefault());
});

// Arrows drive the left stick (yaw/thrust), WASD drives the right stick
// (roll/pitch), each key at full deflection on one axis. A live pointer
// drag on that stick still wins (see current()).
const KEYS = {
  ArrowUp:    ['left', 'y', -1],  ArrowDown:  ['left', 'y', 1],
  ArrowLeft:  ['left', 'x', -1],  ArrowRight: ['left', 'x', 1],
  w: ['right', 'y', -1], W: ['right', 'y', -1],
  s: ['right', 'y', 1],  S: ['right', 'y', 1],
  a: ['right', 'x', -1], A: ['right', 'x', -1],
  d: ['right', 'x', 1],  D: ['right', 'x', 1],
};

function setKey(e, active) {
  const mapped = KEYS[e.key];
  if (!mapped) return;
  e.preventDefault();
  const [name, axis, sign] = mapped;
  const next = active ? sign : 0;
  if (keys[name][axis] === next) return;   // ignore key auto-repeat
  keys[name][axis] = next;
  if (!dragging[name]) sendStick(name);
}
addEventListener('keydown', e => setKey(e, true));
addEventListener('keyup', e => setKey(e, false));

// Releasing on blur stops a stuck key or drag from latching a velocity
// the operator has tabbed away from and can no longer see or cancel.
addEventListener('blur', () => {
  STICKS.forEach(name => {
    dragging[name] = false;
    drag[name] = { x: 0, y: 0 };
    keys[name] = { x: 0, y: 0 };
    sendStick(name);
  });
});

// Keepalive: the server zeroes velocity after 0.5 s of silence, and a
// stick sends a message only when it changes, so an off-center stick
// must be refreshed.
setInterval(() => {
  if (STICKS.some(name => current(name).x || current(name).y)) send({ type: 'ping' });
}, 150);

// A dead socket must not latch the last commanded velocity.
export function resetSticks() {
  STICKS.forEach(name => {
    dragging[name] = false;
    drag[name] = { x: 0, y: 0 };
    keys[name] = { x: 0, y: 0 };
    paint(name);
  });
}
```

- [ ] **Step 4: Rename the import/call in `web-v2/js/main.js`**

Replace `import { resetHeld } from './controls.js';` with `import { resetSticks } from './controls.js';`, and replace the `resetHeld();` call inside `onClose` with `resetSticks();`.

- [ ] **Step 5: Commit**

```bash
git add web-v2/index.html web-v2/css/app.css web-v2/js/controls.js web-v2/js/main.js
git commit -m "$(cat <<'EOF'
feat(web-v2): dual-stick joystick UI replaces WASD/d-pad

Two draggable circular widgets (Mode 2 layout), WASD/arrows kept as a
full-deflection keyboard fallback. Sends {type:stick} over the WS,
matching the server change in the previous commit.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01V6JatY7VqZX14WbuTyrMpC
EOF
)"
```

---

### Task 4: `web/` UI — mirror Task 3

**Files:**
- Modify: `web/index.html`
- Modify: `web/css/app.css`
- Modify: `web/js/controls.js`
- Modify: `web/js/main.js`

**Interfaces:**
- Same as Task 3. `web/` has no `--ui-scale` variable (it's `web-v2/`-only), so its CSS omits the `calc(... * var(--ui-scale))` wrapper — plain pixel values, matching how the rest of `web/css/app.css` is already unscaled.

- [ ] **Step 1: Replace the `#controls` markup in `web/index.html`**

Same replacement as Task 3 Step 1 (the current `#controls` block in `web/index.html` is byte-identical to `web-v2/index.html`'s, confirmed by inspection), producing the same new `#controls` markup.

- [ ] **Step 2: Replace the `#pad`/`#alt`/`#controls button` CSS in `web/css/app.css`**

Replace (this file's version has no `--ui-scale`, plain px — grep confirmed lines 43–54 are the same five rules without `calc()`):

```css
#controls { display:flex; gap:22px; align-items:center; }
#pad { display:grid; grid-template-columns:repeat(3,74px);
       grid-template-rows:repeat(3,74px); gap:6px; }
#alt { display:grid; grid-template-rows:repeat(2,74px); gap:6px; width:74px; }
#controls button { border-radius:10px; border:1px solid #444;
                   background:#222; color:#eee; display:flex;
                   flex-direction:column; align-items:center;
                   justify-content:center; gap:2px; line-height:1; }
#controls button .g { font-size:24px; }
#controls button .t { font-size:10px; letter-spacing:.5px; color:#9a9a9a; }
#controls button.on { background:#2a6; border-color:#6f6; }
#controls button.on .t { color:#dff; }
```

with:

```css
#controls { display:flex; gap:22px; align-items:center; }
.stick { width:148px; height:148px; border-radius:50%; background:#222;
         border:1px solid #444; position:relative; touch-action:none; }
.stick .knob { width:56px; height:56px; border-radius:50%; background:#2a6;
               border:1px solid #6f6; position:absolute; left:50%; top:50%;
               transform:translate(-50%,-50%); pointer-events:none; }
```

- [ ] **Step 3: Replace `web/js/controls.js`**

Same file content as Task 3 Step 3.

- [ ] **Step 4: Rename the import/call in `web/js/main.js`**

Same edit as Task 3 Step 4 (confirmed identical source lines by earlier inspection).

- [ ] **Step 5: Commit**

```bash
git add web/index.html web/css/app.css web/js/controls.js web/js/main.js
git commit -m "$(cat <<'EOF'
feat(web): dual-stick joystick UI replaces WASD/d-pad

Mirrors the web-v2 change: two draggable circular widgets (Mode 2
layout), WASD/arrows kept as a full-deflection keyboard fallback.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01V6JatY7VqZX14WbuTyrMpC
EOF
)"
```

---

### Task 5: Manual verification

**Files:** none (verification only).

- [ ] **Step 1: Start the descriptive server**

Run: `./sim/run-website.sh --descriptive` (or, if Isaac Sim/PX4 aren't up and this isn't a full flight test, run `python joystick-server-descriptive.py` directly from the repo root — it binds its MAVLink UDP socket without needing a PX4 peer present, and the page still loads with telemetry showing disconnected).

- [ ] **Step 2: Load the page and exercise both sticks**

Open `http://localhost:8091` (or the LAN IP) in a browser. Confirm:
- Both stick widgets render (two circles with a green knob each), with no JS console errors.
- Dragging the right stick moves its knob and (per the server log or a temporary `console.log` in `sendStick`) sends `{"type":"stick","stick":"right","x":...,"y":...}`; releasing snaps the knob back to center and sends `x:0,y:0`.
- Dragging the left stick behaves the same way with `"stick":"left"`.
- Holding W/A/S/D moves the right knob to a screen edge (full deflection) and releasing the key returns it to center; arrows do the same for the left knob.
- Dragging a stick while a key for the same stick is held shows the drag winning (per `current()`'s precedence).

- [ ] **Step 3: Confirm the mission-pause edge still fires once per takeover**

With a mission loaded and flying (or, if no PX4/Isaac available, by pointing to `streaming/tests/test_web_ui.py`'s passing `test_deflecting_a_stick_pauses_a_running_mission` from Task 2 as the automated proof of this), confirm deflecting a stick pauses the mission exactly once, and returning to center does not re-trigger it.

- [ ] **Step 4: Report results to the user**

Summarize what was checked and any issues found. If Isaac Sim/PX4 weren't available for a full flight test, say so explicitly rather than claiming the drone was flown — this task only verifies the control surface (widgets, messages, keyboard fallback, pause edge), not proportional flight behavior in the sim.
