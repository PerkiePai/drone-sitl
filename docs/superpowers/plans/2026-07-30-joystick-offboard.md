# Implementation plan: web joystick → PX4 OFFBOARD velocity control

**Spec:** `docs/superpowers/specs/2026-07-30-joystick-offboard-design.md`
**Branch:** `feat/joystick-offboard`
**Date:** 2026-07-30

## Goal

A web page with a four-way pad that flies the Pegasus/Isaac drone through real
PX4 OFFBOARD velocity setpoints. Four commands only: climb, descend, forward,
backward. The Isaac MJPEG camera feed is embedded in the same page.

Done means: open the page from a phone, press ARM → TAKEOFF → OFFBOARD, hold ▶
for three seconds, and watch the drone translate ~6 m forward in the Isaac
viewport and in QGC, then hold position on release.

## Architecture summary

```
Browser ──WS /ws──► joystick-server.py ──20 Hz setpoints──► udpin:0.0.0.0:14540 ──► PX4 SITL
   ▲                  (FastAPI :8090)                                                  │
   └──MJPEG :8080/detect─── drone_setup_px4_cesium.py (Isaac Sim + Pegasus) ◄──TCP 4560─┘
```

One thread owns the MAVLink connection. The web layer only mutates
`CommandState`; one-shot commands go on a queue drained by that same thread.

## Tech stack

Python 3.11 in the `drone` conda env. `pymavlink` (present), `pytest`
(present), `fastapi` + `uvicorn` (Task 0). Front end is one static HTML file,
no build step, no framework.

## Global constraints

- **Never call `conn.mav.*` from the web thread.** pymavlink connections are not
  thread-safe. `SetpointLoop` is the sole owner.
- **The setpoint stream never stops** once started. Zeros are a valid hover
  setpoint; a gap drops OFFBOARD.
- **Use `udpin`, not `udpout`,** for the PX4 link. PX4 binds 14580 and sends
  *to* 14540, so `udpout` would silently never receive a heartbeat.
- **NED sign convention:** `+vz` is *down*. Climbing is negative `vz`.
- Run everything through `conda run -n drone`.
- Tests must pass with no PX4 and no Isaac Sim running.

## Naming contract

Fixed across all tasks; do not rename between steps.

| Symbol | Location |
|---|---|
| `axes_to_body_velocity(held, speed_fwd, speed_up)` | `streaming/offboard.py` |
| `CommandState(speed_fwd, speed_up, watchdog_s)` | `streaming/offboard.py` |
| `.set(direction, pressed, now=None)` / `.touch(now=None)` / `.clear()` / `.velocity(now=None)` | `CommandState` |
| `decode_px4_mode(custom_mode)` | `streaming/offboard.py` |
| `OffboardLink(conn)` | `streaming/offboard.py` |
| `.send_velocity(vx, vy, vz)` / `.arm()` / `.disarm()` / `.takeoff()` / `.land()` / `.offboard()` / `.set_param(name, value, param_type)` / `.bind_target(msg)` | `OffboardLink` |
| `SetpointLoop(conn, state, rate_hz, takeoff_alt, warmup_s)` | `joystick-server.py` |
| `.submit(name)` / `.telemetry()` | `SetpointLoop` |
| `build_app(loop_thread, state, video_port)` | `joystick-server.py` |

Direction strings are exactly `"fwd"`, `"back"`, `"up"`, `"down"`.

---

## Task 0: Dependencies

**Files:** none (environment only)

- [ ] Install:
  ```bash
  conda run -n drone pip install fastapi uvicorn websockets
  ```
  `websockets` is required, not optional: bare `uvicorn` has no WebSocket
  implementation and silently refuses the `/ws` upgrade while HTTP endpoints
  keep returning 200.
- [ ] Verify:
  ```bash
  conda run -n drone python -c "import fastapi, uvicorn, websockets, pymavlink, pytest; print('ok')"
  ```
  Expected: `ok`
- [ ] Commit: `chore: add fastapi/uvicorn to the drone env for the joystick server`

---

## Task 1: Constants + `axes_to_body_velocity`

**Create:** `streaming/offboard.py`, `streaming/tests/test_offboard.py`

### Step 1.1 — Failing test

- [ ] Create `streaming/tests/test_offboard.py`:

  ```python
  """Unit tests for streaming/offboard.py. No PX4 and no Isaac Sim required."""
  import os
  import sys

  import pytest
  from unittest.mock import MagicMock

  ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
  sys.path.insert(0, os.path.join(ROOT, "streaming"))
  import offboard  # noqa: E402


  # --- axes_to_body_velocity -------------------------------------------------

  def test_forward_is_positive_vx():
      assert offboard.axes_to_body_velocity({"fwd"}, 2.0, 1.0) == (2.0, 0.0, 0.0)


  def test_backward_is_negative_vx():
      assert offboard.axes_to_body_velocity({"back"}, 2.0, 1.0) == (-2.0, 0.0, 0.0)


  def test_climb_is_negative_vz_because_ned_down_is_positive():
      assert offboard.axes_to_body_velocity({"up"}, 2.0, 1.0) == (0.0, 0.0, -1.0)


  def test_descend_is_positive_vz():
      assert offboard.axes_to_body_velocity({"down"}, 2.0, 1.0) == (0.0, 0.0, 1.0)


  def test_nothing_held_is_hover():
      assert offboard.axes_to_body_velocity(set(), 2.0, 1.0) == (0.0, 0.0, 0.0)


  def test_forward_and_climb_combine():
      assert offboard.axes_to_body_velocity({"fwd", "up"}, 2.0, 1.0) == (2.0, 0.0, -1.0)


  def test_opposing_directions_cancel():
      assert offboard.axes_to_body_velocity(
          {"fwd", "back", "up", "down"}, 2.0, 1.0) == (0.0, 0.0, 0.0)


  def test_vy_is_always_zero_because_strafe_is_out_of_scope():
      for d in offboard.DIRECTIONS:
          assert offboard.axes_to_body_velocity({d}, 2.0, 1.0)[1] == 0.0


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
  ```
- [ ] Run:
  ```bash
  conda run -n drone pytest streaming/tests/test_offboard.py -q
  ```
  Expected: collection error — `ModuleNotFoundError: No module named 'offboard'`.

### Step 1.2 — Implement

- [ ] Create `streaming/offboard.py`:

  ```python
  """PX4 OFFBOARD velocity control for the web-joystick PoC.

  Four commands only: climb, descend, forward, backward. Strafe and yaw are
  deliberately out of scope -- see
  docs/superpowers/specs/2026-07-30-joystick-offboard-design.md.

  Every MAVLink constant below was read out of ~/PX4-Autopilot rather than
  recalled; the source file and line are on each one. test_offboard.py also
  asserts they agree with pymavlink's dialect.
  """
  import threading
  import time

  # SET_POSITION_TARGET_LOCAL_NED coordinate frame. PX4 rotates vx/vy by yaw
  # only and passes vz through as world-down (mavlink_receiver.cpp:970-991),
  # which is exactly what we want: "forward" tracks the nose, "up" stays up
  # regardless of attitude.
  MAV_FRAME_BODY_NED = 8

  # type_mask: ignore position (bits 0-2), ignore acceleration (bits 6-8),
  # ignore yaw (bit 10). USE velocity (bits 3-5) and yaw_rate (bit 11 clear).
  # Sending yaw_rate = 0 with yaw ignored is what holds the heading.
  VEL_YAWRATE_TYPE_MASK = 1479

  MAV_CMD_DO_SET_MODE = 176
  MAV_CMD_COMPONENT_ARM_DISARM = 400
  MAV_PARAM_TYPE_INT32 = 6
  MAV_PARAM_TYPE_REAL32 = 9
  MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
  MAV_MODE_FLAG_SAFETY_ARMED = 128

  # PX4 custom modes (px4_custom_mode.h:44-67). MAV_CMD_DO_SET_MODE takes these
  # as SEPARATE params -- param1=base_mode, param2=main, param3=sub
  # (Commander.cpp:737-738) -- not bit-packed into one custom_mode field. The
  # bit-packed form appears only in HEARTBEAT, decoded by decode_px4_mode().
  PX4_MAIN_MODE_AUTO = 4
  PX4_MAIN_MODE_OFFBOARD = 6
  PX4_SUB_MODE_AUTO_TAKEOFF = 2
  PX4_SUB_MODE_AUTO_LAND = 6

  # RC-loss exception bitmask (commander_params.c:808-820); bit 2 = Offboard.
  # Without this, PX4's default NAV_RCL_ACT=2 fires a Return-mode failsafe in
  # SITL -- where there is no RC transmitter -- and flies the drone away
  # mid-demo.
  COM_RCL_EXCEPT_OFFBOARD = 4

  DIRECTIONS = ("fwd", "back", "up", "down")


  def axes_to_body_velocity(held, speed_fwd, speed_up):
      """Map held joystick directions to a body-NED velocity setpoint.

      NED means +vx is nose-forward and +vz is DOWN, so climbing is negative vz.
      vy is always 0.0 -- strafe is out of scope. Opposing directions cancel.
      """
      vx = 0.0
      if "fwd" in held:
          vx += speed_fwd
      if "back" in held:
          vx -= speed_fwd
      vz = 0.0
      if "up" in held:
          vz -= speed_up
      if "down" in held:
          vz += speed_up
      return vx, 0.0, vz
  ```
- [ ] Run:
  ```bash
  conda run -n drone pytest streaming/tests/test_offboard.py -q
  ```
  Expected: `9 passed`.
- [ ] Commit: `feat(offboard): add verified PX4 constants + joystick velocity mapping`

---

## Task 2: `CommandState`

**Modify:** `streaming/offboard.py`, `streaming/tests/test_offboard.py`

### Step 2.1 — Failing tests

- [ ] Append to `streaming/tests/test_offboard.py`:

  ```python
  # --- CommandState ----------------------------------------------------------

  def test_state_reports_held_velocity():
      s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
      s.set("fwd", True, now=100.0)
      assert s.velocity(now=100.1) == (2.0, 0.0, 0.0)


  def test_state_release_returns_to_hover():
      s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
      s.set("fwd", True, now=100.0)
      s.set("fwd", False, now=100.2)
      assert s.velocity(now=100.3) == (0.0, 0.0, 0.0)


  def test_watchdog_zeroes_stale_input():
      s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
      s.set("fwd", True, now=100.0)
      assert s.velocity(now=100.4) == (2.0, 0.0, 0.0)   # still fresh
      assert s.velocity(now=101.0) == (0.0, 0.0, 0.0)   # stale -> hover


  def test_touch_keeps_a_held_direction_alive():
      """The page pings every 150 ms while a button is down; without that the
      watchdog would cut hold-to-move off after watchdog_s."""
      s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
      s.set("fwd", True, now=100.0)
      s.touch(now=100.4)
      assert s.velocity(now=100.7) == (2.0, 0.0, 0.0)


  def test_clear_drops_everything():
      s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
      s.set("fwd", True, now=100.0)
      s.clear()
      assert s.velocity(now=100.1) == (0.0, 0.0, 0.0)


  def test_held_reports_current_set():
      s = offboard.CommandState(2.0, 1.0, watchdog_s=0.5)
      s.set("fwd", True, now=100.0)
      s.set("up", True, now=100.0)
      assert s.held() == {"fwd", "up"}


  def test_out_of_scope_direction_is_rejected():
      s = offboard.CommandState()
      with pytest.raises(ValueError):
          s.set("left", True)
  ```
- [ ] Run: `conda run -n drone pytest streaming/tests/test_offboard.py -q`
  Expected: 7 failures, `AttributeError: module 'offboard' has no attribute 'CommandState'`.

### Step 2.2 — Implement

- [ ] Append to `streaming/offboard.py`:

  ```python
  class CommandState:
      """Thread-safe held-direction set with a staleness watchdog.

      The web thread writes; the setpoint thread reads. If the browser stops
      talking -- crash, wifi drop, backgrounded tab -- velocity decays to zero
      instead of latching the last command at full speed.
      """

      def __init__(self, speed_fwd=2.0, speed_up=1.0, watchdog_s=0.5):
          self.speed_fwd = speed_fwd
          self.speed_up = speed_up
          self.watchdog_s = watchdog_s
          self._lock = threading.Lock()
          self._held = set()
          self._last_input = 0.0

      def set(self, direction, pressed, now=None):
          if direction not in DIRECTIONS:
              raise ValueError(
                  f"unknown direction {direction!r}; expected one of {DIRECTIONS}")
          with self._lock:
              if pressed:
                  self._held.add(direction)
              else:
                  self._held.discard(direction)
              self._last_input = time.monotonic() if now is None else now

      def touch(self, now=None):
          """Keepalive. The watchdog measures time since the last message of any
          kind, and holding a button produces exactly one message, so the page
          must refresh the stamp periodically."""
          with self._lock:
              self._last_input = time.monotonic() if now is None else now

      def clear(self):
          with self._lock:
              self._held.clear()

      def held(self):
          with self._lock:
              return set(self._held)

      def velocity(self, now=None):
          now = time.monotonic() if now is None else now
          with self._lock:
              held = set(self._held)
              last = self._last_input
          if held and (now - last) > self.watchdog_s:
              return 0.0, 0.0, 0.0
          return axes_to_body_velocity(held, self.speed_fwd, self.speed_up)
  ```
- [ ] Run: `conda run -n drone pytest streaming/tests/test_offboard.py -q`
  Expected: `16 passed`.
- [ ] Commit: `feat(offboard): add thread-safe CommandState with input watchdog`

---

## Task 3: `decode_px4_mode`

**Modify:** `streaming/offboard.py`, `streaming/tests/test_offboard.py`

### Step 3.1 — Failing tests

- [ ] Append to `streaming/tests/test_offboard.py`:

  ```python
  # --- decode_px4_mode -------------------------------------------------------

  def test_decode_offboard_mode():
      assert offboard.decode_px4_mode(6 << 16) == "OFFBOARD"


  def test_decode_auto_takeoff_mode():
      assert offboard.decode_px4_mode((2 << 24) | (4 << 16)) == "AUTO.TAKEOFF"


  def test_decode_auto_land_mode():
      assert offboard.decode_px4_mode((6 << 24) | (4 << 16)) == "AUTO.LAND"


  def test_decode_posctl_mode():
      assert offboard.decode_px4_mode(3 << 16) == "POSCTL"


  def test_decode_unknown_mode_is_readable_not_a_crash():
      assert "99" in offboard.decode_px4_mode(99 << 16)
  ```
- [ ] Run: `conda run -n drone pytest streaming/tests/test_offboard.py -q`
  Expected: 5 failures, `no attribute 'decode_px4_mode'`.

### Step 3.2 — Implement

- [ ] Append to `streaming/offboard.py`:

  ```python
  PX4_MODE_NAMES = {
      (1, 0): "MANUAL", (2, 0): "ALTCTL", (3, 0): "POSCTL",
      (4, 1): "AUTO.READY", (4, 2): "AUTO.TAKEOFF", (4, 3): "AUTO.LOITER",
      (4, 4): "AUTO.MISSION", (4, 5): "AUTO.RTL", (4, 6): "AUTO.LAND",
      (5, 0): "ACRO", (6, 0): "OFFBOARD", (7, 0): "STABILIZED",
  }


  def decode_px4_mode(custom_mode):
      """HEARTBEAT.custom_mode -> readable PX4 mode name.

      PX4 packs main mode at bits 16-23 and sub mode at bits 24-31. Unknown
      combinations render as mode(main.sub) so an unexpected failsafe shows up
      in the UI as something specific rather than as a blank.
      """
      main = (custom_mode >> 16) & 0xFF
      sub = (custom_mode >> 24) & 0xFF
      name = PX4_MODE_NAMES.get((main, sub))
      if name is None:
          name = PX4_MODE_NAMES.get((main, 0), f"mode({main}.{sub})")
      return name
  ```
- [ ] Run: `conda run -n drone pytest streaming/tests/test_offboard.py -q`
  Expected: `21 passed`.
- [ ] Commit: `feat(offboard): decode PX4 HEARTBEAT custom_mode to readable names`

---

## Task 4: `OffboardLink`

**Modify:** `streaming/offboard.py`, `streaming/tests/test_offboard.py`

### Step 4.1 — Failing tests

- [ ] Append to `streaming/tests/test_offboard.py`:

  ```python
  # --- OffboardLink ----------------------------------------------------------

  def test_send_velocity_uses_body_ned_frame_and_velocity_mask():
      conn = MagicMock()
      offboard.OffboardLink(conn).send_velocity(2.0, 0.0, -1.0)
      args, _ = conn.mav.set_position_target_local_ned_send.call_args
      (_ms, _sys, _comp, frame, mask,
       _x, _y, _z, vx, vy, vz, _ax, _ay, _az, _yaw, yaw_rate) = args
      assert frame == offboard.MAV_FRAME_BODY_NED
      assert mask == offboard.VEL_YAWRATE_TYPE_MASK
      assert (vx, vy, vz) == (2.0, 0.0, -1.0)
      assert yaw_rate == 0.0      # zero yaw_rate is what holds the heading


  def test_offboard_sets_custom_main_mode_6():
      conn = MagicMock()
      offboard.OffboardLink(conn).offboard()
      args, _ = conn.mav.command_long_send.call_args
      _sys, _comp, command, _conf, p1, p2, p3, _p4, _p5, _p6, _p7 = args
      assert command == offboard.MAV_CMD_DO_SET_MODE
      assert p1 == float(offboard.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
      assert p2 == float(offboard.PX4_MAIN_MODE_OFFBOARD)
      assert p3 == 0.0


  def test_takeoff_uses_auto_takeoff_submode():
      conn = MagicMock()
      offboard.OffboardLink(conn).takeoff()
      args, _ = conn.mav.command_long_send.call_args
      assert args[5] == float(offboard.PX4_MAIN_MODE_AUTO)
      assert args[6] == float(offboard.PX4_SUB_MODE_AUTO_TAKEOFF)


  def test_land_uses_auto_land_submode():
      conn = MagicMock()
      offboard.OffboardLink(conn).land()
      args, _ = conn.mav.command_long_send.call_args
      assert args[5] == float(offboard.PX4_MAIN_MODE_AUTO)
      assert args[6] == float(offboard.PX4_SUB_MODE_AUTO_LAND)


  def test_arm_then_disarm():
      conn = MagicMock()
      link = offboard.OffboardLink(conn)
      link.arm()
      args, _ = conn.mav.command_long_send.call_args
      assert args[2] == offboard.MAV_CMD_COMPONENT_ARM_DISARM
      assert args[4] == 1.0
      link.disarm()
      args, _ = conn.mav.command_long_send.call_args
      assert args[4] == 0.0


  def test_rc_loss_exception_param_is_sent_as_int32():
      conn = MagicMock()
      offboard.OffboardLink(conn).set_param(
          "COM_RCL_EXCEPT", offboard.COM_RCL_EXCEPT_OFFBOARD,
          offboard.MAV_PARAM_TYPE_INT32)
      args, _ = conn.mav.param_set_send.call_args
      _sys, _comp, param_id, value, param_type = args
      assert param_id == b"COM_RCL_EXCEPT"
      assert value == 4.0
      assert param_type == offboard.MAV_PARAM_TYPE_INT32


  def test_bind_target_adopts_ids_from_heartbeat():
      conn = MagicMock()
      link = offboard.OffboardLink(conn)
      msg = MagicMock()
      msg.get_srcSystem.return_value = 3
      msg.get_srcComponent.return_value = 7
      link.bind_target(msg)
      assert (link.target_system, link.target_component) == (3, 7)
  ```
- [ ] Run: `conda run -n drone pytest streaming/tests/test_offboard.py -q`
  Expected: 7 failures, `no attribute 'OffboardLink'`.

### Step 4.2 — Implement

- [ ] Append to `streaming/offboard.py`:

  ```python
  class OffboardLink:
      """Every MAVLink write for the joystick PoC.

      Single-threaded by contract: pymavlink connections are not thread-safe,
      so only the setpoint thread may call these methods.
      """

      def __init__(self, conn):
          self.conn = conn
          self.target_system = 1
          self.target_component = 1

      def bind_target(self, msg):
          """Adopt sysid/compid from a received HEARTBEAT."""
          self.target_system = msg.get_srcSystem()
          self.target_component = msg.get_srcComponent()

      def send_velocity(self, vx, vy, vz):
          self.conn.mav.set_position_target_local_ned_send(
              0,                                  # time_boot_ms (PX4 ignores)
              self.target_system, self.target_component,
              MAV_FRAME_BODY_NED,
              VEL_YAWRATE_TYPE_MASK,
              0.0, 0.0, 0.0,                      # x, y, z    -- masked off
              vx, vy, vz,
              0.0, 0.0, 0.0,                      # afx, afy, afz -- masked off
              0.0,                                # yaw        -- masked off
              0.0)                                # yaw_rate=0 -> hold heading

      def _command_long(self, command, *params):
          padded = list(params) + [0.0] * (7 - len(params))
          self.conn.mav.command_long_send(
              self.target_system, self.target_component, command, 0, *padded)

      def set_mode(self, main_mode, sub_mode=0):
          self._command_long(
              MAV_CMD_DO_SET_MODE,
              float(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
              float(main_mode),
              float(sub_mode))

      def arm(self):
          self._command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0)

      def disarm(self):
          self._command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0.0)

      def takeoff(self):
          """AUTO.TAKEOFF climbs to MIS_TAKEOFF_ALT, set at startup."""
          self.set_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_TAKEOFF)

      def land(self):
          self.set_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_LAND)

      def offboard(self):
          self.set_mode(PX4_MAIN_MODE_OFFBOARD)

      def set_param(self, name, value, param_type):
          self.conn.mav.param_set_send(
              self.target_system, self.target_component,
              name.encode() if isinstance(name, str) else name,
              float(value), param_type)
  ```
- [ ] Run: `conda run -n drone pytest streaming/tests/test_offboard.py -q`
  Expected: `28 passed`.
- [ ] Commit: `feat(offboard): add OffboardLink for setpoints, modes, arming, params`

---

## Task 5: `SetpointLoop`

**Create:** `joystick-server.py`

### Step 5.1 — Write the module and the loop

- [ ] Create `joystick-server.py`:

  ```python
  #!/usr/bin/env python3
  """Web joystick -> PX4 OFFBOARD velocity control (proof of concept).

  Serves web/index.html plus a WebSocket at /ws, and streams body-frame velocity
  setpoints to PX4 SITL at 20 Hz. Four commands only: climb, descend, forward,
  backward.

  Run with Isaac Sim already playing drone_setup_px4_cesium.py:

      conda run -n drone python joystick-server.py

  then open http://<box-ip>:8090/ from any device on the LAN.

  Design: docs/superpowers/specs/2026-07-30-joystick-offboard-design.md
  """
  import argparse
  import asyncio
  import json
  import math
  import os
  import queue
  import sys
  import threading
  import time

  from pymavlink import mavutil

  ROOT = os.path.dirname(os.path.abspath(__file__))
  sys.path.insert(0, os.path.join(ROOT, "streaming"))
  import offboard  # noqa: E402


  class SetpointLoop(threading.Thread):
      """Sole owner of the MAVLink connection.

      Sends a velocity setpoint every tick forever -- zeros are a valid hover
      setpoint, and a gap in the stream drops PX4 out of OFFBOARD. One-shot
      commands arrive on a queue and execute on this thread so that nothing
      else ever touches `conn`.
      """

      def __init__(self, conn, state, rate_hz=20.0, takeoff_alt=5.0, warmup_s=1.0):
          super().__init__(daemon=True)
          self.conn = conn
          self.state = state
          self.link = offboard.OffboardLink(conn)
          self.dt = 1.0 / rate_hz
          self.takeoff_alt = takeoff_alt
          self.warmup_s = warmup_s
          self.commands = queue.Queue()
          self._telem_lock = threading.Lock()
          self._telem = {
              "connected": False,
              "armed": False,
              "mode": "--",
              "alt_m": 0.0,
              "vz": 0.0,
              "heading_deg": 0.0,
              "ready_for_offboard": False,
              "streaming_s": 0.0,
          }
          self._stream_start = None
          self._params_sent = False

      def telemetry(self):
          with self._telem_lock:
              return dict(self._telem)

      def submit(self, name):
          """Called from the web thread. Queue only -- never touches `conn`."""
          if name in ("arm", "disarm", "takeoff", "land", "offboard"):
              self.commands.put(name)

      def _run_command(self, name):
          getattr(self.link, name)()
          print(f">>> command: {name}")

      def _send_startup_params(self):
          self.link.set_param("COM_RCL_EXCEPT", offboard.COM_RCL_EXCEPT_OFFBOARD,
                              offboard.MAV_PARAM_TYPE_INT32)
          self.link.set_param("MIS_TAKEOFF_ALT", self.takeoff_alt,
                              offboard.MAV_PARAM_TYPE_REAL32)
          self._params_sent = True
          print(f">>> params: COM_RCL_EXCEPT=4 (offboard exempt from RC-loss "
                f"failsafe), MIS_TAKEOFF_ALT={self.takeoff_alt}")

      def _drain_mavlink(self):
          while True:
              msg = self.conn.recv_match(blocking=False)
              if msg is None:
                  return
              kind = msg.get_type()
              if kind == "HEARTBEAT":
                  self.link.bind_target(msg)
                  armed = bool(msg.base_mode & offboard.MAV_MODE_FLAG_SAFETY_ARMED)
                  mode = offboard.decode_px4_mode(msg.custom_mode)
                  with self._telem_lock:
                      self._telem["connected"] = True
                      self._telem["armed"] = armed
                      self._telem["mode"] = mode
                  if not self._params_sent:
                      self._send_startup_params()
              elif kind == "LOCAL_POSITION_NED":
                  with self._telem_lock:
                      self._telem["alt_m"] = -msg.z    # NED down -> altitude up
                      self._telem["vz"] = -msg.vz
              elif kind == "ATTITUDE":
                  with self._telem_lock:
                      self._telem["heading_deg"] = (
                          math.degrees(msg.yaw) + 360.0) % 360.0

      def run(self):
          next_tick = time.monotonic()
          while True:
              self._drain_mavlink()
              while True:
                  try:
                      self._run_command(self.commands.get_nowait())
                  except queue.Empty:
                      break
              vx, vy, vz = self.state.velocity()
              self.link.send_velocity(vx, vy, vz)

              if self._stream_start is None:
                  self._stream_start = time.monotonic()
              streaming_s = time.monotonic() - self._stream_start
              with self._telem_lock:
                  self._telem["streaming_s"] = streaming_s
                  self._telem["ready_for_offboard"] = (
                      streaming_s >= self.warmup_s and self._telem["connected"])

              next_tick += self.dt
              nap = next_tick - time.monotonic()
              if nap > 0:
                  time.sleep(nap)
              else:
                  next_tick = time.monotonic()   # fell behind; resync
  ```
- [ ] Verify it imports without needing fastapi (deliberate — `build_app` and
  `uvicorn` are imported lazily so the loop test in Task 6 stays lightweight):
  ```bash
  conda run -n drone python -c "
  import importlib.util
  s = importlib.util.spec_from_file_location('js', 'joystick-server.py')
  m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
  print('SetpointLoop', m.SetpointLoop.__name__)"
  ```
  Expected: `SetpointLoop SetpointLoop`
- [ ] Commit: `feat(joystick): add 20 Hz OFFBOARD setpoint thread`

---

## Task 6: Loop integration test against a fake PX4

**Create:** `streaming/tests/test_offboard_loop.py`

### Step 6.1 — Write the test

- [ ] Create `streaming/tests/test_offboard_loop.py`:

  ```python
  """Integration test for the setpoint thread against a fake PX4 over real UDP.

  No PX4 and no Isaac Sim required. Ports are deliberately NOT 14540/14580 so
  the test cannot collide with a real SITL instance running on the same box.
  """
  import importlib.util
  import os
  import sys
  import time

  from pymavlink import mavutil

  ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
  sys.path.insert(0, os.path.join(ROOT, "streaming"))
  import offboard  # noqa: E402

  FAKE_PX4_PORT = 14585


  def _load_server():
      spec = importlib.util.spec_from_file_location(
          "joystick_server", os.path.join(ROOT, "joystick-server.py"))
      mod = importlib.util.module_from_spec(spec)
      spec.loader.exec_module(mod)
      return mod


  def _collect(px4, seconds, kind="SET_POSITION_TARGET_LOCAL_NED"):
      seen = []
      deadline = time.time() + seconds
      while time.time() < deadline:
          msg = px4.recv_match(type=kind, blocking=True, timeout=0.5)
          if msg is not None:
              seen.append(msg)
      return seen


  def test_loop_streams_setpoints_fast_enough_for_offboard():
      """PX4 rejects OFFBOARD unless setpoints arrive above 2 Hz. At 20 Hz we
      should comfortably clear 10 messages in one second."""
      js = _load_server()
      px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{FAKE_PX4_PORT}")
      try:
          conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{FAKE_PX4_PORT}")
          state = offboard.CommandState(2.0, 1.0, watchdog_s=10.0)
          js.SetpointLoop(conn, state, rate_hz=20.0).start()
          state.set("fwd", True)

          seen = _collect(px4, 1.0)
          assert len(seen) >= 10, f"expected >=10 setpoints in 1 s, got {len(seen)}"

          last = seen[-1]
          assert last.coordinate_frame == offboard.MAV_FRAME_BODY_NED
          assert last.type_mask == offboard.VEL_YAWRATE_TYPE_MASK
          assert abs(last.vx - 2.0) < 1e-6
          assert last.vy == 0.0
          assert last.yaw_rate == 0.0
      finally:
          px4.close()


  def test_loop_keeps_streaming_zeros_when_nothing_is_held():
      """Idle must not mean silent: a gap in the stream drops OFFBOARD, so
      hovering has to be expressed as an explicit zero setpoint."""
      js = _load_server()
      px4 = mavutil.mavlink_connection(f"udpin:127.0.0.1:{FAKE_PX4_PORT + 1}")
      try:
          conn = mavutil.mavlink_connection(
              f"udpout:127.0.0.1:{FAKE_PX4_PORT + 1}")
          state = offboard.CommandState(2.0, 1.0, watchdog_s=10.0)
          js.SetpointLoop(conn, state, rate_hz=20.0).start()

          seen = _collect(px4, 1.0)
          assert len(seen) >= 10, f"idle loop went quiet: only {len(seen)} sent"
          assert all(m.vx == 0.0 and m.vy == 0.0 and m.vz == 0.0 for m in seen)
      finally:
          px4.close()
  ```
- [ ] Run:
  ```bash
  conda run -n drone pytest streaming/tests/ -q
  ```
  Expected: `30 passed` (28 unit + 2 loop). Takes ~3 s.
- [ ] Commit: `test(joystick): verify setpoint rate and payload against a fake PX4`

---

## Task 7: FastAPI app + entry point

**Modify:** `joystick-server.py`

### Step 7.1 — Append the web layer

- [ ] Append to `joystick-server.py`:

  ```python
  async def _push_telemetry(sock, loop_thread, hz=5.0):
      try:
          while True:
              await sock.send_text(json.dumps(loop_thread.telemetry()))
              await asyncio.sleep(1.0 / hz)
      except Exception:
          pass          # socket closed; the /ws handler cleans up


  def build_app(loop_thread, state, video_port):
      from fastapi import FastAPI, WebSocket, WebSocketDisconnect
      from fastapi.responses import FileResponse, JSONResponse

      app = FastAPI()

      @app.get("/")
      def index():
          return FileResponse(os.path.join(ROOT, "web", "index.html"))

      @app.get("/config")
      def config():
          return JSONResponse({"video_port": video_port})

      @app.websocket("/ws")
      async def ws(sock: WebSocket):
          await sock.accept()
          state.clear()
          pusher = asyncio.create_task(_push_telemetry(sock, loop_thread))
          try:
              while True:
                  msg = json.loads(await sock.receive_text())
                  kind = msg.get("type")
                  if kind == "axis":
                      state.set(msg["dir"], bool(msg.get("pressed")))
                  elif kind == "cmd":
                      loop_thread.submit(msg["name"])
                  elif kind == "ping":
                      state.touch()
          except (WebSocketDisconnect, json.JSONDecodeError, KeyError, ValueError):
              pass
          finally:
              pusher.cancel()
              # A dropped socket must not latch the last commanded velocity.
              state.clear()

      return app


  def main():
      ap = argparse.ArgumentParser(
          description="Web joystick -> PX4 OFFBOARD velocity control")
      ap.add_argument("--mavlink", default="udpin:0.0.0.0:14540",
                      help="PX4 offboard link. MUST be udpin: PX4 binds 14580 "
                           "and sends TO 14540, so udpout never receives.")
      ap.add_argument("--port", type=int, default=8090, help="web UI port")
      ap.add_argument("--video-port", type=int, default=8080,
                      help="Isaac MJPEG port from drone_setup_px4_cesium.py")
      ap.add_argument("--speed-fwd", type=float, default=2.0, help="m/s")
      ap.add_argument("--speed-up", type=float, default=1.0, help="m/s")
      ap.add_argument("--takeoff-alt", type=float, default=5.0, help="m")
      ap.add_argument("--watchdog", type=float, default=0.5,
                      help="seconds of silence before velocity is forced to zero")
      ap.add_argument("--rate", type=float, default=20.0, help="setpoint Hz")
      ap.add_argument("--offboard-warmup", type=float, default=1.0,
                      help="seconds of streaming before OFFBOARD is offered")
      args = ap.parse_args()

      import uvicorn

      conn = mavutil.mavlink_connection(args.mavlink)
      state = offboard.CommandState(args.speed_fwd, args.speed_up, args.watchdog)
      loop_thread = SetpointLoop(conn, state, args.rate, args.takeoff_alt,
                                 args.offboard_warmup)
      loop_thread.start()

      print(f">>> MAVLink offboard link: {args.mavlink}")
      print(f">>> setpoint loop at {args.rate:.0f} Hz "
            f"({args.speed_fwd} m/s fwd, {args.speed_up} m/s climb)")
      print(f">>> open http://<box-ip>:{args.port}/")
      uvicorn.run(build_app(loop_thread, state, args.video_port),
                  host="0.0.0.0", port=args.port, log_level="warning")


  if __name__ == "__main__":
      main()
  ```
- [ ] Verify the CLI parses and the tests still pass:
  ```bash
  conda run -n drone python joystick-server.py --help
  conda run -n drone pytest streaming/tests/ -q
  ```
  Expected: help text listing all nine flags; then `30 passed`.
- [ ] Commit: `feat(joystick): add FastAPI server with WebSocket control + telemetry`

---

## Task 8: The web UI

**Create:** `web/index.html`

### Step 8.1 — Write the page

- [ ] Create `web/index.html`:

  ```html
  <!doctype html>
  <html>
  <head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
  <title>Drone joystick</title>
  <style>
    :root { color-scheme: dark; }
    body { margin:0; background:#111; color:#ddd; font:14px system-ui,sans-serif;
           display:flex; flex-direction:column; align-items:center; gap:10px;
           padding:10px; touch-action:none; user-select:none;
           -webkit-user-select:none; }
    #video { width:min(92vw,640px); aspect-ratio:16/10; background:#000;
             border:1px solid #333; object-fit:cover; }
    #telem { display:flex; gap:14px; flex-wrap:wrap; justify-content:center;
             font-variant-numeric:tabular-nums; }
    #telem b { color:#fff; font-weight:600; }
    .bad { color:#f66; } .good { color:#6f6; }
    #pad { display:grid; grid-template-columns:repeat(3,72px);
           grid-template-rows:repeat(3,72px); gap:6px; }
    #pad button { font-size:26px; border-radius:10px; border:1px solid #444;
                  background:#222; color:#eee; }
    #pad button.on { background:#2a6; border-color:#6f6; }
    #cmds { display:flex; gap:6px; flex-wrap:wrap; justify-content:center; }
    #cmds button { padding:9px 14px; border-radius:8px; border:1px solid #444;
                   background:#222; color:#eee; font-size:14px; }
    #cmds button:disabled { opacity:.35; }
  </style>
  </head>
  <body>
    <img id="video" alt="sim camera feed">
    <div id="telem">
      <span>link <b id="t-link" class="bad">--</b></span>
      <span>mode <b id="t-mode">--</b></span>
      <span>armed <b id="t-armed">--</b></span>
      <span>alt <b id="t-alt">--</b> m</span>
      <span>hdg <b id="t-hdg">--</b>&deg;</span>
    </div>
    <div id="pad">
      <span></span><button data-dir="up">&#9650;</button><span></span>
      <button data-dir="back">&#9664;</button><span></span><button data-dir="fwd">&#9654;</button>
      <span></span><button data-dir="down">&#9660;</button><span></span>
    </div>
    <div id="cmds">
      <button id="c-arm">ARM</button>
      <button id="c-takeoff">TAKEOFF</button>
      <button id="c-offboard">OFFBOARD</button>
      <button id="c-land">LAND</button>
      <button id="c-disarm">DISARM</button>
    </div>
  <script>
  const held = new Set();
  let ws = null;
  const el = id => document.getElementById(id);

  // The video comes from the Isaac MJPEG server on a different port, so derive
  // the host from the page rather than hard-coding an IP -- this has to work
  // from a phone as well as from the box itself.
  fetch('/config').then(r => r.json()).then(c => {
    el('video').src = `http://${location.hostname}:${c.video_port}/detect`;
  });

  function connect() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onmessage = e => paint(JSON.parse(e.data));
    ws.onclose = () => { held.clear(); paintPad(); setTimeout(connect, 1000); };
  }

  function send(o) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(o)); }

  function paintPad() {
    document.querySelectorAll('#pad button').forEach(
      b => b.classList.toggle('on', held.has(b.dataset.dir)));
  }

  function paint(t) {
    el('t-link').textContent = t.connected ? 'up' : 'down';
    el('t-link').className = t.connected ? 'good' : 'bad';
    el('t-mode').textContent = t.mode;
    el('t-mode').className = t.mode === 'OFFBOARD' ? 'good' : 'bad';
    el('t-armed').textContent = t.armed ? 'yes' : 'no';
    el('t-alt').textContent = t.alt_m.toFixed(1);
    el('t-hdg').textContent = t.heading_deg.toFixed(0);
    el('c-offboard').disabled = !t.ready_for_offboard;
    ['c-arm','c-takeoff','c-land','c-disarm'].forEach(
      id => el(id).disabled = !t.connected);
  }

  function press(dir, on) {
    if (on === held.has(dir)) return;          // ignore key auto-repeat
    on ? held.add(dir) : held.delete(dir);
    send({type:'axis', dir:dir, pressed:on});
    paintPad();
  }

  document.querySelectorAll('#pad button').forEach(b => {
    const d = b.dataset.dir;
    b.addEventListener('pointerdown', e => {
      e.preventDefault(); b.setPointerCapture(e.pointerId); press(d, true); });
    b.addEventListener('pointerup', e => { e.preventDefault(); press(d, false); });
    b.addEventListener('pointercancel', () => press(d, false));
    b.addEventListener('contextmenu', e => e.preventDefault());
  });

  const KEYS = {ArrowUp:'up', ArrowDown:'down', ArrowRight:'fwd', ArrowLeft:'back',
                w:'up', s:'down', d:'fwd', a:'back'};
  addEventListener('keydown', e => {
    if (KEYS[e.key]) { e.preventDefault(); press(KEYS[e.key], true); } });
  addEventListener('keyup', e => {
    if (KEYS[e.key]) { e.preventDefault(); press(KEYS[e.key], false); } });

  // Releasing on blur stops a stuck key from latching a velocity the operator
  // has tabbed away from and can no longer see or cancel.
  addEventListener('blur', () => { [...held].forEach(d => press(d, false)); });

  [['c-arm','arm'], ['c-takeoff','takeoff'], ['c-offboard','offboard'],
   ['c-land','land'], ['c-disarm','disarm']].forEach(([id, name]) =>
    el(id).addEventListener('click', () => send({type:'cmd', name:name})));

  // Keepalive: the server zeroes velocity after 0.5 s of silence, and holding a
  // button sends exactly one message, so a held direction must be refreshed.
  setInterval(() => { if (held.size) send({type:'ping'}); }, 150);

  connect();
  </script>
  </body>
  </html>
  ```
- [ ] Smoke test the page without PX4 (the server starts regardless; telemetry
      will read `link down`):
  ```bash
  conda run -n drone python joystick-server.py &
  sleep 3
  curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/
  curl -s http://127.0.0.1:8090/config
  kill %1
  ```
  Expected: `200`, then `{"video_port":8080}`.
- [ ] Commit: `feat(joystick): add web UI with 4-way pad, telemetry and sim video`

---

## Task 9: Manual acceptance in Isaac Sim

**Files:** none — this is the proof.

- [ ] Open Isaac Sim with the Cesium stage loaded and simulation **stopped**.
- [ ] Script Editor → paste `drone_setup_px4_cesium.py` → Run → press **Play**.
      Confirm the console prints the MJPEG server line for port 8080.
- [ ] Confirm the camera feed is reachable:
  ```bash
  curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/detect
  ```
  Expected: `200`.
- [ ] Start the server:
  ```bash
  conda run -n drone python joystick-server.py
  ```
  Expected within a few seconds: the MAVLink link line, the setpoint-rate line,
  and `>>> params: COM_RCL_EXCEPT=4 ...` — that last line only prints after a
  HEARTBEAT arrives, so it is the proof the `udpin` direction is correct.
- [ ] Open `http://<box-ip>:8090/`. Expect: video visible, `link up`, mode
      reading something like `POSCTL` or `AUTO.LOITER`, OFFBOARD button enabled
      after ~1 s.
- [ ] Press **ARM** → telemetry `armed yes`.
- [ ] Press **TAKEOFF** → mode `AUTO.TAKEOFF`, altitude climbs to ~5 m, settles.
- [ ] Press **OFFBOARD** → mode reads `OFFBOARD` in green.
- [ ] **Hold ▶ for 3 seconds.** Expect the drone to translate roughly 6 m
      forward (2 m/s × 3 s) along its nose in the viewport and in QGC, then hold
      position when released.
- [ ] **Hold ▲ for 3 seconds.** Expect altitude to rise ~3 m, then hold.
- [ ] **Hold ◀ and ▼** and confirm they reverse those motions.
- [ ] **Watchdog check:** hold ▶ and kill the browser tab mid-press. The drone
      must stop within ~0.5 s rather than continuing forward.
- [ ] Press **LAND** → mode `AUTO.LAND`, drone descends and disarms.
- [ ] Commit: `docs(plan): record joystick OFFBOARD manual acceptance results`

---

## Plan validation

**Spec coverage.** Four-command mapping (Task 1), hold-to-move with watchdog and
keepalive (Tasks 2, 8), `MAV_FRAME_BODY_NED` + mask 1479 + `yaw_rate=0` (Tasks 1,
4), `COM_RCL_EXCEPT` and `MIS_TAKEOFF_ALT` at startup (Task 5), OFFBOARD warmup
gate (Tasks 5, 8), arm/takeoff/offboard/land/disarm (Tasks 4, 5, 8), mode-drift
visibility (Tasks 3, 8), continuous stream when idle (Task 6), `/detect` video
feed (Task 8), unit + fake-PX4 + manual testing (Tasks 1–4, 6, 9).

**Consistency.** Every symbol in the naming contract appears with the same
signature in each task that uses it. Direction strings are `"fwd"`, `"back"`,
`"up"`, `"down"` in the mapping function, the state class, the tests, and the
HTML `data-dir` attributes. Test counts are cumulative and stated per task: 9 →
16 → 21 → 28 unit, plus 2 loop = 30.

**No placeholders.** Every code block is complete and runnable as written.

## Follow-ups (deliberately not in this plan)

- `2026-07-13-pipeline-streaming.md:550` specifies `udpout:127.0.0.1:14540` for
  the same PX4 link and needs the same `udpin` correction before v1 is built.
- `fpv_cam` is gone with the old setup script, so `vio-recorder-pai.py` records
  mono cam0. Port the FPV block over if two-sensor VIO recording is wanted.
- The stage up-axis guard from the removed `-pai` script was never carried over.
- Re-enable `ADD_WIND` in `drone_setup_px4_cesium.py:41` once the PoC passes, as
  a robustness demo.
