# Session notes: PX4 SITL never sent a heartbeat after the config-driven stage landed

**Date:** 2026-08-07
**Symptom:** `./sim/launch-sitl.sh` reached `>>> Play pressed`, the camera stream
came up on 8080, PX4 launched (confirmed alive, correct args, direct child of
`kit`) — but QGroundControl and `joystick-server.py` both sat on "waiting for
first heartbeat" indefinitely. Reproduced across multiple clean restarts.

## Root cause

**Not this repo's code.** `~/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/backends/px4_mavlink_backend.py`
carries local edits dated **2026-08-05**, changing `PX4MavlinkBackendConfig`'s
*defaults* for a HITL setup (a real flight controller over a VPN link, not
local SITL):

```
connection_type   tcpin     -> udpin
connection_ip     localhost -> 0.0.0.0
enable_lockstep   True      -> False
```

`drone_setup_px4_cesium.py` builds its `PX4MavlinkBackendConfig` without
setting any of those three keys, so it silently inherited the new HITL
defaults. Two independent failures resulted:

1. **Transport mismatch.** PX4's own `px4-rc.simulator` (the `else` branch,
   taken for `PX4_SIM_MODEL=gazebo-classic_iris`) runs
   `simulator_mavlink start -c $simulator_tcp_port` — PX4 connects **out** over
   **TCP**. With the backend defaulting to `udpin`, Isaac was binding a UDP
   socket on 4560 while PX4 tried to TCP-connect to it. The two sides never
   shared a transport, so a MAVLink session could never open.
2. **Lockstep disabled.** Local PX4 SITL's `simulator_mavlink` module blocks
   each cycle waiting for Isaac to supply synchronized sensor data
   (`HIL_SENSOR`) and expects the physics step to wait on its response
   (`HIL_ACTUATOR_CONTROLS`) in return — that handshake **is** lockstep. With
   `enable_lockstep=False`, PX4 spun forever on
   `ERROR [simulator_mavlink] poll timeout 0, 25` (reproduced directly by
   running PX4's exact launch command by hand).

Neither failure produces an exception anywhere — both sides come up looking
healthy (PX4 alive, Isaac serving its camera stream), which is why this took
extensive process/socket forensics (`/proc` scans, `ss`, GPU app list) across
several turns to pin down instead of a five-minute log read.

## Fix

`drone_setup_px4_cesium.py`'s `PX4MavlinkBackendConfig` now states the three
values explicitly instead of relying on the backend's defaults:

```python
"connection_type": "tcpin",
"connection_ip": "localhost",
"enable_lockstep": True,
```

Rationale for stating them rather than reverting the backend patch: the HITL
change in `px4_mavlink_backend.py` is himself deliberate, dated, and explained
in its own comments (real-hardware-over-VPN needs UDP + no lockstep for
different, valid reasons — see that file's `PX4MavlinkBackendConfig.__init__`).
Reverting it would break whatever HITL flow motivated it. This repo's script
should not depend on Pegasus's current default flavor either way — stating the
SITL-correct values here keeps `drone_setup_px4_cesium.py` working regardless
of what the installed Pegasus defaults to next.

## If this resurfaces

- `grep -n "HITL CHANGE" ~/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/backends/px4_mavlink_backend.py`
  shows the current defaults and their dated rationale.
- A transport mismatch shows as: PX4 process alive, `ss -tanp | grep 4560`
  shows nothing or a stuck `SYN-SENT`, `ss -uanp | grep 4560` shows Isaac's
  `kit` process bound. Fix is `connection_type`.
- A lockstep mismatch shows as: PX4 connects, then its own stdout logs
  `ERROR [simulator_mavlink] poll timeout 0, 25` repeatedly. Fix is
  `enable_lockstep`.
- Both are set in one place: the `PX4MavlinkBackendConfig({...})` dict in
  `drone_setup_px4_cesium.py`, `_spawn_px4_keep_stage()`.
