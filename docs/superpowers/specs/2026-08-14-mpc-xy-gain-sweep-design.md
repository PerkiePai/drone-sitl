# Design: MPC_XY_* gain sweep for vision-only station-keeping

## Why this exists

Task 9 (`docs/superpowers/plans/2026-08-07-vio-gps-denied.md`) classified the
GPS-denied hold's failure mode without touching a single flight parameter,
per ADR-0002. It ruled out, in order: heading/yaw error (9.4d — EKF2's fused
yaw, an independently-computed raw magnetometer heading, and the vision
yaw all agreed within ~2° throughout), an orbit/spiral shape (9.4b — the
track is a straight line at one fixed bearing, a damped overshoot settling
~225 m off-target, not a rotation), a sim-rate-dependent fusion delay (9.4c
— the applied EV delay is a stable ~83 ms, uncorrelated with `sim_rate`),
and every `EKF2_EV_*` vision-fusion tuning parameter (9.4e — an offline
replay sweep over `EKF2_EV_DELAY`, `EKF2_EVP_NOISE`, and `EKF2_EV_CTRL`
with yaw fusion on/off all reproduced the same ~225 m-off result within 4%
of each other).

9.4e's own limitation points at what's left: replay is open-loop — EKF2 is
re-fed the *exact recording* of what the real, closed-loop flight already
did, regardless of `EKF2_EV_*` params. That the result barely changes says
the instability isn't in how EKF2 weighs the vision measurement. It's
either upstream (a closed-loop effect between the real aircraft's motion
and the vision estimator's own output, which no replay can reproduce) or
downstream — in the position controller's own gains, `MPC_XY_*`, which no
diagnostic step has touched yet and which are flying today on PX4's stock
defaults, tuned for GPS, never adapted for a noisier vision source. This
plan tests the downstream half of that split, because it's the only half a
live flight can screen quickly.

## Goal

Find an `MPC_XY_*` gain set that holds the ADR-0001 bar for Task 8.6: 180 s
vision-only, non-growing aircraft-excursion envelope. If nothing in a
targeted, physically-motivated sweep does, that itself is a real result —
it would point the remaining suspicion back upstream, at the vision
estimator's own closed-loop behavior, which is out of this plan's scope.

## Architecture

```
sim/mpc_gain_sweep.py  (new)              joystick-server.py (existing, +1 command)
  connects to ws://<host>:8090/ws   ──►   handles "set_param" over the same
  drives arm/takeoff/offboard/              websocket protocol as every
  gps_denied/gps_restore/land/              existing flight command
  disarm (existing commands,
  unchanged)                          ──►  calls offboard.py's existing
  sets MPC_XY_* between flights             link.set_param() primitive
  writes campaign_<name>.json              (already used for the EKF2
  (candidate order + gains + times)         param bundles)
  reads run.csv afterward for
  excursion_m per candidate
```

One continuous session: Isaac Sim, PX4, and `joystick-server.py` stay up
for the whole campaign. No PX4 reboot between candidates — `MPC_XY_P`,
`MPC_XY_VEL_P_ACC`, `MPC_XY_VEL_I_ACC`, `MPC_XY_VEL_D_ACC` are ordinary
runtime params — confirmed against `mc_pos_control_params.c`: none of the
four carry `@reboot_required`, unlike `EKF2_HGT_REF`. This matters
because Task 9's phase-0 reboot exists specifically to work around
`EKF2_HGT_REF` being reboot-required — the gain sweep has no equivalent
cost per candidate.

`joystick-server.py` already computes and logs `excursion_m` into
`run.csv` every tick (Task 9.3). The sweep needs no new ulog/CSV parsing:
each candidate's hold window is the *k*-th `phase == "2"` segment in the
one continuous `run.csv`, in flight order. The driver's own
`campaign_<name>.json` sidecar records candidate name, gains, and
approximate sim-time boundaries as it flies each one, so the analysis
script doesn't have to guess which segment belongs to which candidate —
order is deterministic within a single continuous session.

## Decisions

### D1 — One new websocket command, not a new connection

`joystick-server.py` already owns the single MAVLink connection to PX4;
nothing else may open a second one (this is the reason `--vision`'s
estimator subprocess talks to `joystick-server.py` over ZMQ rather than
MAVLink directly). The sweep driver is therefore a *websocket* client of
`joystick-server.py`, like the web page, not a second MAVLink client.
`set_param` is one more command alongside `arm`/`takeoff`/`gps_denied`,
handled the same way, and reuses `offboard.py`'s existing `set_param()`
(the one Task 3's INT32-bit-pattern fix already lives in — no new
param-encoding code).

### D2 — Candidates are one-factor-and-one-combo, not a factorial sweep

A full factorial over 4 gains would be dozens of flights. The failure
signature (underdamped, single overshoot, settles on a wrong equilibrium
rather than diverging or oscillating indefinitely) motivates a small,
targeted set instead:

| Candidate | `MPC_XY_P` | `MPC_XY_VEL_P_ACC` | `MPC_XY_VEL_I_ACC` | `MPC_XY_VEL_D_ACC` | Rationale |
|---|---|---|---|---|---|
| `baseline` | 0.95 | 1.8 | 0.4 | 0.2 | stock default (PX4's shipped values), flown fresh under this campaign's own methodology as the reference point |
| `more_damping` | 0.95 | 1.8 | 0.4 | 0.5 | directly targets "underdamped" — more D should cut the overshoot |
| `gentler_p` | 0.5 | 1.2 | 0.4 | 0.2 | less aggressive reaction to perceived position error at all |
| `low_integral` | 0.95 | 1.8 | 0.05 | 0.2 | tests whether the I-term winds onto the wrong equilibrium — live risk if vision's position bias looks locally constant once the aircraft settles into a new spot |
| `gentle_combo` | 0.5 | 1.2 | 0.05 | 0.5 | all three changes together, as a "kitchen sink gentler" candidate |

Each row is a complete override set (all four params always set together,
even where a row matches the default, so every flight's active gain set is
explicit in the sidecar log rather than inferred from what didn't change).

### D3 — Screening bar (70 s) vs. confirmation bar (180 s, ADR-0001)

A full 180 s per candidate, times 5 candidates, is 15 minutes of hold time
alone before climb/settle/land overhead — expensive to spend on candidates
that are obviously worse. 70 s (~1.5 periods of the observed 40-60 s mode)
is enough to see the overshoot magnitude and whether the last third of the
window is flat/falling vs. still rising, not enough to fully trust a
"stable" verdict on its own. Screening ranks candidates on:

1. Peak `excursion_m` during the hold.
2. Trend in the final 20 s of the hold — a linear fit's slope, flagged
   `growing` if positive beyond a small tolerance, `settling` otherwise.

Whichever candidate ranks best on both gets one full 180 s flight, judged
against ADR-0001's actual bar (non-growing excursion envelope over three or
more oscillation periods) — this flight, if it passes, closes 8.6.

### D4 — `RESTORE GNSS` reverts `MPC_XY_*` too, symmetric with `EKF2_GPS_CTRL`

`_go_gps_restore()` already reverts `EKF2_GPS_CTRL` to
`EKF2_GPS_CTRL_DEFAULT` and is deliberately ungated (recovering has no
failure mode the cut needs to guard against — `streaming/vision_bridge.py`
`EKF2_GNSS_RESTORE_PARAMS`). Whatever gain set was active during a
GPS-denied hold must revert to PX4's stock `MPC_XY_*` defaults the instant
GNSS is restored, so a route or manual flight afterward is never silently
flying on gentler-than-GPS-normal gains. This is a `joystick-server.py`
change (`_go_gps_restore`), not sweep-only — it's a permanent, general
fix to `RESTORE GNSS`'s behavior, on by default even outside a sweep
campaign.

### D5 — Screening flights self-abort on a bad candidate

A badly-chosen gain set could plausibly perform worse than anything seen
so far — this project's GPS-denied flights have never crashed (`SESSION.md`
notes every one landed itself safely once commanded), but the sweep is
deliberately trying gains nobody has flown, and 70 s unattended is long
enough for a genuinely unstable candidate to do real damage before the
screen even finishes. The driver aborts a screening flight early
(commands `gps_restore` immediately, skips the rest of that candidate's
hold) if, while `phase == "2"`: `excursion_m` exceeds 400 m (well past the
worst peak seen so far, ~268 m), or `alt` (height above the launch point)
drops by more than 20 m from its value at the cut (a genuine descent, not
the flat-plane sign-convention trap `SESSION.md` documents for
near-`ground_z` readings). An aborted candidate is recorded as `aborted`
in the sidecar log with whatever partial data it collected, not silently
dropped.

## Components

### `sim/mpc_gain_sweep.py` (new, repo root's `sim/`, `conda run -n drone`)

CLI driver. Connects to `joystick-server.py`'s `/ws`, drives the standard
profile per candidate (arm → takeoff → wait `OFFBOARD` → wait vision
fusing healthy → settle 20 s sim-time → `gps_denied` → hold (70 s screen or
180 s confirm) with the D5 abort check → `gps_restore` → wait `AUTO.LAND`
→ disarm → `set_param` the next candidate's four gains → repeat). Takes
`--candidates screen|confirm:<name>` (screen = all 5 at 70 s; confirm:name
= one candidate at 180 s) and `--campaign-name` (names the sidecar JSON and
is passed through as `joystick-server.py`'s `--run-name` at the *caller's*
own server-start time, unchanged from today's manual flow — this script
does not start the server itself, matching how `sim/save-ulog.sh` doesn't
either).

### `joystick-server.py` (modified)

One new websocket command, `set_param` (`{cmd: "set_param", name, value,
type}`), routed the same way `arm`/`gps_denied`/etc. already are —
straight through to `offboard.py`'s `link.set_param()`. `_go_gps_restore`
gains a call to revert `MPC_XY_*` to defaults (D4), unconditionally,
alongside its existing `EKF2_GPS_CTRL` revert.

### `streaming/vision_bridge.py` (modified)

Add `MPC_XY_DEFAULTS` (the four stock values, matching D2's `baseline`
row) alongside the existing `EKF2_*` param tuples, and a
`revert_mpc_gains(link)` helper next to `apply_ekf2_gnss_restore_params` —
same module, since `_go_gps_restore` already imports its EKF2 revert
helper from here and the two reverts belong together.

### `sim/analyze_gain_sweep.py` (new)

Reads a campaign's `run.csv` and its `campaign_<name>.json` sidecar,
slices the *k*-th `phase == "2"` segment per candidate in flight order,
computes peak `excursion_m`, the final-20s trend slope, and prints a
ranked table (same shape as the one in this design doc's D2 section, with
measured values filled in). No ulog parsing needed — everything it reads
already exists in `run.csv`.

## Data flow / error handling

- **Websocket connection lost mid-campaign**: the existing watchdog in
  `joystick-server.py` already zeroes velocity setpoints on a stale
  connection — the sweep driver reconnecting resumes at whatever phase the
  aircraft was actually in; it does not assume state.
- **A candidate flight never reaches `OFFBOARD`** (mirrors 9.4c's `FLY`
  gating logic): the driver times out (30 s sim-time) and marks that
  candidate `failed_to_offboard` in the sidecar rather than hanging the
  whole campaign.
- **`gps_denied` refused** (any of `_go_gps_denied`'s existing gates —
  not fusing, unhealthy estimate, no PX4 pose to align onto): the driver
  retries once after 5 s, then marks the candidate `failed_to_cut` and
  moves on — this should not happen if the 20 s settle already worked for
  every prior candidate in the same session, so a second failure is worth
  surfacing rather than looping.
- **D5's abort fires**: candidate marked `aborted`, campaign continues to
  the next one — a bad candidate must not stop the sweep from learning
  about the rest.

## Testing

### Automated (`~/miniconda3/envs/drone/bin/python -m pytest streaming/tests/ -q`)

- `set_param` command round-trips through the same dispatch path as
  existing commands (extend `streaming/tests/test_web_ui.py`'s command
  dispatch tests).
- `revert_mpc_gains` sends exactly the four `MPC_XY_DEFAULTS` params
  (extend `streaming/tests/test_vision_bridge.py`, matching the existing
  `EKF2_GNSS_RESTORE_PARAMS` test style).
- `analyze_gain_sweep.py`'s segment-slicing and trend-slope math, against
  a small synthetic `run.csv` fixture with known phase-2 segments and a
  known slope.

### Manual (live)

The screening campaign and the confirmation flight themselves — this is
the actual deliverable, not a substitute for it. `sim/save-ulog.sh` still
captures the ulog for the confirmation flight, so a passing 8.6 attempt
has a permanent record.

## Out of scope

- The full factorial over all four gains, or gains beyond the four listed
  (`MPC_XY_VEL_MAX` etc.) — D2's targeted set is deliberately narrow.
- Changing anything about the vision estimator itself (flow-odometry
  accuracy, drift correction) — that's the upstream half of the split this
  design's "Why this exists" section named, not touched here.
- Wind or other environmental disturbance testing — `ADD_WIND` stays off,
  matching every prior GPS-denied flight in this project.
- Persisting a winning gain set as a new default for ordinary (non-VIO)
  flight — D4 already reverts on `RESTORE GNSS`, and nothing in this plan
  proposes changing `MPC_XY_*` outside a GPS-denied hold.
