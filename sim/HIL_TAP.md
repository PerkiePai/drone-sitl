# hil_tap.py — see and freeze the PX4 ↔ Isaac thrust stream

A throwaway MITM tap on the HIL MAVLink link. Two jobs:

1. **Show** what PX4 sends into Isaac — `HIL_ACTUATOR_CONTROLS`, one
   normalised value per rotor (`0.0`–`1.0`), ~250 Hz — and the
   `HIL_SENSOR` / `HIL_GPS` stream coming back.
2. **Freeze** that stream on command and measure what the sim does.
   Under lockstep, withholding the PX4→Isaac direction halts Isaac's
   physics step within one frame; the tap prints how fast the
   `HIL_SENSOR` return stream flatlines.

## Topology

PX4's `simulator_mavlink` always dials TCP `127.0.0.1:4560` (hardcoded
`4560+instance` in `px4-rc.simulator`). The tap takes that port and
forwards to Isaac on **4561**:

```
PX4  --connect-->  hil_tap :4560  --connect-->  Isaac :4561
```

So Isaac has to be told to listen on 4561. That is the
`SIM_TCP_BASEPORT` tunable in `drone_setup_px4_cesium.py` (default
4560, unchanged for normal runs).

## Run it

```bash
# 1. bring the sim up with Isaac on 4561
DRONE_SETUP_SIM_TCP_BASEPORT=4561 ./sim/run-website.sh

# 2. start the tap (needs pymavlink -- the `drone` env has it)
conda run -n drone python sim/hil_tap.py
```

Order does not matter — the tap retries the Isaac connection for 120 s,
and PX4 retries its side on its own. `stop-website` then a normal
`run-website` (no env var) puts everything back on 4560.

## On the webpage

`http://<box>:8090/` has a **FREEZE SIM** button and a live readout line
under the command row, both wired through `joystick-server.py`:

- `POST /hil/freeze` / `/hil/thaw` write to `logs/hil_tap.ctl` (same file
  as the terminal control-file trick below) -- plain HTTP, not a flight
  command, never touches the MAVLink queue.
- `GET /hil/status` reads `logs/hil_tap.status.json`, which the tap
  rewrites every `--interval` (atomically) regardless of frozen state.
  If that file's mtime is more than 3s old, the endpoint reports
  `{"up": false}` -- that's how the page tells "tap not running" apart
  from "frozen" (a freeze keeps `ts` fresh; only a dead tap goes stale).

Both are no-ops if `hil_tap.py` isn't running -- the button will still
say `{"ok":true}` (the file write succeeds) but nothing will actually
freeze, and the readout will say `tap: not running`.

## Controls

| key | effect |
|-----|--------|
| `<Enter>` | toggle freeze / thaw |
| `s<Enter>` | one-line status |
| `q<Enter>` / `Ctrl-C` | quit |
| `kill -USR1 <pid>` | toggle freeze from another shell |
| `echo freeze > logs/hil_tap.ctl` | toggle freeze -- no PID needed |
| `echo thaw   > logs/hil_tap.ctl` | toggle thaw -- no PID needed |

The control file (`--control-file`, default `logs/hil_tap.ctl`) is polled
every 0.2s and only acts on a *change* in its content, so writing the
same word twice is a no-op.

## What you should see

Each line is the exact message count seen in the last `--interval`
(default 1s) -- not a smoothed rate:

```
[  live] t=   5.960s  P->I m=[0.00 0.00 0.00 0.00] acts= 151/1s   I->P sensor= 150/1s gps= 150/1s
```

After freezing:

```
[tap] FREEZE -- withholding PX4->Isaac thrust. watching HIL_SENSOR return stream (auto-thaw in 20s)...
[tap] Isaac HALTED: HIL_SENSOR return stream went silent 160 ms ago (1 sensor msg(s)
      got through in the 1 ms after freeze, then nothing). sim clock stuck at t=6.220s
[FROZEN] t=   6.220s  P->I m=[0.00 0.00 0.00 0.00] acts=   0/1s   I->P sensor=   0/1s gps=   0/1s
```

`kill -USR1` needs the tap's own python pid — it prints it on startup
(`[tap] up. pid NNNN`), or `pgrep -f sim/hil_tap.py`.

Thaw (`<Enter>`, `SIGUSR1`, or the control file) flushes the held bytes
directly and *then* resumes — `t=` jumps forward to catch up and counts
recover. That is the lockstep guarantee made visible: PX4 does not race
ahead and nothing is dropped — the whole loop parks until the thrust
packet is delivered.

Both directions read `0/1s` within a second of a freeze because the
wire really is silent — PX4 sends one last `HIL_ACTUATOR_CONTROLS` (the
~90 bytes the tap holds) then its scheduler blocks; Isaac is blocked in
its physics step. Only PX4's MAVLink receive thread keeps ticking,
logging `ERROR poll timeout` once a second.

## Keep freezes short — `--max-freeze`

A freeze longer than ~30 s makes PX4's `simulator_mavlink` give up on
the link **for good**: endless `poll timeout`, no reconnect, and the
PX4 process orphans past `stop-website` — a full restart is the only
way back. So the tap **auto-thaws after 20 s** by default:

```
[tap] hit 20s freeze limit
[tap] AUTO-THAW  -- releasing 93 held bytes, sim resumes
```

`--max-freeze 0` disables the guard (don't, unless you mean to restart
afterwards); `--max-freeze 10` for a tighter limit.

## Zero-code alternative

To just confirm the freeze behaviour without the tap:
`kill -STOP $(pgrep -f 'bin/px4')` — PX4 stops sending thrust, Isaac's
sim clock stalls; `kill -CONT` resumes. No stream view, but proves the
"PX4 holds → Isaac waits" path in 5 seconds.

See memory `px4-isaac-lockstep-protocol.md`.
