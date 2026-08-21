#!/usr/bin/env python3
"""Did the GNSS cut hand over cleanly? PASS/FAIL per cut, read from a ulog.

    conda run -n drone python sim/check_handover.py logs/<run>/<file>.ulg

The one number that says whether the handover worked. It finds every cut in
the log itself -- `EKF2_GPS_CTRL` going to 0 -- and asks whether EKF2 jumped
its OWN horizontal position at that moment.

It did at all five cuts on record, by 9.2, 14.1, 28.4, 39.9 and 69.7 m, each
matching `estimator_ev_pos_bias` at the cut to within 0.1 m. The cause: PX4
performs the handover natively -- `resetHorizontalPositionTo(measurement)` the
instant `flags.gps` drops (ev_pos_control.cpp:230-233) -- while EKF2 fuses at a
DELAYED horizon, so a realignment issued in the same tick as the param write
has not reached that horizon and PX4 resets onto the PRE-realignment sample.
`joystick-server.py`'s `GPS_DENIED_SETTLE_S` is the fix; this is its test.

Requires the `estimator_ev_pos_bias` and full-rate replay topics, which means
the flight must have been logged with `SDLOG_PROFILE=131` -- phase 0 sets it
under `--vision`, so any ordinary GPS-denied run already has them (ADR-0003).
`sim/save-ulog.sh` is what gets the file out of Pegasus's temp rootfs.
"""
import argparse
import math

CUT_PARAM = "EKF2_GPS_CTRL"
CUT_VALUE = 0
"""`EKF2_GPS_CTRL=0` disables all GNSS fusion -- the cut. 7 is the restore and
is deliberately not counted, or every flight would report twice the handovers
it flew."""

RESET_WINDOW_S = (-0.2, 2.0)
"""Around the cut, in SIM seconds, within which a position reset belongs to the
handover rather than to the hold.

Wide enough at the top to catch the ~1.04 s bounce-back as well as the
immediate jump -- both are the same failure, the second being PX4 hitting its
compile-time 1 s `no_aid_timeout_max` (common.h:449) after rejecting the
vision. Deliberately no wider: phase 2 produces resets of its own -- eight of
20-35 m during Step 10.7's hold alone -- and charging those to the cut would
make the result unfalsifiable.
"""

RESET_TOLERANCE_M = 1.0
"""Above this, the cut moved the aircraft's estimate somewhere it was not.

Not zero, because PX4 always resets at the cut: `bias_estimator_change` makes
`reset` true and `resetHorizontalPositionTo` runs unconditionally
(ev_pos_control.cpp:182-233). The fix cannot remove that reset and does not try
to -- it makes the reset land where EKF2 already was. So the bar is the
magnitude, not the absence.
"""


def cut_times(changed_parameters):
    """Sim seconds at which GNSS was cut, from pyulog's changed_parameters.

    Takes the (timestamp_us, name, value) triples rather than a ULog so the
    logic is testable without a 176 MB fixture.
    """
    return [ts / 1e6 for ts, name, value in changed_parameters
            if name == CUT_PARAM and value == CUT_VALUE]


def worst_reset(cut_s, resets, window=RESET_WINDOW_S):
    """Largest position reset attributable to this cut, in metres; 0.0 if none.

    `resets` is (sim_seconds, magnitude_m) pairs -- every point at which
    `vehicle_local_position.xy_reset_counter` incremented.
    """
    lo, hi = window
    near = [m for t, m in resets if lo <= t - cut_s <= hi]
    return max(near) if near else 0.0


def verdict(worst_m):
    return "FAIL" if worst_m > RESET_TOLERANCE_M else "PASS"


def read_ulog(path):
    """(cuts, resets, bias_lookup) from a ulog. The only impure part."""
    import numpy as np
    from pyulog import ULog

    u = ULog(path, ["vehicle_local_position", "estimator_ev_pos_bias"])
    data = {d.name: d.data for d in u.data_list}
    lp = data["vehicle_local_position"]
    t = lp["timestamp"] / 1e6
    counter, dx, dy = lp["xy_reset_counter"], lp["delta_xy[0]"], lp["delta_xy[1]"]
    resets = [(t[j], math.hypot(dx[j], dy[j]))
              for j in (np.where(np.diff(counter) != 0)[0] + 1)]

    bias = data.get("estimator_ev_pos_bias")

    def bias_at(when_s):
        """`estimator_ev_pos_bias` just BEFORE a cut -- the drift EKF2 was
        holding, which is what the reset magnitude should match."""
        if bias is None:
            return None
        tb = bias["timestamp"] / 1e6
        i = int(np.argmin(np.abs(tb - (when_s - 0.05))))
        return math.hypot(bias["bias[0]"][i], bias["bias[1]"][i])

    return cut_times(u.changed_parameters), resets, bias_at


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulog")
    args = ap.parse_args()

    cuts, resets, bias_at = read_ulog(args.ulog)
    if not cuts:
        print(f"no {CUT_PARAM} -> {CUT_VALUE} in this log: no cut was taken")
        return 2

    failed = 0
    for cut in cuts:
        worst = worst_reset(cut, resets)
        result = verdict(worst)
        failed += result == "FAIL"
        bias = bias_at(cut)
        note = "" if bias is None else f"   ev_pos_bias at cut {bias:6.2f} m"
        detail = (f"EKF2 reset {worst:6.2f} m" if result == "FAIL"
                  else f"no reset ({worst:.2f} m)")
        print(f"{result}  cut t={cut:9.2f}s  {detail}{note}")

    print(f"\n{len(cuts) - failed}/{len(cuts)} cuts clean")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
