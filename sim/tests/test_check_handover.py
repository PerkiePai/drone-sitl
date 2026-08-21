"""sim/check_handover.py's cut-finding and reset-matching, against synthetic
parameter changes and reset lists with known answers. No PX4, no Isaac Sim,
no ulog -- the ULog reading lives in main() precisely so this can be plain
arrays."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import check_handover as ch  # noqa: E402


def test_cut_times_finds_every_gnss_cut():
    """A campaign flies several candidates down one log, so a cut is not a
    single event -- the four screening cuts all live in 10_45_50.ulg."""
    changed = [(761_430_000, "EKF2_GPS_CTRL", 0),
               (831_750_000, "EKF2_GPS_CTRL", 7),
               (879_470_000, "EKF2_GPS_CTRL", 0)]
    assert ch.cut_times(changed) == [761.43, 879.47]


def test_cut_times_ignores_the_restore():
    """`EKF2_GPS_CTRL=7` is the restore, the exact inverse of the cut. Counting
    it would report twice as many handovers as were actually flown."""
    changed = [(100_000_000, "EKF2_GPS_CTRL", 7)]
    assert ch.cut_times(changed) == []


def test_cut_times_ignores_other_params():
    """The gain sweep rewrites all four MPC_XY_* on every candidate, so the
    changed-parameter list is mostly noise as far as this is concerned."""
    changed = [(100_000_000, "MPC_XY_VEL_I_ACC", 0.0),
               (100_000_000, "EKF2_EV_CTRL", 0)]
    assert ch.cut_times(changed) == []


def test_worst_reset_picks_the_largest_in_the_window():
    """The 39.9 m reset and its ~1 s bounce-back are one failure, and the
    headline number is the bigger of the two."""
    resets = [(1786.48, 39.94), (1787.52, 39.92)]
    assert ch.worst_reset(1786.44, resets) == 39.94


def test_a_reset_from_earlier_in_the_hold_is_not_counted():
    """Phase 2 produces resets of its own -- eight of 20-35 m during Step
    10.7's hold alone. Those are the hold failing, not the handover, and
    charging them to the cut would make every flight unfalsifiable."""
    resets = [(1750.00, 30.0), (1816.00, 22.0)]
    assert ch.worst_reset(1786.44, resets) == 0.0


def test_no_reset_at_all_is_zero_not_an_error():
    assert ch.worst_reset(1786.44, []) == 0.0


def test_a_tiny_reset_still_passes():
    """PX4 ALWAYS resets position at the cut -- `bias_estimator_change` makes
    `reset` true and `resetHorizontalPositionTo` runs unconditionally
    (ev_pos_control.cpp:182-233). The fix cannot remove the reset; it can only
    make the reset land where EKF2 already was. So the bar is magnitude, not
    the absence of a reset."""
    assert ch.verdict(0.04) == "PASS"


def test_the_known_failure_magnitudes_all_fail():
    """The five cuts on record. A checker that cannot fail these proves
    nothing when it passes."""
    for m in (9.21, 14.15, 28.38, 39.94, 69.75):
        assert ch.verdict(m) == "FAIL", m
