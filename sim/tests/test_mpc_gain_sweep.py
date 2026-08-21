"""D5's screening-abort check. No PX4, no Isaac Sim, no socket."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import mpc_gain_sweep as sweep  # noqa: E402


def test_should_abort_is_false_with_no_data_yet():
    assert not sweep.should_abort(None, None, None)


def test_should_abort_on_excursion_past_the_worst_peak_seen():
    assert sweep.should_abort(401.0, alt_m=20.0, alt_at_cut_m=20.0)
    assert not sweep.should_abort(399.0, alt_m=20.0, alt_at_cut_m=20.0)


def test_should_abort_on_a_genuine_descent():
    assert sweep.should_abort(0.0, alt_m=-1.0, alt_at_cut_m=20.0)
    assert not sweep.should_abort(0.0, alt_m=5.0, alt_at_cut_m=20.0)


def test_should_abort_ignores_a_near_ground_z_reading_without_a_cut_baseline():
    """SESSION.md, run 6: a very negative alt_m near ground_z is not
    necessarily a strike -- without alt_at_cut_m there is nothing to compare
    it against, so it must not trip the abort."""
    assert not sweep.should_abort(0.0, alt_m=-24.89, alt_at_cut_m=None)


def test_climbed_is_false_while_still_sitting_on_the_pad():
    """The bug this predicate exists for. The aircraft sits in AUTO.LOITER on
    the pad, so a bare `mode == "AUTO.LOITER"` wait is ALREADY satisfied before
    `takeoff` is even sent -- it falls straight through, `offboard` goes out
    0.1 s into AUTO.TAKEOFF, and OFFBOARD's zero-velocity setpoint wins the
    race against the climb. Confirmed live 2026-08-21: nav_state 4 -> 17 -> 14
    inside 0.1 s, then auto preflight disarm 10 s later, never off the ground.
    """
    assert not sweep.climbed({"mode": "AUTO.LOITER", "alt_m": -24.93}, -24.93)


def test_climbed_is_false_while_still_climbing():
    """AUTO.TAKEOFF is not level flight -- switching to OFFBOARD mid-climb is
    the same race, just later in it."""
    assert not sweep.climbed({"mode": "AUTO.TAKEOFF", "alt_m": 10.0}, -24.93)


def test_climbed_is_true_once_levelled_off_at_altitude():
    assert sweep.climbed({"mode": "AUTO.LOITER", "alt_m": 25.0}, -24.93)


def test_climbed_is_false_without_an_altitude_reading():
    """No altitude is not evidence of a climb. Waiting the timeout out and
    reporting failed_to_climb beats flying a candidate that never left."""
    assert not sweep.climbed({"mode": "AUTO.LOITER", "alt_m": None}, -24.93)
    assert not sweep.climbed({"mode": "AUTO.LOITER", "alt_m": 25.0}, None)
