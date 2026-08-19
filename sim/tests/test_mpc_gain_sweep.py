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
