"""sim/analyze_gain_sweep.py's segment-slicing and trend-slope math, against
a synthetic run.csv + sidecar with known phase-2 segments and a known slope.
No PX4, no Isaac Sim."""
import json
import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import analyze_gain_sweep as ags  # noqa: E402


def _row(sim_s, phase, excursion_m):
    return {"sim_s": str(sim_s), "phase": phase,
            "excursion_m": "" if excursion_m is None else str(excursion_m)}


def test_phase2_segments_splits_on_phase_boundaries():
    rows = ([_row(0, "1", None), _row(1, "1b", None)]
           + [_row(t, "2", 1.0) for t in range(2, 5)]
           + [_row(5, "1", None)]
           + [_row(t, "2", 2.0) for t in range(6, 9)])
    segments = ags.phase2_segments(rows)
    assert len(segments) == 2
    assert len(segments[0]) == 3
    assert len(segments[1]) == 3


def test_peak_excursion_ignores_blank_values():
    rows = [_row(0, "2", 1.0), _row(1, "2", None), _row(2, "2", 5.5),
            _row(3, "2", 3.0)]
    assert ags.peak_excursion(rows) == 5.5


def test_trend_slope_is_flat_for_a_settled_hold():
    rows = [_row(t, "2", 10.0) for t in range(0, 30)]
    slope = ags.trend_slope(rows, window_s=20.0)
    assert abs(slope) < ags.SLOPE_TOLERANCE
    assert ags.classify(slope) == "settling"


def test_trend_slope_is_positive_for_a_linear_growth():
    rows = [_row(t, "2", float(t)) for t in range(0, 30)]   # 1 m/s growth
    slope = ags.trend_slope(rows, window_s=20.0)
    assert slope == pytest.approx(1.0, abs=0.01)
    assert ags.classify(slope) == "growing"


def test_rank_candidates_orders_by_peak_excursion(tmp_path):
    csv_path = tmp_path / "run.csv"
    with open(csv_path, "w") as f:
        f.write("sim_s,phase,excursion_m\n")
        for t in range(0, 10):
            f.write(f"{t},2,20.0\n")             # candidate A: flat at 20 m
        for t in range(10, 20):
            f.write(f"{t},1,\n")                  # gap between candidates
        for t in range(20, 30):
            f.write(f"{t},2,5.0\n")              # candidate B: flat at 5 m

    sidecar_path = tmp_path / "campaign.json"
    sidecar = {"campaign": "test", "candidates": [
        {"name": "A", "gains": {}, "hold_s": 10, "status": "flown"},
        {"name": "B", "gains": {}, "hold_s": 10, "status": "flown"},
    ]}
    with open(sidecar_path, "w") as f:
        json.dump(sidecar, f)

    results = ags.rank_candidates(str(csv_path), str(sidecar_path))
    assert [r["name"] for r in results] == ["B", "A"]   # 5 m ranks before 20 m


def test_rank_candidates_includes_aborted_not_just_flown(tmp_path):
    """D5's whole point is that a bad candidate still teaches the sweep
    something -- an aborted candidate's real phase-2 data must not be
    silently dropped just because it did not finish its full hold."""
    csv_path = tmp_path / "run.csv"
    with open(csv_path, "w") as f:
        f.write("sim_s,phase,excursion_m\n")
        for t in range(0, 10):
            f.write(f"{t},2,50.0\n")

    sidecar_path = tmp_path / "campaign.json"
    sidecar = {"campaign": "test", "candidates": [
        {"name": "A", "gains": {}, "hold_s": 70, "status": "aborted"},
        {"name": "B", "gains": {}, "hold_s": 70, "status": "failed_to_climb"},
    ]}
    with open(sidecar_path, "w") as f:
        json.dump(sidecar, f)

    results = ags.rank_candidates(str(csv_path), str(sidecar_path))
    assert [r["name"] for r in results] == ["A"]
    assert results[0]["status"] == "aborted"


def test_a_bounded_wander_is_not_read_as_growth():
    """The 20 s default was chosen against a 40-60 s oscillation, where it is
    a fraction of a period. A hold that has actually settled does not sit
    still -- it wanders inside a bounded envelope -- and on that signal a 20 s
    window measures only which way the wander happened to be going when the
    clock stopped.

    Flown 2026-08-22 (`logs/20260822-gyrobias`): a 180 s hold whose envelope
    is flat at 1.1-2.0 m and whose whole-hold slope is +0.0029 m/s scored
    0.055 m/s, "growing", off its final 20 s alone.
    """
    period, amplitude = 25.0, 1.0
    rows = [_row(t / 10.0, "2", 1.5 + amplitude * math.sin(2 * math.pi * (t / 10.0) / period))
            for t in range(0, 1800)]

    assert ags.classify(ags.trend_slope(rows)) == "settling"


def test_a_real_ramp_is_still_read_as_growth():
    """The control: widening the window must not blind it to the runaway it
    exists to catch. `baseline` in Step 10.6 grew at 4.5 m/s."""
    rows = [_row(t / 10.0, "2", 0.1 * (t / 10.0)) for t in range(0, 1800)]

    assert ags.classify(ags.trend_slope(rows)) == "growing"
