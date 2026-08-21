#!/usr/bin/env python3
"""Reads a gain-sweep campaign's run.csv + campaign_<name>.json sidecar and
prints a ranked table: peak excursion_m and final-20s trend slope per
candidate, in flight order (D2, D3). See
docs/superpowers/specs/2026-08-14-mpc-xy-gain-sweep-design.md.
"""
import argparse
import csv
import json

SLOPE_TOLERANCE = 0.01
"""m/s. Below this, a fit slope is noise, not a real trend (D3)."""


def load_run_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def phase2_segments(rows):
    """Every contiguous run of phase=="2" rows, in flight order -- the k-th
    segment belongs to the k-th candidate flown (the campaign sidecar
    preserves that order)."""
    segments = []
    current = []
    for row in rows:
        if row["phase"] == "2":
            current.append(row)
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    return segments


def peak_excursion(segment):
    values = [float(r["excursion_m"]) for r in segment if r["excursion_m"]]
    return max(values) if values else None


TREND_WINDOW_S = 120.0
"""Sim seconds of hold the trend is fitted over, counting back from the end.

Was 20 s, chosen against run 5's 40-60 s oscillation, where it is a fraction
of a period. That is the wrong window for a hold that has actually settled: a
settled hold does not sit still, it wanders inside a bounded envelope, and
over 20 s of that the fit measures only which way the wander happened to be
going when the clock stopped. `logs/20260822-gyrobias` -- 180 s, envelope flat
at 1.1-2.0 m, whole-hold slope +0.0029 m/s -- scored 0.055 m/s and "growing"
off its final 20 s alone.

120 s is several periods of anything this project has seen wander, and still
leaves a real ramp unmistakable: Step 10.6's `baseline` grew at 4.5 m/s, three
orders of magnitude above SLOPE_TOLERANCE.
"""


def trend_slope(segment, window_s=TREND_WINDOW_S):
    """Least-squares slope of excursion_m over the final window_s sim
    seconds of the hold. Positive beyond SLOPE_TOLERANCE means growing (D3);
    this returns the number, classify() applies the label."""
    timed = [(float(r["sim_s"]), float(r["excursion_m"]))
            for r in segment if r["sim_s"] and r["excursion_m"]]
    if len(timed) < 2:
        return None
    end_t = timed[-1][0]
    window = [(t, x) for t, x in timed if t >= end_t - window_s]
    if len(window) < 2:
        window = timed
    n = len(window)
    mean_t = sum(t for t, _ in window) / n
    mean_x = sum(x for _, x in window) / n
    num = sum((t - mean_t) * (x - mean_x) for t, x in window)
    den = sum((t - mean_t) ** 2 for t, _ in window)
    return num / den if den else 0.0


def classify(slope):
    if slope is None:
        return "unknown"
    return "growing" if slope > SLOPE_TOLERANCE else "settling"


def rank_candidates(run_csv_path, sidecar_path):
    rows = load_run_csv(run_csv_path)
    with open(sidecar_path) as f:
        sidecar = json.load(f)
    segments = phase2_segments(rows)
    # "aborted" candidates reached a real GPS-denied hold and produced a real
    # phase-2 segment too -- D5 exists precisely so a bad candidate still
    # teaches the sweep something, and an early abort is often the MOST
    # informative point in the data, not something to discard.
    with_data = [c for c in sidecar["candidates"]
                if c["status"] in ("flown", "aborted")]
    results = []
    for candidate, segment in zip(with_data, segments):
        slope = trend_slope(segment)
        results.append({
            "name": candidate["name"],
            "gains": candidate["gains"],
            "status": candidate["status"],
            "peak_excursion_m": peak_excursion(segment),
            "trend_slope_mps": slope,
            "trend": classify(slope),
        })
    results.sort(key=lambda r: (r["peak_excursion_m"] is None,
                                r["peak_excursion_m"] or 0.0))
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_csv")
    ap.add_argument("sidecar_json")
    args = ap.parse_args()
    results = rank_candidates(args.run_csv, args.sidecar_json)
    print(f"{'candidate':<15} {'status':>10} {'peak_m':>8} "
          f"{'slope_m/s':>10} {'trend':>10}")
    for r in results:
        print(f"{r['name']:<15} {r['status']:>10} {r['peak_excursion_m']:>8.1f} "
              f"{r['trend_slope_mps']:>10.3f} {r['trend']:>10}")


if __name__ == "__main__":
    main()
