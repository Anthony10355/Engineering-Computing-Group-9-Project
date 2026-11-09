#!/usr/bin/env python3
"""Evaluate planners on scenarios without a window and print a score table.

Examples
--------
    python evaluate.py                                  # baseline on every scenario
    python evaluate.py --planner student
    python evaluate.py --planner baseline student       # compare planners
    python evaluate.py --scenario lane_change_overtake --planner student

Results are also written to results/results.json and results/results.csv.
Every run starts from a NEW planner object, so nothing carries over
between scenarios.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from planners import PLANNERS, make_planner          # noqa: E402
from scenarios import all_scenarios, get_scenario    # noqa: E402
from simulator.simulation import Simulation          # noqa: E402

COLUMNS = ("scenario", "planner", "score", "collision", "road_departure", "completed", "progress",
           "simulation_time_s", "completion_time_s", "planner_errors", "red_light_violations",
           "stop_sign_violations", "wrong_side_time_s", "collision_from_behind", "end_reason")


def evaluate(scenario_names, planner_names, verbose_errors: bool = False):
    rows = []
    for planner_name in planner_names:
        for name in scenario_names:
            sim = Simulation(get_scenario(name), make_planner(planner_name), planner_name, verbose_errors)
            result = sim.run()
            rows.append(result.as_dict())
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--planner", nargs="+", default=["baseline"], choices=list(PLANNERS))
    ap.add_argument("--scenario", nargs="+", default=None, help="default: every scenario")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent / "results")
    args = ap.parse_args()

    names = args.scenario or list(all_scenarios())
    rows = evaluate(names, args.planner, verbose_errors=True)

    pw = max(10, *(len(p) for p in args.planner))
    print(f"\n{'scenario':32s} {'planner':{pw}s} {'score':>6s}  {'collision':9s} {'off-road':8s} "
          f"{'goal':5s} {'progress':>8s} {'time':>6s}  {'errors':6s}  {'violations':10s}  end reason")
    for r in rows:
        violations = r["red_light_violations"] + r["stop_sign_violations"] + (r["wrong_side_time_s"] > 0.0)
        print(f"{r['scenario']:32s} {r['planner']:{pw}s} {r['score']:6.1f}  {str(r['collision']):9s} "
              f"{str(r['road_departure']):8s} {str(r['completed']):5s} {100 * r['progress']:7.0f}% "
              f"{r['simulation_time_s']:6.1f}  {r['planner_errors']:<6d}  {violations:<10d}  {r['end_reason']}")
        for note in r["notes"]:
            print(f"{'':{34 + pw}s}rule: {note}")
    for planner in args.planner:
        total = sum(r["score"] for r in rows if r["planner"] == planner)
        print(f"TOTAL {planner}: {total:.1f} / {100 * len(names)}")

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "results.json").write_text(json.dumps(rows, indent=2))
    with open(args.out / "results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nwrote {args.out / 'results.json'} and {args.out / 'results.csv'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
