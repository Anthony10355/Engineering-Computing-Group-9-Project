"""Run many scenarios in parallel (headless) and print a compact table.

    venv\\Scripts\\python.exe tools\\stress.py                         # every scenario, student planner
    venv\\Scripts\\python.exe tools\\stress.py --prefix rb_ hz_        # only names starting with these
    venv\\Scripts\\python.exe tools\\stress.py --planner baseline student
    venv\\Scripts\\python.exe tools\\stress.py --save results/stress_student.json

Also reports planner compute time per call (mean / max, ms) and rule violations.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def _run(job):
    scenario_name, planner_name = job
    from planners import make_planner
    from scenarios import get_scenario
    from simulator.simulation import Simulation

    planner = make_planner(planner_name)
    times = []
    plan = planner.plan

    def timed(obs):
        t0 = time.perf_counter()
        try:
            return plan(obs)
        finally:
            times.append(time.perf_counter() - t0)

    planner.plan = timed
    r = Simulation(get_scenario(scenario_name), planner, planner_name, False).run().as_dict()
    r["plan_ms_mean"] = 1000 * sum(times) / max(len(times), 1)
    r["plan_ms_max"] = 1000 * max(times, default=0.0)
    return r


def main():
    from scenarios import all_scenarios
    ap = argparse.ArgumentParser()
    ap.add_argument("--planner", nargs="+", default=["student"])
    ap.add_argument("--prefix", nargs="*", default=None)
    ap.add_argument("--scenario", nargs="*", default=None)
    ap.add_argument("--jobs", type=int, default=max(1, mp.cpu_count() - 1))
    ap.add_argument("--save", type=Path, default=None)
    args = ap.parse_args()

    names = list(all_scenarios())
    if args.scenario:
        names = args.scenario
    elif args.prefix:
        names = [n for n in names if any(n.startswith(p) for p in args.prefix)]
    jobs = [(n, p) for n in names for p in args.planner]
    with mp.Pool(args.jobs) as pool:
        rows = pool.map(_run, jobs, chunksize=1)

    print(f"{'scenario':30s} {'planner':8s} {'score':>6s} {'time':>6s} {'prog':>5s} {'ms avg/max':>11s}  "
          f"{'viol':4s} end")
    for r in rows:
        viol = []
        if r["red_light_violations"]:
            viol.append("RED")
        if r["stop_sign_violations"]:
            viol.append("STOP")
        if r["wrong_side_time_s"] > 0:
            viol.append(f"WS{r['wrong_side_time_s']:.1f}")
        flag = "  <<<" if r["score"] < 80 or viol else ""
        print(f"{r['scenario']:30s} {r['planner']:8s} {r['score']:6.1f} {r['simulation_time_s']:6.1f} "
              f"{100 * r['progress']:4.0f}% {r['plan_ms_mean']:5.1f}/{r['plan_ms_max']:5.0f}  "
              f"{','.join(viol) or '-':4s} {r['end_reason']}{flag}")
    for p in args.planner:
        sc = [r["score"] for r in rows if r["planner"] == p]
        fails = sum(1 for s in sc if s < 80)
        print(f"TOTAL {p}: {sum(sc):.1f} / {100 * len(sc)}   mean {sum(sc) / max(len(sc), 1):.2f}   "
              f"below-80: {fails}")
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        args.save.write_text(json.dumps(rows, indent=1))


if __name__ == "__main__":
    main()
