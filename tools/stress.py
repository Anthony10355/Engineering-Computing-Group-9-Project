"""Run many scenarios in parallel (headless) and print a compact table.

    venv\\Scripts\\python.exe tools\\stress.py                         # every scenario, student planner
    venv\\Scripts\\python.exe tools\\stress.py --prefix rb_ hz_        # only names starting with these
    venv\\Scripts\\python.exe tools\\stress.py --planner baseline student
    venv\\Scripts\\python.exe tools\\stress.py --save results/stress_student.json
    venv\\Scripts\\python.exe tools\\stress.py --planner baseline team9 --baseline-cache --valid-only

Also reports planner compute time per call (mean / max, ms) and rule violations.

Faster testing:
  --baseline-cache  the baseline's results never change unless the baseline,
                    the simulator or a scenario file changes, so they are saved
                    once (results/baseline_cache.json) and reused; only missing
                    scenes are run.  The cache is thrown away automatically when
                    any of those files changes.
  --valid-only      run only the scenes the baseline completes without a
                    collision (the same rule the leaderboard uses to keep a
                    scene).  Implies --baseline-cache.
"""
from __future__ import annotations

import argparse
import hashlib
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


CACHE = ROOT / "results" / "baseline_cache.json"


def _baseline_fingerprint() -> str:
    """Changes whenever anything that decides a baseline result changes."""
    h = hashlib.sha256()
    files = [ROOT / "planners" / "baseline_planner.py", ROOT / "planners" / "planner_tools.py",
             *sorted((ROOT / "simulator").glob("*.py")), *sorted((ROOT / "scenarios").glob("*.py"))]
    for p in files:
        h.update(p.name.encode())
        h.update(p.read_bytes())
    return h.hexdigest()[:16]


def _load_cache() -> dict:
    try:
        data = json.loads(CACHE.read_text())
    except (OSError, ValueError):
        return {}
    return data.get("runs", {}) if data.get("fingerprint") == _baseline_fingerprint() else {}


def _save_cache(runs: dict) -> None:
    CACHE.parent.mkdir(parents=True, exist_ok=True)
    CACHE.write_text(json.dumps({"fingerprint": _baseline_fingerprint(), "runs": runs}))


def _baseline_ok(r: dict) -> bool:
    return bool(r.get("completed") and not r.get("collision") and not r.get("road_departure"))


def _run_jobs(jobs, n_jobs):
    if not jobs:
        return []
    with mp.Pool(min(n_jobs, len(jobs))) as pool:
        return pool.map(_run, jobs, chunksize=1)


def main():
    from scenarios import all_scenarios
    ap = argparse.ArgumentParser()
    ap.add_argument("--planner", nargs="+", default=["student"])
    ap.add_argument("--prefix", nargs="*", default=None)
    ap.add_argument("--scenario", nargs="*", default=None)
    ap.add_argument("--jobs", type=int, default=max(1, mp.cpu_count() - 1))
    ap.add_argument("--save", type=Path, default=None)
    ap.add_argument("--baseline-cache", action="store_true", help="reuse saved baseline results")
    ap.add_argument("--valid-only", action="store_true", help="only scenes the baseline completes (implies cache)")
    args = ap.parse_args()

    names = list(all_scenarios())
    if args.scenario:
        names = args.scenario
    elif args.prefix:
        names = [n for n in names if any(n.startswith(p) for p in args.prefix)]

    use_cache = args.baseline_cache or args.valid_only
    base_rows = {}
    if use_cache:
        cache = _load_cache()
        missing = [n for n in names if n not in cache]
        if missing:
            print(f"baseline cache: running the baseline on {len(missing)} scene(s) not cached yet")
            cache.update({r["scenario"]: r for r in _run_jobs([(n, "baseline") for n in missing], args.jobs)})
            _save_cache(cache)
        base_rows = {n: cache[n] for n in names}
        if args.valid_only:
            dropped = [n for n in names if not _baseline_ok(base_rows[n])]
            names = [n for n in names if _baseline_ok(base_rows[n])]
            print(f"valid-only: {len(names)} scenes (skipped {len(dropped)} the baseline does not complete)")

    jobs = [(n, p) for n in names for p in args.planner if not (use_cache and p == "baseline")]
    by_key = {(r["scenario"], r["planner"]): r for r in _run_jobs(jobs, args.jobs)}
    by_key.update({(n, "baseline"): base_rows[n] for n in names if n in base_rows})
    rows = [by_key[(n, p)] for n in names for p in args.planner]

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
