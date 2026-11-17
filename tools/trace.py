"""Print what the planner saw and chose, cycle by cycle, for one scenario.

    venv\\Scripts\\python.exe tools\\trace.py rb_fast_cross_right --from 5 --to 10 --every 2
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from planners import make_planner                 # noqa: E402
from scenarios import get_scenario                # noqa: E402
from simulator.simulation import Simulation       # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument("--planner", default="student")
    ap.add_argument("--from", dest="t0", type=float, default=0.0)
    ap.add_argument("--to", dest="t1", type=float, default=1e9)
    ap.add_argument("--every", type=int, default=1, help="print every n-th cycle")
    ap.add_argument("--actors", action="store_true", help="also list the road users the planner sees")
    args = ap.parse_args()

    planner = make_planner(args.planner)
    plan = planner.plan
    k = [0]

    def traced(obs):
        traj = plan(obs)
        if args.t0 <= obs.time_s <= args.t1 and k[0] % args.every == 0:
            e = obs.ego
            lane = obs.lanes["current"]
            print(f"t={obs.time_s:6.2f} v={e.speed_mps:5.2f} a={e.acceleration_mps2:5.2f} "
                  f"lane={lane.lane_id} d={lane.ego_d_m:+.2f} goal={obs.distance_to_goal_m:6.1f} "
                  f"{getattr(traj, 'behavior', '')}  v_end={traj.speed_mps[-1]:.2f}")
            for n in getattr(traj, "notes", [])[:8]:
                print("      ", n)
            if args.actors:
                for a in obs.actors:
                    print(f"       #{a.actor_id:<4d} {a.kind:10s} x={a.x_m:7.1f} y={a.y_m:6.1f} "
                          f"vx={a.vx_mps:5.1f} vy={a.vy_mps:5.1f} v={a.speed_mps:5.1f}")
            if obs.stop_lines or obs.junctions or obs.crosswalks:
                print("       stop_lines", [(round(s.distance_m, 1), s.control, s.signal_state) for s in obs.stop_lines],
                      "junctions", [(round(j.entry_distance_m, 1), round(j.exit_distance_m, 1), j.movement)
                                    for j in obs.junctions], "crosswalks", obs.crosswalks)
        k[0] += 1
        return traj

    planner.plan = traced
    r = Simulation(get_scenario(args.scenario), planner, args.planner, True).run()
    print(r.end_reason, r.score, r.notes)


if __name__ == "__main__":
    main()
