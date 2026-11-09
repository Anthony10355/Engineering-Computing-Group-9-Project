#!/usr/bin/env python3
"""Run one scenario with one planner in the visual simulator.

Examples
--------
    python run_simulation.py                                   # default scenario, baseline planner
    python run_simulation.py --scenario lane_change_overtake
    python run_simulation.py --scenario intersection_crossing_traffic --planner student
    python run_simulation.py --list                            # list scenarios and planners
    python run_simulation.py --scenario lead_vehicle_brakes --headless   # no window, just the result
    python run_simulation.py --scenario lane_change_overtake --save-frames frames/   # save PNG images

Keys in the window: q / ESC quit, p pause, d debug overlays, v cycle views.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))      # works from any folder

from planners import PLANNERS, make_planner                   # noqa: E402
from scenarios import all_scenarios, get_scenario             # noqa: E402
from simulator.simulation import Simulation                   # noqa: E402

WINDOW = "Autonomous Driving Challenge"


def print_result(result) -> None:
    print(f"\n=== {result.scenario}  (planner: {result.planner}) ===")
    print(f"  end reason      : {result.end_reason}")
    print(f"  collision       : {result.collision}" + (f"  (actor #{result.collision_actor_id} at "
                                                         f"{result.collision_time_s:.1f} s)" if result.collision else ""))
    print(f"  road departure  : {result.road_departure}")
    print(f"  goal reached    : {result.completed}" + (f"  (at {result.completion_time_s:.1f} s)" if result.completed else ""))
    print(f"  progress        : {100 * result.progress:.0f} %")
    print(f"  simulation time : {result.simulation_time_s:.1f} s")
    print(f"  planner errors  : {result.planner_errors}")
    print(f"  rule violations : {result.violation_summary}" + (f"  ({'; '.join(result.notes)})" if result.notes else ""))
    print(f"  SCORE           : {result.score:.1f} / 100")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="lead_vehicle_brakes")
    ap.add_argument("--planner", default="baseline")
    ap.add_argument("--list", action="store_true", help="list scenarios and planners, then exit")
    ap.add_argument("--headless", action="store_true", help="no window (as fast as possible)")
    ap.add_argument("--fast", action="store_true", help="with a window, but do not slow down to real time")
    ap.add_argument("--view", default="local_bev", choices=["local_bev", "global_map", "both"])
    ap.add_argument("--debug", action="store_true", help="start with debug overlays on")
    ap.add_argument("--save-frames", type=Path, default=None, help="save every 10th frame as PNG here")
    args = ap.parse_args()

    if args.list:
        print("Scenarios:")
        for name, build in all_scenarios().items():
            print(f"  {name:32s} {build().description}")
        print("Planners:  " + ", ".join(PLANNERS))
        return 0

    scenario = get_scenario(args.scenario)
    sim = Simulation(scenario, make_planner(args.planner), args.planner)
    print(f"[{scenario.name}] {scenario.description}   (planner: {args.planner})")

    renderer = None
    if not args.headless or args.save_frames is not None:
        from simulator.renderer import VIEWS, Renderer         # OpenCV is only needed for drawing
        import cv2
        renderer = Renderer(debug=args.debug, view=args.view)
        if args.save_frames is not None:
            args.save_frames.mkdir(parents=True, exist_ok=True)

    paused = False
    frame_index = 0
    try:
        while not sim.done:
            loop_start = time.perf_counter()
            frame = sim.step()
            if renderer is not None:
                image = None
                if not args.headless or frame_index % 10 == 0:
                    image = renderer.render(frame)
                if args.save_frames is not None and frame_index % 10 == 0:
                    cv2.imwrite(str(args.save_frames / f"{scenario.name}_{frame.time_s:06.2f}s.png"), image)
                if not args.headless:
                    cv2.imshow(WINDOW, image)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        sim.result.end_reason = "stopped by user"
                        break
                    if key == ord("d"):
                        renderer.debug = not renderer.debug
                    if key == ord("v"):
                        renderer.view = VIEWS[(VIEWS.index(renderer.view) + 1) % len(VIEWS)]
                    if key == ord("p"):
                        paused = not paused
                    while paused:
                        key = cv2.waitKey(30) & 0xFF
                        if key in (27, ord("q")):
                            sim.result.end_reason = "stopped by user"
                        if key in (27, ord("q"), ord("p")):
                            paused = False
                    if sim.result.end_reason == "stopped by user":
                        break
                    if not args.fast:
                        remaining = sim.cycle_dt_s - (time.perf_counter() - loop_start)
                        if remaining > 0:
                            time.sleep(remaining)
            frame_index += 1
        if renderer is not None and not args.headless:
            image = renderer.render(frame)                  # final frame with the result
            cv2.imshow(WINDOW, image)
            print("Finished - press any key in the window to close it.")
            cv2.waitKey(0)
    finally:
        if renderer is not None and not args.headless:
            cv2.destroyAllWindows()
    print_result(sim.result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
