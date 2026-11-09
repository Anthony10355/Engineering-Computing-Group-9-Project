#!/usr/bin/env python3
"""Self-test of the whole pipeline (no window needed).

Run:   python tests/test_challenge.py        (or:  python -m pytest tests)
"""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np                                                     # noqa: E402

from planners import PLANNERS, make_planner                           # noqa: E402
from scenarios import all_scenarios, get_scenario                     # noqa: E402
from scenarios.example_scenarios import EXAMPLE_SCENARIOS             # noqa: E402
from scenarios.scenario_tools import (                                # noqa: E402
    RIGHT_LANE, STRAIGHT_THROUGH, Scenario, car, intersection, intersection_signals, two_lane_road,
)
from simulator.geometry import cartesian_to_frenet, ego_to_world, point_in_polygon   # noqa: E402
from simulator.observation import build_observation                   # noqa: E402
from simulator.simulation import Simulation                           # noqa: E402
from simulator.trajectory import Trajectory                           # noqa: E402
from simulator.world import World                                     # noqa: E402


# ---------------------------------------------------------------------------
# helper planners used only by the tests
# ---------------------------------------------------------------------------
class StraightAheadPlanner:
    """Drives straight ahead at a fixed speed and ignores everything (unsafe on purpose)."""

    def __init__(self, speed=10.0, curve_right_1pm=0.0):
        self.speed, self.k = speed, curve_right_1pm

    def plan(self, obs):
        t = np.arange(0.0, 4.01, 0.1)
        s = self.speed * t
        if self.k == 0.0:
            x, y = s, np.zeros_like(s)
        else:
            x, y = np.sin(self.k * s) / self.k, (1 - np.cos(self.k * s)) / self.k
        return Trajectory(t, x, y, np.full_like(t, self.speed))


class CrashingPlanner:
    def plan(self, obs):
        raise RuntimeError("deliberate test failure")


class InvalidPlanner:
    def plan(self, obs):
        return Trajectory([0.0, 0.1], [0.0, float("nan")], [0.0, 0.0], [5.0, 5.0])


def run(scenario, planner, name="test"):
    sim = Simulation(scenario, planner, name, verbose_errors=False)
    return sim, sim.run()


# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------
def test_baseline_solves_all_example_scenarios():
    for name in EXAMPLE_SCENARIOS:
        sim, r = run(get_scenario(name), make_planner("baseline"), "baseline")
        assert not r.collision, (name, r.end_reason)
        assert not r.road_departure, name
        assert r.completed, (name, r.end_reason, r.progress)
        assert r.planner_errors == 0, name
        assert r.score > 80.0, (name, r.score)
        moved = [a for a in sim.world.actors if a.speed_mps > 0.0 or a.speed_profile]
        assert moved, f"{name}: no actor moved"


def test_vehicles_move():
    sim = Simulation(get_scenario("intersection_crossing_traffic"), make_planner("baseline"), "baseline")
    start = {a.actor_id: (a.x_m, a.y_m) for a in sim.world.actors}
    ego0 = (sim.world.ego.x_m, sim.world.ego.y_m)
    for _ in range(40):
        sim.step()
    for a in sim.world.actors:
        assert math.hypot(a.x_m - start[a.actor_id][0], a.y_m - start[a.actor_id][1]) > 5.0
    assert math.hypot(sim.world.ego.x_m - ego0[0], sim.world.ego.y_m - ego0[1]) > 5.0


def test_pure_pursuit_tracks_the_plan():
    """Where the car actually is 0.5 s after planning vs. where the plan said."""
    for name in ("lead_vehicle_brakes", "lane_change_overtake", "intersection_crossing_traffic"):
        sim = Simulation(get_scenario(name), make_planner("baseline"), "baseline")
        errors = []
        pending = []
        while not sim.done:
            ego = sim.world.ego
            frame = sim.step()
            traj = frame.trajectory
            px, py = np.interp(0.5, traj.t_s, traj.x_m), np.interp(0.5, traj.t_s, traj.y_m)
            pending.append((sim.world.time_s + 0.45, *ego_to_world(ego.x_m, ego.y_m, ego.yaw_rad, px, py)))
            while pending and pending[0][0] <= sim.world.time_s + 1e-9:
                _, gx, gy = pending.pop(0)
                errors.append(math.hypot(sim.world.ego.x_m - gx, sim.world.ego.y_m - gy))
        errors = np.array(errors)
        assert np.percentile(errors, 95) < 0.6, (name, float(np.percentile(errors, 95)))


def test_collision_is_detected_and_scores_zero():
    sim, r = run(get_scenario("lead_vehicle_brakes"), StraightAheadPlanner(10.0))
    assert r.collision and r.collision_actor_id == 1, r
    assert r.score == 0.0 and r.end_reason.startswith("collision")


def test_road_departure_is_detected():
    sim, r = run(get_scenario("lead_vehicle_brakes"), StraightAheadPlanner(5.0, curve_right_1pm=0.05))
    assert r.road_departure and r.score == 0.0, r


def test_planner_crash_and_invalid_output_are_contained():
    for planner in (CrashingPlanner(), InvalidPlanner()):
        sim, r = run(get_scenario("lead_vehicle_brakes"), planner)
        assert r.planner_errors > 0
        assert r.end_reason != "running"                  # the simulation still terminates
        assert sim.world.ego.speed_mps < 0.1              # fallback braking stopped the car
        assert not r.collision


def test_red_light_then_green():
    sc = Scenario("red_then_green", "red light turns green at 14 s", intersection("signals"), STRAIGHT_THROUGH,
                  ego_start_s_m=60.0, ego_speed_mps=8.0, desired_speed_mps=8.0, time_limit_s=45.0,
                  signal_phases=intersection_signals([(0.0, "RED"), (14.0, "GREEN")], [(0.0, "GREEN"), (14.0, "RED")]))
    sim = Simulation(sc, make_planner("baseline"), "baseline")
    line_s = 133.0                                        # stop line along the route (front bumper)
    stopped_during_red = False
    while not sim.done:
        sim.step()
        s, _ = sim.world.ego_route_frenet()
        if sim.world.time_s < 14.0:
            assert s + 3.9 <= line_s + 0.3, f"front passed the stop line on red at t={sim.world.time_s:.1f}"
            stopped_during_red |= sim.world.ego.speed_mps < 0.05
    assert stopped_during_red and sim.result.completed, sim.result


def test_stop_sign():
    sc = Scenario("stop_sign", "all-way stop", intersection("stop_signs"), STRAIGHT_THROUGH,
                  ego_start_s_m=60.0, ego_speed_mps=8.0, desired_speed_mps=8.0, time_limit_s=45.0)
    sim = Simulation(sc, make_planner("baseline"), "baseline")
    stop_time = 0.0
    while not sim.done:
        sim.step()
        s, _ = sim.world.ego_route_frenet()
        if sim.world.ego.speed_mps < 0.05 and 120.0 < s < 133.0:
            stop_time += 0.05
    assert stop_time >= 2.0 and sim.result.completed, (stop_time, sim.result)


def test_speed_limit_is_observed_and_checked():
    sc = get_scenario("lead_vehicle_brakes")
    obs = build_observation(World(sc))
    assert math.isclose(obs.speed_limit_mps, 25 * 0.44704) and obs.desired_speed_mps <= obs.speed_limit_mps
    try:
        replace(sc, desired_speed_mps=sc.speed_limit_mps + 1.0)
    except ValueError:
        pass
    else:
        raise AssertionError("a desired speed above the speed limit must be rejected")


def test_switching_scenarios_and_planners():
    assert set(EXAMPLE_SCENARIOS) <= set(all_scenarios())
    for name in all_scenarios():
        assert get_scenario(name).name == name
    for name in PLANNERS:
        assert hasattr(make_planner(name), "plan")
    for bad in (lambda: get_scenario("nope"), lambda: make_planner("nope")):
        try:
            bad()
        except KeyError:
            pass
        else:
            raise AssertionError("unknown names must raise KeyError")
    sim, r = run(get_scenario("my_crosswalk"), make_planner("student"), "student")
    assert r.end_reason != "running" and r.planner_errors == 0


def test_runs_are_deterministic():
    a = run(get_scenario("lane_change_overtake"), make_planner("baseline"))[1]
    b = run(get_scenario("lane_change_overtake"), make_planner("baseline"))[1]
    assert a.as_dict() == b.as_dict()


def test_every_run_terminates_within_time_limit():
    for name in all_scenarios():
        for planner in PLANNERS:
            sim, r = run(get_scenario(name), make_planner(planner), planner)
            assert r.end_reason != "running"
            assert r.simulation_time_s <= sim.scenario.time_limit_s + 0.06


def test_package_is_independent_of_the_research_code():
    banned = ("longitudinal_control", "synthetic_trajectory_world", "trajectory_closed_loop_sim", "metadrive",
              "planner_interface", "local_planner", "safety_certificate")
    for path in ROOT.rglob("*.py"):
        if path.name == Path(__file__).name:
            continue
        text = path.read_text()
        for word in banned:
            assert word not in text, f"{path.relative_to(ROOT)} mentions '{word}'"


def test_renderer_draws_every_view():
    from simulator.renderer import VIEWS, Renderer
    sim = Simulation(get_scenario("intersection_crossing_traffic"), make_planner("baseline"), "baseline")
    frame = None
    for _ in range(30):
        frame = sim.step()
    for view in VIEWS:
        for debug in (False, True):
            image = Renderer(debug=debug, view=view).render(frame)
            assert image.ndim == 3 and image.shape[0] == 760 and image.std() > 5.0


def test_point_in_polygon():
    square = np.array([(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0)])
    x = np.array([1.0, 3.0, -0.5, 1.9, 1.0])
    y = np.array([1.0, 1.0, 1.0, 0.1, 2.5])
    assert point_in_polygon(x, y, square).tolist() == [True, False, False, True, False]
    many = np.random.default_rng(0).uniform(-1.0, 3.0, size=(50_000, 2))
    inside = point_in_polygon(many[:, 0], many[:, 1], square)
    assert np.array_equal(inside, (many > 0).all(axis=1) & (many < 2).all(axis=1))


def test_late_actor_appears_where_it_was_placed():
    sc = Scenario("late", "late car", two_lane_road(), ("two_lane_road:F2",),
                  actors=(car(1, 100.0, RIGHT_LANE, 10.0, appear_at_s=3.0),))
    world = World(sc)
    reference = world.network.roads["two_lane_road"].reference
    while world.time_s < 3.0 - 1e-9:
        assert not world.present_actors()
        world.advance(0.0, 0.0, 0.01)
    a = world.present_actors()[0]
    assert abs(cartesian_to_frenet(reference, a.x_m, a.y_m)[0] - 100.0) < 1e-6
    for _ in range(100):
        world.advance(0.0, 0.0, 0.01)
    assert abs(cartesian_to_frenet(reference, a.x_m, a.y_m)[0] - 110.0) < 0.05


def test_rule_violations_are_reported():
    red = Scenario("always_red", "red light", intersection("signals"), STRAIGHT_THROUGH, ego_start_s_m=60.0,
                   ego_speed_mps=8.0, desired_speed_mps=8.0, time_limit_s=30.0,
                   signal_phases=intersection_signals([(0.0, "RED")]))
    _, r = run(red, StraightAheadPlanner(8.0))
    assert r.red_light_violations == 1 and r.completed, r
    stop = Scenario("stop_runner", "stop sign", intersection("stop_signs"), STRAIGHT_THROUGH, ego_start_s_m=60.0,
                    ego_speed_mps=8.0, desired_speed_mps=8.0, time_limit_s=30.0)
    _, r = run(stop, StraightAheadPlanner(8.0))
    assert r.stop_sign_violations == 1 and r.red_light_violations == 0, r
    wrong = Scenario("wrong_side", "left of the centre line", two_lane_road(), ("two_lane_road:F2",),
                     ego_start_offset_m=-6.0, ego_speed_mps=10.0, goal_distance_m=50.0, time_limit_s=20.0)
    _, r = run(wrong, StraightAheadPlanner(10.0))
    assert r.completed and abs(r.wrong_side_time_s - r.simulation_time_s) < 0.02, r
    for name in EXAMPLE_SCENARIOS:                         # the baseline obeys the rules
        _, r = run(get_scenario(name), make_planner("baseline"), "baseline")
        assert r.violation_summary == "none", (name, r.notes)


def test_collision_from_behind_is_flagged():
    sc = Scenario("tailgater", "fast car from behind", two_lane_road(), ("two_lane_road:F2",), ego_speed_mps=0.0,
                  actors=(car(1, 10.0, RIGHT_LANE, 10.0),))
    _, r = run(sc, StraightAheadPlanner(0.0))
    assert r.collision and r.collision_from_behind, r
    _, r = run(get_scenario("lead_vehicle_brakes"), StraightAheadPlanner(10.0))
    assert r.collision and not r.collision_from_behind, r


def test_bad_display_fields_do_not_crash_the_renderer():
    from simulator.renderer import Renderer

    class OddLabels(StraightAheadPlanner):
        def plan(self, obs):
            traj = super().plan(obs)
            traj.behavior, traj.notes = 7, "one string, not a list"
            traj.alternatives = [None, "x", Trajectory([0.0], [0.0], [0.0], [0.0])]
            return traj

    sim = Simulation(get_scenario("lead_vehicle_brakes"), OddLabels(5.0), "odd", verbose_errors=False)
    frame = sim.step()
    assert frame.planner_error is None and sim.result.planner_errors == 0
    Renderer(debug=True, view="both").render(frame)


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  {name}")
        except Exception as exc:                          # noqa: BLE001
            failed += 1
            print(f"FAIL  {name}: {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - failed}/{len(tests)} tests passed")
    sys.exit(1 if failed else 0)
