"""Record (features, expert trajectory) pairs from closed-loop runs on random scenarios."""

import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]           # the AutonomousDrivingChallenge folder
sys.path.insert(0, str(ROOT))

from planners import make_planner                                    # noqa: E402
from planners.learned_example.features import NUM_FEATURES, encode   # noqa: E402
from planners.learned_example.model import NUM_OUTPUTS, target_from_trajectory   # noqa: E402
from scenarios.scenario_tools import (                               # noqa: E402
    INTERSECTION_LANE, LEFT_LANE, MPH, ONCOMING_LANE, RIGHT_LANE, STRAIGHT_THROUGH, TURN_LEFT, TURN_RIGHT, Scenario, car,
    crosswalk_road, curved_two_lane_road, intersection, intersection_signals, pedestrian, truck, two_lane_road,
)
from simulator.simulation import Simulation                          # noqa: E402


class Recorder:
    """Drives with the expert and records what it saw and what it planned."""

    def __init__(self, expert) -> None:
        self.expert, self.features, self.targets = expert, [], []

    def plan(self, obs):
        traj = self.expert.plan(obs)
        self.features.append(encode(obs))
        self.targets.append(target_from_trajectory(traj, obs))
        return traj


def _road_traffic(rng, limit):
    """Slow / braking vehicles ahead, traffic in the left lane, oncoming traffic (2+2 roads)."""
    actors = []
    s = 30.0 + rng.uniform(25.0, 60.0)
    for i in range(rng.integers(0, 4)):                               # slow vehicles ahead in our lane
        build = truck if rng.random() < 0.3 else car
        v = rng.uniform(2.0, limit)
        brake_at = rng.uniform(3.0, 20.0)
        actors.append(build(1 + i, s, RIGHT_LANE, v, speed_changes=[(brake_at, v), (brake_at + 3.0, rng.uniform(0.0, v))]))
        s += rng.uniform(30.0, 80.0)
    for i in range(rng.integers(0, 6)):                               # traffic in the left lane
        actors.append(car(20 + i, rng.uniform(-60.0, 200.0), LEFT_LANE, rng.uniform(6.0, 15.0)))
    for i in range(rng.integers(0, 3)):                               # oncoming traffic
        actors.append(car(40 + i, rng.uniform(100.0, 400.0), ONCOMING_LANE, rng.uniform(6.0, 14.0), oncoming=True))
    return [a for a in actors if all(abs(a.road_s_m - b.road_s_m) > 12.0 or a.road_d_m != b.road_d_m
                                     for b in actors if b is not a)]    # drop overlapping starts


def random_scenario(seed: int) -> Scenario:
    """A random scene built only from scenario_tools; the seed picks the road type and everything else.

    Every scene has a posted speed limit (typical US urban limits).  The expert drives AT the limit
    (desired_speed_mps = speed_limit_mps), so the network learns its cruise speed from the limit alone."""
    rng = np.random.default_rng(seed)
    kind = ("straight", "curve", "crosswalk", "junction")[seed % 4]
    if kind in ("straight", "curve"):
        limit = rng.choice([25, 30, 35, 40]) * MPH
        road, lane = (two_lane_road(), "two_lane_road:F2") if kind == "straight" else (curved_two_lane_road(), "curved_road:F2")
        return Scenario(f"random_{seed}", kind, road, (lane,), ego_start_s_m=30.0,
                        ego_speed_mps=rng.uniform(0.0, limit), desired_speed_mps=limit, speed_limit_mps=limit,
                        goal_distance_m=300.0, time_limit_s=50.0, actors=tuple(_road_traffic(rng, limit)))
    if kind == "crosswalk":
        limit = rng.choice([20, 25, 30]) * MPH
        actors = [a for a in _road_traffic(rng, limit) if a.road_s_m < 300.0]
        for i in range(rng.integers(1, 4)):                           # pedestrians at the crosswalk (s = 150 m)
            side = rng.choice([1.0, -1.0])
            v, start = rng.uniform(0.9, 1.8), rng.uniform(0.0, 15.0)
            actors.append(pedestrian(60 + i, 150.0 + rng.uniform(-1.0, 1.0), 8.95 * side,
                                     "left" if side > 0 else "right", 0.0, speed_changes=[(start, 0.0), (start + 0.5, v)]))
        return Scenario(f"random_{seed}", kind, crosswalk_road(), ("crosswalk_road:F2",), ego_start_s_m=rng.uniform(30.0, 80.0),
                        ego_speed_mps=rng.uniform(0.0, limit), desired_speed_mps=limit, speed_limit_mps=limit,
                        goal_distance_m=180.0, time_limit_s=50.0, actors=tuple(actors))
    control = rng.choice([None, "stop_signs", "signals"])
    route = (STRAIGHT_THROUGH, TURN_LEFT, TURN_RIGHT)[rng.integers(3)]
    limit = rng.choice([20, 25, 30]) * MPH
    actors, phases = [], {}
    if control == "signals":                                          # crossing cars ignore lights: none here
        green = rng.uniform(0.0, 20.0)
        phases = intersection_signals([(0.0, "RED"), (green, "GREEN")] if rng.random() < 0.6 else
                                      [(0.0, "GREEN"), (green, "YELLOW"), (green + 3.0, "RED"), (green + 15.0, "GREEN")])
    else:
        for i, arm in enumerate(rng.choice(["A1", "A3"], size=rng.integers(0, 4))):
            actors.append(car(1 + i, rng.uniform(0.0, 120.0) - 30.0 * i, INTERSECTION_LANE, rng.uniform(5.0, 8.0), road=str(arm)))
    if rng.random() < 0.5:
        actors.append(car(10, rng.uniform(0.0, 100.0), INTERSECTION_LANE, rng.uniform(5.0, 8.0), road="A2"))   # oncoming
    return Scenario(f"random_{seed}", kind, intersection(control), route, ego_start_s_m=rng.uniform(40.0, 90.0),
                    ego_speed_mps=rng.uniform(0.0, limit), desired_speed_mps=limit, speed_limit_mps=limit,
                    time_limit_s=50.0,
                    actors=tuple(actors), signal_phases=phases)


def run_episode(seed: int):
    recorder = Recorder(make_planner("baseline"))               # or your own expert
    result = Simulation(random_scenario(seed), recorder, "recorder", verbose_errors=False).run()
    if result.collision or result.road_departure:                 # never learn from an expert crash
        return np.zeros((0, NUM_FEATURES), np.float32), np.zeros((0, NUM_OUTPUTS), np.float32), result.score
    return np.array(recorder.features), np.array(recorder.targets), result.score


if __name__ == "__main__":
    seeds = range(int(sys.argv[1]) if len(sys.argv) > 1 else 400)
    with Pool() as pool:                                         # one simulation per CPU core
        episodes = pool.map(run_episode, seeds)
    X = np.concatenate([e[0] for e in episodes])
    Y = np.concatenate([e[1] for e in episodes])
    np.savez_compressed(ROOT / "training" / "data.npz", X=X, Y=Y)
    kept = sum(len(e[0]) > 0 for e in episodes)
    print(f"{len(X)} samples from {kept} of {len(episodes)} episodes (expert crashes dropped), "
          f"mean expert score {np.mean([e[2] for e in episodes]):.1f}")
