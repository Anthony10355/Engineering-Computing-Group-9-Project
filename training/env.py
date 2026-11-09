"""Gymnasium environment around the simulator (for reinforcement learning)."""

import sys
from pathlib import Path

import gymnasium as gym
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from planners.planner_tools import build_trajectory, lateral_profile, speed_profile, time_grid   # noqa: E402
from planners.learned_example.features import NUM_FEATURES, encode                                       # noqa: E402
from simulator.observation import build_observation                                             # noqa: E402
from simulator.simulation import Simulation                                                     # noqa: E402


def action_to_trajectory(obs, action, t=time_grid()):
    """action in [-1, 1]^2: [target speed (-1 = stop, +1 = the speed limit), lateral offset in lanes (+ = right)]."""
    lane = obs.lanes["current"]
    v_target = 0.5 * (float(np.clip(action[0], -1.0, 1.0)) + 1.0) * obs.speed_limit_mps
    s, v, a = speed_profile(t, obs.ego.speed_mps, obs.ego.acceleration_mps2, v_target, lane=lane)
    d = lateral_profile(lane.ego_d_m, 0.0, float(np.clip(action[1], -1.0, 1.0)) * lane.width_m,
                        max(20.0, 3.0 * obs.ego.speed_mps), s)
    return build_trajectory(lane, t, s, d, v, a, behavior="RL")


class _Puppet:
    """The 'planner' the simulation calls: it returns whatever the agent chose."""
    trajectory = None

    def plan(self, obs):
        return self.trajectory


class DrivingEnv(gym.Env):
    def __init__(self, make_scenario):
        self.make_scenario = make_scenario                       # seed -> Scenario
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (NUM_FEATURES,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.puppet = _Puppet()
        self.sim = Simulation(self.make_scenario(int(self.np_random.integers(1 << 30))), self.puppet, "rl", False)
        self.obs = build_observation(self.sim.world)
        return encode(self.obs), {}

    def step(self, action):
        r = self.sim.result
        before = (r.progress, r.red_light_violations + r.stop_sign_violations, r.wrong_side_time_s)
        self.puppet.trajectory = action_to_trajectory(self.obs, action)
        self.sim.step()                                           # 50 ms: controller, physics, evaluation
        r = self.sim.result
        reward = 100.0 * (r.progress - before[0])                 # progress along the route
        reward -= 50.0 * (r.red_light_violations + r.stop_sign_violations - before[1])
        reward -= 5.0 * (r.wrong_side_time_s - before[2])
        if r.collision or r.road_departure:
            reward -= 100.0
        self.obs = build_observation(self.sim.world)
        terminated = r.collision or r.road_departure or r.completed
        truncated = self.sim.done and not terminated              # time limit
        return encode(self.obs), reward, terminated, truncated, {"result": r}
