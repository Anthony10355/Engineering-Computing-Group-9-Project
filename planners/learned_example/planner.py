"""Learned example planner.  Loads its weights ONCE in __init__; plan() only runs the network."""

from pathlib import Path

import numpy as np
import torch

from planners.planner_tools import SpeedLimits, build_trajectory, extend_trajectory, first_collision, speed_profile, time_grid
from planners.learned_example.features import FEATURE_VERSION, NUM_FEATURES, encode
from planners.learned_example.model import T_OUT, PlannerNet, trajectory_from_output

WEIGHTS = Path(__file__).resolve().parent / "weights" / "model.pt"   # relative to THIS file, not the cwd
BRAKE = SpeedLimits(max_decel_mps2=6.0, max_jerk_mps3=30.0)


class LearnedExamplePlanner:
    def __init__(self) -> None:
        torch.set_num_threads(1)                      # small nets run faster single-threaded
        ckpt = torch.load(WEIGHTS, map_location="cpu", weights_only=True)
        if int(ckpt["feature_version"]) != FEATURE_VERSION:
            raise RuntimeError("these weights were trained with a different features.py")
        self.model = PlannerNet(NUM_FEATURES)
        self.model.load_state_dict(ckpt["model"])
        self.model.eval()
        self.x_mean, self.x_std = ckpt["x_mean"], ckpt["x_std"]
        self.y_mean, self.y_std = ckpt["y_mean"], ckpt["y_std"]
        self.t = time_grid()

    def plan(self, obs):
        x = (torch.from_numpy(encode(obs)) - self.x_mean) / self.x_std
        with torch.inference_mode():
            out = self.model(x[None])[0] * self.y_std + self.y_mean
        out = out.numpy().astype(float)
        lane = obs.lanes["current"]

        # safety filter: brake in the lane instead if the network output is absurd (a situation it never
        # saw in training) or if the learned plan would hit a road user ahead
        traj = trajectory_from_output(out, obs) if self._plausible(out, obs) else None
        if traj is not None:
            t, px, py, h = extend_trajectory(lane, traj, 6.0)
            behind = {a.actor_id for a in obs.actors if a.x_m < 0.0}
        if traj is None or first_collision(t, px, py, h, obs.actors, behind) is not None:
            s, v, a = speed_profile(self.t, obs.ego.speed_mps, obs.ego.acceleration_mps2, 0.0, lane=lane,
                                    stop_s_m=0.0, limits=BRAKE)
            traj = build_trajectory(lane, self.t, s, np.full_like(s, lane.ego_d_m), v, a, behavior="SAFETY_BRAKE")
        return traj

    @staticmethod
    def _plausible(out, obs) -> bool:
        n = T_OUT.size
        d, v = out[:n], out[n:]
        return bool(np.all(np.isfinite(out)) and np.all(np.abs(d) < 6.0)
                    and np.all(v < 1.5 * obs.speed_limit_mps + 3.0))
