"""Network definition + the output <-> Trajectory conversion.  Used by training AND by the planner.

The network predicts, at the times T_OUT, the car's sideways offset d from the
CURRENT LANE's centre line (+ = right) and its speed.  Lane-relative offsets
make the car return to the lane centre after it drifts, and follow curves and
junction turns (on the route the lane centre line is the route itself)."""

import numpy as np
from torch import nn

from planners.planner_tools import build_trajectory, time_grid
from simulator.geometry import cartesian_to_frenet_many
from simulator.trajectory import Trajectory

T_OUT = np.arange(0.5, 4.0 + 1e-9, 0.5)            # predicted times (s): 0.5, 1.0, ... 4.0
NUM_OUTPUTS = 2 * T_OUT.size                       # lane offset d and speed at each time
T_PLAN = time_grid(4.0, 0.1)                       # times of the trajectory handed to the simulator


class PlannerNet(nn.Module):
    def __init__(self, num_features: int, hidden: int = 256) -> None:
        super().__init__()
        self.net = nn.Sequential(nn.Linear(num_features, hidden), nn.ReLU(), nn.Linear(hidden, hidden), nn.ReLU(),
                                 nn.Linear(hidden, NUM_OUTPUTS))

    def forward(self, x):
        return self.net(x)


def target_from_trajectory(traj: Trajectory, obs) -> np.ndarray:
    """Label for training: the expert trajectory as (d, speed) on the current lane at T_OUT."""
    lane = obs.lanes["current"]
    _, d = cartesian_to_frenet_many(lane.centerline, traj.x_m, traj.y_m)
    return np.concatenate([np.interp(T_OUT, traj.t_s, d), np.interp(T_OUT, traj.t_s, traj.speed_mps)]).astype(np.float32)


def trajectory_from_output(out: np.ndarray, obs) -> Trajectory:
    """Network output -> a Trajectory along the current lane that passes the simulator's checks.
    The distance driven comes from the predicted speeds, the sideways offset from the predicted d."""
    lane = obs.lanes["current"]
    n = T_OUT.size
    t = np.concatenate(([0.0], T_OUT))
    # never plan into a lane that does not exist or across a line that must not be crossed
    left, right = obs.lanes.get("left"), obs.lanes.get("right")
    d_min = -lane.width_m if left is not None and left.change_allowed else -0.3
    d_max = lane.width_m if right is not None and right.change_allowed else 0.3
    d = np.concatenate(([lane.ego_d_m], np.clip(out[:n], d_min, d_max)))
    v = np.concatenate(([max(obs.ego.speed_mps, 0.0)], np.clip(out[n:], 0.0, obs.speed_limit_mps)))   # never above the limit
    s = np.concatenate(([0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))))     # distance driven
    return build_trajectory(lane, T_PLAN, np.interp(T_PLAN, t, s), np.interp(T_PLAN, t, d),
                            np.interp(T_PLAN, t, v), behavior="LEARNED")
