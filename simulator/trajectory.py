"""The trajectory a planner returns (the planner -> controller contract).

A trajectory is a list of timed points in the EGO frame of the planning
cycle that produced it:

    x = forward [m], y = RIGHT [m], ego rear axle at (0, 0) facing +x.

Required: ``t_s`` (starting at 0, strictly increasing), ``x_m``, ``y_m`` and
``speed_mps`` (the speed the car should have at each point).  Heading and
acceleration are filled in automatically if you do not give them.

``behavior`` and ``notes`` are only shown on screen.  They never affect the
simulation or the score.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import List, Optional

import numpy as np


@dataclass(eq=False)
class Trajectory:
    t_s: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray
    speed_mps: np.ndarray
    heading_rad: Optional[np.ndarray] = None
    #: acceleration applied from point k to point k+1 (computed if omitted)
    acceleration_mps2: Optional[np.ndarray] = None
    #: display only: short behaviour label (e.g. "FOLLOW_LEAD") and text lines
    behavior: str = "PLAN"
    notes: List[str] = field(default_factory=list)
    #: display only: other trajectories you considered (drawn in debug view)
    alternatives: List["Trajectory"] = field(default_factory=list)
    #: display only: True if this alternative was rejected
    rejected: bool = False

    def __post_init__(self) -> None:
        self.t_s = np.asarray(self.t_s, dtype=float).ravel()
        self.x_m = np.asarray(self.x_m, dtype=float).ravel()
        self.y_m = np.asarray(self.y_m, dtype=float).ravel()
        self.speed_mps = np.asarray(self.speed_mps, dtype=float).ravel()
        n = self.t_s.size
        if self.heading_rad is None and n >= 2:
            dx, dy = np.diff(self.x_m), np.diff(self.y_m)
            seg = np.arctan2(dy, dx)
            moving = np.hypot(dx, dy) > 1e-4
            heading = np.zeros(n)
            last = 0.0
            for k in range(n - 1):                  # keep the last heading while standing still
                last = float(seg[k]) if moving[k] else last
                heading[k] = last
            heading[-1] = last
            self.heading_rad = heading
        elif self.heading_rad is not None:
            self.heading_rad = np.asarray(self.heading_rad, dtype=float).ravel()
        if self.acceleration_mps2 is None and n >= 2:
            dt = np.maximum(np.diff(self.t_s), 1e-6)
            acc = np.diff(self.speed_mps) / dt
            self.acceleration_mps2 = np.append(acc, acc[-1])
        elif self.acceleration_mps2 is not None:
            self.acceleration_mps2 = np.asarray(self.acceleration_mps2, dtype=float).ravel()

    # --- convenience -----------------------------------------------------
    @property
    def num_points(self) -> int:
        return int(self.t_s.size)

    @property
    def terminal_speed_mps(self) -> float:
        return float(self.speed_mps[-1])

    def speed_at(self, t_s: float) -> float:
        return float(np.interp(t_s, self.t_s, self.speed_mps))


def check_trajectory(traj) -> Optional[str]:
    """Return None if the trajectory can be executed, else the reason it cannot.

    The SIMULATOR runs this check (not the planner).  A trajectory that fails
    is not executed: the car brakes instead and a planner error is counted.
    """
    if not isinstance(traj, Trajectory):
        return f"planner returned {type(traj).__name__}, not a Trajectory"
    n = traj.num_points
    if n < 2:
        return "a trajectory needs at least 2 points"
    arrays = (traj.t_s, traj.x_m, traj.y_m, traj.speed_mps, traj.heading_rad, traj.acceleration_mps2)
    if any(a is None or a.size != n for a in arrays):
        return "all trajectory arrays must have the same length"
    if not all(np.all(np.isfinite(a)) for a in arrays):
        return "trajectory contains NaN or infinite values"
    if abs(traj.t_s[0]) > 0.05:
        return "t_s must start at 0"
    if not np.all(np.diff(traj.t_s) > 0.0):
        return "t_s must be strictly increasing"
    if traj.t_s[-1] < 1.0:
        return "the trajectory must cover at least 1 s"
    if np.any(traj.speed_mps < -1e-6) or np.any(traj.speed_mps > 40.0):
        return "speeds must be between 0 and 40 m/s"
    if math.hypot(traj.x_m[0], traj.y_m[0]) > 3.0:
        return "the trajectory must start at the car (within 3 m of (0, 0))"
    return None
