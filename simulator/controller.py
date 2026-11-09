"""Trajectory follower: pure-pursuit steering + speed feedback.

Copied from the original research simulator (only the trajectory field name
changed: ``speed_mps`` instead of ``s_dot_mps``).  This is NOT the planner and
students should not need to change it.

Lateral (pure pursuit about the rear axle)
    The goal point lies on the trajectory polyline at path distance
    ``Ld = clip(L0 + k_v * v, Lmin, Lmax)`` from the vehicle.
    ``delta = atan(2 * L * sin(alpha) / Ld)``, alpha = goal bearing.
    Positive steering = right.

Longitudinal
    ``a = a_plan(t + t_lag) + kp * (v_plan(t) - v)``, clipped.  Sampling the
    planned acceleration one actuator time constant ahead cancels the
    actuator lag.  Standstill hold: when the plan has stopped but the car
    still rolls, brake the remaining speed out.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Tuple

import numpy as np

from simulator.trajectory import Trajectory


@dataclass(frozen=True)
class TrajectoryFollowerConfig:
    wheelbase_m: float = 3.0
    lookahead_base_m: float = 1.0
    lookahead_gain_s: float = 0.6
    lookahead_min_m: float = 3.0
    lookahead_max_m: float = 20.0
    max_steering_rad: float = 0.55
    speed_kp_per_s: float = 1.0
    acceleration_preview_s: float = 0.2      # = vehicle acceleration time constant
    max_acceleration_mps2: float = 2.0
    max_braking_mps2: float = 6.0


@dataclass(frozen=True)
class ControlCommand:
    steering_angle_rad: float
    acceleration_mps2: float
    reference_speed_mps: float
    lookahead_distance_m: float
    lookahead_point_xy_m: Tuple[float, float]   # trajectory (ego) frame
    compute_time_ms: float


class TrajectoryFollower:
    def __init__(self, config: TrajectoryFollowerConfig | None = None) -> None:
        self.config = config or TrajectoryFollowerConfig()

    def compute(self, trajectory: Trajectory, ego_speed_mps: float,
                elapsed_since_plan_s: float = 0.0,
                ego_pose_in_trajectory_frame: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> ControlCommand:
        started = time.perf_counter()
        cfg = self.config
        ex, ey, eh = ego_pose_in_trajectory_frame
        lookahead = min(cfg.lookahead_max_m,
                        max(cfg.lookahead_min_m, cfg.lookahead_base_m + cfg.lookahead_gain_s * ego_speed_mps))
        gx, gy = self._goal_point(trajectory, ex, ey, lookahead)

        # goal bearing in the vehicle frame (x forward, y right)
        dx, dy = gx - ex, gy - ey
        local_x = math.cos(eh) * dx + math.sin(eh) * dy
        local_y = -math.sin(eh) * dx + math.cos(eh) * dy
        distance = math.hypot(local_x, local_y)
        alpha = math.atan2(local_y, local_x)
        steering = 0.0 if distance < 1e-6 else math.atan2(2.0 * cfg.wheelbase_m * math.sin(alpha), distance)
        steering = max(-cfg.max_steering_rad, min(cfg.max_steering_rad, steering))

        v_ref = trajectory.speed_at(elapsed_since_plan_s)
        t_lead = elapsed_since_plan_s + cfg.acceleration_preview_s
        a_ff = float(np.interp(t_lead, trajectory.t_s, trajectory.acceleration_mps2))
        accel = a_ff + cfg.speed_kp_per_s * (v_ref - ego_speed_mps)
        if trajectory.speed_at(t_lead) <= 1e-6 and ego_speed_mps > 0.0:
            accel = min(accel, -ego_speed_mps / cfg.acceleration_preview_s)   # standstill hold
        accel = max(-cfg.max_braking_mps2, min(cfg.max_acceleration_mps2, accel))
        return ControlCommand(
            steering_angle_rad=float(steering), acceleration_mps2=float(accel),
            reference_speed_mps=float(v_ref), lookahead_distance_m=float(lookahead),
            lookahead_point_xy_m=(float(gx), float(gy)),
            compute_time_ms=(time.perf_counter() - started) * 1e3)

    @staticmethod
    def _goal_point(trajectory: Trajectory, ex: float, ey: float, lookahead_m: float) -> Tuple[float, float]:
        xy = np.column_stack((trajectory.x_m, trajectory.y_m))
        nearest = int(np.argmin(np.hypot(xy[:, 0] - ex, xy[:, 1] - ey)))
        pts = xy[nearest:]
        seg = np.hypot(*np.diff(pts, axis=0).T) if len(pts) > 1 else np.zeros(0)
        cumulative = np.concatenate(([0.0], np.cumsum(seg)))
        if cumulative[-1] >= lookahead_m:
            k = int(np.searchsorted(cumulative, lookahead_m))
            frac = (lookahead_m - cumulative[k - 1]) / max(seg[k - 1], 1e-12)
            p = pts[k - 1] + frac * (pts[k] - pts[k - 1])
            return float(p[0]), float(p[1])
        # trajectory shorter than the lookahead (slow / stopping): extend it
        # along its last heading so that steering stays defined
        remaining = lookahead_m - cumulative[-1]
        h = float(trajectory.heading_rad[-1])
        return float(pts[-1, 0] + remaining * math.cos(h)), float(pts[-1, 1] + remaining * math.sin(h))
