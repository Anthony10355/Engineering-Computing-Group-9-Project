"""Helper functions for writing planners (free to use in your own planner).

Everything is in the EGO frame: x = forward, y = RIGHT, ego rear axle at
(0, 0) facing +x.  Positions along a lane use Frenet coordinates on the
lane's centre line: s = distance along the lane, d = sideways offset
(+ = right).

Main helpers
------------
time_grid()                    -> times 0, 0.1, ... 4.0 s
actor_position_on_lane(...)    -> where an actor is on a lane (s, d)
find_lead(...)                 -> the vehicle ahead of us in a lane
speed_profile(...)             -> how far we drive and how fast, over time
lateral_profile(...)           -> smooth sideways motion (quintic polynomial)
build_trajectory(...)          -> turn (s, d, speed) over time into a Trajectory
predict_actor(...)             -> constant-velocity prediction of a road user
first_collision(...)           -> does a trajectory hit any predicted actor?
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from simulator.geometry import (
    boxes_overlap_many, cartesian_to_frenet, frenet_heading_to_cartesian, frenet_to_cartesian, reference_state,
)
from simulator.trajectory import Trajectory

# ---------------------------------------------------------------------------
# Constants (ego vehicle and default limits)
# ---------------------------------------------------------------------------
EGO_LENGTH_M = 4.66
EGO_WIDTH_M = 1.89
EGO_REAR_AXLE_TO_FRONT_M = 3.90
EGO_CENTER_OFFSET_M = 0.5 * (3.90 - (4.66 - 3.90))   # rear axle -> footprint centre

DT_S = 0.1
HORIZON_S = 4.0


def time_grid(horizon_s: float = HORIZON_S, dt_s: float = DT_S) -> np.ndarray:
    return np.arange(0.0, horizon_s + 1e-9, dt_s)


# ---------------------------------------------------------------------------
# Lanes and actors
# ---------------------------------------------------------------------------
def actor_position_on_lane(lane, actor) -> Tuple[float, float]:
    """(s, d) of an actor's centre on ``lane.centerline``."""
    return cartesian_to_frenet(lane.centerline, actor.x_m, actor.y_m)


def speed_along_lane(lane, actor, s_m: float) -> float:
    """Component of the actor's velocity along the lane direction."""
    heading = float(reference_state(lane.centerline, s_m)[2])
    return actor.vx_mps * math.cos(heading) + actor.vy_mps * math.sin(heading)


def is_in_lane(lane, actor, s_m: float, d_m: float, extra_m: float = 0.3) -> bool:
    """True if the actor's footprint overlaps the lane (sideways)."""
    return abs(d_m) < 0.5 * lane.width_m + 0.5 * actor.width_m + extra_m - 0.5


@dataclass(frozen=True)
class Lead:
    actor_id: int
    gap_m: float          # from our front bumper to the lead's rear
    speed_mps: float      # lead speed along the lane
    rear_s_m: float       # lead rear position on the lane (s)


def find_lead(lane, actors) -> Optional[Lead]:
    """Closest road user ahead of the ego in ``lane`` (or None)."""
    best = None
    ego_front = lane.ego_s_m + EGO_REAR_AXLE_TO_FRONT_M
    for a in actors:
        s, d = actor_position_on_lane(lane, a)
        if s <= lane.ego_s_m or not is_in_lane(lane, a, s, d):
            continue
        rear = s - 0.5 * a.length_m
        gap = rear - ego_front
        if best is None or gap < best.gap_m:
            best = Lead(a.actor_id, gap, speed_along_lane(lane, a, s), rear)
    return best


# ---------------------------------------------------------------------------
# Longitudinal motion (adapted from the original Planner 0)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SpeedLimits:
    max_accel_mps2: float = 1.5
    comfort_decel_mps2: float = 2.0
    max_decel_mps2: float = 3.0
    max_jerk_mps3: float = 4.0
    max_lateral_accel_mps2: float = 1.8     # limits speed in curves
    speed_time_constant_s: float = 0.5
    idm_standstill_gap_m: float = 2.5
    idm_time_gap_s: float = 1.5


def curve_speed_limit(lane, s_from: float, desired_speed_mps: float, limits: SpeedLimits,
                      step_m: float = 0.5, reach_m: float = 120.0):
    """Speed envelope along the lane: never faster than the curve allows,
    and slowing down early enough (comfort deceleration) before curves."""
    grid = s_from + step_m * np.arange(int(reach_m / step_m) + 1)
    kappa = np.abs(reference_state(lane.centerline, grid)[3])
    v_lim2 = np.minimum(desired_speed_mps ** 2, limits.max_lateral_accel_mps2 / np.maximum(kappa, 1e-9))
    rel = grid - grid[0]
    a_c = limits.comfort_decel_mps2
    v_env2 = np.minimum.accumulate((v_lim2 + 2.0 * a_c * rel)[::-1])[::-1] - 2.0 * a_c * rel
    return grid, np.sqrt(np.maximum(v_env2, 0.0))


def speed_profile(t: np.ndarray, v0: float, a0: float, desired_speed_mps: float, lane=None,
                  stop_s_m: Optional[float] = None, lead: Optional[Lead] = None,
                  limits: SpeedLimits = SpeedLimits()):
    """Integrate the longitudinal motion over the time grid ``t``.

    * cruise toward ``desired_speed_mps`` (time constant 0.5 s), slower in curves;
    * ``stop_s_m``: lane position (rear-axle s, measured from the ego) where
      the car must be stopped -- uses the stop law a = -v^2 / (2 r);
    * ``lead``: follow it with the Intelligent Driver Model (IDM), assuming it
      keeps its current speed;
    * acceleration and jerk limits, never reversing.
    Returns (distance travelled s_rel, speed, acceleration) arrays.
    """
    n = t.size
    dt = float(t[1] - t[0])
    grid = v_env = None
    if lane is not None:
        grid, v_env = curve_speed_limit(lane, lane.ego_s_m, desired_speed_mps, limits)
    s_rel, v_out, a_out = np.zeros(n), np.zeros(n), np.zeros(n)
    s, v = 0.0, max(float(v0), 0.0)
    a_prev = max(-limits.max_decel_mps2, min(limits.max_accel_mps2, float(a0)))
    two_sqrt_ab = 2.0 * math.sqrt(limits.max_accel_mps2 * limits.comfort_decel_mps2)
    for k in range(n):
        v_target = desired_speed_mps
        if grid is not None:
            v_target = min(v_target, float(np.interp(lane.ego_s_m + s + 0.5 * v, grid, v_env)))
        a_des = (v_target - v) / limits.speed_time_constant_s
        if stop_s_m is not None:
            r = stop_s_m - s
            if r <= 0.15:
                a_des = -v / dt
            else:
                a_req = v * v / (2.0 * r)
                if a_req >= limits.comfort_decel_mps2:
                    a_des = min(a_des, -a_req)
                a_des = min(a_des, (math.sqrt(2.0 * limits.comfort_decel_mps2 * r) - v) / limits.speed_time_constant_s)
        if lead is not None:
            lead_rear = lead.rear_s_m - lane.ego_s_m + lead.speed_mps * t[k] if lane is not None else \
                lead.gap_m + EGO_REAR_AXLE_TO_FRONT_M + lead.speed_mps * t[k]
            gap = lead_rear - (s + EGO_REAR_AXLE_TO_FRONT_M)
            s_star = max(limits.idm_standstill_gap_m,
                         limits.idm_standstill_gap_m + v * limits.idm_time_gap_s
                         + v * (v - lead.speed_mps) / two_sqrt_ab)
            a_idm = limits.max_accel_mps2 * (1.0 - (s_star / max(gap, 0.1)) ** 2)
            a_des = min(a_des, a_idm)
        a_k = max(-limits.max_decel_mps2, min(limits.max_accel_mps2, a_des))
        jerk_step = limits.max_jerk_mps3 * dt
        a_k = max(a_prev - jerk_step, min(a_prev + jerk_step, a_k))
        if v + a_k * dt < 0.0:
            a_k = -v / dt
        if stop_s_m is not None:                       # never roll past the stop point
            remaining = stop_s_m + 0.2 - s
            if v * dt + 0.5 * a_k * dt * dt > remaining:
                a_k = max(-v / dt, min(a_k, 2.0 * (remaining - v * dt) / (dt * dt)))
        s_rel[k], v_out[k], a_out[k] = s, v, a_k
        a_prev = a_k
        s = s + v * dt + 0.5 * a_k * dt * dt
        v = max(0.0, v + a_k * dt)
    return s_rel, v_out, a_out


# ---------------------------------------------------------------------------
# Lateral motion
# ---------------------------------------------------------------------------
def lateral_profile(d0: float, slope0: float, d_target: float, length_m: float, u: np.ndarray) -> np.ndarray:
    """Quintic d(u) from offset d0 (with slope dd/ds = slope0, zero curvature)
    to ``d_target`` over ``length_m`` metres of travel; constant afterwards."""
    L = max(length_m, 1.0)
    D = d_target - (d0 + slope0 * L)
    V = -slope0
    a3 = (20.0 * D - 8.0 * V * L) / (2.0 * L ** 3)
    a4 = (-30.0 * D + 14.0 * V * L) / (2.0 * L ** 4)
    a5 = (12.0 * D - 6.0 * V * L) / (2.0 * L ** 5)
    uc = np.clip(u, 0.0, L)
    return d0 + uc * (slope0 + uc * uc * (a3 + uc * (a4 + uc * a5)))


def build_trajectory(lane, t: np.ndarray, s_rel: np.ndarray, d: np.ndarray, speed: np.ndarray,
                     accel: Optional[np.ndarray] = None, behavior: str = "PLAN", notes=None) -> Trajectory:
    """Convert motion along a lane into an ego-frame Trajectory."""
    s = lane.ego_s_m + s_rel
    x, y = frenet_to_cartesian(lane.centerline, s, d)
    ds, dd = np.gradient(s), np.gradient(d)
    moving = ds > 1e-6
    dd_ds = np.zeros_like(d)
    dd_ds[moving] = dd[moving] / ds[moving]        # slope of the sideways motion (0 while standing still)
    heading = frenet_heading_to_cartesian(lane.centerline, s, d, dd_ds)
    return Trajectory(t, np.asarray(x, float), np.asarray(y, float), speed, heading_rad=np.asarray(heading, float),
                      acceleration_mps2=accel, behavior=behavior, notes=list(notes or []))


# ---------------------------------------------------------------------------
# Prediction and collision checking
# ---------------------------------------------------------------------------
SAFETY_MARGIN_M = {"pedestrian": 0.6, "bicycle": 0.5, "car": 0.3, "truck": 0.3}


def predict_actor(actor, t: np.ndarray):
    """Constant-velocity prediction: (x(t), y(t)) of the actor centre."""
    return actor.x_m + actor.vx_mps * t, actor.y_m + actor.vy_mps * t


def extend_trajectory(lane, traj: Trajectory, until_s: float):
    """Continue a trajectory at its final speed along the lane (prediction only)."""
    dt = float(traj.t_s[1] - traj.t_s[0])
    t_ext = np.arange(traj.t_s[-1] + dt, until_s + 1e-9, dt)
    if t_ext.size == 0:
        return traj.t_s, traj.x_m, traj.y_m, traj.heading_rad
    s_end, d_end = cartesian_to_frenet(lane.centerline, float(traj.x_m[-1]), float(traj.y_m[-1]))
    s_ext = s_end + traj.speed_mps[-1] * (t_ext - traj.t_s[-1])
    x, y = frenet_to_cartesian(lane.centerline, s_ext, np.full_like(s_ext, d_end))
    h = reference_state(lane.centerline, s_ext)[2]
    return (np.concatenate((traj.t_s, t_ext)), np.concatenate((traj.x_m, x)),
            np.concatenate((traj.y_m, y)), np.concatenate((traj.heading_rad, h)))


def first_collision(t, x, y, heading, actors, ignore_ids=(), longitudinal_margin_m: float = 0.5,
                    crossing_time_buffer_s: float = 0.0):
    """First (time, actor_id, ego_x, ego_y) at which the ego footprint along
    (t, x, y, heading) overlaps a constant-velocity predicted actor, or None.

    ``crossing_time_buffer_s``: for road users whose direction CROSSES ours
    (more than 30 degrees off), stretch their footprint along their motion by
    speed x buffer at both ends, i.e. keep that much TIME between us and them."""
    t, x, y, heading = (np.asarray(v, float) for v in (t, x, y, heading))
    cx = x + EGO_CENTER_OFFSET_M * np.cos(heading)
    cy = y + EGO_CENTER_OFFSET_M * np.sin(heading)
    best = None
    for a in actors:
        if a.actor_id in ignore_ids:
            continue
        ax, ay = predict_actor(a, t)
        length = a.length_m
        relative = abs((a.heading_rad + math.pi) % (2.0 * math.pi) - math.pi)
        if crossing_time_buffer_s > 0.0 and math.radians(30.0) < relative < math.radians(150.0):
            length += 2.0 * a.speed_mps * crossing_time_buffer_s
        near = np.hypot(cx - ax, cy - ay) < 0.5 * (EGO_LENGTH_M + length) + 4.0
        if not near.any():
            continue
        margin = SAFETY_MARGIN_M.get(a.kind, 0.3)
        hit = boxes_overlap_many(cx, cy, heading, EGO_LENGTH_M + 2 * longitudinal_margin_m, EGO_WIDTH_M + 2 * margin,
                                 ax, ay, np.full_like(t, a.heading_rad), length, a.width_m)
        idx = np.flatnonzero(hit & near)
        if idx.size and (best is None or t[idx[0]] < best[0]):
            k = int(idx[0])
            best = (float(t[k]), a.actor_id, float(x[k]), float(y[k]))
    return best
