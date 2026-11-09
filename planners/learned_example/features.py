"""Observation -> fixed-size feature vector.  Used by training AND by the planner.

Uses the posted speed limit (obs.speed_limit_mps), never obs.desired_speed_mps: a real car knows the
limit, not the speed somebody "wants"."""

import math

import numpy as np

from planners.planner_tools import actor_position_on_lane
from simulator.geometry import reference_state

FEATURE_VERSION = 2                     # 2: speed limit instead of desired speed
LANE_AHEAD_M = (0.0, 10.0, 20.0, 30.0, 40.0, 50.0)
MAX_ACTORS = 8
KINDS = ("car", "truck", "bicycle", "pedestrian")
CONTROLS = (None, "stop_sign", "signal")
SIGNALS = ("RED", "YELLOW", "GREEN")


def encode(obs) -> np.ndarray:
    lane = obs.lanes["current"]
    f = [obs.ego.speed_mps / 15.0, obs.ego.acceleration_mps2 / 3.0, obs.speed_limit_mps / 15.0,
         min(obs.distance_to_goal_m, 100.0) / 100.0, lane.ego_d_m / 2.0,
         float(reference_state(lane.centerline, lane.ego_s_m)[2])]
    x, y = reference_state(lane.centerline, lane.ego_s_m + np.array(LANE_AHEAD_M))[:2]
    f += list(np.asarray(x) / 50.0) + list(np.asarray(y) / 10.0)          # lane shape ahead (ego frame)
    for side in ("left", "right"):
        other = obs.lanes.get(side)
        f += [float(other is not None), float(other is not None and other.change_allowed)]
    line = obs.stop_lines[0] if obs.stop_lines else None
    f += [min(line.distance_m, 100.0) / 100.0 if line else 1.0]
    f += [float(line is not None and line.control == c) for c in CONTROLS]
    f += [float(line is not None and line.signal_state == s) for s in SIGNALS]
    actors = sorted(obs.actors, key=lambda a: math.hypot(a.x_m, a.y_m))[:MAX_ACTORS]
    for k in range(MAX_ACTORS):
        if k < len(actors):
            a = actors[k]
            s, d = actor_position_on_lane(lane, a)
            f += [1.0, a.x_m / 50.0, a.y_m / 10.0, (s - lane.ego_s_m) / 50.0, d / 10.0, a.vx_mps / 15.0,
                  a.vy_mps / 15.0, math.cos(a.heading_rad), math.sin(a.heading_rad), a.length_m / 10.0]
            f += [float(a.kind == kind) for kind in KINDS]
        else:
            f += [0.0] * (10 + len(KINDS))                                  # empty slot
    return np.asarray(f, dtype=np.float32)


NUM_FEATURES = len(LANE_AHEAD_M) * 2 + 6 + 4 + 1 + len(CONTROLS) + len(SIGNALS) + MAX_ACTORS * (10 + len(KINDS))
