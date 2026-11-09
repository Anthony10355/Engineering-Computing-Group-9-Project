"""Observation: everything the planner is allowed to know, in the EGO frame.

This module is the ONLY bridge from world truth to the planner.  The planner
receives a fresh ``Observation`` every planning cycle (20 times per second).

Ego frame: x = forward [m], y = RIGHT [m], the ego REAR AXLE is at (0, 0)
facing +x.  Headings are relative to the ego heading (positive = to the
right).

Perception model (as in the original simulator): perfect positions and
velocities, but only for road users inside the 120 m x 80 m camera window
(about 80 m ahead, 40 m behind, 40 m to each side).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from simulator.geometry import (
    ReferencePath, cartesian_to_frenet, frenet_to_cartesian, world_to_ego, world_vector_to_ego, wrap_angle,
)
from simulator.road_network import CROSSABLE_BOUNDARIES, Lane
from simulator.scenario_types import CLASS_TO_KIND
from simulator.vehicle_model import EGO_GEOMETRY
from simulator.world import World


#: perception window (same as the original 120 x 80 BEV grid, 1 m cells, ego at row 80 / col 40)
SENSOR_AHEAD_M, SENSOR_BEHIND_M, SENSOR_SIDE_M = 80.5, 39.5, 40.0
LANE_BEHIND_M, LANE_AHEAD_M = 15.0, 105.0
GOAL_MARGIN_M = 12.0


@dataclass(frozen=True)
class EgoObservation:
    speed_mps: float
    acceleration_mps2: float
    yaw_rate_rps: float            # + = turning right
    steering_rad: float            # + = steering right
    length_m: float = EGO_GEOMETRY.length_m
    width_m: float = EGO_GEOMETRY.width_m
    rear_axle_to_front_m: float = EGO_GEOMETRY.rear_axle_to_front_m


@dataclass(frozen=True)
class ActorObservation:
    actor_id: int
    kind: str                      # "car", "truck", "bicycle", "pedestrian"
    x_m: float                     # centre, ego frame (forward)
    y_m: float                     # centre, ego frame (right)
    heading_rad: float             # relative to the ego heading
    speed_mps: float
    vx_mps: float                  # velocity over ground, in ego axes
    vy_mps: float
    length_m: float
    width_m: float


@dataclass(frozen=True, eq=False)
class LaneObservation:
    """A lane the ego could drive in, as a centre line in the ego frame."""

    name: str                      # "current", "left" or "right"
    lane_id: str                   # map id, e.g. "two_lane_road:F2"
    centerline: ReferencePath      # ego frame, from ~15 m behind to ~105 m ahead
    width_m: float
    ego_s_m: float                 # where the ego rear axle is on this centre line
    ego_d_m: float                 # ego sideways offset from the centre line (+ = right)
    is_route_lane: bool            # True if the mission route continues in this lane
    change_allowed: bool           # left/right lanes: lane-change marking allows crossing here


@dataclass(frozen=True)
class StopLineObservation:
    distance_m: float              # along the route, from the ego REAR AXLE to the line
    control: Optional[str]         # None (yield line), "stop_sign" or "signal"
    signal_state: Optional[str]    # "RED" / "YELLOW" / "GREEN" for signals, else None
    stop_line_id: str


@dataclass(frozen=True)
class JunctionObservation:
    entry_distance_m: float        # along the route, from the ego rear axle
    exit_distance_m: float
    movement: str                  # "STRAIGHT", "LEFT" or "RIGHT"


@dataclass(frozen=True, eq=False)
class Observation:
    time_s: float
    ego: EgoObservation
    lanes: Dict[str, LaneObservation]          # keys "current" and, if they exist, "left" / "right"
    actors: Tuple[ActorObservation, ...]
    stop_lines: Tuple[StopLineObservation, ...]
    junctions: Tuple[JunctionObservation, ...]
    crosswalks: Tuple[Tuple[float, float], ...]  # (start, end) distances along the route
    desired_speed_mps: float
    distance_to_goal_m: float
    speed_limit_mps: float                     # posted maximum speed


def _lane_corridor(world: World, lane: Lane, route_lanes: Tuple[str, ...]) -> Tuple[ReferencePath, bool]:
    """World-frame path to follow when driving in ``lane``."""
    if lane.lane_id in route_lanes:
        return world.route_path, True
    return lane.centerline, False


def _window(world: World, path: ReferencePath) -> Tuple[ReferencePath, float, float]:
    """Cut ``path`` around the ego and express it in the ego frame."""
    ego = world.ego
    s_ego, _ = cartesian_to_frenet(path, ego.x_m, ego.y_m)
    s = np.arange(s_ego - LANE_BEHIND_M, s_ego + LANE_AHEAD_M + 1e-9, 1.0)
    gx, gy = frenet_to_cartesian(path, s, np.zeros_like(s))
    x, y = world_to_ego(ego.x_m, ego.y_m, ego.yaw_rad, gx, gy)
    local = ReferencePath.from_xy(np.column_stack((x, y)))
    s_local, d_local = cartesian_to_frenet(local, 0.0, 0.0)
    return local, s_local, d_local


def _route_lane_at(world: World, s_route: float) -> str:
    offset = 0.0
    for lane_id in world.scenario.route:
        length = world.network.lanes[lane_id].centerline.length_m
        if s_route < offset + length:
            return lane_id
        offset += length
    return world.scenario.route[-1]


def _current_lane(world: World, s_route: float) -> Lane:
    """Lane whose centre line is closest to the ego (route lane or a neighbour)."""
    lanes = world.network.lanes
    base = lanes[_route_lane_at(world, s_route)]
    options = [base]
    for side in ("left_lane_id", "right_lane_id"):
        lane = base
        while getattr(lane, side) is not None:
            lane = lanes[getattr(lane, side)]
            options.append(lane)
    best, best_d = base, math.inf
    for lane in options:
        s, d = cartesian_to_frenet(lane.centerline, world.ego.x_m, world.ego.y_m)
        if -5.0 <= s <= lane.centerline.length_m + 5.0 and abs(d) < best_d:
            best, best_d = lane, abs(d)
    return best


def build_observation(world: World) -> Observation:
    ego, net, sc = world.ego, world.network, world.scenario
    route = tuple(sc.route)
    s_route, _ = world.ego_route_frenet()

    # --- lanes ------------------------------------------------------------
    current = _current_lane(world, s_route)
    lanes: Dict[str, LaneObservation] = {}
    s_cur_lane, _ = cartesian_to_frenet(current.centerline, ego.x_m, ego.y_m)
    for name, lane_id in (("current", current.lane_id), ("left", current.left_lane_id),
                          ("right", current.right_lane_id)):
        if lane_id is None:
            continue
        lane = net.lanes[lane_id]
        path, on_route = _lane_corridor(world, lane, route)
        local, s_l, d_l = _window(world, path)
        allowed = True
        if name != "current":
            allowed = current.boundary_at(name, s_cur_lane) in CROSSABLE_BOUNDARIES
        lanes[name] = LaneObservation(name, lane_id, local, lane.width_m, s_l, d_l, on_route, allowed)

    # --- actors inside the sensor window -------------------------------
    actors: List[ActorObservation] = []
    for a in world.present_actors():
        x, y = (float(v) for v in world_to_ego(ego.x_m, ego.y_m, ego.yaw_rad, a.x_m, a.y_m))
        if not (-SENSOR_BEHIND_M < x <= SENSOR_AHEAD_M and -SENSOR_SIDE_M - 0.5 <= y < SENSOR_SIDE_M - 0.5):
            continue
        vx, vy = (float(v) for v in world_vector_to_ego(
            ego.yaw_rad, a.speed_mps * math.cos(a.heading_rad), a.speed_mps * math.sin(a.heading_rad)))
        actors.append(ActorObservation(a.actor_id, CLASS_TO_KIND.get(a.class_id, "car"), x, y,
                                       wrap_angle(a.heading_rad - ego.yaw_rad), abs(a.speed_mps), vx, vy,
                                       a.length_m, a.width_m))

    # --- traffic controls / junctions / crosswalks along the route ------
    sign_of = {sign.stop_line_id: sign for sign in net.stop_signs}
    signal_of = {sig.stop_line_id: sig for sig in net.signals.values()}
    stops = []
    for line in net.stop_lines.values():
        if line.lane_id not in route:
            continue
        dist = net.lane_start_on_route(route, line.lane_id) + line.s_on_lane_m - s_route
        if dist < -5.0 or dist > 150.0:
            continue
        if line.stop_line_id in signal_of:
            state = world.signal_state(signal_of[line.stop_line_id].signal_id).value
            stops.append(StopLineObservation(dist, "signal", state, line.stop_line_id))
        elif line.stop_line_id in sign_of:
            stops.append(StopLineObservation(dist, "stop_sign", None, line.stop_line_id))
        else:
            stops.append(StopLineObservation(dist, None, None, line.stop_line_id))
    junctions = []
    for lane_id in route:
        lane = net.lanes[lane_id]
        if lane.junction_id is None:
            continue
        start = net.lane_start_on_route(route, lane_id) - s_route
        end = start + lane.centerline.length_m
        if end > -5.0 and start < 150.0:
            junctions.append(JunctionObservation(start, end, lane.movement or "STRAIGHT"))
    crosswalks = []
    for cw in net.crosswalks:
        sd = [cartesian_to_frenet(world.route_path, px, py) for px, py in cw.polygon_m]
        d_vals = [d for _, d in sd]
        if min(d_vals) < 0.0 < max(d_vals):
            s_vals = [s for s, _ in sd]
            start, end = min(s_vals) - s_route, max(s_vals) - s_route
            if end > -5.0 and start < 150.0:
                crosswalks.append((start, end))

    goal_s = sc.ego_start_s_m + (sc.goal_distance_m if sc.goal_distance_m is not None
                                 else world.route_path.length_m - GOAL_MARGIN_M - sc.ego_start_s_m)
    return Observation(
        time_s=world.time_s,
        ego=EgoObservation(ego.speed_mps, ego.acceleration_mps2, world.ego_yaw_rate_rps(), ego.steering_rad),
        lanes=lanes, actors=tuple(actors), stop_lines=tuple(sorted(stops, key=lambda s: s.distance_m)),
        junctions=tuple(junctions), crosswalks=tuple(sorted(crosswalks)),
        desired_speed_mps=sc.desired_speed_mps, distance_to_goal_m=goal_s - s_route,
        speed_limit_mps=sc.speed_limit_mps)


def goal_route_s(world: World) -> float:
    """Route distance s at which the mission counts as completed."""
    sc = world.scenario
    if sc.goal_distance_m is not None:
        return sc.ego_start_s_m + sc.goal_distance_m
    return world.route_path.length_m - GOAL_MARGIN_M
