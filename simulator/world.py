"""World truth: road network, ego vehicle state, actors and traffic signals.

Simulation only -- the planner never sees this module.  It receives an
``Observation`` built by ``simulator/observation.py``.

Copied (simplified) from the original research simulator: scripted actors,
signal timing and the ego plant are unchanged; the random-traffic and
scripted-maneuver machinery of the research version was removed.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

from simulator.geometry import ReferencePath, cartesian_to_frenet, frenet_to_cartesian, reference_state
from simulator.road_network import SignalState
from simulator.scenario_types import ActorSpec, Scenario
from simulator.vehicle_model import BicycleState, KinematicBicycle


def path_pose(path: ReferencePath, s_m: float, d_m: float) -> Tuple[float, float, float]:
    x, y = frenet_to_cartesian(path, s_m, d_m)
    return float(x), float(y), float(reference_state(path, s_m)[2])


def _smooth_keyframes(keys, t: float) -> Tuple[float, float]:
    """Value and time derivative of smoothstep-joined keyframes."""
    if t <= keys[0][0]:
        return keys[0][1], 0.0
    for (t0, v0), (t1, v1) in zip(keys, keys[1:]):
        if t0 <= t <= t1:
            u = (t - t0) / max(t1 - t0, 1e-9)
            return v0 + (v1 - v0) * u * u * (3 - 2 * u), (v1 - v0) * 6 * u * (1 - u) / max(t1 - t0, 1e-9)
    return keys[-1][1], 0.0


def _profile_speed(keys, t: float) -> float:
    ts = [k[0] for k in keys]
    vs = [k[1] for k in keys]
    if t <= ts[0]:
        return vs[0]
    if t >= ts[-1]:
        return vs[-1]
    i = bisect.bisect_right(ts, t) - 1
    u = (t - ts[i]) / (ts[i + 1] - ts[i])
    return vs[i] + u * (vs[i + 1] - vs[i])


@dataclass
class Actor:
    """One scripted road user (simulation truth)."""

    actor_id: int
    class_id: int
    kind: str
    x_m: float
    y_m: float
    heading_rad: float
    speed_mps: float
    length_m: float
    width_m: float
    road: ReferencePath
    road_s_m: Optional[float] = None      # set when the actor follows the road
    road_d_m: float = 0.0
    oncoming: bool = False
    speed_profile: Tuple[Tuple[float, float], ...] = ()
    lateral_profile: Tuple[Tuple[float, float], ...] = ()
    time_s: float = 0.0
    appear_time_s: float = 0.0

    @classmethod
    def from_spec(cls, spec: ActorSpec, road: ReferencePath) -> "Actor":
        x, y, h = path_pose(road, spec.road_s_m, spec.road_d_m)
        h = h + (math.pi if spec.oncoming else 0.0) if spec.follow_road else h + spec.heading_offset_rad
        speed = _profile_speed(spec.speed_profile, 0.0) if spec.speed_profile else spec.speed_mps
        return cls(spec.actor_id, spec.class_id, spec.kind, x, y, h, speed, spec.length_m, spec.width_m, road,
                   spec.road_s_m if spec.follow_road else None, spec.road_d_m,
                   spec.oncoming and spec.follow_road, spec.speed_profile, spec.lateral_profile,
                   appear_time_s=spec.appear_time_s)

    def advance(self, dt_s: float) -> None:
        v_old = self.speed_mps
        self.time_s += dt_s
        if self.speed_profile:
            self.speed_mps = _profile_speed(self.speed_profile, self.time_s)
        if self.time_s <= self.appear_time_s + 1e-9:
            return                    # not there yet: appear exactly where the scenario placed it
        step = 0.5 * (v_old + self.speed_mps) * dt_s
        if self.road_s_m is not None:
            direction = -1.0 if self.oncoming else 1.0
            self.road_s_m += direction * step
            dd_dt = 0.0
            if self.lateral_profile:
                self.road_d_m, dd_dt = _smooth_keyframes(self.lateral_profile, self.time_s)
            self.x_m, self.y_m, self.heading_rad = path_pose(self.road, self.road_s_m, self.road_d_m)
            if self.speed_mps > 0.1 and dd_dt:
                self.heading_rad += math.atan2(direction * dd_dt, self.speed_mps)
            if self.oncoming:
                self.heading_rad += math.pi
        else:
            self.x_m += step * math.cos(self.heading_rad)
            self.y_m += step * math.sin(self.heading_rad)


class World:
    def __init__(self, scenario: Scenario, plant: Optional[KinematicBicycle] = None) -> None:
        self.scenario = scenario
        self.network = scenario.road
        self.plant = plant or KinematicBicycle()
        self.time_s = 0.0
        self.route_path = self.network.route_path(scenario.route)
        if scenario.ego_start_lane:
            start_path = self.network.lanes[scenario.ego_start_lane].centerline
            gx, gy, _ = path_pose(self.route_path, scenario.ego_start_s_m, 0.0)
            s_on_lane, _ = cartesian_to_frenet(start_path, gx, gy)
            x, y, h = path_pose(start_path, s_on_lane, scenario.ego_start_offset_m)
        else:
            x, y, h = path_pose(self.route_path, scenario.ego_start_s_m, scenario.ego_start_offset_m)
        self.ego = BicycleState(x, y, h, scenario.ego_speed_mps)
        roads = {rid: r.reference for rid, r in self.network.roads.items()}
        main = roads[self.network.main_road_id]
        self.actors: List[Actor] = []
        for spec in scenario.actors:
            if spec.road_id is not None and spec.road_id not in roads:
                raise KeyError(f"actor {spec.actor_id}: unknown road '{spec.road_id}'. Roads: {sorted(roads)}")
            self.actors.append(Actor.from_spec(spec, roads[spec.road_id] if spec.road_id else main))

    # --- queries -------------------------------------------------------
    def present_actors(self) -> List[Actor]:
        """Actors that exist now (``appear_time_s`` reached)."""
        return [a for a in self.actors if self.time_s >= a.appear_time_s - 1e-9]

    def ego_route_frenet(self) -> Tuple[float, float]:
        """TRUE ego (s, d) on the route path (rear axle)."""
        return cartesian_to_frenet(self.route_path, self.ego.x_m, self.ego.y_m)

    def ego_yaw_rate_rps(self) -> float:
        return self.plant.yaw_rate_rps(self.ego)

    def signal_state(self, signal_id: str) -> SignalState:
        phases = self.scenario.signal_phases.get(signal_id, ((0.0, "GREEN"),))
        state = phases[0][1]
        for start, value in phases:
            if self.time_s + 1e-9 >= start:
                state = value
        return SignalState(state)

    # --- simulation step -------------------------------------------------
    def advance(self, steering_command_rad: float, acceleration_command_mps2: float, dt_s: float) -> None:
        self.ego = self.plant.step(self.ego, steering_command_rad, acceleration_command_mps2, dt_s)
        for actor in self.actors:
            actor.advance(dt_s)
        self.time_s += dt_s
