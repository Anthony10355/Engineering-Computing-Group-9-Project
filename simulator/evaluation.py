"""Independent evaluation: metrics and score.

This module reads ONLY the simulated world (true ego footprint, true actor
footprints, road surface, route).  It never looks at what the planner
returned, so a planner cannot change its score except by how the car
actually drives.

Metrics
-------
collision         the ego footprint overlapped an actor footprint
road_departure    a corner of the ego footprint left the road surface (asphalt)
completed         the ego reached the goal distance along its route in time
progress          fraction (0..1) of the goal distance driven along the route
simulation_time   seconds simulated until the run ended

Traffic-rule checks (REPORTED for the judges; they do not change the score)
---------------------------------------------------------------------------
red_light_violations   the front bumper crossed a stop line while its signal was RED
stop_sign_violations   the front bumper crossed a stop-sign line without a full stop
                       (speed < 0.2 m/s) within the 10 m before the line
wrong_side_time_s      time the footprint centre spent on the opposing-traffic side
                       of a road (over the centre line)
collision_from_behind  the collision was a road user running into the ego from
                       behind (scripted road users do not brake for the ego)

All checks run at every vehicle-model sub-step (100 Hz).

Score (0 .. 100 points per scenario)
------------------------------------
* collision or road departure  -> 0 points (safety comes first)
* otherwise: 50 x progress
             + 30 if completed
             + 20 x min(1, ideal_time / time_used)  if completed
  where ideal_time = goal distance / desired speed.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
from typing import List, Optional

from simulator.geometry import (
    boxes_overlap, box_corners, cartesian_to_frenet, point_in_polygon, reference_state, wrap_angle,
)
from simulator.observation import goal_route_s
from simulator.vehicle_model import EGO_GEOMETRY
from simulator.world import World


@dataclass
class EvaluationResult:
    scenario: str
    planner: str
    collision: bool = False
    collision_actor_id: Optional[int] = None
    collision_time_s: Optional[float] = None
    road_departure: bool = False
    road_departure_time_s: Optional[float] = None
    completed: bool = False
    completion_time_s: Optional[float] = None
    progress: float = 0.0
    simulation_time_s: float = 0.0
    planner_errors: int = 0
    red_light_violations: int = 0
    stop_sign_violations: int = 0
    wrong_side_time_s: float = 0.0
    collision_from_behind: bool = False
    end_reason: str = "running"
    score: float = 0.0
    notes: List[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return asdict(self)

    @property
    def violation_summary(self) -> str:
        parts = []
        if self.red_light_violations:
            parts.append(f"red light x{self.red_light_violations}")
        if self.stop_sign_violations:
            parts.append(f"stop sign x{self.stop_sign_violations}")
        if self.wrong_side_time_s > 0.0:
            parts.append(f"wrong side {self.wrong_side_time_s:.1f} s")
        return ", ".join(parts) or "none"


def ego_footprint(world: World):
    """(centre, heading, length, width) of the TRUE ego footprint."""
    g, e = EGO_GEOMETRY, world.ego
    cx = e.x_m + g.center_offset_m * math.cos(e.yaw_rad)
    cy = e.y_m + g.center_offset_m * math.sin(e.yaw_rad)
    return (cx, cy), e.yaw_rad, g.length_m, g.width_m


def colliding_actor(world: World) -> Optional[int]:
    """Id of an actor whose true footprint overlaps the ego footprint, else None."""
    centre, heading, length, width = ego_footprint(world)
    for a in world.present_actors():
        if math.hypot(a.x_m - centre[0], a.y_m - centre[1]) > 0.5 * (length + a.length_m) + 2.0:
            continue
        if boxes_overlap(centre, heading, length, width, (a.x_m, a.y_m), a.heading_rad, a.length_m, a.width_m):
            return a.actor_id
    return None


def off_road(world: World) -> bool:
    """True if any corner of the ego footprint is outside every road surface."""
    centre, heading, length, width = ego_footprint(world)
    corners = box_corners(centre[0], centre[1], heading, length, width)
    on_road = None
    for surface in world.network.surfaces:
        inside = point_in_polygon(corners[:, 0], corners[:, 1], surface)
        on_road = inside if on_road is None else (on_road | inside)
    return bool(on_road is not None and not on_road.all())


def hit_from_behind(world: World, actor_id: int) -> bool:
    """True if the actor's centre is behind the ego's rear bumper, it overlaps
    the ego sideways and it drives in the ego's direction faster than the ego."""
    (cx, cy), heading, length, width = ego_footprint(world)
    a = next(a for a in world.actors if a.actor_id == actor_id)
    dx, dy = a.x_m - cx, a.y_m - cy
    along = dx * math.cos(heading) + dy * math.sin(heading)
    across = -dx * math.sin(heading) + dy * math.cos(heading)
    same_direction = abs(wrap_angle(a.heading_rad - heading)) < math.radians(45.0)
    return (along < -0.5 * length and abs(across) < 0.5 * (width + a.width_m) and same_direction
            and a.speed_mps > world.ego.speed_mps)


STOP_SIGN_ZONE_M = 10.0          # a full stop must happen within this distance before the line
FULL_STOP_MPS = 0.2


class Evaluator:
    def __init__(self, world: World, planner_name: str) -> None:
        self.world = world
        self.result = EvaluationResult(world.scenario.name, planner_name)
        self.start_s = world.scenario.ego_start_s_m
        self.goal_s = goal_route_s(world)
        self._last_time_s = world.time_s
        # stop lines on the route: (route s of the line, kind, signal id, stop line id)
        net, route = world.network, tuple(world.scenario.route)
        signal_of = {sig.stop_line_id: sig.signal_id for sig in net.signals.values()}
        stop_sign_lines = {sign.stop_line_id for sign in net.stop_signs}
        self._lines = []
        for line in net.stop_lines.values():
            if line.lane_id not in route:
                continue
            s_line = net.lane_start_on_route(route, line.lane_id) + line.s_on_lane_m
            if line.stop_line_id in signal_of:
                self._lines.append((s_line, "signal", signal_of[line.stop_line_id], line.stop_line_id))
            elif line.stop_line_id in stop_sign_lines:
                self._lines.append((s_line, "stop_sign", None, line.stop_line_id))
        self._prev_front_s = world.ego_route_frenet()[0] + EGO_GEOMETRY.rear_axle_to_front_m
        self._full_stop_at = set()
        self._roads = [r.reference for r in net.roads.values()]

    @property
    def finished(self) -> bool:
        return self.result.end_reason != "running"

    def _check_traffic_controls(self, front_s: float) -> None:
        w, r = self.world, self.result
        for s_line, kind, signal_id, line_id in self._lines:
            if kind == "stop_sign" and s_line - STOP_SIGN_ZONE_M <= front_s <= s_line + 0.5 \
                    and w.ego.speed_mps < FULL_STOP_MPS:
                self._full_stop_at.add(line_id)
            if not (self._prev_front_s < s_line <= front_s):
                continue                                     # the front bumper did not cross this line now
            if kind == "signal" and w.signal_state(signal_id).value == "RED":
                r.red_light_violations += 1
                r.notes.append(f"red light at {line_id}, t = {w.time_s:.2f} s")
            if kind == "stop_sign" and line_id not in self._full_stop_at:
                r.stop_sign_violations += 1
                r.notes.append(f"no full stop at {line_id}, t = {w.time_s:.2f} s")
        self._prev_front_s = front_s

    def _on_wrong_side(self) -> bool:
        """Footprint centre on the opposing-traffic side of the road it is on.
        Road reference lines are the centre lines; traffic keeps right of them."""
        (cx, cy), heading, _, _ = ego_footprint(self.world)
        for ref in self._roads:
            s, d = cartesian_to_frenet(ref, cx, cy)
            if not (0.0 <= s <= ref.length_m) or abs(d) > 12.0:
                continue                                     # not on this road (e.g. inside the junction box)
            along = math.cos(heading - float(reference_state(ref, s)[2]))
            return d * along < 0.0
        return False

    def update(self) -> None:
        """Call after every vehicle-model sub-step (the simulation does)."""
        w, r = self.world, self.result
        dt = w.time_s - self._last_time_s
        self._last_time_s = w.time_s
        r.simulation_time_s = w.time_s
        s, _ = w.ego_route_frenet()
        r.progress = max(0.0, min(1.0, (s - self.start_s) / max(self.goal_s - self.start_s, 1e-6)))
        self._check_traffic_controls(s + EGO_GEOMETRY.rear_axle_to_front_m)
        if self._on_wrong_side():
            if r.wrong_side_time_s == 0.0:
                r.notes.append(f"on the opposing side of the road from t = {w.time_s:.2f} s")
            r.wrong_side_time_s = round(r.wrong_side_time_s + dt, 6)
        actor = colliding_actor(w)
        if actor is not None and not r.collision:
            r.collision, r.collision_actor_id, r.collision_time_s = True, actor, w.time_s
            r.collision_from_behind = hit_from_behind(w, actor)
            r.end_reason = f"collision with actor #{actor}" + (" (hit from behind)" if r.collision_from_behind else "")
        if not r.road_departure and off_road(w):
            r.road_departure, r.road_departure_time_s = True, w.time_s
            if r.end_reason == "running":
                r.end_reason = "left the road"
        if r.end_reason == "running" and s >= self.goal_s:
            r.completed, r.completion_time_s = True, w.time_s
            r.end_reason = "goal reached"
        if r.end_reason == "running" and w.time_s >= w.scenario.time_limit_s - 1e-9:
            r.end_reason = "time limit"
        r.score = compute_score(r, (self.goal_s - self.start_s) / max(w.scenario.desired_speed_mps, 0.1))


def compute_score(r: EvaluationResult, ideal_time_s: float) -> float:
    if r.collision or r.road_departure:
        return 0.0
    score = 50.0 * r.progress
    if r.completed and r.completion_time_s:
        score += 30.0 + 20.0 * min(1.0, ideal_time_s / r.completion_time_s)
    return round(score, 1)
