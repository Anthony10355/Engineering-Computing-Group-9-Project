"""Building blocks for scenarios.  Scenario authors only need this file.

Roads
-----
two_lane_road()          straight road, 2 lanes in each direction (900 m)
curved_two_lane_road()   gentle S-curve, 2 lanes in each direction
crosswalk_road()         straight 2+2 road with sidewalks and a crosswalk at 150 m
intersection(control)    four-way junction, 1 lane per direction;
                         control = None (uncontrolled), "stop_signs" or "signals"

Positions on a road are (s, offset): s = metres along the road's reference
line, offset = metres to the RIGHT of it (negative = left).  On the 2+2
roads the reference line is the yellow centre line, so
    LEFT_LANE  = 1.75 m  (inner forward lane, "F1")
    RIGHT_LANE = 5.25 m  (outer forward lane, "F2")
    ONCOMING_LANE = -1.75 m (inner opposing lane, "O1")

Road users
----------
car(...), truck(...), bicycle(...), pedestrian(...)
    speed_changes = [(time_s, speed_mps), ...]   speed keyframes (linear in between)
    lane_changes  = [(time_s, offset_m), ...]    sideways keyframes (smooth in between)
    appear_at_s   = time when the road user appears (late detection), at its start
                    position (s_m, offset_m); keyframe times stay absolute
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

from simulator.road_network import RoadNetwork, four_way_intersection, road_segment, s_curve_reference, straight_reference
from simulator.scenario_types import MPH, ActorSpec, Scenario

__all__ = [
    "Scenario", "MPH", "two_lane_road", "curved_two_lane_road", "crosswalk_road", "intersection",
    "car", "truck", "bicycle", "pedestrian", "intersection_signals",
    "LEFT_LANE", "RIGHT_LANE", "ONCOMING_LANE", "ONCOMING_OUTER_LANE",
    "STRAIGHT_THROUGH", "TURN_LEFT", "TURN_RIGHT", "INTERSECTION_LANE",
]

LANE_WIDTH = 3.5
LEFT_LANE = 0.5 * LANE_WIDTH            # inner forward lane (F1)
RIGHT_LANE = 1.5 * LANE_WIDTH           # outer forward lane (F2)
ONCOMING_LANE = -0.5 * LANE_WIDTH       # inner opposing lane (O1)
ONCOMING_OUTER_LANE = -1.5 * LANE_WIDTH  # outer opposing lane (O2)

#: Intersection: each arm road A0 (south), A1 (west), A2 (north), A3 (east)
#: runs from the outer end TOWARD the junction centre (centre at s = 150 m).
#: Its inbound lane is INTERSECTION_LANE metres to the right of it.
INTERSECTION_LANE = 0.5 * LANE_WIDTH
#: Ego routes through the intersection, starting on arm A0 (from the south)
STRAIGHT_THROUGH = ("A0:in", "A0>A2:S", "A2:out")
TURN_LEFT = ("A0:in", "A0>A1:L", "A1:out")
TURN_RIGHT = ("A0:in", "A0>A3:R", "A3:out")


# ---------------------------------------------------------------------------
# Roads
# ---------------------------------------------------------------------------
_CACHE: Dict[tuple, RoadNetwork] = {}


def _cached(key, build) -> RoadNetwork:
    if key not in _CACHE:
        _CACHE[key] = build()
    return _CACHE[key]


def two_lane_road(length_m: float = 900.0, solid_divider_until_m: float = 20.0) -> RoadNetwork:
    """Straight road, lanes "two_lane_road:F1" (left) and "two_lane_road:F2" (right)."""
    return _cached(("two_lane_road", length_m, solid_divider_until_m), lambda: road_segment(
        "two_lane_road", straight_reference(length_m), 2, solid_divider_until_s_m=solid_divider_until_m))


def curved_two_lane_road() -> RoadNetwork:
    """Gentle S-curve (radius 100 m), lanes "curved_road:F1" / "curved_road:F2"."""
    return _cached(("curved_road",), lambda: road_segment("curved_road", s_curve_reference(100.0), 2))


def crosswalk_road() -> RoadNetwork:
    """Straight 2+2 road with sidewalks (centre at offset 8.95 m) and a crosswalk at s = 150 m."""
    return _cached(("crosswalk_road",), lambda: road_segment(
        "crosswalk_road", straight_reference(400.0), 2, sidewalk_width_m=2.5, crosswalk_s_m=(150.0,)))


def intersection(control: Optional[str] = None) -> RoadNetwork:
    """Four-way junction at the origin.  control: None, "stop_signs" or "signals"."""
    name = {None: "intersection", "stop_signs": "intersection_stop", "signals": "intersection_signals"}[control]
    return _cached(("intersection", control), lambda: four_way_intersection(name, control=control))


def intersection_signals(ego_phases: Sequence[Tuple[float, str]],
                         crossing_phases: Sequence[Tuple[float, str]] = ((0.0, "GREEN"),)):
    """Signal timing for ``intersection("signals")``: arms A0/A2 (the ego's road)
    use ``ego_phases``, the crossing arms A1/A3 use ``crossing_phases``.
    Phases are ((start_time_s, "RED" | "YELLOW" | "GREEN"), ...)."""
    ego, cross = tuple(ego_phases), tuple(crossing_phases)
    return {"SIG_A0": ego, "SIG_A2": ego, "SIG_A1": cross, "SIG_A3": cross}


# ---------------------------------------------------------------------------
# Road users
# ---------------------------------------------------------------------------
def _keyframes(first_value: float, changes: Iterable[Tuple[float, float]]) -> Tuple[Tuple[float, float], ...]:
    changes = tuple((float(t), float(v)) for t, v in changes)
    if not changes:
        return ()
    if changes[0][0] > 0.0:
        changes = ((0.0, float(first_value)),) + changes
    return changes


def car(actor_id: int, s_m: float, offset_m: float = RIGHT_LANE, speed_mps: float = 8.0, *,
        speed_changes=(), lane_changes=(), road: Optional[str] = None, oncoming: bool = False,
        appear_at_s: float = 0.0, length_m: float = 4.6, width_m: float = 1.9, kind: str = "car") -> ActorSpec:
    """A vehicle that drives along a road.

    ``road``: None = the main road; on the intersection use "A0".."A3"
    (the arm the car starts on; it drives toward and straight across the
    junction).  ``oncoming=True`` drives against the road direction."""
    return ActorSpec(actor_id, kind, float(s_m), float(offset_m), float(speed_mps), length_m, width_m,
                     follow_road=True, oncoming=oncoming, road_id=road,
                     speed_profile=_keyframes(speed_mps, speed_changes),
                     lateral_profile=_keyframes(offset_m, lane_changes), appear_time_s=appear_at_s)


def truck(actor_id: int, s_m: float, offset_m: float = RIGHT_LANE, speed_mps: float = 6.0, **kwargs) -> ActorSpec:
    return car(actor_id, s_m, offset_m, speed_mps, length_m=9.0, width_m=2.5, kind="truck", **kwargs)


def bicycle(actor_id: int, s_m: float, offset_m: float, speed_mps: float = 4.0, **kwargs) -> ActorSpec:
    return car(actor_id, s_m, offset_m, speed_mps, length_m=1.8, width_m=0.6, kind="bicycle", **kwargs)


def pedestrian(actor_id: int, s_m: float, offset_m: float, walk_toward: str = "left", speed_mps: float = 1.3, *,
               speed_changes=(), road: Optional[str] = None, appear_at_s: float = 0.0) -> ActorSpec:
    """A pedestrian that walks straight ACROSS the road (``walk_toward``
    "left" or "right" as seen from the road direction) or "along"."""
    heading = {"left": -math.pi / 2, "right": math.pi / 2, "along": 0.0}[walk_toward]
    return ActorSpec(actor_id, "pedestrian", float(s_m), float(offset_m), float(speed_mps), 0.5, 0.5,
                     follow_road=False, heading_offset_rad=heading, road_id=road,
                     speed_profile=_keyframes(speed_mps, speed_changes), appear_time_s=appear_at_s)
