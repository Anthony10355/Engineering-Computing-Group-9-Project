"""Data types that describe a scenario.  Students build these through the
friendly helper functions in ``scenarios/scenario_tools.py``.

Actor motion (copied from the original simulator's scripted actors):

* ``follow_road=True``: the actor drives along a road reference line at a
  constant sideways offset ``road_d_m`` (``oncoming=True`` drives the other
  way).  A ``lateral_profile`` moves it sideways (lane change / cut-in).
* ``follow_road=False``: the actor moves in a straight line with heading
  road_heading(s) + ``heading_offset_rad`` (pedestrians crossing the road).
* ``speed_profile``: ((time_s, speed_mps), ...) keyframes; the speed is
  interpolated linearly between keyframes.  Empty = constant ``speed_mps``.
  Keyframe times are absolute simulation times (also for late actors).
* ``appear_time_s``: the actor is absent before this time and then appears
  at its start position (road_s_m, road_d_m); it does not move before.
* Actors are SCRIPTED: they do not react to the ego car.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from simulator.road_network import RoadNetwork


#: Object classes (numbers follow the original perception convention; they
#: also choose the drawing colour).
PEDESTRIAN, BICYCLE, CAR, TRUCK = 1, 2, 3, 6

#: metres per second in one mile per hour (speed limits: 25 * MPH)
MPH = 0.44704
KIND_TO_CLASS = {"pedestrian": PEDESTRIAN, "bicycle": BICYCLE, "car": CAR, "truck": TRUCK}
CLASS_TO_KIND = {v: k for k, v in KIND_TO_CLASS.items()}


@dataclass(frozen=True)
class ActorSpec:
    actor_id: int
    kind: str                         # "car", "truck", "bicycle" or "pedestrian"
    road_s_m: float                   # start position along the road reference line
    road_d_m: float                   # start sideways offset (+ = right of the reference line)
    speed_mps: float
    length_m: float = 4.6
    width_m: float = 1.9
    follow_road: bool = True
    oncoming: bool = False
    heading_offset_rad: float = 0.0
    road_id: Optional[str] = None     # None = the main road of the network
    speed_profile: Tuple[Tuple[float, float], ...] = ()
    lateral_profile: Tuple[Tuple[float, float], ...] = ()
    appear_time_s: float = 0.0        # the actor exists only from this time on, and appears at its start pose

    @property
    def class_id(self) -> int:
        return KIND_TO_CLASS[self.kind]


@dataclass(frozen=True, eq=False)
class Scenario:
    name: str
    description: str
    road: RoadNetwork
    route: Tuple[str, ...]            # lane IDs the ego car must follow (its mission)
    ego_start_s_m: float = 30.0       # start position along the route
    ego_start_offset_m: float = 0.0   # start sideways offset (+ = right)
    ego_speed_mps: float = 8.0
    desired_speed_mps: float = 10.0   # cruise speed the planner should aim for (also sets the ideal time)
    speed_limit_mps: float = 25 * MPH  # posted maximum speed (default 25 mph = 11.18 m/s)
    goal_distance_m: Optional[float] = None   # distance to drive along the route (None = to route end - 12 m)
    time_limit_s: float = 40.0
    actors: Tuple[ActorSpec, ...] = ()
    #: signal_id -> ((start_time_s, "RED" | "YELLOW" | "GREEN"), ...)
    signal_phases: Dict[str, Tuple[Tuple[float, str], ...]] = field(default_factory=dict)
    #: start in a different lane than the first route lane (e.g. the left lane)
    ego_start_lane: Optional[str] = None

    def __post_init__(self) -> None:
        if not 0.0 < self.speed_limit_mps:
            raise ValueError(f"scenario {self.name}: speed_limit_mps must be positive")
        if self.desired_speed_mps > self.speed_limit_mps + 1e-6:
            raise ValueError(f"scenario {self.name}: desired_speed_mps ({self.desired_speed_mps:.2f}) is above "
                             f"speed_limit_mps ({self.speed_limit_mps:.2f})")
