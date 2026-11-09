"""The three introductory scenarios.

Do not edit these (they are the common reference for everybody).  Copy one
into scenarios/student_scenario.py and change the copy instead.
"""

from __future__ import annotations

from scenarios.scenario_tools import (
    INTERSECTION_LANE, LEFT_LANE, ONCOMING_LANE, RIGHT_LANE, STRAIGHT_THROUGH, MPH, Scenario, car, intersection, truck,
    two_lane_road,
)


def lead_vehicle_brakes() -> Scenario:
    """A slower car ahead brakes to a stop, waits, then drives on.  The left
    lane is full of traffic, so the ego car must slow down and stop behind it."""
    lead = car(1, s_m=75.0, offset_m=RIGHT_LANE, speed_mps=8.0,
               speed_changes=[(5.0, 8.0), (8.0, 0.0), (15.0, 0.0), (19.0, 8.0)])
    left_lane_traffic = [car(100 + i, s_m=-150.0 + 13.0 * i, offset_m=LEFT_LANE, speed_mps=9.5) for i in range(26)]
    return Scenario(
        name="lead_vehicle_brakes",
        description="Slower lead car brakes to a stop, waits, then drives on; the left lane is busy",
        road=two_lane_road(), route=("two_lane_road:F2",),
        ego_start_s_m=30.0, ego_speed_mps=10.0, desired_speed_mps=10.0, speed_limit_mps=25 * MPH,
        goal_distance_m=250.0, time_limit_s=45.0,
        actors=tuple([lead] + left_lane_traffic))


def lane_change_overtake() -> Scenario:
    """A slow truck blocks the right lane.  A fast car is coming from behind in
    the left lane: let it pass, then change left, overtake and return right."""
    return Scenario(
        name="lane_change_overtake",
        description="Slow truck ahead; a fast car approaches in the left lane - let it pass, overtake, return",
        road=two_lane_road(), route=("two_lane_road:F2",),
        ego_start_s_m=30.0, ego_speed_mps=10.0, desired_speed_mps=10.0, speed_limit_mps=25 * MPH,
        goal_distance_m=400.0, time_limit_s=55.0,
        actors=(truck(1, s_m=80.0, offset_m=RIGHT_LANE, speed_mps=5.0),
                car(2, s_m=-5.0, offset_m=LEFT_LANE, speed_mps=14.0),
                car(3, s_m=300.0, offset_m=ONCOMING_LANE, speed_mps=9.0, oncoming=True)))


def intersection_crossing_traffic() -> Scenario:
    """Uncontrolled four-way junction.  Cars cross from the left and from the
    right; the ego car must yield, wait for a safe gap and drive straight on."""
    return Scenario(
        name="intersection_crossing_traffic",
        description="Uncontrolled junction, straight through; cars cross from the left and right - yield, then go",
        road=intersection(), route=STRAIGHT_THROUGH,
        ego_start_s_m=60.0, ego_speed_mps=8.0, desired_speed_mps=8.0, speed_limit_mps=25 * MPH,
        time_limit_s=45.0,
        actors=(car(1, s_m=74.0, offset_m=INTERSECTION_LANE, speed_mps=7.0, road="A1"),     # from the left (west)
                car(2, s_m=50.0, offset_m=INTERSECTION_LANE, speed_mps=7.0, road="A3"),     # from the right (east)
                car(3, s_m=40.0, offset_m=INTERSECTION_LANE, speed_mps=7.0, road="A2")))    # oncoming, straight


EXAMPLE_SCENARIOS = {
    "lead_vehicle_brakes": lead_vehicle_brakes,
    "lane_change_overtake": lane_change_overtake,
    "intersection_crossing_traffic": intersection_crossing_traffic,
}
