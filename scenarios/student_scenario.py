"""SCENARIO TEAM: write your scenarios here.

Team members: Anthony Pollard, Cole Pulaski, Will Cunningham, Brandon Debrosse

1. Write a function that returns a ``Scenario`` (see the example below and
   SCENARIO_GUIDE.md).
2. Add it to ``STUDENT_SCENARIOS`` at the bottom of this file.
3. Run it:   python run_simulation.py --scenario my_crosswalk

You never need to change anything in the simulator/ folder.
"""

from __future__ import annotations

from scenarios.scenario_tools import (   # noqa: F401  (everything you may need)
    INTERSECTION_LANE, LEFT_LANE, ONCOMING_LANE, RIGHT_LANE, STRAIGHT_THROUGH, TURN_LEFT, TURN_RIGHT,
    MPH, Scenario, bicycle, car, crosswalk_road, curved_two_lane_road, intersection, intersection_signals,
    pedestrian, truck, two_lane_road,
)


def my_crosswalk() -> Scenario:
    """Example: a pedestrian waits at a crosswalk, then crosses in front of the car."""
    walker = pedestrian(1, s_m=150.0, offset_m=8.95, walk_toward="left", speed_mps=0.0,
                        speed_changes=[(7.0, 0.0), (7.5, 1.3)])   # waits 7 s, then walks at 1.3 m/s
    return Scenario(
        name="my_crosswalk",
        description="A pedestrian waits at the crosswalk and then crosses in front of the car",
        road=crosswalk_road(), route=("crosswalk_road:F2",),
        ego_start_s_m=60.0, ego_speed_mps=8.0, desired_speed_mps=8.0, speed_limit_mps=25 * MPH,
        goal_distance_m=200.0, time_limit_s=45.0,
        actors=(walker,))


def city_mix() -> Scenario:
    """Three events in a row on one 400 m street (crosswalk_road):

    1. Braking truck: a delivery truck ahead brakes to a stop and waits; the
       left lane is busy, so the ego car has to stop behind it rather than swerve.
    2. Crosswalk: the truck drives on through the crosswalk (s = 150 m); then a
       pedestrian who was waiting there crosses in front of the ego car.
    3. Overtake: after the crosswalk the truck slows to 4 m/s; the ego car must
       wait for a gap in the left lane, overtake, and return before the goal.

    One truck plays all three parts, so no two scripted road users ever drive
    into each other (scripted road users don't react to each other).
    """
    # 1 + 3. Truck: brakes at t = 4 s, stopped 6.5-10 s, drives on at 8 m/s,
    #        then slows to 4 m/s from t = 22 s (about s = 200 m).
    lead_truck = truck(1, s_m=60.0, offset_m=RIGHT_LANE, speed_mps=8.0,
                       speed_changes=[(4.0, 8.0), (6.5, 0.0), (10.0, 0.0), (13.0, 8.0),
                                      (22.0, 8.0), (24.0, 4.0)])
    # Busy left lane at the start (blocks an early overtake; it catches up again later).
    left_traffic = [car(100 + i, s_m=-90.0 + 16.0 * i, offset_m=LEFT_LANE, speed_mps=9.5) for i in range(6)]

    # 2. Pedestrian on the right sidewalk: waits until the truck has passed, then crosses left.
    walker = pedestrian(2, s_m=150.0, offset_m=8.95, walk_toward="left", speed_mps=0.0,
                        speed_changes=[(20.0, 0.0), (20.5, 1.3)])

    # Oncoming traffic on the other side of the yellow line (not in our way, but visible).
    oncoming = car(5, s_m=390.0, offset_m=ONCOMING_LANE, speed_mps=9.0, oncoming=True)

    return Scenario(
        name="city_mix",
        description="Truck brakes ahead, then a pedestrian at a crosswalk, then overtaking the slow truck",
        road=crosswalk_road(), route=("crosswalk_road:F2",),
        ego_start_s_m=20.0, ego_speed_mps=8.0, desired_speed_mps=10.0, speed_limit_mps=25 * MPH,
        goal_distance_m=340.0, time_limit_s=80.0,
        actors=tuple([lead_truck, walker, oncoming] + left_traffic))


# Add every scenario function you write here: "name used on the command line": function
STUDENT_SCENARIOS = {
    "my_crosswalk": my_crosswalk,
    "city_mix": city_mix,
}

# Planner stress tests (scenarios/stress_tests.py), submitted with our scenes.
from scenarios.stress_tests import STRESS_TESTS  # noqa: E402
STUDENT_SCENARIOS.update(STRESS_TESTS)
