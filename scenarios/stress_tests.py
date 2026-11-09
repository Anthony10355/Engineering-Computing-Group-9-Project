"""Team members: Anthony Pollard, Cole Pulaski, Will Cunningham, Brandon Debrosse

Stress-test scenes for checking the planner on situations the example scenes
don't cover (stop signs, traffic lights, turns, cyclists, occlusion, cut-ins,
tight junction gaps).  Names start with "st_".

They are added to the scenario list at the bottom of student_scenario.py.
"""

from __future__ import annotations

from scenarios.scenario_tools import (
    INTERSECTION_LANE, LEFT_LANE, ONCOMING_LANE, RIGHT_LANE, STRAIGHT_THROUGH, TURN_LEFT, TURN_RIGHT,
    MPH, Scenario, bicycle, car, curved_two_lane_road, intersection, intersection_signals, pedestrian, truck,
    two_lane_road,
)


def st_stop_sign_left_turn() -> Scenario:
    """All-way stop, ego turns LEFT.  Must stop fully at the line, then wait for
    the oncoming car (which goes straight) and a car from the right."""
    return Scenario(
        name="st_stop_sign_left_turn",
        description="All-way stop, left turn across an oncoming car, plus a car arriving from the right",
        road=intersection("stop_signs"), route=TURN_LEFT,
        ego_start_s_m=70.0, ego_speed_mps=8.0, desired_speed_mps=8.0, time_limit_s=50.0,
        actors=(car(1, s_m=60.0, offset_m=INTERSECTION_LANE, speed_mps=7.0, road="A2",       # oncoming
                    speed_changes=[(9.0, 7.0), (12.0, 0.0), (14.0, 0.0), (17.0, 7.0)]),
                car(2, s_m=70.0, offset_m=INTERSECTION_LANE, speed_mps=7.0, road="A3",       # from the right
                    speed_changes=[(10.0, 7.0), (13.0, 0.0), (19.0, 0.0), (22.0, 7.0)])))


def st_red_then_green() -> Scenario:
    """Traffic light: red when the ego arrives, green at 14 s.  Cross traffic
    keeps driving on its green until 14 s."""
    return Scenario(
        name="st_red_then_green",
        description="Arrive at a red light, wait, go on green; cross traffic uses its green meanwhile",
        road=intersection("signals"), route=STRAIGHT_THROUGH,
        ego_start_s_m=60.0, ego_speed_mps=8.0, desired_speed_mps=8.0, time_limit_s=45.0,
        signal_phases=intersection_signals(ego_phases=[(0.0, "RED"), (14.0, "GREEN")],
                                           crossing_phases=[(0.0, "GREEN"), (12.0, "YELLOW"), (14.0, "RED")]),
        actors=tuple(car(10 + i, s_m=20.0 + 25.0 * i, offset_m=INTERSECTION_LANE, speed_mps=8.0, road="A1")
                     for i in range(4)))


def st_yellow_dilemma() -> Scenario:
    """Light turns YELLOW when the ego is ~25 m from the line at speed, then RED
    3 s later.  Either stop smoothly or clear the line before red - never run the red."""
    return Scenario(
        name="st_yellow_dilemma",
        description="Light turns yellow in the dilemma zone, red 3 s later",
        road=intersection("signals"), route=STRAIGHT_THROUGH,
        ego_start_s_m=40.0, ego_speed_mps=10.0, desired_speed_mps=10.0, time_limit_s=45.0,
        signal_phases=intersection_signals(ego_phases=[(0.0, "GREEN"), (7.0, "YELLOW"), (10.0, "RED"), (22.0, "GREEN")],
                                           crossing_phases=[(0.0, "RED"), (11.0, "GREEN"), (20.0, "YELLOW"),
                                                            (22.0, "RED")]))


def st_right_turn_pedestrian() -> Scenario:
    """Uncontrolled junction, ego turns RIGHT; a car comes from the left."""
    return Scenario(
        name="st_right_turn_cross_car",
        description="Right turn at an uncontrolled junction while a car approaches from the left",
        road=intersection(), route=TURN_RIGHT,
        ego_start_s_m=60.0, ego_speed_mps=8.0, desired_speed_mps=8.0, time_limit_s=45.0,
        actors=(car(1, s_m=70.0, offset_m=INTERSECTION_LANE, speed_mps=7.0, road="A1"),))


def st_cyclist_on_curve() -> Scenario:
    """S-curve with a cyclist at the right edge of our lane and traffic in the
    left lane: follow the cyclist until there is room to pass."""
    return Scenario(
        name="st_cyclist_on_curve",
        description="Cyclist at the lane edge on an S-curve, busy left lane - follow, then pass with room",
        road=curved_two_lane_road(), route=("curved_road:F2",),
        ego_start_s_m=20.0, ego_speed_mps=9.0, desired_speed_mps=10.0, time_limit_s=60.0,
        goal_distance_m=300.0,
        actors=tuple([bicycle(1, s_m=70.0, offset_m=RIGHT_LANE + 0.9, speed_mps=4.5)]
                     + [car(100 + i, s_m=-40.0 + 18.0 * i, offset_m=LEFT_LANE, speed_mps=9.0) for i in range(4)]))


def st_occluded_pedestrian() -> Scenario:
    """Ego drives in the LEFT lane past a truck parked in the right lane.  A
    pedestrian steps out from in front of the truck when the ego is ~35 m away."""
    return Scenario(
        name="st_occluded_pedestrian",
        description="Pedestrian steps out from in front of a parked truck (late detection)",
        road=two_lane_road(), route=("two_lane_road:F1",),
        ego_start_s_m=30.0, ego_speed_mps=10.0, desired_speed_mps=10.0, time_limit_s=45.0,
        goal_distance_m=250.0,
        actors=(truck(1, s_m=120.0, offset_m=RIGHT_LANE, speed_mps=0.0),
                pedestrian(2, s_m=127.0, offset_m=RIGHT_LANE, walk_toward="left", speed_mps=1.5,
                           appear_at_s=5.5)))


def st_cut_in() -> Scenario:
    """A car in the left lane overtakes and cuts in ~10 m in front, then brakes."""
    return Scenario(
        name="st_cut_in",
        description="Car cuts in close in front, then brakes",
        road=two_lane_road(), route=("two_lane_road:F2",),
        ego_start_s_m=30.0, ego_speed_mps=10.0, desired_speed_mps=10.0, time_limit_s=45.0,
        goal_distance_m=300.0,
        actors=(car(1, s_m=28.0, offset_m=LEFT_LANE, speed_mps=13.0,
                    lane_changes=[(5.0, LEFT_LANE), (7.0, RIGHT_LANE)],
                    speed_changes=[(8.0, 13.0), (10.0, 6.0), (14.0, 6.0), (17.0, 10.0)]),))


def st_tight_gaps() -> Scenario:
    """Uncontrolled junction: a stream of cars from the left with only one gap
    that is big enough to cross."""
    stream = [car(10 + i, s_m=100.0 - 14.0 * i, offset_m=INTERSECTION_LANE, speed_mps=7.0, road="A1")
              for i in range(4)]
    later = [car(20 + i, s_m=100.0 - 14.0 * 4 - 60.0 - 14.0 * i, offset_m=INTERSECTION_LANE, speed_mps=7.0,
                 road="A1") for i in range(3)]
    return Scenario(
        name="st_tight_gaps",
        description="Uncontrolled junction: platoon of cars from the left with one usable gap",
        road=intersection(), route=STRAIGHT_THROUGH,
        ego_start_s_m=60.0, ego_speed_mps=8.0, desired_speed_mps=8.0, time_limit_s=50.0,
        actors=tuple(stream + later))


STRESS_TESTS = {
    "st_stop_sign_left_turn": st_stop_sign_left_turn,
    "st_red_then_green": st_red_then_green,
    "st_yellow_dilemma": st_yellow_dilemma,
    "st_right_turn_cross_car": st_right_turn_pedestrian,
    "st_cyclist_on_curve": st_cyclist_on_curve,
    "st_occluded_pedestrian": st_occluded_pedestrian,
    "st_cut_in": st_cut_in,
    "st_tight_gaps": st_tight_gaps,
}
