"""Team members: Anthony Pollard, Cole Pulaski, Will Cunningham, Brandon Debrosse

Robustness suite: many short scenes in the style a hidden test set is likely
to use, for checking the planner beyond the scenes we were given.  Names start
with "rb_".  Every scene is meant to be fair (SCENARIO_GUIDE.md section 7):
anything that blocks the ego lane does so at least the "fair" distance ahead.

Run them all with:   python tools/stress.py --prefix rb_
"""

from __future__ import annotations

from scenarios.scenario_tools import (
    INTERSECTION_LANE, LEFT_LANE, MPH, ONCOMING_LANE, ONCOMING_OUTER_LANE, RIGHT_LANE, STRAIGHT_THROUGH,
    TURN_LEFT, TURN_RIGHT, Scenario, bicycle, car, crosswalk_road, curved_two_lane_road, intersection,
    intersection_signals, pedestrian, truck, two_lane_road,
)

F1, F2 = "two_lane_road:F1", "two_lane_road:F2"
C1, C2 = "curved_road:F1", "curved_road:F2"
X2 = "crosswalk_road:F2"
X1 = "crosswalk_road:F1"


def _road(name, road, route, actors, *, start=30.0, v0=10.0, vd=10.0, limit=25 * MPH, goal=250.0, t=60.0,
          desc="", **kw):
    return Scenario(name=name, description=desc or name, road=road, route=route, ego_start_s_m=start,
                    ego_speed_mps=v0, desired_speed_mps=vd, speed_limit_mps=limit, goal_distance_m=goal,
                    time_limit_s=t, actors=tuple(actors), **kw)


def _busy(first_id, offset, speed, start, spacing, n):
    return [car(first_id + i, start + spacing * i, offset, speed) for i in range(n)]


# ---------------------------------------------------------------------------
# Following
# ---------------------------------------------------------------------------
def rb_stop_and_go():
    lead = car(1, 70.0, RIGHT_LANE, 10.0, speed_changes=[
        (4, 10), (7, 2), (10, 2), (13, 11), (18, 11), (21, 0), (25, 0), (29, 10), (34, 10), (36, 4), (40, 12)])
    return _road("rb_stop_and_go", two_lane_road(), (F2,), [lead] + _busy(100, LEFT_LANE, 11.0, -200, 12, 30),
                 limit=30 * MPH, vd=12.0, v0=11.0, goal=420.0, t=80.0,
                 desc="Lead car in stop-and-go waves, left lane full")


def rb_hard_brake_fast():
    lead = car(1, 70.0, RIGHT_LANE, 14.0, speed_changes=[(6, 14), (8.5, 0), (14, 0), (19, 12)])
    return _road("rb_hard_brake_fast", two_lane_road(), (F2,), [lead] + _busy(100, LEFT_LANE, 14.5, -260, 13, 30),
                 v0=14.0, vd=14.0, limit=35 * MPH, goal=400.0, t=70.0,
                 desc="Lead car at 14 m/s brakes at 5.6 m/s^2 to a stop; left lane full")


def rb_lead_brake_curve():
    lead = car(1, 65.0, RIGHT_LANE, 10.0, speed_changes=[(5, 10), (7, 0), (12, 0), (16, 10)])
    return _road("rb_lead_brake_curve", curved_two_lane_road(), (C2,),
                 [lead] + _busy(100, LEFT_LANE, 10.5, -200, 12, 25), goal=300.0, t=60.0,
                 desc="Lead car brakes to a stop on the S-curve; left lane full")


def rb_slow_both_lanes():
    return _road("rb_slow_both_lanes", two_lane_road(), (F2,),
                 [truck(1, 70.0, RIGHT_LANE, 5.0), truck(2, 75.0, LEFT_LANE, 5.5)], goal=200.0, t=70.0,
                 desc="Slow trucks side by side in both lanes: no way past, just follow")


def rb_gap_overtake():
    left = [car(100 + i, -30.0 - 20.0 * i - (25.0 if i >= 3 else 0.0), LEFT_LANE, 12.0) for i in range(6)]
    return _road("rb_gap_overtake", two_lane_road(), (F2,), [truck(1, 60.0, RIGHT_LANE, 4.0)] + left,
                 goal=350.0, t=70.0, desc="Slow truck; left lane cars with short gaps then one long gap")


# ---------------------------------------------------------------------------
# Cut-ins
# ---------------------------------------------------------------------------
def rb_cut_in_from_right():
    c = car(1, 36.0, RIGHT_LANE, 12.0, lane_changes=[(3.0, RIGHT_LANE), (5.0, LEFT_LANE)],
            speed_changes=[(6.0, 12.0), (8.0, 5.0), (12.0, 5.0), (15.0, 10.0)])
    return _road("rb_cut_in_from_right", two_lane_road(), (F1,), [c], goal=300.0, t=50.0,
                 desc="Ego in the left lane; a car from the right cuts in ahead and brakes")


def rb_double_cut_in():
    a = car(1, 30.0, LEFT_LANE, 13.0, lane_changes=[(4.0, LEFT_LANE), (6.0, RIGHT_LANE)],
            speed_changes=[(7.0, 13.0), (9.0, 7.0)])
    b = car(2, 10.0, LEFT_LANE, 14.0, lane_changes=[(9.0, LEFT_LANE), (11.0, RIGHT_LANE)],
            speed_changes=[(12.0, 14.0), (14.0, 7.0), (20.0, 7.0), (24.0, 11.0)])
    return _road("rb_double_cut_in", two_lane_road(), (F2,), [a, b], goal=320.0, t=60.0,
                 desc="Two cars cut in one after the other and slow down")


def rb_cut_in_stop():
    c = car(1, 40.0, LEFT_LANE, 12.0, lane_changes=[(4.0, LEFT_LANE), (6.0, RIGHT_LANE)],
            speed_changes=[(7.0, 12.0), (10.0, 0.0), (16.0, 0.0), (20.0, 9.0)])
    return _road("rb_cut_in_stop", two_lane_road(), (F2,), [c] + _busy(100, LEFT_LANE, 10.0, -150, 14, 15),
                 goal=280.0, t=60.0, desc="A car cuts in and brakes to a full stop; left lane busy")


# ---------------------------------------------------------------------------
# Obstacles
# ---------------------------------------------------------------------------
def rb_parked_row():
    parked = [car(1 + i, 90.0 + 30.0 * i, RIGHT_LANE, 0.0) for i in range(4)]
    left = [car(100 + i, -20.0 - 35.0 * i, LEFT_LANE, 11.0) for i in range(4)]
    return _road("rb_parked_row", two_lane_road(), (F2,), parked + left, goal=320.0, t=70.0,
                 desc="Row of parked cars in the ego lane, traffic in the left lane")


def rb_stalled_curve():
    return _road("rb_stalled_curve", curved_two_lane_road(), (C2,),
                 [car(1, 120.0, RIGHT_LANE, 0.0), car(100, -30.0, LEFT_LANE, 12.0)], goal=300.0, t=60.0,
                 desc="Stalled car in the ego lane on the S-curve")


def rb_appearing_car():
    return _road("rb_appearing_car", two_lane_road(), (F2,),
                 [car(1, 108.0, RIGHT_LANE, 0.0, appear_at_s=4.0), car(100, -40.0, LEFT_LANE, 11.0)],
                 goal=250.0, t=55.0, desc="A stopped car appears ~33 m ahead at 10 m/s (fair: 27 m)")


def rb_debris_left_while_passing():
    return _road("rb_debris_left_while_passing", two_lane_road(), (F2,),
                 [truck(1, 60.0, RIGHT_LANE, 4.0), car(2, 160.0, LEFT_LANE, 0.0, length_m=1.5, width_m=1.5,
                                                          kind="truck")],
                 goal=320.0, t=70.0, desc="Overtaking a slow truck while debris lies in the left lane ahead")


def rb_weave():
    return _road("rb_weave", two_lane_road(), (F2,),
                 [car(1, 100.0, RIGHT_LANE, 0.0), car(2, 150.0, LEFT_LANE, 0.0), car(3, 210.0, RIGHT_LANE, 0.0)],
                 goal=320.0, t=70.0, desc="Stopped cars alternate between lanes: weave through")


def rb_start_wrong_lane():
    return _road("rb_start_wrong_lane", two_lane_road(), (F2,),
                 [car(1, 80.0, LEFT_LANE, 0.0), car(100, -20.0, RIGHT_LANE, 12.0)], goal=250.0, t=60.0,
                 ego_start_lane=F1, desc="Ego starts in the left lane, which is blocked ahead; merge right behind a car")


def rb_wrong_way_in_lane():
    return _road("rb_wrong_way_in_lane", two_lane_road(), (F2,),
                 [car(1, 330.0, RIGHT_LANE, 9.0, oncoming=True, lane_changes=[(22.0, RIGHT_LANE), (25.0, 9.0)])],
                 goal=300.0, t=60.0, desc="Wrong-way car comes toward the ego in its lane; left lane is free")


# ---------------------------------------------------------------------------
# Vulnerable road users
# ---------------------------------------------------------------------------
def rb_jaywalker():
    p = pedestrian(1, 110.0, 8.0, walk_toward="left", speed_mps=0.0, speed_changes=[(3.5, 0.0), (4.0, 1.5)])
    return _road("rb_jaywalker", two_lane_road(), (F2,), [p], goal=250.0, t=50.0,
                 desc="Pedestrian crosses mid-block, no crosswalk")


def rb_ped_hesitates():
    p = pedestrian(1, 150.0, 8.95, walk_toward="left", speed_mps=0.0,
                   speed_changes=[(5.0, 0.0), (5.5, 1.3), (7.0, 1.3), (7.3, 0.0), (10.0, 0.0), (10.3, 1.3)])
    return _road("rb_ped_hesitates", crosswalk_road(), (X2,), [p], start=40.0, goal=220.0, t=50.0, vd=10.0,
                 desc="Pedestrian starts crossing, stops in the road, then continues")


def rb_ped_along_edge():
    return _road("rb_ped_along_edge", two_lane_road(), (F2,),
                 [pedestrian(1, 120.0, 6.6, walk_toward="along", speed_mps=1.2), car(100, -60.0, LEFT_LANE, 11.0)],
                 goal=250.0, t=50.0, desc="Pedestrian walks along the right edge of the ego lane")


def rb_cyclist_35mph():
    left = [car(100 + i, -40.0 - 30.0 * i, LEFT_LANE, 15.0) for i in range(5)]
    return _road("rb_cyclist_35mph", two_lane_road(), (F2,), [bicycle(1, 90.0, RIGHT_LANE + 0.8, 5.0)] + left,
                 v0=13.0, vd=14.0, limit=35 * MPH, goal=400.0, t=60.0,
                 desc="Cyclist on a 35 mph road with fast cars passing on the left")


def rb_crosswalk_group():
    peds = [pedestrian(1, 149.0, 8.95, "left", 0.0, speed_changes=[(6.0, 0.0), (6.3, 1.4)]),
            pedestrian(2, 151.0, -8.95, "right", 0.0, speed_changes=[(7.5, 0.0), (7.8, 1.2)]),
            pedestrian(3, 150.0, 8.95, "left", 0.0, speed_changes=[(11.0, 0.0), (11.3, 1.6)])]
    return _road("rb_crosswalk_group", crosswalk_road(), (X2,), peds, start=40.0, goal=230.0, t=60.0,
                 desc="Several pedestrians cross from both sides at staggered times")


def rb_crosswalk_left_lane():
    p = pedestrian(1, 150.0, -8.95, "right", 0.0, speed_changes=[(5.0, 0.0), (5.3, 1.3)])
    return _road("rb_crosswalk_left_lane", crosswalk_road(), (X1,), [p], start=40.0, goal=220.0, t=50.0,
                 desc="Ego in the left lane; pedestrian crosses from the far (left) sidewalk")


def rb_occluded_left():
    return _road("rb_occluded_left", two_lane_road(), (F2,),
                 [truck(1, 125.0, LEFT_LANE, 0.0),
                  pedestrian(2, 131.0, 2.6, "right", 2.0, appear_at_s=6.2)],
                 goal=250.0, t=50.0, desc="Pedestrian runs out from in front of a truck parked in the LEFT lane")


def rb_child_runs():
    return _road("rb_child_runs", two_lane_road(), (F2,),
                 [car(1, 95.0, 7.6, 0.0, width_m=1.8), pedestrian(2, 100.0, 7.4, "left", 2.5, appear_at_s=5.0)],
                 v0=8.0, vd=8.0, goal=200.0, t=45.0, desc="A child runs out between parked cars at the kerb")


# ---------------------------------------------------------------------------
# Junctions
# ---------------------------------------------------------------------------
def rb_left_turn_stream():
    onc = [car(1 + i, 40.0 - 22.0 * i, INTERSECTION_LANE, 7.0, road="A2") for i in range(4)]
    return _road("rb_left_turn_stream", intersection(), TURN_LEFT, onc, start=60.0, v0=8.0, vd=8.0, goal=None,
                 t=60.0, desc="Uncontrolled left turn across a stream of oncoming cars")


def rb_red_light_runner():
    runner = car(1, 60.0, INTERSECTION_LANE, 11.0, road="A3")
    return _road("rb_red_light_runner", intersection("signals"), STRAIGHT_THROUGH, [runner], start=60.0, v0=8.0,
                 vd=8.0, goal=None, t=50.0,
                 signal_phases=intersection_signals([(0.0, "GREEN")], [(0.0, "RED")]),
                 desc="Green light, but a car from the right runs its red")


def rb_signal_left_turn():
    onc = [car(1 + i, 30.0 - 25.0 * i, INTERSECTION_LANE, 8.0, road="A2") for i in range(3)]
    return _road("rb_signal_left_turn", intersection("signals"), TURN_LEFT, onc, start=60.0, v0=8.0, vd=8.0,
                 goal=None, t=60.0, desc="Green light, left turn, oncoming cars go straight (no arrow)")


def rb_stop_sign_cross():
    cars = [car(1, 80.0, INTERSECTION_LANE, 7.0, road="A1", speed_changes=[(6.0, 7.0), (9.0, 0.0), (11.0, 0.0),
                                                                         (14.0, 7.0)]),
            car(2, 40.0, INTERSECTION_LANE, 9.0, road="A3")]
    return _road("rb_stop_sign_cross", intersection("stop_signs"), STRAIGHT_THROUGH, cars, start=60.0, v0=8.0,
                 vd=8.0, goal=None, t=50.0, desc="All-way stop, straight; cross traffic from both sides")


def rb_yellow_close():
    return _road("rb_yellow_close", intersection("signals"), STRAIGHT_THROUGH, [], start=110.0, v0=8.0, vd=8.0,
                 goal=None, t=45.0,
                 signal_phases=intersection_signals([(0.0, "GREEN"), (1.2, "YELLOW"), (4.2, "RED")],
                                                    [(0.0, "RED"), (6.0, "GREEN")]),
                 desc="Light turns yellow when the ego is too close to stop comfortably: go through")


def rb_red_queue():
    lead = car(1, 100.0, INTERSECTION_LANE, 7.0, road="A0",
               speed_changes=[(1.0, 7.0), (4.5, 0.0), (15.0, 0.0), (18.0, 7.0)])
    return _road("rb_red_queue", intersection("signals"), STRAIGHT_THROUGH, [lead], start=60.0, v0=8.0, vd=8.0,
                 goal=None, t=50.0,
                 signal_phases=intersection_signals([(0.0, "RED"), (14.0, "GREEN")], [(0.0, "GREEN"), (11.0, "YELLOW"),
                                                                                     (14.0, "RED")]),
                 desc="Queue behind a car waiting at a red light")


def rb_fast_cross_right():
    return _road("rb_fast_cross_right", intersection(), STRAIGHT_THROUGH,
                 [car(1, 30.0, INTERSECTION_LANE, 12.0, road="A3"), car(2, -10.0, INTERSECTION_LANE, 12.0, road="A1")],
                 start=60.0, v0=8.0, vd=8.0, goal=None, t=50.0,
                 desc="Uncontrolled junction: fast cars from the right and the left")


def rb_right_turn_ped():
    return _road("rb_right_turn_ped", intersection(), TURN_RIGHT,
                 [pedestrian(1, 135.0, 5.0, "left", 0.0, road="A3", speed_changes=[(6.0, 0.0), (6.3, 1.3)])],
                 start=60.0, v0=8.0, vd=8.0, goal=None, t=50.0,
                 desc="Right turn; a pedestrian crosses the road the ego turns into")


def rb_stop_sign_right_turn():
    return _road("rb_stop_sign_right_turn", intersection("stop_signs"), TURN_RIGHT,
                 [car(1, 70.0, INTERSECTION_LANE, 8.0, road="A1", speed_changes=[(7.0, 8.0), (10.0, 0.0),
                                                                                  (12.0, 0.0), (15.0, 8.0)])],
                 start=60.0, v0=8.0, vd=8.0, goal=None, t=50.0,
                 desc="All-way stop, right turn, car from the left")


# ---------------------------------------------------------------------------
# Higher speed and combinations
# ---------------------------------------------------------------------------
def rb_35mph_overtake():
    left = [car(100 + i, -60.0 - 18.0 * i, LEFT_LANE, 15.5) for i in range(3)]
    return _road("rb_35mph_overtake", two_lane_road(), (F2,),
                 [truck(1, 90.0, RIGHT_LANE, 8.0), truck(2, 300.0, RIGHT_LANE, 7.0)] + left,
                 v0=14.0, vd=15.0, limit=35 * MPH, goal=550.0, t=70.0,
                 desc="35 mph road: two slow trucks to overtake, fast cars in the left lane")


def rb_standing_start_signal():
    return _road("rb_standing_start_signal", intersection("signals"), STRAIGHT_THROUGH,
                 [car(1, 60.0, INTERSECTION_LANE, 8.0, road="A1", speed_changes=[(2.0, 8.0), (3.5, 0.0)])],
                 start=125.0, v0=0.0, vd=8.0, goal=None, t=45.0,
                 signal_phases=intersection_signals([(0.0, "RED"), (3.0, "GREEN")], [(0.0, "GREEN"), (1.0, "YELLOW"),
                                                                                    (3.0, "RED")]),
                 desc="Standing start at a red light that turns green")


def rb_curve_cut_in_ped():
    c = car(1, 30.0, LEFT_LANE, 12.0, lane_changes=[(4.0, LEFT_LANE), (6.0, RIGHT_LANE)],
            speed_changes=[(7.0, 12.0), (9.0, 6.0), (15.0, 6.0), (18.0, 10.0)])
    p = pedestrian(2, 230.0, 8.0, "left", 0.0, speed_changes=[(18.0, 0.0), (18.5, 1.4)])
    return _road("rb_curve_cut_in_ped", curved_two_lane_road(), (C2,), [c, p], goal=320.0, t=70.0,
                 desc="S-curve: a car cuts in and slows, later a pedestrian crosses")


def rb_crash_then_crosswalk():
    a = car(1, 70.0, RIGHT_LANE, 9.0, speed_changes=[(5.0, 9.0), (6.0, 0.0)])
    p = pedestrian(2, 150.0, -8.95, "right", 0.0, speed_changes=[(12.0, 0.0), (12.3, 1.3)])
    left = [car(100 + i, -40.0 - 40.0 * i, LEFT_LANE, 11.0) for i in range(3)]
    return _road("rb_crash_then_crosswalk", crosswalk_road(), (X2,), [a, p] + left, start=20.0, goal=300.0,
                 t=70.0, desc="A car ahead stops abruptly (breakdown), then a pedestrian at the crosswalk")


def rb_oncoming_overtaker():
    o = car(1, 300.0, ONCOMING_LANE, 13.0, oncoming=True,
            lane_changes=[(5.0, ONCOMING_LANE), (7.0, LEFT_LANE), (12.0, LEFT_LANE), (14.0, ONCOMING_LANE)])
    slow = car(2, 330.0, ONCOMING_OUTER_LANE, 6.0, oncoming=True)
    return _road("rb_oncoming_overtaker", two_lane_road(), (F2,), [o, slow, truck(3, 100.0, RIGHT_LANE, 6.0)],
                 goal=320.0, t=60.0, desc="Oncoming car swings into our left lane to overtake while we follow a truck")


ROBUSTNESS_SCENARIOS = {f.__name__: f for f in (
    rb_stop_and_go, rb_hard_brake_fast, rb_lead_brake_curve, rb_slow_both_lanes, rb_gap_overtake,
    rb_cut_in_from_right, rb_double_cut_in, rb_cut_in_stop,
    rb_parked_row, rb_stalled_curve, rb_appearing_car, rb_debris_left_while_passing, rb_weave,
    rb_start_wrong_lane, rb_wrong_way_in_lane,
    rb_jaywalker, rb_ped_hesitates, rb_ped_along_edge, rb_cyclist_35mph, rb_crosswalk_group,
    rb_crosswalk_left_lane, rb_occluded_left, rb_child_runs,
    rb_left_turn_stream, rb_red_light_runner, rb_signal_left_turn, rb_stop_sign_cross, rb_yellow_close,
    rb_red_queue, rb_fast_cross_right, rb_right_turn_ped, rb_stop_sign_right_turn,
    rb_35mph_overtake, rb_standing_start_signal, rb_curve_cut_in_ped, rb_crash_then_crosswalk,
    rb_oncoming_overtaker,
)}
