"""Team members: Anthony Pollard, Cole Pulaski, Will Cunningham, Brandon Debrosse

Hold-out scenes: written AFTER the planner was tuned on the rb_ and pg_
scenes, to check that the fixes carry over to situations the planner was not
tuned on.  They follow the themes of the leaderboard's scene teams (school
bus, police stop, construction, jaywalkers, busy junctions, rush hour,
parking, yielding, moving over).  Names start with "ho_".
"""

from __future__ import annotations

from scenarios.scenario_tools import (
    INTERSECTION_LANE, LEFT_LANE, MPH, ONCOMING_LANE, ONCOMING_OUTER_LANE, RIGHT_LANE, STRAIGHT_THROUGH,
    TURN_LEFT, TURN_RIGHT, Scenario, bicycle, car, crosswalk_road, curved_two_lane_road, intersection,
    intersection_signals, pedestrian, truck, two_lane_road,
)

F1, F2 = "two_lane_road:F1", "two_lane_road:F2"
C1, C2 = "curved_road:F1", "curved_road:F2"
X1, X2 = "crosswalk_road:F1", "crosswalk_road:F2"


def _sc(name, road, route, actors, *, start=30.0, v0=10.0, vd=10.0, limit=25 * MPH, goal=250.0, t=60.0,
        desc="", **kw):
    return Scenario(name=name, description=desc or name, road=road, route=route, ego_start_s_m=start,
                    ego_speed_mps=v0, desired_speed_mps=vd, speed_limit_mps=limit, goal_distance_m=goal,
                    time_limit_s=t, actors=tuple(actors), **kw)


def _cone(i, s, offset):
    return car(i, s, offset, 0.0, length_m=0.6, width_m=0.6, kind="truck")


def ho_school_bus_ahead():
    bus = car(1, 80.0, RIGHT_LANE, 8.0, length_m=12.0, width_m=2.5, kind="truck",
              speed_changes=[(3.0, 8.0), (6.0, 0.0), (16.0, 0.0), (20.0, 8.0), (30.0, 8.0), (33.0, 0.0),
                             (40.0, 0.0), (44.0, 8.0)])
    kids = [pedestrian(10 + i, 96.0 + 1.5 * i, 8.0, "left", 0.0, appear_at_s=7.0,
                       speed_changes=[(8.0 + 1.2 * i, 0.0), (8.3 + 1.2 * i, 1.5), (12.5 + 1.2 * i, 0.0)])
            for i in range(3)]
    return _sc("ho_school_bus_ahead", two_lane_road(), (F2,), [bus] + kids + [car(30, -50.0, LEFT_LANE, 11.0)],
               goal=330.0, t=90.0, desc="School bus stops twice; children cross in front of it")


def ho_school_bus_oncoming():
    bus = car(1, 260.0, ONCOMING_LANE, 8.0, oncoming=True, length_m=12.0, width_m=2.5, kind="truck",
              speed_changes=[(4.0, 8.0), (7.0, 0.0), (18.0, 0.0), (21.0, 8.0)])
    kids = [pedestrian(10 + i, 198.0 - 2.0 * i, -6.5, "right", 0.0, appear_at_s=8.0,
                       speed_changes=[(9.0 + 1.5 * i, 0.0), (9.3 + 1.5 * i, 1.4), (19.0 + 1.5 * i, 1.4)])
            for i in range(2)]
    return _sc("ho_school_bus_oncoming", two_lane_road(), (F2,), [bus] + kids, goal=320.0, t=70.0,
               desc="Oncoming school bus stops; children cross all lanes in front of the ego")


def ho_traffic_stop_curve():
    return _sc("ho_traffic_stop_curve", curved_two_lane_road(), (C2,),
               [car(1, 140.0, RIGHT_LANE + 0.6, 0.0), car(2, 150.0, RIGHT_LANE + 0.6, 0.0),
                pedestrian(3, 151.0, 3.9, "along", 0.0)] + [car(20 + i, -10.0 - 30.0 * i, LEFT_LANE, 10.5)
                                                           for i in range(3)],
               goal=300.0, t=70.0, desc="Police traffic stop on the S-curve, officer at the driver door")


def ho_construction_chicane():
    cones = [_cone(1 + i, 110.0 + 2.0 * i, RIGHT_LANE + (0.0 if i % 2 else 0.8)) for i in range(15)]
    cones += [_cone(30 + i, 170.0 + 2.0 * i, LEFT_LANE) for i in range(12)]
    worker = pedestrian(50, 125.0, 6.0, "along", 0.5, speed_changes=[(10.0, 0.5), (11.0, 0.0)])
    return _sc("ho_construction_chicane", two_lane_road(), (F2,),
               cones + [worker, car(60, -40.0, LEFT_LANE, 11.0), car(61, 330.0, ONCOMING_LANE, 9.0, oncoming=True)],
               goal=300.0, t=70.0, desc="Cones close the right lane, then the left lane: weave through")


def ho_jaywalkers_group():
    peds = [pedestrian(1, 120.0, 8.0, "left", 0.0, speed_changes=[(4.0, 0.0), (4.3, 1.4)]),
            pedestrian(2, 124.0, -6.0, "right", 0.0, speed_changes=[(5.0, 0.0), (5.3, 1.8)]),
            pedestrian(3, 190.0, 8.0, "left", 0.0, speed_changes=[(11.0, 0.0), (11.3, 2.2), (13.0, 0.0),
                                                                  (15.0, 0.0), (15.3, 2.2)])]
    return _sc("ho_jaywalkers_group", two_lane_road(), (F2,), peds + [car(20, -60.0, LEFT_LANE, 11.0)],
               goal=280.0, t=60.0, desc="Several jaywalkers from both sides; one stops in the road and goes on")


def ho_busy_signal_left():
    onc = [car(1 + i, 60.0 - 18.0 * i, INTERSECTION_LANE, 9.0, road="A2") for i in range(4)]
    cross = [car(20 + i, 30.0 - 20.0 * i, INTERSECTION_LANE, 9.0, road="A3",
                 speed_changes=[(9.0 - 1.0 * i, 9.0), (12.0 - 1.0 * i, 0.0)]) for i in range(3)]
    walker = pedestrian(40, 140.0, -5.0, "right", 0.0, road="A1", speed_changes=[(14.0, 0.0), (14.3, 1.3)])
    return _sc("ho_busy_signal_left", intersection("signals"), TURN_LEFT, onc + cross + [walker], start=70.0,
               v0=8.0, vd=8.0, goal=None, t=70.0,
               signal_phases=intersection_signals([(0.0, "RED"), (12.0, "GREEN")],
                                                  [(0.0, "GREEN"), (9.0, "YELLOW"), (12.0, "RED")]),
               desc="Busy signalised junction: left turn after red, oncoming stream, pedestrian on the exit road")


def ho_rush_curve():
    lead = [car(1 + i, 55.0 + 14.0 * i, RIGHT_LANE, 7.0,
                speed_changes=[(3.0 + i, 7.0), (6.0 + i, 0.0), (9.0 + i, 0.0), (13.0 + i, 7.0),
                               (22.0 + i, 7.0), (25.0 + i, 1.0), (28.0 + i, 7.0)]) for i in range(3)]
    left = [car(20 + i, -70.0 + 13.0 * i, LEFT_LANE, 8.0) for i in range(10)]
    return _sc("ho_rush_curve", curved_two_lane_road(), (C2,), lead + left, v0=7.0, goal=300.0, t=80.0,
               desc="Rush hour on the S-curve: stop-and-go queue, full left lane")


def ho_rush_peds_signal():
    peds = [pedestrian(10 + i, 128.0 + 1.0 * i, 6.0, "along", 0.0) for i in range(3)]
    return _sc("ho_rush_peds_signal", intersection("signals"), STRAIGHT_THROUGH,
               [car(1, 105.0, INTERSECTION_LANE, 6.0, road="A0", speed_changes=[(1.0, 6.0), (4.0, 0.0),
                                                                                 (12.0, 0.0), (15.0, 7.0)])]
               + peds, start=50.0, v0=7.0, vd=8.0, goal=None, t=60.0,
               signal_phases=intersection_signals([(0.0, "RED"), (11.0, "GREEN")], [(0.0, "GREEN"), (8.0, "YELLOW"),
                                                                                   (11.0, "RED")]),
               desc="Downtown corner: people waiting on the corner, queue at a red light")


def ho_park_reverse_35():
    parker = car(1, 90.0, RIGHT_LANE, 12.0, speed_changes=[(2.0, 12.0), (6.0, 0.0), (7.0, 0.0), (8.0, -2.0),
                                                           (11.0, -2.0), (12.0, 0.0)],
                 lane_changes=[(10.0, RIGHT_LANE), (12.0, 7.5)])
    return _sc("ho_park_reverse_35", two_lane_road(), (F2,),
               [parker, car(10, 70.0, 7.5, 0.0)] + [car(20 + i, -30.0 - 40.0 * i, LEFT_LANE, 14.5) for i in range(3)],
               v0=13.0, vd=14.0, limit=35 * MPH, goal=350.0, t=60.0,
               desc="35 mph: the car ahead brakes and reverses toward a parking spot")


def ho_park_pull_out_close():
    out = car(1, 75.0, 7.5, 0.0, speed_changes=[(1.5, 0.0), (3.0, 3.0), (6.0, 9.0)],
              lane_changes=[(1.5, 7.5), (4.0, RIGHT_LANE)])
    return _sc("ho_park_pull_out_close", two_lane_road(), (F2,),
               [out, car(2, 60.0, 7.5, 0.0), car(3, 90.0, 7.5, 0.0), car(20, 0.0, LEFT_LANE, 10.0)],
               goal=260.0, t=60.0, desc="A parked car pulls out ~35 m ahead while a car is beside us")


def ho_yield_right_both():
    a1 = [car(1 + i, 120.0 - g, INTERSECTION_LANE, 10.0, road="A1") for i, g in enumerate([0, 15, 30, 80, 95])]
    a3 = [car(20 + i, 110.0 - g, INTERSECTION_LANE, 10.0, road="A3") for i, g in enumerate([0, 20, 40, 60])]
    return _sc("ho_yield_right_both", intersection(), TURN_RIGHT, a1 + a3, start=60.0, v0=8.0, vd=8.0, goal=None,
               t=70.0, desc="Right turn onto a busy road with traffic from both sides")


def ho_yield_straight_fast():
    return _sc("ho_yield_straight_fast", intersection(), STRAIGHT_THROUGH,
               [car(1, 40.0, INTERSECTION_LANE, 14.0, road="A1"), car(2, 0.0, INTERSECTION_LANE, 14.0, road="A1"),
                car(3, 50.0, INTERSECTION_LANE, 13.0, road="A3"), car(4, -30.0, INTERSECTION_LANE, 13.0, road="A3")],
               start=50.0, v0=8.0, vd=8.0, goal=None, t=60.0,
               desc="Uncontrolled junction with fast traffic (14 m/s) from both sides")


def ho_police_behind_35():
    return _sc("ho_police_behind_35", two_lane_road(), (F2,),
               [car(1, -45.0, RIGHT_LANE, 20.0), car(20, 20.0, LEFT_LANE, 14.0), car(21, -60.0, LEFT_LANE, 14.0)],
               v0=14.0, vd=15.0, limit=35 * MPH, goal=400.0, t=60.0,
               desc="35 mph: police at 20 m/s from behind, cars in the left lane")


def ho_police_shoulder_curve():
    return _sc("ho_police_shoulder_curve", curved_two_lane_road(), (C2,),
               [car(1, 160.0, 6.7, 0.0), car(2, 170.0, 6.7, 0.0)] + [car(20 + i, -20.0 - 25.0 * i, LEFT_LANE, 10.0)
                                                                     for i in range(4)],
               goal=300.0, t=60.0, desc="Police on the shoulder of a curve, sticking into the lane; busy left lane")


def ho_tailgater_red():
    behind = car(1, 40.0, INTERSECTION_LANE, 9.0, road="A0",
                 speed_changes=[(6.0, 9.0), (9.5, 0.0), (20.0, 0.0), (23.0, 8.0)])
    return _sc("ho_tailgater_red", intersection("signals"), STRAIGHT_THROUGH, [behind], start=60.0, v0=8.0,
               vd=8.0, goal=None, t=60.0,
               signal_phases=intersection_signals([(0.0, "GREEN"), (5.0, "YELLOW"), (8.0, "RED"), (18.0, "GREEN")],
                                                  [(0.0, "RED"), (9.0, "GREEN"), (16.0, "YELLOW"), (18.0, "RED")]),
               desc="Light turns red with a car following close behind (it stops late)")


def ho_bike_swerve():
    bike = bicycle(1, 80.0, RIGHT_LANE + 1.0, 5.0, lane_changes=[(6.0, RIGHT_LANE + 1.0), (7.5, RIGHT_LANE - 0.3),
                                                                (10.0, RIGHT_LANE - 0.3), (11.5, RIGHT_LANE + 1.0)])
    return _sc("ho_bike_swerve", two_lane_road(), (F2,), [bike] + [car(20 + i, -40.0 - 22.0 * i, LEFT_LANE, 11.0)
                                                              for i in range(5)],
               goal=280.0, t=60.0, desc="Cyclist swerves into the middle of the lane around a pothole")


def ho_wrong_way_35():
    return _sc("ho_wrong_way_35", two_lane_road(), (F1,),
               [car(1, 420.0, LEFT_LANE, 12.0, oncoming=True), car(20, 40.0, RIGHT_LANE, 12.0)],
               v0=14.0, vd=15.0, limit=35 * MPH, goal=380.0, t=60.0,
               desc="35 mph, ego in the left lane: a wrong-way car comes down it; a car beside us on the right")


HOLDOUT_SCENARIOS = {f.__name__: f for f in (
    ho_school_bus_ahead, ho_school_bus_oncoming, ho_traffic_stop_curve, ho_construction_chicane,
    ho_jaywalkers_group, ho_busy_signal_left, ho_rush_curve, ho_rush_peds_signal, ho_park_reverse_35,
    ho_park_pull_out_close, ho_yield_right_both, ho_yield_straight_fast, ho_police_behind_35,
    ho_police_shoulder_curve, ho_tailgater_red, ho_bike_swerve, ho_wrong_way_35,
)}
