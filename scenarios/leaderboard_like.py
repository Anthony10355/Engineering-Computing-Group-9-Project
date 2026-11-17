"""Team members: Anthony Pollard, Cole Pulaski, Will Cunningham, Brandon Debrosse

Our own versions of the leaderboard scenes we could not see (team 5's
"downtown rush hour", "parallel parking", "yield busy road" and "police move
over").  We only know their names, so each one is built in several ways that
fit the name.  Names start with "pg_".

Things these scenes exercise that the others do not:
  * scripted road users never brake, so a fast vehicle coming from BEHIND in
    the ego lane will drive into the ego unless the ego moves over;
  * a car can drive backwards (negative speed), e.g. when parallel parking;
  * pedestrians standing on the sidewalk that never cross;
  * parked cars that stick out into the ego lane.
"""

from __future__ import annotations

from scenarios.scenario_tools import (
    INTERSECTION_LANE, LEFT_LANE, MPH, ONCOMING_LANE, RIGHT_LANE, STRAIGHT_THROUGH, TURN_LEFT, TURN_RIGHT,
    Scenario, car, crosswalk_road, intersection, intersection_signals, pedestrian, truck, two_lane_road,
)

F1, F2 = "two_lane_road:F1", "two_lane_road:F2"
X1, X2 = "crosswalk_road:F1", "crosswalk_road:F2"
KERB = 7.5                     # a parked car centred here sticks 0.45 m into the right lane (lane edge 7.0 m)


def _sc(name, road, route, actors, *, start=30.0, v0=10.0, vd=10.0, limit=25 * MPH, goal=250.0, t=60.0,
        desc="", **kw):
    return Scenario(name=name, description=desc or name, road=road, route=route, ego_start_s_m=start,
                    ego_speed_mps=v0, desired_speed_mps=vd, speed_limit_mps=limit, goal_distance_m=goal,
                    time_limit_s=t, actors=tuple(actors), **kw)


def _wave(t0, period, v_hi, v_lo, n):
    """speed keyframes for stop-and-go waves"""
    out, t = [], t0
    for _ in range(n):
        out += [(t, v_hi), (t + 2.5, v_lo), (t + 2.5 + period * 0.3, v_lo), (t + 2.5 + period * 0.3 + 3.0, v_hi)]
        t += period
    return out


# ---------------------------------------------------------------------------
# Downtown rush hour
# ---------------------------------------------------------------------------
def pg_rush_sidewalk():
    """Slow dense traffic, people standing on both sidewalks who never cross,
    some who do cross at the crosswalk."""
    lead = [car(1 + i, 50.0 + 14.0 * i, RIGHT_LANE, 6.0, speed_changes=_wave(3.0 + 1.0 * i, 14.0, 6.0, 0.0, 5))
            for i in range(3)]
    left = [car(20 + i, -60.0 + 12.0 * i, LEFT_LANE, 6.5, speed_changes=_wave(5.0 + 0.8 * i, 15.0, 6.5, 1.0, 5))
            for i in range(10)]
    standing = [pedestrian(40 + i, s, side, "left" if side > 0 else "right", 0.0)
                for i, (s, side) in enumerate([(120.0, 8.95), (146.0, 8.95), (155.0, -8.95), (190.0, 8.95),
                                               (230.0, 8.6), (260.0, -8.95)])]
    crossing = [pedestrian(60, 150.0, 8.95, "left", 0.0, speed_changes=[(14.0, 0.0), (14.3, 1.3)])]
    return _sc("pg_rush_sidewalk", crosswalk_road(), (X2,), lead + left + standing + crossing, start=20.0,
               v0=6.0, vd=10.0, goal=300.0, t=90.0, desc="Rush hour: stop-and-go traffic, people waiting on sidewalks")


def pg_rush_cut_ins():
    """Both lanes in stop-and-go waves; cars squeeze from one lane into the other."""
    right = [car(1 + i, 45.0 + 15.0 * i, RIGHT_LANE, 7.0, speed_changes=_wave(2.0 + 0.7 * i, 12.0, 7.0, 0.5, 6))
             for i in range(4)]
    left = [car(20 + i, -40.0 + 13.0 * i, LEFT_LANE, 7.5, speed_changes=_wave(4.0 + 0.7 * i, 13.0, 7.5, 1.0, 6))
            for i in range(9)]
    cutter = car(50, 40.0, LEFT_LANE, 9.0, lane_changes=[(3.0, LEFT_LANE), (5.0, RIGHT_LANE)],
                 speed_changes=[(5.0, 9.0), (7.0, 6.0)] + _wave(12.0, 12.0, 7.0, 0.5, 4))
    return _sc("pg_rush_cut_ins", two_lane_road(), (F2,), right + left + [cutter], v0=7.0, vd=10.0, goal=300.0,
               t=90.0, desc="Rush hour: both lanes in stop-and-go waves with a car cutting in")


def pg_rush_signal():
    """Queue at a red light downtown; cross traffic and a pedestrian crossing the exit road."""
    queue = [car(1 + i, 110.0 - 9.0 * i, INTERSECTION_LANE, 6.0, road="A0",
                 speed_changes=[(0.5 + 0.4 * i, 6.0), (3.5 + 0.5 * i, 0.0), (16.0 + 1.2 * i, 0.0),
                                (19.0 + 1.2 * i, 7.0)]) for i in range(3)]
    cross = [car(20 + i, 40.0 - 20.0 * i, INTERSECTION_LANE, 8.0, road="A1",
                 speed_changes=[(12.0 - 1.5 * i, 8.0), (14.5 - 1.5 * i, 0.0)]) for i in range(4)]
    walker = pedestrian(40, 166.0, -5.0, "right", 0.0, road="A2", speed_changes=[(18.0, 0.0), (18.3, 1.3)])
    return _sc("pg_rush_signal", intersection("signals"), STRAIGHT_THROUGH, queue + cross + [walker], start=45.0,
               v0=7.0, vd=8.0, goal=None, t=70.0,
               signal_phases=intersection_signals([(0.0, "RED"), (15.0, "GREEN")],
                                                  [(0.0, "GREEN"), (12.0, "YELLOW"), (15.0, "RED")]),
               desc="Rush hour: queue at a red light, cross traffic, pedestrian on the far side")


def pg_rush_walkers_along():
    """Crowded sidewalks: people walking along both sidewalks, a jaywalker, slow traffic."""
    along = [pedestrian(40 + i, 60.0 + 25.0 * i, 8.95 if i % 2 else -8.95, "along", 1.3) for i in range(8)]
    jay = pedestrian(60, 220.0, 8.95, "left", 0.0, speed_changes=[(15.0, 0.0), (15.3, 1.5)])
    slow = [car(1, 70.0, RIGHT_LANE, 6.0), car(2, 200.0, LEFT_LANE, 6.0)] + \
           [car(20 + i, -50.0 - 25.0 * i, LEFT_LANE, 9.0) for i in range(4)]
    return _sc("pg_rush_walkers_along", crosswalk_road(), (X2,), along + [jay] + slow, start=20.0, goal=320.0,
               t=80.0, desc="Rush hour: crowded sidewalks, a jaywalker and slow cars")


# ---------------------------------------------------------------------------
# Parallel parking
# ---------------------------------------------------------------------------
def _parked_row(first_id, s_list, offset=KERB):
    return [car(first_id + i, s, offset, 0.0) for i, s in enumerate(s_list)]


def pg_park_reverse_in():
    """A car stops just past a gap at the kerb and reverses into it."""
    parker = car(1, 70.0, RIGHT_LANE, 7.0, speed_changes=[(3.0, 7.0), (6.0, 0.0), (7.0, 0.0), (8.0, -1.5),
                                                          (11.5, -1.5), (12.5, 0.0)],
                 lane_changes=[(8.0, RIGHT_LANE), (12.5, KERB)])
    return _sc("pg_park_reverse_in", two_lane_road(), (F2,),
               [parker] + _parked_row(10, [70.0, 100.0, 108.0]) + [car(30, -40.0, LEFT_LANE, 11.0),
                                                                    car(31, -90.0, LEFT_LANE, 11.0)],
               goal=260.0, t=60.0, desc="Parallel parking: the car ahead stops and reverses into a gap at the kerb")


def pg_park_reverse_long():
    """Same, but the parker backs up a long way straight back in the lane first."""
    parker = car(1, 75.0, RIGHT_LANE, 8.0, speed_changes=[(3.0, 8.0), (5.0, 0.0), (6.0, 0.0), (7.0, -2.0),
                                                          (10.0, -2.0), (11.0, -1.0), (14.0, -1.0), (15.0, 0.0)],
                 lane_changes=[(10.0, RIGHT_LANE), (15.0, KERB)])
    return _sc("pg_park_reverse_long", two_lane_road(), (F2,),
               [parker] + _parked_row(10, [62.0, 92.0]) + [car(30, -20.0, LEFT_LANE, 10.0),
                                                           car(31, -70.0, LEFT_LANE, 10.0)],
               goal=260.0, t=60.0, desc="Parallel parking: the car ahead backs up 8 m, then into the spot")


def pg_park_pull_out():
    """A parked car pulls out of its spot into the ego lane."""
    out = car(1, 100.0, KERB, 0.0, speed_changes=[(3.5, 0.0), (5.0, 4.0), (9.0, 8.0)],
              lane_changes=[(3.5, KERB), (6.5, RIGHT_LANE)])
    return _sc("pg_park_pull_out", two_lane_road(), (F2,),
               [out] + _parked_row(10, [85.0, 115.0, 140.0]) + [car(30, 10.0, LEFT_LANE, 10.0)],
               goal=260.0, t=60.0, desc="Parallel parking: a parked car pulls out into the ego lane")


def pg_park_kerb_row():
    """A long row of parked cars sticking slightly into the ego lane."""
    return _sc("pg_park_kerb_row", two_lane_road(), (F2,),
               _parked_row(10, [70.0 + 8.0 * i for i in range(12)], offset=6.4) + [car(30, -30.0, LEFT_LANE, 11.0)],
               goal=260.0, t=60.0, desc="Parked cars stick 0.5 m into the ego lane all along the street")


def pg_park_double():
    """Car ahead stops in the lane to parallel park while a delivery truck is double-parked further on."""
    parker = car(1, 60.0, RIGHT_LANE, 6.0, speed_changes=[(2.0, 6.0), (5.0, 0.0), (8.0, 0.0), (9.0, -1.2),
                                                          (13.0, -1.2), (14.0, 0.0)],
                 lane_changes=[(9.0, RIGHT_LANE), (14.0, KERB)])
    return _sc("pg_park_double", crosswalk_road(), (X2,),
               [parker, truck(2, 200.0, RIGHT_LANE, 0.0)] + _parked_row(10, [52.0, 85.0, 230.0], offset=6.9)
               + [car(30, -30.0, LEFT_LANE, 9.0), car(31, -80.0, LEFT_LANE, 9.0)],
               start=20.0, goal=300.0, t=70.0, desc="A car parallel parks ahead, then a double-parked truck")


# ---------------------------------------------------------------------------
# Yield on a busy road
# ---------------------------------------------------------------------------
def _stream(first_id, road, s0, gaps, speed):
    out, s = [], s0
    for i, g in enumerate(gaps):
        out.append(car(first_id + i, s, INTERSECTION_LANE, speed, road=road))
        s -= g
    return out


def pg_yield_right():
    """Right turn onto a busy road: cars from the left keep coming, with gaps of different sizes."""
    return _sc("pg_yield_right", intersection(), TURN_RIGHT,
               _stream(1, "A1", 110.0, [14, 16, 18, 15, 40, 14, 15, 60, 15], 9.0), start=60.0, v0=8.0, vd=8.0,
               goal=None, t=70.0, desc="Right turn onto a busy road: wait for a big enough gap")


def pg_yield_left():
    """Left turn across a busy road with traffic from both sides."""
    return _sc("pg_yield_left", intersection(), TURN_LEFT,
               _stream(1, "A1", 115.0, [15, 17, 45, 15, 70, 15], 9.0)
               + _stream(20, "A3", 100.0, [16, 18, 15, 60, 15, 18], 9.0), start=60.0, v0=8.0, vd=8.0,
               goal=None, t=80.0, desc="Left turn across traffic from both sides")


def pg_yield_straight():
    """Straight across a busy road; one fast car at the edge of the sensor range."""
    return _sc("pg_yield_straight", intersection(), STRAIGHT_THROUGH,
               _stream(1, "A1", 100.0, [15, 15, 50, 16, 16], 9.0)
               + _stream(20, "A3", 80.0, [14, 60, 15], 10.0) + [car(40, -60.0, INTERSECTION_LANE, 13.0, road="A3")],
               start=60.0, v0=8.0, vd=8.0, goal=None, t=70.0, desc="Straight across a busy road")


def pg_yield_stop_sign():
    """All-way stop, but the crossing cars do not stop (busy road); turn right."""
    return _sc("pg_yield_stop_sign", intersection("stop_signs"), TURN_RIGHT,
               _stream(1, "A1", 105.0, [15, 15, 15, 45, 15, 15, 70], 8.0), start=60.0, v0=8.0, vd=8.0,
               goal=None, t=80.0, desc="Stop sign onto a busy road whose traffic does not stop")


def pg_yield_merge():
    """Ego in a lane that ends (blocked ahead); merge into a busy lane."""
    left = [car(20 + i, 60.0 - g, LEFT_LANE, 10.0) for i, g in enumerate([0, 14, 28, 42, 80, 94, 108])]
    return _sc("pg_yield_merge", two_lane_road(), (F2,),
               [car(1, 150.0, RIGHT_LANE, 0.0, length_m=30.0, width_m=2.5, kind="truck")] + left,
               goal=300.0, t=60.0, desc="The ego lane is blocked: merge into a busy lane")


# ---------------------------------------------------------------------------
# Police: move over
# ---------------------------------------------------------------------------
def pg_police_shoulder():
    """Police car stopped at the kerb behind a pulled-over car; move over to the left lane."""
    return _sc("pg_police_shoulder", two_lane_road(), (F2,),
               [car(1, 130.0, 6.5, 0.0), car(2, 140.0, 6.5, 0.0),
                pedestrian(3, 136.0, 4.9, "along", 0.0, speed_changes=[(6.0, 0.0), (6.5, 0.6), (12.0, 0.6),
                                                                      (12.5, 0.0)])]
               + [car(20 + i, -20.0 - 30.0 * i, LEFT_LANE, 11.0) for i in range(3)],
               goal=260.0, t=60.0, desc="Police car stopped at the kerb, officer standing beside it: move over")


def pg_police_from_behind():
    """A police car comes up fast from behind in the ego lane; it does not brake."""
    return _sc("pg_police_from_behind", two_lane_road(), (F2,),
               [car(1, -40.0, RIGHT_LANE, 15.0), car(2, 120.0, RIGHT_LANE, 8.0),
                car(20, -10.0, LEFT_LANE, 11.0)],
               goal=300.0, t=60.0, desc="Police car with siren approaching fast from behind: move over and let it pass")


def pg_police_behind_brake():
    """Police comes from behind while the ego is following a car that brakes."""
    return _sc("pg_police_behind_brake", two_lane_road(), (F2,),
               [car(1, -60.0, RIGHT_LANE, 15.0, lane_changes=[(9.0, RIGHT_LANE)]),
                car(2, 70.0, RIGHT_LANE, 10.0, speed_changes=[(4.0, 10.0), (7.0, 3.0)]),
                car(20, -30.0, LEFT_LANE, 12.0), car(21, -80.0, LEFT_LANE, 12.0)],
               goal=300.0, t=60.0, desc="Police from behind while the car ahead slows down")


def pg_police_in_lane():
    """Police car and stopped car fully in the right lane, officer walking between them and the left lane."""
    return _sc("pg_police_in_lane", two_lane_road(), (F2,),
               [car(1, 140.0, RIGHT_LANE, 0.0), car(2, 150.0, RIGHT_LANE, 0.0),
                pedestrian(3, 147.0, 4.0, "along", 0.0, speed_changes=[(8.0, 0.0), (8.5, -0.8), (11.0, -0.8),
                                                                      (11.5, 0.0)])]
               + [car(20 + i, -15.0 - 35.0 * i, LEFT_LANE, 11.0) for i in range(3)]
               + [car(30, 400.0, ONCOMING_LANE, 10.0, oncoming=True)],
               goal=280.0, t=70.0, desc="Traffic stop in the right lane, officer beside it: pass with room")


def pg_police_left_lane():
    """Ego in the LEFT lane; police comes from behind in the left lane; move right."""
    return _sc("pg_police_left_lane", two_lane_road(), (F1,),
               [car(1, -50.0, LEFT_LANE, 15.5), car(20, 60.0, RIGHT_LANE, 9.0)],
               goal=300.0, t=60.0, desc="Police from behind in the left lane: move over to the right")


LEADERBOARD_LIKE = {f.__name__: f for f in (
    pg_rush_sidewalk, pg_rush_cut_ins, pg_rush_signal, pg_rush_walkers_along,
    pg_park_reverse_in, pg_park_reverse_long, pg_park_pull_out, pg_park_kerb_row, pg_park_double,
    pg_yield_right, pg_yield_left, pg_yield_straight, pg_yield_stop_sign, pg_yield_merge,
    pg_police_shoulder, pg_police_from_behind, pg_police_behind_brake, pg_police_in_lane, pg_police_left_lane,
)}
