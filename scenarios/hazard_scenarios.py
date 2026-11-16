"""Team members: Anthony Pollard, Cole Pulaski, Will Cunningham, Brandon Debrosse

Hazard scenes: things that are not normal traffic.  Names start with "hz_".

    hz_fallen_tree     a tree falls across the ego lane; drive around it
    hz_tornado         a tornado crosses the road and drops debris
    hz_crash_ahead     two cars ahead crash into each other and block the lane

The simulator only knows cars, trucks, bicycles and pedestrians, so the tree,
the tornado and the debris are built from the ``car`` building block with
their own length and width (drawn in the truck colour).  The planner sees
them as ordinary obstacles, which is all it needs to know.

They are added to the scenario list at the bottom of student_scenario.py.
"""

from __future__ import annotations

from scenarios.scenario_tools import (
    LEFT_LANE, MPH, ONCOMING_LANE, RIGHT_LANE, Scenario, car, pedestrian, two_lane_road,
)

ROAD_EDGE = 7.0          # right edge of the asphalt on the 2+2 roads [m right of the centre line]


def _obstacle(actor_id: int, s_m: float, offset_m: float, along_m: float, across_m: float, **kwargs):
    """A box that is not a vehicle: ``along_m`` long (road direction), ``across_m`` wide."""
    kwargs.setdefault("speed_mps", 0.0)
    return car(actor_id, s_m, offset_m, length_m=along_m, width_m=across_m, kind="truck", **kwargs)


def hz_fallen_tree() -> Scenario:
    """A tree at the right roadside falls across the RIGHT lane (the ego lane).

    * The tree stands beside the road (off the asphalt) until t = 4 s and lies
      across the whole right lane from t = 5.2 s.  Its tip ends just short of
      the lane line, so the left lane stays free.
    * When it starts to fall the ego's front bumper is about 38 m away at
      10 m/s (fair distance at 10 m/s: 27 m; still 33 m at the 25 mph limit).
    * Two cars come from behind in the left lane, so the ego cannot simply
      swerve: brake, let them pass, change left, pass the tree, come back.
    """
    tree = _obstacle(1, s_m=112.0, offset_m=10.6, along_m=1.2, across_m=6.0,      # 7.6 .. 13.6 m: off the road
                     lane_changes=[(4.0, 10.6), (5.2, 6.4)])                        # 3.4 ..  9.4 m: across the lane
    passing = [car(10, s_m=-10.0, offset_m=LEFT_LANE, speed_mps=12.0),
               car(11, s_m=-45.0, offset_m=LEFT_LANE, speed_mps=12.0)]
    oncoming = car(20, s_m=320.0, offset_m=ONCOMING_LANE, speed_mps=9.0, oncoming=True)
    return Scenario(
        name="hz_fallen_tree",
        description="A tree falls across the ego lane 38 m ahead; brake, let left-lane cars pass, drive around it",
        road=two_lane_road(), route=("two_lane_road:F2",),
        ego_start_s_m=30.0, ego_speed_mps=10.0, desired_speed_mps=10.0, speed_limit_mps=25 * MPH,
        goal_distance_m=250.0, time_limit_s=55.0,
        actors=tuple([tree, oncoming] + passing))


TORNADO_RADIUS = 25.0    # closer than this to the tornado's centre = sucked in (counts as a collision)
TORNADO_STRIPS = 60      # the disc is built from this many strips lying side by side


def _tornado(first_id: int, s_m: float, centre_offsets, speed_changes):
    """A circular danger zone of radius TORNADO_RADIUS around the tornado.

    The simulator only has boxes, so the disc is made of TORNADO_STRIPS narrow
    strips that all move together.  Every strip reaches at least as far as the
    circle, so touching the disc means being within 25 m of the centre.

    ``centre_offsets``: sideways keyframes (time, offset) of the CENTRE.
    ``speed_changes``: speed along the road; POSITIVE = toward the ego (the
    strips are "oncoming"), negative = away from it.
    While it moves TOWARD the ego it must not move sideways as well;
    otherwise the strips would turn and the disc would fall apart.
    """
    width = 2.0 * TORNADO_RADIUS / TORNADO_STRIPS
    strips = []
    for i in range(TORNADO_STRIPS):
        left, right = -TORNADO_RADIUS + i * width, -TORNADO_RADIUS + (i + 1) * width
        inner = 0.0 if left < 0.0 < right else min(abs(left), abs(right))
        along = 2.0 * (TORNADO_RADIUS ** 2 - inner ** 2) ** 0.5
        mid = 0.5 * (left + right)
        strips.append(_obstacle(first_id + i, s_m, centre_offsets[0][1] + mid, along_m=along, across_m=width - 0.02,
                                speed_changes=speed_changes, oncoming=True,
                                lane_changes=[(t, offset + mid) for t, offset in centre_offsets]))
    return strips


def _thrown(actor_id: int, s_m: float, offset_m: float, at_s: float, speed_mps: float, flight_s: float):
    """A branch hurled out of the tornado: appears at ``at_s`` and flies down
    the road TOWARD the ego, slowing to rest after ``flight_s`` seconds."""
    return _obstacle(actor_id, s_m, offset_m, along_m=1.5, across_m=2.0, oncoming=True, appear_at_s=at_s,
                     speed_mps=speed_mps, speed_changes=[(at_s, speed_mps), (at_s + flight_s, 0.0)])


def hz_tornado() -> Scenario:
    """An erratic tornado on the road: it charges at the ego, lurches away,
    comes back, throws debris down the road and finally leaves to the right.

    * Getting within 25 m of the tornado's centre means being sucked in: the
      tornado is a disc of radius 25 m, and touching it is a collision (score 0).
      While it is on the road it covers BOTH forward lanes: no way past.
    * 2 - 9.5 s    CHARGE: it comes down the road toward the ego at 12 m/s and
                   stops with its edge at s = 157 m.  A car that never braked
                   would have its front bumper at about 140 m by then.
    * 9.5 - 12 s   stalls and darts to the right.
    * 12 - 16 s    lurches 42 m away at 14 m/s, weaving to the left, and hurls
                   branch A back down the RIGHT lane (flies 12 m, lands at
                   s = 166 m).
    * 16 - 18.5 s  comes back 8 m toward the ego (edge at s = 191 m: even a
                   car that chased it at full throttle is still short of that).
    * 19 - 24 s    darts right, lurches 20 m away and hurls branch B down the
                   LEFT lane (flies 15 m, lands at s = 210 m).
    * 25.5 - 30 s  races off the road to the right, throwing branch C into the
                   right lane (lands at s = 251 m), then drifts on beside the road.
    * The ego has to stop for the charge, keep clear while the tornado moves
      about, and then slalom: left around A, right around B, left around C.
    """
    tornado = _tornado(
        100, s_m=260.0,
        centre_offsets=[(9.5, 3.5), (11.5, 8.0),           # darts right
                        (12.0, 8.0), (16.0, 1.0),          # weaves left while lurching away
                        (19.0, 1.0), (20.5, 6.0),          # darts right
                        (21.0, 6.0), (24.0, 2.0),          # weaves left while lurching away
                        (25.5, 2.0), (30.0, 50.0)],        # races off the road to the right
        speed_changes=[(2.0, 0.0), (3.0, 12.0), (8.5, 12.0), (9.5, 0.0),          # charges 78 m toward the ego
                       (12.0, 0.0), (13.0, -14.0), (15.0, -14.0), (16.0, 0.0),    # lurches 42 m away
                       (17.0, 5.33), (17.5, 5.33), (18.5, 0.0),                   # comes back 8 m
                       (21.0, 0.0), (22.0, -10.0), (23.0, -10.0), (24.0, 0.0),    # lurches 20 m away
                       (30.0, 0.0), (32.0, -3.0)])                                # drifts on beside the road
    branch_a = _thrown(2, s_m=178.0, offset_m=RIGHT_LANE, at_s=14.0, speed_mps=10.0, flight_s=2.4)
    branch_b = _thrown(3, s_m=225.0, offset_m=LEFT_LANE, at_s=22.5, speed_mps=12.0, flight_s=2.5)
    branch_c = _thrown(4, s_m=266.0, offset_m=RIGHT_LANE, at_s=27.0, speed_mps=12.0, flight_s=2.5)
    return Scenario(
        name="hz_tornado",
        description="Erratic tornado charges, retreats and throws debris: stay 25 m away or be sucked in, then slalom",
        road=two_lane_road(), route=("two_lane_road:F2",),
        ego_start_s_m=30.0, ego_speed_mps=10.0, desired_speed_mps=10.0, speed_limit_mps=25 * MPH,
        goal_distance_m=300.0, time_limit_s=90.0,
        actors=tuple(tornado + [branch_a, branch_b, branch_c]))


def hz_crash_ahead() -> Scenario:
    """Two cars crash right in front of the ego.

    * Car B drives ahead in the right lane at 9 m/s.
    * Car A overtakes the ego in the left lane at 14 m/s, swerves right at
      4.6 s and hits B's rear corner at 6 s.  A loses speed, B is pushed
      forward and to the right; both are standing still at 7.8 s, wedged
      together across the right lane.  The left lane stays free (0.85 m of
      room beside the wreck).
    * At the impact the wreck is 23 m ahead of the ego's front bumper and it
      stops 33 m ahead: an emergency stop, not a comfortable one (fair
      distance at 10 m/s: 27 m).
    * Car C comes from behind in the left lane and passes the wreck at about
      17 s: the ego must not pull out in front of it.
    """
    crash_t, stop_t = 6.0, 7.8
    car_b = car(1, s_m=75.0, offset_m=RIGHT_LANE, speed_mps=9.0,
                speed_changes=[(crash_t, 9.0), (crash_t + 0.2, 11.0), (stop_t, 0.0)],     # shoved, then stops
                lane_changes=[(crash_t, RIGHT_LANE), (crash_t + 0.6, 5.9)])                # pushed to the right
    car_a = car(2, s_m=40.1, offset_m=LEFT_LANE, speed_mps=14.0,
                speed_changes=[(crash_t, 14.0), (crash_t + 0.2, 11.0), (stop_t, 0.0)],
                lane_changes=[(4.6, LEFT_LANE), (6.2, 4.5)])                               # swerves into B
    car_c = car(3, s_m=-70.0, offset_m=LEFT_LANE, speed_mps=12.0)
    return Scenario(
        name="hz_crash_ahead",
        description="Two cars ahead crash and block the ego lane; emergency stop, then pass the wreck on the left",
        road=two_lane_road(), route=("two_lane_road:F2",),
        ego_start_s_m=30.0, ego_speed_mps=10.0, desired_speed_mps=10.0, speed_limit_mps=25 * MPH,
        goal_distance_m=250.0, time_limit_s=55.0,
        actors=(car_b, car_a, car_c))


def hz_everything_at_once() -> Scenario:
    """A chain reaction: every obvious reaction to one event is wrong for the next.

    1. RUNAWAY TRUCK (0 - 8 s).  Car B drives ahead in the right lane at only
       9 m/s, which invites an overtake - but a truck comes up from behind in the
       left lane at 15 m/s and passes the ego in the first two seconds.
    2. CRASH (8 - 9.8 s).  The truck swerves right and hits B.  Both stop wedged
       across the right lane (s = 148 .. 161 m); the left lane stays free
       (0.85 m of room beside the truck).  Emergency stop.
    3. WRONG-WAY DRIVER (until about 14.5 s).  A car comes TOWARD the ego in the
       left lane - the only way around the wreck - and passes the wreck at
       13.5 s.  Pulling out as soon as the ego has stopped means a head-on
       crash.  It moves over to its own side of the road afterwards.
    4. PEOPLE FLEE THE WRECK.  Three people appear in front of the wreck (hidden
       by it until then), stand for 0.7 s and run across the road to the left,
       one after the other, 5 - 15 m beyond the wreck.
    5. TORNADO.  It waits beside the road, moves onto it (9.5 - 12.5 s, behind
       the wrong-way car) and charges 78 m toward the ego (13 - 20.5 s),
       stopping with its edge at s = 251 m.  It blocks both lanes until 37.5 s,
       then does the same dance as in hz_tornado: lurches away, comes back,
       throws three branches (they land at s = 260 m right lane, 304 m left
       lane, 345 m right lane) and leaves to the right at 51 - 55.5 s.

    Why it is still solvable: the truck and the wrong-way car only use the
    LEFT lane, so an ego that stays in its lane and brakes for the wreck is
    never hit.  A law-abiding ego cannot be past the wreck before 13.5 s, so
    even at full throttle its front bumper is short of s = 227 m when the
    charge ends at 20.5 s - the same 17 m margin as in hz_tornado.
    """
    crash_t, stop_t = 8.0, 9.8
    car_b = car(1, s_m=76.0, offset_m=RIGHT_LANE, speed_mps=9.0,
                speed_changes=[(crash_t, 9.0), (crash_t + 0.2, 11.0), (stop_t, 0.0)],      # shoved, then stops
                lane_changes=[(crash_t, RIGHT_LANE), (crash_t + 0.6, 5.9)])                 # pushed to the right
    runaway = car(2, s_m=21.2, offset_m=LEFT_LANE, speed_mps=15.0, length_m=9.0, width_m=2.5, kind="truck",
                  speed_changes=[(crash_t, 15.0), (crash_t + 0.2, 11.0), (stop_t, 0.0)],
                  lane_changes=[(6.6, LEFT_LANE), (8.2, 4.8)])                              # swerves into B

    wrong_way = car(3, s_m=316.0, offset_m=LEFT_LANE, speed_mps=12.0, oncoming=True,
                    lane_changes=[(16.0, LEFT_LANE), (19.0, ONCOMING_LANE)])                # back to its own side

    people = [pedestrian(10 + i, s_m=s, offset_m=6.2, walk_toward="left", appear_at_s=t, speed_mps=0.0,
                         speed_changes=[(t + 0.7, 0.0), (t + 1.0, 2.5)])
              for i, (s, t) in enumerate(PEOPLE)]

    dt, ds = 25.5, 94.0      # the dance after the charge: this much later and further on than in hz_tornado
    tornado = _tornado(
        100, s_m=260.0 + ds,
        centre_offsets=[(9.5, 45.0), (12.5, 3.5),                        # moves onto the road from the right
                        (9.5 + dt, 3.5), (11.5 + dt, 8.0),               # darts right
                        (12.0 + dt, 8.0), (16.0 + dt, 1.0),              # weaves left while lurching away
                        (19.0 + dt, 1.0), (20.5 + dt, 6.0),              # darts right
                        (21.0 + dt, 6.0), (24.0 + dt, 2.0),              # weaves left while lurching away
                        (25.5 + dt, 2.0), (30.0 + dt, 50.0)],            # races off the road to the right
        speed_changes=[(13.0, 0.0), (14.0, 12.0), (19.5, 12.0), (20.5, 0.0),                       # charges 78 m
                       (12.0 + dt, 0.0), (13.0 + dt, -14.0), (15.0 + dt, -14.0), (16.0 + dt, 0.0),  # lurches 42 m away
                       (17.0 + dt, 5.33), (17.5 + dt, 5.33), (18.5 + dt, 0.0),                     # comes back 8 m
                       (21.0 + dt, 0.0), (22.0 + dt, -10.0), (23.0 + dt, -10.0), (24.0 + dt, 0.0),  # lurches 20 m away
                       (30.0 + dt, 0.0), (32.0 + dt, -3.0)])                                       # drifts on
    branch_a = _thrown(4, s_m=178.0 + ds, offset_m=RIGHT_LANE, at_s=14.0 + dt, speed_mps=10.0, flight_s=2.4)
    branch_b = _thrown(5, s_m=225.0 + ds, offset_m=LEFT_LANE, at_s=22.5 + dt, speed_mps=12.0, flight_s=2.5)
    branch_c = _thrown(6, s_m=266.0 + ds, offset_m=RIGHT_LANE, at_s=27.0 + dt, speed_mps=12.0, flight_s=2.5)
    return Scenario(
        name="hz_everything_at_once",
        description="Runaway truck crashes ahead, wrong-way driver in the only free lane, people flee the wreck, "
                    "then a tornado charges and throws debris",
        road=two_lane_road(), route=("two_lane_road:F2",),
        ego_start_s_m=30.0, ego_speed_mps=10.0, desired_speed_mps=10.0, speed_limit_mps=25 * MPH,
        goal_distance_m=394.0, time_limit_s=110.0,
        actors=tuple([car_b, runaway, wrong_way, branch_a, branch_b, branch_c] + people + tornado))


#: People fleeing the wreck in hz_everything_at_once: (s where they cross, time they appear)
PEOPLE = ((166.0, 17.0), (171.0, 20.0), (176.0, 23.0))


HAZARD_SCENARIOS = {
    "hz_fallen_tree": hz_fallen_tree,
    "hz_tornado": hz_tornado,
    "hz_crash_ahead": hz_crash_ahead,
    "hz_everything_at_once": hz_everything_at_once,
}
