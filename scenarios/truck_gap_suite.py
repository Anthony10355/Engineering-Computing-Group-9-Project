"""Team members: Anthony Pollard, Cole Pulaski, Will Cunningham, Brandon Debrosse

Local stand-ins for the leaderboard's parked-truck scenes (team 6: parked
truck merge / decoy / gap ladder / truck merge queue).  A truck stands in the
ego lane; the left lane carries traffic whose gaps grow one after another, so
the scene measures how soon a planner accepts a gap that is safe.  Names start
with "tg_".
"""

from __future__ import annotations

from scenarios.scenario_tools import LEFT_LANE, MPH, RIGHT_LANE, Scenario, car, truck, two_lane_road

F2 = "two_lane_road:F2"


def _ladder(first_id, front_s, gaps, speed):
    out, s = [], front_s
    for i, g in enumerate(gaps):
        out.append(car(first_id + i, s, LEFT_LANE, speed))
        s -= g
    return out


def _scene(name, truck_s, gaps, speed, *, front_s=20.0, v0=10.0, extra=(), desc=""):
    return Scenario(name=name, description=desc or name, road=two_lane_road(), route=(F2,), ego_start_s_m=30.0,
                    ego_speed_mps=v0, desired_speed_mps=10.0, speed_limit_mps=25 * MPH, goal_distance_m=260.0,
                    time_limit_s=80.0,
                    actors=tuple([truck(1, truck_s, RIGHT_LANE, 0.0)] + _ladder(100, front_s, gaps, speed)
                                 + list(extra)))


def tg_ladder_slow():
    return _scene("tg_ladder_slow", 110.0, [12, 16, 20, 25, 30, 36, 44, 55], 8.0)


def tg_ladder_fast():
    return _scene("tg_ladder_fast", 110.0, [16, 20, 25, 30, 36, 44, 55, 70], 11.0)


def tg_ladder_far():
    return _scene("tg_ladder_far", 160.0, [14, 18, 24, 30, 38, 48, 60], 10.0, front_s=60.0)


def tg_queue_behind():
    """Two trucks parked one after another, ladder traffic, ego arrives slower."""
    return _scene("tg_queue_behind", 100.0, [14, 17, 21, 26, 32, 40, 50], 9.0, v0=6.0,
                  extra=[truck(2, 112.0, RIGHT_LANE, 0.0)])


def tg_decoy():
    """The 'parked' truck drives off after a while."""
    return Scenario(name="tg_decoy", description="Truck that looks parked drives off", road=two_lane_road(),
                    route=(F2,), ego_start_s_m=30.0, ego_speed_mps=10.0, desired_speed_mps=10.0,
                    speed_limit_mps=25 * MPH, goal_distance_m=260.0, time_limit_s=80.0,
                    actors=tuple([truck(1, 100.0, RIGHT_LANE, 0.0, speed_changes=[(9.0, 0.0), (12.0, 6.0)])]
                                 + _ladder(100, 20.0, [14, 18, 24, 30, 40, 55], 10.0)))


def tg_release():
    """Truck stands, then drives off slowly; left lane busy."""
    return Scenario(name="tg_release", description="Truck releases and drives on slowly", road=two_lane_road(),
                    route=(F2,), ego_start_s_m=30.0, ego_speed_mps=10.0, desired_speed_mps=10.0,
                    speed_limit_mps=25 * MPH, goal_distance_m=260.0, time_limit_s=80.0,
                    actors=tuple([truck(1, 95.0, RIGHT_LANE, 0.0, speed_changes=[(6.0, 0.0), (9.0, 5.0)])]
                                 + _ladder(100, 10.0, [15, 19, 25, 33, 45, 60], 10.0)))


TRUCK_GAP_SCENARIOS = {f.__name__: f for f in (
    tg_ladder_slow, tg_ladder_fast, tg_ladder_far, tg_queue_behind, tg_decoy, tg_release)}
