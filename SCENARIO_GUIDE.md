# Scenario Guide (Scenario Team)

Your job: build driving scenes that are **as hard as possible, but still solvable by a careful driver** (see COMPETITION_RULES.md).

You write scenarios in **`scenarios/student_scenario.py`** (more files are fine, see section 9). Everything you need is imported there from `scenarios/scenario_tools.py`. You never change the `simulator/` folder.

**Workflow in short:**

1. Write a function that returns a `Scenario` (sections 1–5).
2. Register it in `STUDENT_SCENARIOS`.
3. Watch it: `python run_simulation.py --scenario <name> --debug`.
4. Check that it is fair (section 7) and scores it: `python evaluate.py --scenario <name> --planner baseline student`.
5. Make it harder (section 6) and repeat.

## 1. A scenario in one picture

```python
def my_scene() -> Scenario:
    return Scenario(
        name="my_scene",                      # must equal the key in STUDENT_SCENARIOS
        description="What happens and what the car should do",
        road=two_lane_road(),                 # which road (section 2)
        route=("two_lane_road:F2",),          # which lanes the ego car must follow (its mission)
        ego_start_s_m=30.0,                   # start position along the route [m]
        ego_speed_mps=10.0,                   # start speed [m/s]
        desired_speed_mps=10.0,               # speed the planner should aim for [m/s]
        speed_limit_mps=25 * MPH,             # posted maximum speed (MPH = 0.44704 m/s; default 25 mph)
        goal_distance_m=250.0,                # distance to drive to "reach the goal" [m]
        time_limit_s=45.0,                    # the run stops after this many seconds
        actors=(car(1, s_m=75.0, offset_m=RIGHT_LANE, speed_mps=8.0),),   # other road users (section 4)
    )

STUDENT_SCENARIOS = {"my_scene": my_scene}
```

Then run it: `python run_simulation.py --scenario my_scene`.

**Optional fields:**

* `ego_start_offset_m`: start a little left or right of the lane centre (positive = right).
* `ego_start_lane`: start in another lane, e.g. `"two_lane_road:F1"`.
* `signal_phases`: traffic-light timing (section 5).
* `goal_distance_m=None`: drive to the end of the route (minus 12 m).

**Choosing the time limit.** The score gives 20 points for time only if the goal is reached, compared with `goal distance / desired speed`. Give enough time for a careful driver to finish (e.g. 1.5–2× the ideal time plus any waiting you force). A scenario whose time limit is too short to finish *safely* is unfair.

## 2. Roads

| Function | Road | Lane IDs for `route` |
|---|---|---|
| `two_lane_road(length_m=900, solid_divider_until_m=20)` | straight, 2 lanes each direction; the divider between the two forward lanes is solid until `solid_divider_until_m`, dashed after | `"two_lane_road:F1"` (left), `"two_lane_road:F2"` (right) |
| `curved_two_lane_road()` | gentle S-curve (radius 100 m), 2 lanes each direction | `"curved_road:F1"`, `"curved_road:F2"` |
| `crosswalk_road()` | straight 2+2 (400 m) with sidewalks and a crosswalk at s = 150 m | `"crosswalk_road:F1"`, `"crosswalk_road:F2"` |
| `intersection(control=None)` | four-way junction, 1 lane each direction | use `STRAIGHT_THROUGH`, `TURN_LEFT` or `TURN_RIGHT` |

**Intersection control:** `None` (uncontrolled, the ego must yield by itself), `"stop_signs"` (all-way stop) or `"signals"` (traffic lights).

**Positions on a road:** `s` is metres along the road's reference line, and `offset` is metres to the **right** of it (negative = left). On the 2+2 roads the reference line is the yellow centre line:

```
                 ONCOMING_OUTER_LANE = -5.25     (traffic coming toward you)
                 ONCOMING_LANE       = -1.75
================ yellow centre line  =  0 ================
                 LEFT_LANE           = +1.75     (F1)  -> direction of travel
                 RIGHT_LANE          = +5.25     (F2)  ->
sidewalk centre (crosswalk_road only) = +8.95 / -8.95
```

**Intersection.** The four arms are roads `"A0"` (south), `"A1"` (west), `"A2"` (north) and `"A3"` (east).

* Each arm road starts 150 m from the centre and points **toward** the centre, which is at s = 150 m along every arm. The junction box spans s = 138 … 162 m.
* A car on arm `"A1"` at `offset_m=INTERSECTION_LANE` drives toward the junction and **straight across** it.
* The ego routes start on arm A0 (from the south). The stop line on A0 is at s = 133 m.

## 3. The ego car

* `ego_start_s_m` is measured along the **route**, from the start of its first lane.
* `desired_speed_mps` is the cruise speed given to the planner; it also sets the ideal time of the score. On the intersection, keep it at or below about 8 m/s; turns are slower.
* `speed_limit_mps` is the posted maximum speed, also given to the planner. Use real posted limits, e.g. `25 * MPH` (residential, the default), `35 * MPH` (urban arterial). `desired_speed_mps` may not be above it (the scenario raises an error). A planner may cruise at the limit instead of the desired speed (the `learned_example` planner does), so also check that your scenario is solvable for a car arriving at the limit.
* `goal_distance_m` is how far the car must drive along the route to "reach the goal".
* What the ego can physically do: accelerate up to 2 m/s², brake up to 6 m/s² (the controller's limit), with a 0.2 s actuator lag; steering is limited to ±0.55 rad at 0.8 rad/s. It cannot reverse.

## 4. Other road users

```python
car(actor_id, s_m, offset_m=RIGHT_LANE, speed_mps=8.0,
    speed_changes=[(t, speed), ...],    # speed keyframes; linear in between
    lane_changes=[(t, offset), ...],    # sideways keyframes; smooth in between
    road=None,                          # "A0".."A3" on the intersection
    oncoming=False,                     # True = drive against the road direction
    appear_at_s=0.0)                    # appears only at this time (late detection)
truck(actor_id, s_m, offset_m, speed_mps, ...)     # 9.0 m x 2.5 m
bicycle(actor_id, s_m, offset_m, speed_mps, ...)   # 1.8 m x 0.6 m
pedestrian(actor_id, s_m, offset_m, walk_toward="left", speed_mps=1.3,
           speed_changes=[...], road=None, appear_at_s=0.0)
```

**Timing rules (important):**

* All keyframe times are **absolute simulation times** in seconds, starting at 0.
* The start speed (`speed_mps`) and start offset count as a keyframe at time 0. Between keyframes the speed changes **linearly** and the offset changes smoothly (S-shaped). To *hold* a speed until time T and only then change it, add a keyframe `(T, same speed)`, as in the braking example below. After the last keyframe the value stays constant.
* `appear_at_s=T`: the road user does not exist before time T. At time T it **appears exactly at `(s_m, offset_m)`** and only then starts moving (its speed at T is taken from `speed_changes`). Use it for a car that comes out from behind an obstacle or a pedestrian stepping out between parked cars.

**Changing speed:**

```python
# 8 m/s, starts braking at 5 s, standing still from 8 s to 15 s, back to 8 m/s at 19 s
car(1, 75.0, RIGHT_LANE, 8.0, speed_changes=[(5.0, 8.0), (8.0, 0.0), (15.0, 0.0), (19.0, 8.0)])
```

**Changing lanes (cut-in):**

```python
# drives in the left lane, moves into the right lane between 2.5 s and 5 s
car(2, 46.0, LEFT_LANE, 8.0, lane_changes=[(2.5, LEFT_LANE), (5.0, RIGHT_LANE)])
```

**Pedestrian crossing the road:**

```python
# waits on the right sidewalk for 7 s, then walks to the left at 1.3 m/s
pedestrian(1, 150.0, 8.95, walk_toward="left", speed_mps=0.0, speed_changes=[(7.0, 0.0), (7.5, 1.3)])
```

**Other cases:**

* Oncoming traffic: `car(3, 300.0, ONCOMING_LANE, 9.0, oncoming=True)`.
* Parked car: `car(4, 120.0, RIGHT_LANE, 0.0)`.
* A car that suddenly appears (occlusion): `car(5, 95.0, RIGHT_LANE, 0.0, appear_at_s=3.0)`.
* Every road user needs a **unique** `actor_id`.

**What road users can NOT do (know the limits of the tools):**

* They are **scripted**. They never react to the ego car: they do not brake for it, do not yield, and will drive into it.
* They **ignore traffic lights and stop signs.** If a crossing car should stop at its red light, give it `speed_changes` that make it stop.
* Vehicles follow their road at a fixed offset (plus your `lane_changes`). On the intersection they can only drive **straight across**; they cannot turn.
* Pedestrians walk in a straight line (across the road, or `"along"` it).

## 5. Traffic lights

Use `intersection("signals")`. Lights are green unless you set phases:

```python
signal_phases=intersection_signals(
    ego_phases=[(0.0, "RED"), (14.0, "GREEN")],          # arms A0 and A2 (your road)
    crossing_phases=[(0.0, "GREEN"), (14.0, "RED")]),     # arms A1 and A3
```

Give a realistic yellow phase (about 3 s) between green and red, e.g. `[(0.0, "GREEN"), (6.0, "YELLOW"), (9.0, "RED")]`.

## 6. Making scenarios harder

Increase difficulty step by step and say in the `description` what each scene tests. Ideas, roughly from easy to hard:

| Lever | Example |
|---|---|
| Lead vehicle behaviour | a lead car that brakes hard, stops, restarts, or drives in stop-and-go waves |
| Cut-ins | a car from the left lane cuts in close in front, then brakes |
| Blocked lane | a parked car or slow truck in the ego lane, with traffic in the neighbouring lane that leaves only short gaps |
| Overtaking pressure | a slow truck plus fast cars approaching from behind in the left lane |
| Occlusion | a pedestrian or car that appears late (`appear_at_s`) behind a parked truck |
| Vulnerable road users | cyclists riding at the lane edge; pedestrians who start, stop and continue walking |
| Junction gap acceptance | crossing traffic on an uncontrolled junction with gaps that are *just* big enough |
| Signals | a light that turns yellow when the ego is at the dilemma-zone distance; a car from the side that crosses late on its red |
| Curves | the same scenes on `curved_two_lane_road()`, where the view along the lane is curved |
| Combinations | two or three of the above at the same time |

## 7. Fair play: is the scenario solvable?

The rules require scenarios that a **careful** driver can get through without a collision (COMPETITION_RULES.md). This section gives the numbers to check that.

**Reaction distance.** If something appears in or moves into the ego's path, it must happen far enough ahead. The table gives, for a road user that is standing still in the ego lane, the gap between the ego's front bumper and the road user at the moment it appears (or blocks the lane):

| ego speed | physical minimum (unsolvable below this) | fair: at least |
|---|---|---|
| 8 m/s | 7 m | 19 m |
| 10 m/s | 11 m | 27 m |
| 13 m/s | 17 m | 41 m |
| 15 m/s | 23 m | 53 m |

The physical minimum is `0.25 v + v²/12` (one planning cycle plus actuator lag, then 6 m/s² braking). The fair distance is `1.0 v + v²/6` (one second to react, then 3 m/s²). Scenes in between are emergency situations. For reference, the baseline survives a stopped car appearing 12 m ahead at 8 m/s and 18 m ahead at 10 m/s.

**Things that make a scene unsolvable or meaningless** (check for them):

* road users that start overlapping each other or the ego car (forbidden by the rules);
* unrealistic speeds (the rules give the limits);
* a road user driven into the ego from behind or from the side while the ego drives normally. Scripted road users do not brake, so such a collision says nothing about the planner. The evaluation flags it as `collision_from_behind`;
* a road user that appears on top of, or right next to, the ego;
* a scene where the only way through is to break a traffic rule (run a red light, cross the centre line).

**Check your scene:**

1. Watch it with the baseline: `python run_simulation.py --scenario <name> --debug`. Press **p** to pause and look at the gaps.
2. Score it: `python evaluate.py --scenario <name> --planner baseline student`.
3. If the baseline crashes, find out whether a careful driver would have crashed too. If yes, the scenario is unfair: make it easier. If no, you found a weakness: write down why the scene is hard.
4. Check the reported rule violations (see the table printed by `evaluate.py`).

## 8. What the evaluation reports

For every run the simulator records:

* the **score** (collision or road departure gives 0; see README section 6);
* **rule violations** (reported only, they do not change the score): running a red light, not stopping fully at a stop sign, driving on the opposing side of the road;
* `collision_from_behind`: the ego was hit by a road user coming from behind.

## 9. Organising your files

* Small sets: everything in `scenarios/student_scenario.py`.
* Larger sets: create e.g. `scenarios/team_b_scenarios.py`, import its functions in `student_scenario.py` and add them to `STUDENT_SCENARIOS`.
* Names must be unique and must not repeat the example scenario names.
* Build scenarios from the `scenario_tools.py` building blocks. Plain Python (loops, helper functions, constants) to build many road users works fine. If you use random numbers, use a fixed seed: the evaluation relies on every run of a scenario being identical.

**Submission checklist:**

* [ ] each scenario has a clear `description` of what it tests;
* [ ] `python run_simulation.py --list` shows them;
* [ ] `python evaluate.py --planner baseline student` runs without errors;
* [ ] you checked each scenario with section 7, and you have written down why it is hard.
