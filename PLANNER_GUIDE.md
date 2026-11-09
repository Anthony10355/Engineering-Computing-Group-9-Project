# Planner Guide (Planner Team)

* Classical planner (rules, sampling, optimisation): this guide.
* Your own existing trajectory generator: section 8.
* Learning-based planner (neural network, imitation or reinforcement learning): this guide, then [LEARNING_GUIDE.md](LEARNING_GUIDE.md).

Your planner lives in **`planners/student_planner.py`**. The simulator calls its `plan(observation)` method **20 times per second**. Each time it must return a **trajectory**: where the car should be over the next few seconds, and how fast it should go there. A controller (pure pursuit + speed control) then drives the car along it.

```
observation --> YOUR plan() --> trajectory --> controller --> car moves --> new observation ...
```

## 1. Coordinates (read this first)

Everything you get and return is in the **ego frame**:

```
            x (forward)
            ^
            |
            |        the car's REAR AXLE is at (0, 0), facing +x
   (left)   o------> y (RIGHT)
```

* **y is positive to the RIGHT.**
* Headings are measured from +x toward +y, so a positive heading means pointing to the right.
* All distances are in metres, speeds in m/s, times in s.
* The car is 4.66 m long and 1.89 m wide. Its front bumper is 3.90 m ahead of the rear axle.

Along a lane we often use **Frenet coordinates**:

* **s**: distance along the lane centre line.
* **d**: sideways offset from it, **positive = right**.

## 2. What the observation contains

`obs` is an `Observation` (defined in `simulator/observation.py`):

| Field | Meaning |
|---|---|
| `obs.time_s` | simulation time |
| `obs.desired_speed_mps` | the cruise speed you should aim for (the score's ideal time uses it) |
| `obs.speed_limit_mps` | the posted maximum speed (e.g. 25 mph = 11.18 m/s); the desired speed is never above it |
| `obs.distance_to_goal_m` | route distance still to drive |
| `obs.ego.speed_mps`, `.acceleration_mps2`, `.yaw_rate_rps`, `.steering_rad` | your car |
| `obs.lanes["current"]` | the lane you are in (always present) |
| `obs.lanes["left"]`, `obs.lanes["right"]` | the neighbouring lanes in your direction (only if they exist) |
| `obs.actors` | road users within the sensor range (about 80 m ahead, 40 m behind and to each side) |
| `obs.stop_lines` | stop or yield lines on your route ahead |
| `obs.junctions` | junctions on your route ahead |
| `obs.crosswalks` | `(start, end)` distances of crosswalks on your route |

**Lane** (`LaneObservation`):

* `centerline`: a `ReferencePath` in the ego frame, from about 15 m behind to about 105 m ahead.
* `ego_s_m`, `ego_d_m`: where your rear axle is on that centre line.
* `width_m`.
* `is_route_lane`: True if your mission continues in this lane.
* `change_allowed`: True if the lane marking allows changing into this lane here. Solid lines and junctions do not.

**Road user** (`ActorObservation`):

* `actor_id`, and `kind`: "car", "truck", "bicycle" or "pedestrian".
* `x_m`, `y_m`: the centre, in the ego frame.
* `heading_rad`: relative to your heading.
* `speed_mps`, and `vx_mps`, `vy_mps`: its velocity over the ground, in ego axes.
* `length_m`, `width_m`.

**Stop line** (`StopLineObservation`):

* `distance_m`: along the route, from your **rear axle** to the line. Subtract 3.90 m for your front bumper.
* `control`: `None` (a yield line at an uncontrolled junction), `"stop_sign"` or `"signal"`.
* `signal_state`: "RED", "YELLOW" or "GREEN" when there is a signal.

**Junction** (`JunctionObservation`): `entry_distance_m`, `exit_distance_m` and `movement` ("STRAIGHT", "LEFT" or "RIGHT").

The observation is a **copy**. Changing it does not change the world.

## 3. What you return

```python
from simulator.trajectory import Trajectory

return Trajectory(
    t_s=t,              # times: 0, 0.1, 0.2, ..., 4.0   (must start at 0 and increase)
    x_m=x, y_m=y,       # rear-axle positions at those times (ego frame)
    speed_mps=v,        # speed at those times (>= 0)
    behavior="FOLLOW_LEAD",          # optional: label shown on screen
    notes=["lead #1 20 m"],          # optional: text shown on screen
)
```

* Heading and acceleration are computed for you, or you can pass `heading_rad=` and `acceleration_mps2=`.
* `alternatives=[...]` (other trajectories you considered) are drawn in debug view (**d**).

**The simulator checks every trajectory:**

* at least 2 points covering at least 1 s;
* no NaN values;
* time starting at 0 and increasing;
* speeds between 0 and 40 m/s;
* the first point within 3 m of the car.

If a check fails, or your planner crashes, the car **brakes** for that cycle and a *planner error* is counted.

## 4. How the controller follows your trajectory

You do **not** steer or brake directly. The controller does (`simulator/controller.py`):

* **Steering (pure pursuit).**
  * It picks a *goal point* on your trajectory, `Ld` metres ahead. `Ld = 1.0 + 0.6 × speed`, kept between 3 and 20 m.
  * It steers along the circular arc that reaches that point: `steering = atan(2 × wheelbase × sin(alpha) / Ld)`, where `alpha` is the angle to the goal point.
  * Result: smooth paths are followed accurately (about 3 cm at 0.5 s in our tests). Sharp kinks are cut.
* **Speed.**
  * `acceleration = planned acceleration (0.2 s ahead) + 1.0 × (planned speed now − actual speed)`, between −6 and +2 m/s².
  * If your plan stops, the controller brakes to a standstill and holds it.
* **The car** (`simulator/vehicle_model.py`).
  * A kinematic bicycle model. Steering is limited to ±0.55 rad and 0.8 rad/s.
  * Acceleration reacts with a 0.2 s lag.
  * The car cannot reverse.

**What this means for your planner:**

* Make the trajectory smooth and drivable.
* Start it at the car (0, 0).
* Keep the speed profile realistic: brake at most about 3 m/s² normally, and about 6 m/s² in an emergency.
* You plan again every 0.05 s, so only the first part of each plan is ever driven. The rest must still be sensible, because it decides what you do next.

## 5. Helper functions (`planners/planner_tools.py`)

| Function | What it does |
|---|---|
| `time_grid(4.0, 0.1)` | the times 0 … 4 s |
| `find_lead(lane, obs.actors)` | the road user ahead in a lane → `Lead(actor_id, gap_m, speed_mps, rear_s_m)` or None |
| `actor_position_on_lane(lane, actor)` | (s, d) of a road user on a lane |
| `speed_profile(t, v0, a0, desired, lane=, stop_s_m=, lead=)` | speeds over time: cruise, slow down in curves, follow a lead (IDM), stop at a point → (distance, speed, acceleration) |
| `lateral_profile(d0, slope0, d_target, length_m, s)` | smooth sideways motion (quintic polynomial) |
| `build_trajectory(lane, t, s, d, v, a)` | (distance, offset, speed) along a lane → `Trajectory` |
| `predict_actor(actor, t)` | where a road user will be if it keeps its velocity |
| `extend_trajectory(lane, traj, 8.0)` | continue a plan at its final speed (for checking further ahead) |
| `first_collision(t, x, y, heading, obs.actors)` | first predicted overlap with a road user, or None |

## 6. How the baseline planner works (`planners/baseline_planner.py`)

The baseline is a simplified version of the research "Planner 0". Every cycle it:

1. **Makes candidates:**
   * cruise in the current lane (following the car ahead);
   * cruise in the left or right lane, if the marking allows a change;
   * stop at the next stop or yield line;
   * stop 2 m before a predicted conflict (never past a stop line).
2. **Obeys traffic controls:**
   * stops for red (and for yellow if it can stop comfortably);
   * stops at stop signs for 2 s, then continues when the junction is clear.
3. **Checks each candidate:**
   * The plan is extended to 8 s and compared with **constant-velocity predictions** of every road user (small safety margins; 1 s time buffer for crossing traffic).
   * Lane changes also need enough room in front and behind.
4. **Picks the cheapest valid candidate.**
   * Cost = lost progress + lane-change cost + not-in-route-lane cost + harsh-braking cost.
   * If nothing is valid, it brakes harder (up to 6 m/s²). If even that fails, it brakes at the limit.

**Its known weaknesses**, which are ideas for you:

* Constant-velocity prediction: it does not know that cars brake, turn or follow lanes.
* Only one speed choice per lane.
* No pedestrian-intent reasoning.
* A fixed 4 s plan and 8 s check.
* Conservative gaps, which cost time.

## 7. Write a replacement planner

1. **Easiest:** edit `StudentPlanner.plan` in `student_planner.py`. It already works: it keeps its lane, follows the car ahead and stops for red lights.
2. **Or** copy `baseline_planner.py` into `student_planner.py`, rename the class to `StudentPlanner`, and improve it.
3. **Or** add a new planner class and register it in `planners/__init__.py`, e.g. `PLANNERS["team_a"] = TeamAPlanner`.

The rules for any planner:

* a class with `__init__(self)` (no arguments) and `plan(self, obs) -> Trajectory`;
* memory between cycles in `self` is allowed (e.g. "I am in the middle of a lane change");
* use only `obs` (see COMPETITION_RULES.md).

**Testing and debugging:**

```bash
python run_simulation.py --scenario lane_change_overtake --planner student --debug
python evaluate.py --planner baseline student
```

* Press **d** to see the lane centre lines the planner receives, your `alternatives`, the pure-pursuit goal point and the true outlines of all road users.
* Press **p** to pause.
* `print()` inside `plan()` works, but it slows the simulation down.

**What is checked besides the score.** The evaluation also reports traffic-rule violations: running a red light, not stopping fully (below 0.2 m/s) within 10 m before a stop-sign line, and driving with the car's centre on the opposing side of the road. They are printed by `evaluate.py` and shown on screen; they do not change the score. How they are judged is up to the competition rules.

## 8. Bring your own trajectory generator

You may already have a planner (a lattice planner, an MPC, a planner from another course or simulator). Keep it as it is and write a thin **adapter** around it.

**1. Put your code in its own folder** and register the adapter:

```
planners/
    team_a/
        __init__.py          (empty)
        core.py              your planner code, any structure
        planner.py           class TeamAPlanner: the adapter
        requirements.txt     extra packages you need (if any)
```

```python
# planners/__init__.py
from planners.team_a.planner import TeamAPlanner
PLANNERS["team_a"] = TeamAPlanner     # or add it to the PLANNERS dict
```

Run it with `--planner team_a`. Use imports relative to the project (`from planners.team_a.core import ...`) and paths relative to your file (`Path(__file__).parent / "config.yaml"`). Never rely on the current working directory.

**2. The adapter converts in both directions:**

| Your code expects | Get it from the observation |
|---|---|
| reference path / lane centre | `obs.lanes["current"].centerline.xy_m` ((N, 2) points, ego frame), `.s_m`, `.heading_rad`; the ego is at `ego_s_m` on it |
| neighbouring lanes | `obs.lanes.get("left")`, `obs.lanes.get("right")`; use them only if `change_allowed` |
| obstacles | `obs.actors`: centre, heading, size and velocity of each road user (ego frame) |
| ego state | `obs.ego.speed_mps`, `.acceleration_mps2`, `.yaw_rate_rps`, `.steering_rad` |
| traffic rules | `obs.stop_lines` (distance, type, signal state), `obs.junctions`, `obs.crosswalks` |

| Your code returns | Give the simulator |
|---|---|
| timed states (t, x, y, v) | `Trajectory(t_s, x_m, y_m, speed_mps)` directly |
| a path only (no timing) | a speed profile along it (example below) |
| a frame with **y to the left** (ROS, most textbooks) | flip it: `y_m = -y_left`, heading = `-heading_left`. Do the same with every y, vy and heading you read from `obs` |
| positions of the vehicle **centre** or **front axle** | shift to the **rear axle**: `x_rear = x_centre - 1.57 * cos(heading)` (front axle: 3.0 m instead of 1.57 m) |
| world coordinates | the simulator gives none on purpose; plan in the ego frame (the ego is always at (0, 0) facing +x) |

**3. Example: a path-only generator plus a speed profile.** `build_trajectory`, `speed_profile` and `find_lead` work on any object with a `centerline` (a `ReferencePath`), an `ego_s_m` and a `width_m`, so your own path can be used like a lane:

```python
from types import SimpleNamespace

import numpy as np

from planners.planner_tools import build_trajectory, find_lead, speed_profile, time_grid
from simulator.geometry import ReferencePath


class PathAdapterPlanner:
    def __init__(self):
        self.t = time_grid(4.0, 0.1)

    def plan(self, obs):
        xy = my_path_generator(obs)                  # YOUR code: (N, 2) points, ego frame, y RIGHT
        xy = np.vstack(([0.0, 0.0], xy[np.hypot(*xy.T) > 0.5]))       # make it start at the rear axle
        path = SimpleNamespace(centerline=ReferencePath.from_xy(xy), ego_s_m=0.0, width_m=3.5)
        lead = find_lead(path, obs.actors)                             # who is ahead on MY path?
        s, v, a = speed_profile(self.t, obs.ego.speed_mps, obs.ego.acceleration_mps2, obs.desired_speed_mps,
                                lane=path, lead=lead)                  # also slows down in curves of my path
        return build_trajectory(path, self.t, s, np.zeros_like(s), v, a, behavior="MY_PATH")
```

**4. Rules that matter for an adapter:**

* `plan()` is called **20 times per second**, and only the first 50 ms of each plan are driven. If your planner is slow, plan less often inside `plan()` and re-use (shift) the last plan in between; memory in `self` is allowed.
* The trajectory must start within 3 m of (0, 0) and cover at least 1 s (4–8 s is better). Speeds are 0…40 m/s.
* Keep `plan()` under about 50 ms on a normal laptop (the time is shown on screen as *planner compute*).
* Only the `obs` is allowed as input: no reading the world, the scenario files, or other planners.
