# Autonomous Driving Challenge

A small 2-D driving simulator for a competition between two groups:

* **Scene Generation Group:** builds driving scenes that are hard but fair. The guides call this group the *Scenario Team*.
* **Trajectory Planning Group:** builds planners that drive through those scenes safely and efficiently. The guides call this group the *Planner Team*.

Both groups can see the whole project, but each one only needs part of it.

| Who you are | Read this |
|---|---|
| **Everyone** | [General section](#general-section-for-both-groups), including how to run the code and how [scoring](#6-scoring) works |
| **Scene Generation Group** | [Scene Generation Group section](#scene-generation-group) |
| **Trajectory Planning Group** | [Trajectory Planning Group section](#trajectory-planning-group) |
| **New to Claude Code?** | [CLAUDE_CODE_TUTORIAL.md](CLAUDE_CODE_TUTORIAL.md) |

---

# General section (for both groups)

## 1. Setup (once)

You need **Python 3**. The project has been tested with Python 3.10 on Linux. It has not been tested on Windows or Mac yet, so tell your instructor if something fails there.

1. Download the project: click the green **Code** button on this page and choose **Download ZIP**, or use `git clone`.
2. Open a terminal in the project folder.
3. Run these commands:

```bash
python -m venv venv
# Windows:      venv\Scripts\activate
# macOS/Linux:  source venv/bin/activate
pip install -r requirements.txt          # numpy + opencv-python
python tests/test_challenge.py           # takes a few minutes; should end with "19/19 tests passed"
```

Run the `activate` line again each time you open a new terminal.

## 2. How the project fits together

**The loop, 20 times per second:** world → observation → planner → trajectory check → controller (pure pursuit + speed) → vehicle model and other road users → evaluation → display.

A **scene** sets up the world: the road, the other road users and the goal. A **planner** only sees the observation and returns a trajectory. The simulator drives the car along it and scores what really happened.

```
run_simulation.py           runs one scene with one planner, usually in a window
evaluate.py                 runs planners on scenes and prints a score table
requirements.txt            the packages needed to run the project

scenarios/                  SCENE GENERATION GROUP
    student_scenario.py     <- write your scenes here
    scenario_tools.py       building blocks: roads, cars, trucks, bicycles, pedestrians
    example_scenarios.py    the three common scenes (do not edit)

planners/                   TRAJECTORY PLANNING GROUP
    student_planner.py      <- write your planner here
    <team>/                 <- or put your own planner (code + trained weights) here
    __init__.py             the list of planners; add your own planner here
    baseline_planner.py     the reference planner to read, copy ideas from and beat
    planner_tools.py        helper functions for planners
    learned_example/        example neural-network planner: code, weights, README

training/                   how learned_example was trained (optional)
    collect.py              records training data from the baseline driving random scenes
    train.py                trains the network on that data
    data.npz                the recorded dataset (25 MB)
    env.py                  environment for reinforcement learning (not used for the shipped weights)

simulator/                  the simulator itself (read it if you like, do not change it)
    world.py                the truth: road, your car, other road users, signals
    observation.py          what a planner is allowed to see
    trajectory.py           the trajectory format, and the check on it
    controller.py           pure pursuit + speed control
    vehicle_model.py        kinematic bicycle model
    evaluation.py           metrics and score
    renderer.py             the display
    simulation.py           the loop that connects everything
    scenario_types.py       the Scenario data format
    road_network.py         roads, lanes and junctions
    geometry.py             paths and coordinate maths

tests/test_challenge.py     self-test
results/                    created by evaluate.py; not uploaded to GitHub
```

## 3. Run the visual simulator

```bash
python run_simulation.py
```

A window opens with:

* **Left:** a top-down view that moves with the car.
  * Your car (orange) is at the bottom middle, facing up.
  * Other road users are blue, and pedestrians are red dots. The white arrows show where each one moves in 1 s.
  * The green line is the **planned trajectory**, with a dot every 0.5 s. A red bar marks where the plan stops.
* **Right:** the planner's behaviour, the speed, the controller commands and the live score.

**Keys:** **q** or ESC quit · **p** pause · **d** debug overlays · **v** switch view

When the run ends, the result appears in the window and in the terminal. Press any key in the window to close it.

## 4. All the ways to run the code

Scene and planner files don't run on their own. `python scenarios/student_scenario.py` only gives an error. Instead, pass the **name** of the scene or planner to `run_simulation.py` or `evaluate.py`.

### See what is available

```bash
python run_simulation.py --list
```

| Scene | What happens |
|---|---|
| `lead_vehicle_brakes` | A slower car ahead brakes to a stop, waits, then drives on |
| `lane_change_overtake` | A slow truck is ahead, and a fast car approaches in the left lane |
| `intersection_crossing_traffic` | Cars cross from both sides at a junction with no signals |
| `my_crosswalk` | Example student scene: a pedestrian crosses in front of the car |

| Planner | What it is |
|---|---|
| `baseline` | The reference planner |
| `student` | The simple starter planner; it doesn't change lanes or yield at junctions |
| `learned_example` | Example neural-network planner; only listed after `pip install torch` |

### Watch one run: `run_simulation.py`

```bash
python run_simulation.py                                                 # default: lead_vehicle_brakes with baseline
python run_simulation.py --scenario my_crosswalk                         # choose the scene
python run_simulation.py --scenario lane_change_overtake --planner student   # choose the scene and planner
```

Add any of these options:

| Option | What it does |
|---|---|
| `--scenario NAME` | Choose the scene |
| `--planner NAME` | Choose the planner |
| `--view local_bev`, `global_map` or `both` | Choose the starting view: following the car, the whole map, or both |
| `--debug` | Start with the debug overlays on |
| `--fast` | Keep the window, but don't slow down to real time |
| `--headless` | No window; just print the result |
| `--save-frames frames/` | Save a PNG picture every 0.5 s of simulated time |
| `--list` | List the scenes and planners, then exit |

If no window can open, combine `--headless --save-frames frames/` and look at the pictures instead.

### Get scores: `evaluate.py`

This runs without a window, and every run starts with a new planner object.

```bash
python evaluate.py                                              # baseline on every scene
python evaluate.py --planner student                            # your planner on every scene
python evaluate.py --planner baseline student                   # compare planners side by side
python evaluate.py --scenario lane_change_overtake --planner student         # one scene
python evaluate.py --scenario my_crosswalk lead_vehicle_brakes --planner baseline student   # several of each
python evaluate.py --out my_results                             # save the files to another folder
```

It prints a table and saves `results.json` and `results.csv` in `results/`.

### Other commands

```bash
python tests/test_challenge.py      # self-test: checks that the simulator still works
python training/collect.py 40       # record training data (see the warning in the planning section)
python training/train.py            # train the example network
```

## 5. What the other documents are about

| Document | What it covers | Who needs it |
|---|---|---|
| [CLAUDE_CODE_TUTORIAL.md](CLAUDE_CODE_TUTORIAL.md) | Installing Claude Code on Windows or Mac, writing prompts, and using it to build a scene and train a planner | Everyone |
| [COMPETITION_RULES.md](COMPETITION_RULES.md) | What each group may change, fair play and the submission checklist | Everyone |
| [SCENARIO_GUIDE.md](SCENARIO_GUIDE.md) | How to build scenes: roads, road users, traffic lights, making scenes harder, checking they are fair | Scene Generation Group |
| [PLANNER_GUIDE.md](PLANNER_GUIDE.md) | How to write a planner: coordinates, observation, trajectory, controller, helpers, how the baseline works, using your own generator | Trajectory Planning Group |
| [LEARNING_GUIDE.md](LEARNING_GUIDE.md) | How to train a neural-network planner, anywhere, so that its weights work here | Trajectory Planning Group, if using machine learning |
| [planners/learned_example/README.md](planners/learned_example/README.md) | The example network: inputs, outputs, dataset, training, results and what went wrong | Trajectory Planning Group, if using machine learning |

## 6. Scoring

### Planner score (Trajectory Planning Group)

Each planner gets **0 to 100 points on each scene**. The score comes from `simulator/evaluation.py`, which only looks at what the car **actually did** in the simulated world, never at what your planner returned.

| Result | Points |
|---|---|
| Collision or road departure | **0, and the run ends** |
| Progress | 50 × the fraction of the goal distance driven |
| Goal reached before the time limit | + 30 |
| Speed, only if the goal is reached | + 20 × min(1, ideal time / your time) |

The ideal time is the goal distance divided by the scene's desired speed. The **team total** is the sum of its scores over all scenes.

**Real examples:**

| Run | Score | Why |
|---|---|---|
| `baseline` on `lane_change_overtake` | 98.2 | Overtakes the truck safely and reaches the goal |
| `student` on `lane_change_overtake` | 38.3 | Follows the truck until the time limit, with 77 % progress |
| `student` on `intersection_crossing_traffic` | 0.0 | Crashes into a crossing car |

**What else the table shows:**

| Metric | Meaning |
|---|---|
| collision | Your car's outline touched another road user |
| off-road | A corner of your car left the road surface |
| goal | You drove the goal distance along your route before the time limit |
| progress | The fraction of the goal distance you drove (0–100 %) |
| time | Seconds simulated until the run ended |
| errors | How often your planner crashed or returned an invalid trajectory |
| violations | Red light, no full stop at a stop sign, or driving on the opposing side |

* A run ends at the first collision, road departure, goal or time limit. These are checked every 10 ms of simulated time.
* **Rule violations don't change the score automatically.** Running a red light or a stop sign is still forbidden, and judges may check the replay and disqualify that scene's result.
* **Planner errors cost no points directly.** The simulator brakes the car for that cycle and shows the error in red, and the braking usually costs time.

### Scene score (Scene Generation Group)

A good scene is one that the planners find hard, as long as a careful driver could still solve it.

**Scene score = 100 − the average score of the Trajectory Planning Group's planners on that scene**

**The scene must be valid to earn points.** A scene that isn't valid scores 0. To be valid, it must:

* follow the submission rules in [COMPETITION_RULES.md](COMPETITION_RULES.md), section 5;
* be solvable, shown by the `baseline` planner reaching the goal with no collision or road departure. If the baseline fails, the judges may still approve the scene if they agree a careful driver could solve it.

The baseline is only used for this check. It doesn't count in the average.

The **team score** is the average of its scene scores, so submitting extra easy scenes doesn't help.

**Example with real scores.** Suppose the planners are `student` and `learned_example`, and these four scenes were submitted. The baseline reaches the goal on all of them.

| Scene | `student` | `learned_example` | Average | Scene score |
|---|---|---|---|---|
| `lane_change_overtake` | 38.3 | 37.8 | 38.05 | **61.95** |
| `intersection_crossing_traffic` | 0.0 | 100.0 | 50.00 | **50.00** |
| `lead_vehicle_brakes` | 92.9 | 44.1 | 68.50 | **31.50** |
| `my_crosswalk` | 97.3 | 100.0 | 98.65 | **1.35** |

The team score is (61.95 + 50.00 + 31.50 + 1.35) ÷ 4 = **36.2**.

`evaluate.py` does not calculate the scene score. Work it out from the planner scores in its table, or from `results/results.csv`:

```bash
python evaluate.py --scenario my_scene --planner baseline team_a team_b
```

---

# Scene Generation Group

Your job is to build scenes that are **as hard as possible, but still solvable by a careful driver**. Your goal is a high [scene score](#scene-score-scene-generation-group).

## What you need

| | Files |
|---|---|
| **Change and extend** | `scenarios/student_scenario.py`, plus your own new files in `scenarios/` |
| **Read and understand** | [SCENARIO_GUIDE.md](SCENARIO_GUIDE.md), `scenarios/scenario_tools.py`, section 5 of [COMPETITION_RULES.md](COMPETITION_RULES.md) |
| **Use as examples** | `scenarios/example_scenarios.py`, and `my_crosswalk` in `student_scenario.py` |
| **Look up if needed** | `simulator/scenario_types.py`, which lists every setting a scene can have |
| **Do not change** | `simulator/`, `scenarios/example_scenarios.py`, `evaluate.py`, `run_simulation.py` |
| **Can skip** | `planners/`, `training/`, `PLANNER_GUIDE.md`, `LEARNING_GUIDE.md` |

## What the code gives you

`scenarios/scenario_tools.py` holds your building blocks:

| Type | Available |
|---|---|
| Roads | `two_lane_road`, `curved_two_lane_road`, `crosswalk_road`, `intersection` |
| Traffic lights | `intersection_signals` |
| Road users | `car`, `truck`, `bicycle`, `pedestrian` |
| Lane positions | `LEFT_LANE`, `RIGHT_LANE`, `ONCOMING_LANE`, `INTERSECTION_LANE` |
| Routes through a junction | `STRAIGHT_THROUGH`, `TURN_LEFT`, `TURN_RIGHT` |

## How to add a scene

**1.** Write a function in `scenarios/student_scenario.py`. This is the example already in that file:

```python
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
```

**2.** Register it at the bottom of the file. The name here must match `name=` in the scene, and must be different from the example scene names.

```python
STUDENT_SCENARIOS = {
    "my_crosswalk": my_crosswalk,
    "my_scene": my_scene,
}
```

**3.** Check that it appears, watch it, then score it:

```bash
python run_simulation.py --list
python run_simulation.py --scenario my_scene
python evaluate.py --scenario my_scene --planner baseline student
```

**4.** Watch the replay and ask whether the scene really tests what you intended. A high score doesn't prove that. In the [tutorial](CLAUDE_CODE_TUTORIAL.md), the baseline scored 99.4 on a school-zone scene only because the child never came near the car.

**5.** Make it harder using section 6 of [SCENARIO_GUIDE.md](SCENARIO_GUIDE.md), check that it is fair using section 7, and repeat.

## Example: keep your scenes in your own file

A few scenes fit fine in `student_scenario.py`. For a larger set, or to keep each team's work apart, use your own file. Here is a complete example for a team called `team_b`.

**1.** Create a new file in the `scenarios` folder:

```
scenarios/
    team_b_scenarios.py     <- new: your scenes
    student_scenario.py     <- already there: connect your scenes here
```

**2.** Put your scene in `scenarios/team_b_scenarios.py`:

```python
"""Team B's scenes."""

from scenarios.scenario_tools import LEFT_LANE, MPH, RIGHT_LANE, Scenario, car, curved_two_lane_road, truck


def truck_on_curve() -> Scenario:
    """A slow truck on an S-curve, with traffic passing in the left lane."""
    return Scenario(
        name="truck_on_curve",
        description="Slow truck on an S-curve; a faster car passes in the left lane",
        road=curved_two_lane_road(), route=("curved_road:F2",),
        ego_start_s_m=30.0, ego_speed_mps=8.0, desired_speed_mps=8.0, speed_limit_mps=25 * MPH,
        goal_distance_m=200.0, time_limit_s=45.0,
        actors=(truck(1, s_m=70.0, offset_m=RIGHT_LANE, speed_mps=5.0),
                car(2, s_m=0.0, offset_m=LEFT_LANE, speed_mps=11.0)))
```

**3.** Connect it in `scenarios/student_scenario.py`. Add one line after the other imports at the top, and one line at the bottom:

```python
from scenarios.team_b_scenarios import truck_on_curve        # add this line at the top

STUDENT_SCENARIOS = {
    "my_crosswalk": my_crosswalk,
    "truck_on_curve": truck_on_curve,                        # add this line
}
```

**4.** Run it:

```bash
python run_simulation.py --scenario truck_on_curve
python evaluate.py --scenario truck_on_curve --planner baseline student
```

Both planners score 94.0. If `student` were the only planner on the other team, the scene score would only be 6, so this scene is too easy. It is a starting point for you to make harder.

## Rules for scenes

* Road users must not overlap at the start.
* Speeds must be realistic: pedestrians up to about 2.5 m/s, cyclists up to about 8 m/s, cars and trucks up to about 15 m/s.
* Use only the `scenario_tools.py` building blocks. Plain Python such as loops and helper functions is fine.
* If you use random numbers, use a fixed seed, so that every run of a scene is identical.
* Give each scene a clear `description`, and write down what makes it hard.
* A scene where any car must crash will be removed. For example, a car appearing 2 m ahead at 15 m/s isn't fair.

---

# Trajectory Planning Group

Your job is to get the car to its goal **safely first, then quickly**. Your goal is a high [planner score](#planner-score-trajectory-planning-group) on every scene, including scenes you haven't seen.

## What you need

| | Files |
|---|---|
| **Change and extend** | `planners/student_planner.py` or your own `planners/<team>/` folder; `planners/__init__.py` to register a new planner |
| **Read and understand** | [PLANNER_GUIDE.md](PLANNER_GUIDE.md), `planners/baseline_planner.py`, `planners/planner_tools.py` |
| **Look up if needed** | `simulator/observation.py` (what you can see), `simulator/trajectory.py` (what you return), `simulator/controller.py` (how the car follows it) |
| **Only for machine learning** | [LEARNING_GUIDE.md](LEARNING_GUIDE.md), `planners/learned_example/`, `training/` |
| **Do not change** | `simulator/`, `scenarios/example_scenarios.py`, `evaluate.py`, `run_simulation.py` |
| **Can skip** | `SCENARIO_GUIDE.md` (though `scenarios/` is useful for making your own practice scenes) |

## What the code gives you

| File | What it is |
|---|---|
| `planners/student_planner.py` | A working starter. It keeps its lane, follows the car ahead and stops for red lights. It doesn't change lanes or yield at junctions; that is your job. |
| `planners/baseline_planner.py` | The reference planner. Read it, copy ideas from it, and try to beat it. |
| `planners/planner_tools.py` | Helpers such as `find_lead`, `speed_profile`, `lateral_profile`, `build_trajectory`, `predict_actor` and `first_collision` |

## What a planner must do

The simulator calls `plan()` 20 times per second, and your planner returns a trajectory:

```python
class StudentPlanner:
    def __init__(self):                    # no arguments
        ...

    def plan(self, obs) -> Trajectory:
        ...
        return Trajectory(t_s=..., x_m=..., y_m=..., speed_mps=...)
```

The trajectory gives the position of the car's **rear axle** in the car's own frame, with **x forward and y to the RIGHT**. Many textbooks use y to the left, so read section 1 of the [Planner Guide](PLANNER_GUIDE.md) first.

## Choose your approach

| Approach | Where to start | Guide |
|---|---|---|
| **Improve the starter** | Edit `planners/student_planner.py` | [PLANNER_GUIDE.md](PLANNER_GUIDE.md), section 7 |
| **Improve a copy of the baseline** | Copy `baseline_planner.py` into `student_planner.py` and rename the class to `StudentPlanner` | [PLANNER_GUIDE.md](PLANNER_GUIDE.md), sections 6–7 |
| **Use your own generator** | Put your code and a thin adapter in `planners/<team>/` | [PLANNER_GUIDE.md](PLANNER_GUIDE.md), section 8 |
| **Train a neural network** | Follow `planners/learned_example/` | [LEARNING_GUIDE.md](LEARNING_GUIDE.md) |

## Example: create your own planner

Here is a complete planner that runs, for a team called `team_a`. To work in `student_planner.py` instead, you can skip steps 1 and 3.

**1.** Create a folder with two files in it:

```
planners/
    team_a/
        __init__.py     an empty file; it just has to exist
        planner.py      your planner
```

**2.** Put this in `planners/team_a/planner.py`:

```python
"""Team A's planner: stay in the lane and follow the car ahead."""

from planners.planner_tools import build_trajectory, find_lead, lateral_profile, speed_profile, time_grid


class TeamAPlanner:
    def __init__(self):
        self.t = time_grid(horizon_s=4.0, dt_s=0.1)      # plan 4 s ahead, one point every 0.1 s

    def plan(self, obs):
        lane = obs.lanes["current"]                      # the lane we are driving in
        lead = find_lead(lane, obs.actors)               # the vehicle ahead of us, if any

        # How far to drive, and how fast: cruise, but slow down for the vehicle ahead
        s, v, a = speed_profile(self.t, obs.ego.speed_mps, obs.ego.acceleration_mps2,
                                obs.desired_speed_mps, lane=lane, lead=lead)

        # Where to be sideways: steer smoothly to the lane centre (offset 0)
        d = lateral_profile(lane.ego_d_m, 0.0, 0.0, max(8.0, 1.5 * obs.ego.speed_mps), s)

        # Turn these into the x, y and speed points the simulator needs
        return build_trajectory(lane, self.t, s, d, v, a, behavior="KEEP_LANE")
```

**3.** Register it in `planners/__init__.py` by adding the two marked lines:

```python
from planners.baseline_planner import BaselinePlanner
from planners.student_planner import StudentPlanner
from planners.team_a.planner import TeamAPlanner         # add this line

PLANNERS = {
    "baseline": BaselinePlanner,
    "student": StudentPlanner,
    "team_a": TeamAPlanner,                              # add this line
}
```

**4.** Run it:

```bash
python run_simulation.py --scenario lane_change_overtake --planner team_a
python evaluate.py --planner baseline team_a
```

It scores 228.5 out of 400 on the four shipped scenes, because it never overtakes and crashes at the junction. It also ignores traffic lights; `student_planner.py` shows how to stop for them. Those are the problems for you to solve.

## Test your planner

```bash
python run_simulation.py --scenario lane_change_overtake --planner student --debug   # watch it, with overlays
python evaluate.py --planner baseline student                                         # compare with the baseline
```

Press **d** in the window to turn the overlays on or off, and **p** to pause. `print()` works inside `plan()`, but it slows the simulation down.

Current scores to beat, out of 400:

| Planner | Total |
|---|---|
| `baseline` | 385.4 |
| `learned_example` | 281.9 |
| `student` (starter) | 228.5 |

## Train a neural network (optional)

You don't need any of this unless you choose machine learning. To try the example:

```bash
pip install -r training/requirements.txt                             # PyTorch and gymnasium
python run_simulation.py --scenario my_crosswalk --planner learned_example
```

To train it again:

```bash
python training/collect.py 40      # record data from 40 random-scene episodes (400 if you leave out the number)
python training/train.py           # train the network
```

> **Warning:** these two commands **overwrite** `training/data.npz` and the shipped weights in `planners/learned_example/weights/model.pt`. Copy `learned_example` into your own folder and change the paths first, as Part B of the [tutorial](CLAUDE_CODE_TUTORIAL.md) shows.

In our test on 4 CPU cores, recording 40–120 episodes took about 10–20 minutes, and training took under a minute. No graphics card is needed.

More data doesn't always give a better planner, so always check the result with `evaluate.py`.

## Rules for planners

* Use **only the `obs` you are given**. Don't read the simulated world, the scene definitions or other planners.
* Don't check which scene is running, so no `if scenario == ...`. Judges may also use hidden scenes.
* Inside `plan()`, don't read or write files, don't use the network, and don't use random numbers without a fixed seed.
* Keep `plan()` under about 50 ms per call on a normal laptop. Nothing enforces this automatically, but it is a fairness rule. The time appears on screen as *planner compute*.

Before you submit, go through the checklist in section 6 of [COMPETITION_RULES.md](COMPETITION_RULES.md).
