# Competition Rules

## 1. Teams and responsibilities

**Planner Team**

* Writes a planner in `planners/student_planner.py`, plus extra planner files if needed, registered in `planners/__init__.py`.
* Makes the car reach its goal **safely**: no collisions, staying on the road, obeying red lights and stop signs.
* Makes it efficient as well: progress and time.

**Scenario Team**

* Writes scenarios in `scenarios/student_scenario.py`, plus extra scenario files if needed, registered in `STUDENT_SCENARIOS`.
* Makes scenarios increasingly difficult **but solvable** by a careful driver.
* Gives each one a clear description of what it tests.

## 2. Common interfaces (do not change them)

| Interface | Contract |
|---|---|
| Planner | a class with `__init__(self)` (no arguments) and `plan(self, obs) -> Trajectory`; registered in `planners/__init__.py` |
| Observation | `simulator/observation.py`: ego state, lanes, road users within sensor range, stop lines, junctions, crosswalks, desired speed, speed limit, distance to goal |
| Trajectory | `simulator/trajectory.py`: `t_s` (from 0, increasing), `x_m`, `y_m` (rear axle, ego frame), `speed_mps`; checked by the simulator |
| Scenario | a function returning `Scenario` (`simulator/scenario_types.py`), built with `scenarios/scenario_tools.py`; registered in `STUDENT_SCENARIOS` |

## 3. Permitted and forbidden modifications

**Allowed for the Planner Team:**

* any code inside your planner files;
* new helper functions of your own;
* reading any file in the project.

**Allowed for the Scenario Team:**

* any scenario built from `scenario_tools.py`;
* new helper functions in your own scenario files.

**Not allowed, for either team:**

* changing anything in `simulator/` (world, observation, controller, vehicle model, evaluation, renderer, simulation loop);
* changing `scenarios/example_scenarios.py`, `evaluate.py` or `run_simulation.py`;
* in a planner, importing or reading the simulated world, other planners' internals, or the scenario definitions. **Only use the `obs` you are given.** Planners must not know which scenario is running: no `if scenario == ...`;
* reading or writing files, using the network, or using random numbers without a fixed seed inside `plan()`;
* making `plan()` very slow on purpose. As a guideline, keep it under about 50 ms per call on a normal laptop.
  * The simulation does not stop on time, so this is a fairness rule, not an automatic check.
  * The time is shown on screen as *planner compute*.

**Why these rules:** the score is computed by the simulator from what the car actually did. Changing the simulator, or giving the planner information a real car would not have, makes the comparison meaningless.

## 4. Safety and scoring

**Score per scenario** (computed only from the simulated world, never from planner return values):

* **Collision or road departure: 0 points.**
* Otherwise: **50 × progress**,
  * plus **30** if the goal is reached,
  * plus **20 × min(1, ideal time / your time)** if the goal is reached.

**Ending a run:**

* A run ends at the first collision, road departure, goal, or time limit.
* A crashing or invalid planner makes the car brake that cycle and counts a *planner error*. The error itself is not punished, but braking usually costs time.

**Safety comes first.** A slower planner that never crashes beats a fast planner that sometimes crashes. Running a red light or a stop sign is not scored automatically, but it is forbidden: judges may check it in the replay and disqualify that scenario's result.

## 5. Fair comparison

* Every planner is evaluated with the **same** command: `python evaluate.py --planner <names> --scenario <names>`.
  * It runs every planner on the **same** scenarios, which are fully deterministic.
  * It creates a **new** planner object for every scenario, so nothing carries over.
* The three example scenarios, plus the scenarios submitted by the Scenario Team, form the test set. The judges may also hold back hidden scenarios built with the same tools.
* The baseline planner is the reference. Do not weaken it; beat it.

**Scenario Team submissions must:**

* start without overlapping road users;
* use realistic speeds (pedestrians up to about 2.5 m/s, cyclists up to about 8 m/s, cars and trucks up to about 15 m/s);
* use only `scenario_tools.py` building blocks;
* be solvable by a careful driver. A scenario where any car must crash (e.g. a car appearing 2 m in front at 15 m/s) will be removed.
* Run the baseline on your scenario and describe what makes it hard.

## 6. Submission checklist

* [ ] `python tests/test_challenge.py` still passes. The tests check the simulator, so they must pass unchanged.
* [ ] `python evaluate.py --planner baseline student` runs without errors.
* [ ] Your planner and scenario files contain your team names.
* [ ] A short README section describes your idea.
