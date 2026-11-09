# Learning-Based Planner Guide

This guide is for Planner Teams that want a **neural network** (or any other trained model) to plan. Read [PLANNER_GUIDE.md](PLANNER_GUIDE.md) first: it explains the observation, the trajectory and the controller.

**The idea.** You may train **anywhere**: on your laptop, on a GPU server, in Colab, with data from this simulator or from somewhere else. What ends up in this project is **code + weights** inside `planners/<your_team>/`, run with the normal commands (`run_simulation.py`, `evaluate.py`). The weights work here only if the model sees **exactly the same inputs** and produces **exactly the same kind of outputs** during training and here.

**A complete working example is included:** the planner `learned_example` (code and weights in `planners/learned_example/`, training code and dataset in `training/`). Its README, [planners/learned_example/README.md](planners/learned_example/README.md), describes its features, outputs, dataset, training and results. Run it with:

```bash
pip install torch
python run_simulation.py --scenario intersection_crossing_traffic --planner learned_example
```

The easiest start: copy `planners/learned_example/` to `planners/<your_team>/` and the scripts in `training/`, rename the imports, register your planner, and improve it.

## 1. The contract your model must respect

| | In this simulator |
|---|---|
| Called | `plan(obs)` 20 times per second (every 50 ms of simulated time) |
| Input | `obs`, an `Observation` (`simulator/observation.py`). Nothing else may be used (COMPETITION_RULES.md) |
| Frame | ego frame: **x forward, y to the RIGHT**, origin at the **rear axle**, headings positive to the right |
| Units | metres, seconds, m/s, radians |
| Perception | perfect positions and velocities of road users inside about 80 m ahead / 40 m behind / 40 m to the sides |
| Output | a `Trajectory`: times from 0, rear-axle positions (ego frame) and speeds; at least 1 s long, starting within 3 m of (0, 0) |
| Execution | pure pursuit steering + speed controller (max +2 / −6 m/s²), kinematic bicycle, 0.2 s actuator lag. Only the first 50 ms of each plan are driven before you plan again |
| Hardware | the simulator sets up no GPU: plan for CPU inference, about 50 ms per `plan()` call at most (COMPETITION_RULES.md) |
| Determinism | the same scenario must give the same result every run: `model.eval()`, no sampling at inference |
| Weights | load them **once**, in `__init__`, from a path relative to your planner file (the rules forbid reading files inside `plan()`) |

## 2. The golden rule: one copy of the conversion code

The code that turns an observation into numbers (`features.py`) and the network's numbers into a trajectory (`model.py`) lives **inside your planner folder** and is imported by **both** your training scripts and your planner. Never copy it into a notebook and change it there. If the features differ by one sign or one scale factor between training and running, the weights are useless, and nothing will crash to tell you.

```
planners/
    <your_team>/
        __init__.py          (empty)
        features.py          obs -> feature vector           (training AND planner)
        model.py             network + outputs <-> Trajectory  (training AND planner)
        planner.py           your planner class: loads the weights, runs the network
        weights/model.pt     the trained weights
        requirements.txt     extra packages your planner needs, e.g. "torch>=2.0"
training/                    your training code (any place; the example scripts expect it in the project folder)
```

Register the planner in `planners/__init__.py`, e.g. `PLANNERS["my_team"] = MyTeamPlanner`. `learned_example` is registered with a small function instead of the class, so that the project still runs for people without PyTorch. Copy that pattern if your planner needs extra packages.

## 3. Step by step, following the example

### Step 1: features (`features.py`)

A **fixed-length** vector built only from `obs`: ego state, lane shape ahead, neighbouring lanes, the next stop line and signal, and the nearest road users (fixed number of slots, empty slots are zeros). `FEATURE_VERSION` is stored with the weights, so that old weights refuse to load after you change the features.

Use inputs a real car would have. The example uses the **posted speed limit** (`obs.speed_limit_mps`) and never `obs.desired_speed_mps`: on a real road you know the limit (25 mph in a neighbourhood, 35 mph on an urban street), not the speed someone "wants". In the training scenes the expert drives at the limit, so the network learns its cruise speed from the limit alone, and its speeds are clipped to the limit.

Better features are the easiest improvement: more lane points, the neighbouring lanes' shapes, junction distance and movement, crosswalk distances, or a bird's-eye image for a CNN built from the same `obs`.

### Step 2: what the network outputs (`model.py`)

Every choice must end in a `Trajectory`:

| Network output | Converted with | Notes |
|---|---|---|
| waypoints (x, y, v) relative to the car | `Trajectory(t, x, y, v)` | simplest, but drifts (see section 5) |
| **offset from the lane centre and speed** at fixed times | integrate speed → distance, then `planner_tools.build_trajectory(lane, t, s, d, v)` | **used by the example**; returns to the lane centre, follows curves and turns |
| a high-level decision (target speed, target lane, stop point) | `speed_profile` + `lateral_profile` + `build_trajectory` | smooth by construction; good for reinforcement learning (`training/env.py` does this) |
| a score for each candidate trajectory of a classical generator | pick the best candidate | "learned cost function"; very safe |

`model.py` also contains the label conversion (`target_from_trajectory`): it turns an expert's trajectory into exactly the numbers the network must output.

### Step 3: training data

**a) Imitation learning in this simulator** (`training/collect.py`, used by the example):

* `random_scenario(seed)` builds random scenes **from `scenario_tools` only**: random roads, routes, traffic, pedestrians, signals. They never need to be registered; `Simulation(scenario, planner)` runs them directly.
* A `Recorder` wraps an expert planner (the baseline, or your own classical planner). It drives with the expert and stores `encode(obs)` and `target_from_trajectory(expert_trajectory, obs)` at every step.
* Episodes where the expert crashed are dropped. One simulation runs per CPU core (`multiprocessing.Pool`).

**b) Reinforcement learning** (`training/env.py`): a [gymnasium](https://gymnasium.farama.org/) environment. Each `step(action)` is one 50 ms planning cycle: the action becomes a trajectory, then the simulator's controller, physics and evaluation run. The reward is built from the same evaluation as the score (progress, collision, road departure, rule violations). It passes `gymnasium.utils.env_checker.check_env`:

```python
from training.env import DrivingEnv
from training.collect import random_scenario

env = DrivingEnv(random_scenario)          # any function seed -> Scenario
obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

In your planner, use the **same** `encode` and action conversion as in the environment.

**c) Data from another simulator or dataset.** Allowed, but convert it to this simulator's conventions **before** training:

* **y to the right.** Most simulators, ROS and datasets use y to the left: flip the sign of every y, vy and heading.
* **Rear axle.** Convert vehicle-centre positions to the rear axle (1.57 m behind the centre of this car; the wheelbase is 3.0 m).
* **Ego frame**, re-centred and rotated at every planning step.
* **Timing:** 20 Hz planning, outputs at the times your `model.py` uses.
* **Same feature code:** build an `Observation` (or an object with the same fields) from the other data and call the same `encode()`.

Then fine-tune in this simulator: closed-loop behaviour here (controller, lag, braking limits) is what is scored.

### Step 4: training (`training/train.py`)

* Normalise inputs and outputs, and **save the normalisation together with the weights**: it is part of the model.
* Features that never vary in your data must not be divided by their (zero) standard deviation (section 5).
* Save tensors only, and load with `torch.load(..., map_location="cpu", weights_only=True)`.

### Step 5: the planner (`planner.py`)

* Load weights in `__init__` with a path relative to the file: `Path(__file__).resolve().parent / "weights" / "model.pt"`. It then works from any working directory.
* `model.eval()`, `torch.inference_mode()`, `torch.set_num_threads(1)` (small networks are faster single-threaded).
* Add a **safety filter**. A network will sometimes output something unsafe or absurd, and one collision scores 0. The example brakes when the output is implausible or when the plan would hit a predicted road user (`planner_tools.first_collision`).

## 4. Verify before you hand in

```bash
python evaluate.py --planner baseline <your_team>     # the "errors" column must be 0
python run_simulation.py --scenario lane_change_overtake --planner <your_team> --debug
cd /tmp && python /path/to/AutonomousDrivingChallenge/evaluate.py --planner <your_team>   # from another folder
```

* [ ] 0 planner errors in every scenario. An invalid trajectory is replaced by braking and counted as an error.
* [ ] Running twice gives the same scores.
* [ ] *planner compute* on screen stays well below 50 ms.
* [ ] It works in a fresh virtual environment with `requirements.txt` + your `requirements.txt`.
* [ ] `python tests/test_challenge.py` still passes.

## 5. Lessons from building the example

These all happened while building `learned_example`. Details are in its README.

1. **A feature that never varied in training explodes at test time** when you normalise by its standard deviation (signal states on a dataset without traffic lights). The network then output 180,000 m/s.
2. **The network only knows what it has seen.** Trained on straight roads only, it failed every junction and crosswalk. Random scenes of every type fixed that. The test scenarios can include ones you never saw.
3. **Imitation learning never shows how to recover.** An expert is always centred, so a network predicting positions relative to the car never learned to steer back after a drift, and slowly left the road. Lane-relative outputs fixed it. DAgger is the general fix: drive with your network, record what the expert would do in those states, retrain.
4. **Constrain the output with what you know for sure.** Clip lateral offsets to lanes that exist and may be entered. Check plausibility. Check collisions.
5. **A lower training loss is not a better driver.** A training change that lowered the loss made the example crash at the intersection. Judge models with `evaluate.py` (closed loop), not by their loss.
6. **Hybrid planners are fine and often best**, for example: a network chooses speed and lane, classical tools build the trajectory, and a rule-based check vetoes unsafe plans.
