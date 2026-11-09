"""The closed simulation loop (same structure and rates as the original):

    world truth -> observation -> PLANNER -> trajectory check -> controller
                -> vehicle model (5 sub-steps) + actors -> evaluation

Planning runs at 20 Hz; the controller command is held for the 50 ms cycle
while the vehicle model integrates it in 5 sub-steps of 10 ms.  The
evaluation runs after every sub-step, so short contacts are not missed and
the run ends at the sub-step where it is decided.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import math
import time
import traceback
from typing import Callable, Optional, Tuple

import numpy as np

from simulator.controller import ControlCommand, TrajectoryFollower
from simulator.evaluation import EvaluationResult, Evaluator
from simulator.geometry import frenet_to_cartesian, world_to_ego
from simulator.observation import Observation, build_observation
from simulator.road_network import RoadNetwork
from simulator.scenario_types import Scenario
from simulator.trajectory import Trajectory, check_trajectory
from simulator.world import World


PLANNING_RATE_HZ = 20.0
PLANT_SUBSTEPS = 5
EMERGENCY_DECEL_MPS2 = 6.0


# ---------------------------------------------------------------------------
# Display-only data (never given to the planner)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ActorTruth:
    actor_id: int
    class_id: int
    x_forward_m: float
    y_right_m: float
    heading_rad: float
    length_m: float
    width_m: float


@dataclass(frozen=True, eq=False)
class SceneSnapshot:
    """Road geometry and traffic controls in the EGO frame (for drawing)."""

    actors: tuple
    surfaces: tuple = ()
    markings: tuple = ()          # (style, polyline, arc length)
    stop_lines: tuple = ()
    crosswalks: tuple = ()
    stop_signs: tuple = ()
    signals: tuple = ()           # (x, y, state)
    sidewalks: tuple = ()         # (polyline, width)


@dataclass(frozen=True, eq=False)
class GlobalView:
    """World-frame data for the north-up overview map."""

    network: RoadNetwork
    ego_pose: tuple
    ego_history_xy: np.ndarray
    actors: tuple = ()            # (id, class, x, y, heading, length, width)
    signals: tuple = ()           # (x, y, state)


@dataclass(frozen=True, eq=False)
class DisplayFrame:
    scenario_name: str
    scenario_description: str
    planner_name: str
    time_s: float
    observation: Observation
    trajectory: Trajectory
    command: ControlCommand
    scene: SceneSnapshot
    global_view: GlobalView
    result: EvaluationResult
    planner_ms: float
    planner_error: Optional[str] = None


_VIEW = (-70.0, 110.0, -70.0, 70.0)     # ego-frame window kept for drawing


def _ego_points(ego, points: np.ndarray) -> np.ndarray:
    x, y = world_to_ego(ego.x_m, ego.y_m, ego.yaw_rad, points[:, 0], points[:, 1])
    return np.column_stack((x, y))


def _visible_runs(points: np.ndarray, arc=None):
    x0, x1, y0, y1 = _VIEW
    inside = (points[:, 0] >= x0) & (points[:, 0] <= x1) & (points[:, 1] >= y0) & (points[:, 1] <= y1)
    if not inside.any():
        return []
    idx = np.flatnonzero(inside)
    runs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
    return [(points[r[0]:r[-1] + 1], None if arc is None else arc[r[0]:r[-1] + 1]) for r in runs if r.size >= 2]


def scene_snapshot(world: World) -> SceneSnapshot:
    ego, net = world.ego, world.network
    actors = []
    for a in world.present_actors():
        x, y = world_to_ego(ego.x_m, ego.y_m, ego.yaw_rad, a.x_m, a.y_m)
        actors.append(ActorTruth(a.actor_id, a.class_id, float(x), float(y), a.heading_rad - ego.yaw_rad,
                                 a.length_m, a.width_m))
    markings = []
    for m in net.markings:
        for run, arc in _visible_runs(_ego_points(ego, m.points_m), m.arc_m):
            markings.append((m.style.value, run, arc))
    signals = []
    for sig in net.signals.values():
        x, y = world_to_ego(ego.x_m, ego.y_m, ego.yaw_rad, *sig.position_m)
        signals.append((float(x), float(y), world.signal_state(sig.signal_id).value))
    return SceneSnapshot(
        actors=tuple(actors),
        surfaces=tuple(_ego_points(ego, poly) for poly in net.surfaces),
        markings=tuple(markings),
        stop_lines=tuple(_ego_points(ego, np.array([sl.p0_m, sl.p1_m])) for sl in net.stop_lines.values()),
        crosswalks=tuple(_ego_points(ego, cw.polygon_m) for cw in net.crosswalks),
        stop_signs=tuple(tuple(float(v) for v in world_to_ego(ego.x_m, ego.y_m, ego.yaw_rad, *s.position_m))
                         for s in net.stop_signs),
        signals=tuple(signals),
        sidewalks=tuple((run, sw.width_m) for sw in net.sidewalks
                        for run, _ in _visible_runs(_ego_points(ego, sw.centerline_m))),
    )


# ---------------------------------------------------------------------------
# Fallback used when the planner fails (simulator side, not a planner)
# ---------------------------------------------------------------------------
def braking_trajectory(obs: Observation, decel_mps2: float = EMERGENCY_DECEL_MPS2) -> Trajectory:
    """Brake to a stop along the current lane, keeping the current offset."""
    lane = obs.lanes["current"]
    t = np.arange(0.0, 4.0 + 1e-9, 0.1)
    v0 = obs.ego.speed_mps
    v = np.maximum(v0 - decel_mps2 * t, 0.0)
    t_stop = v0 / decel_mps2
    s = np.where(t < t_stop, v0 * t - 0.5 * decel_mps2 * t ** 2, v0 * t_stop - 0.5 * decel_mps2 * t_stop ** 2)
    x, y = frenet_to_cartesian(lane.centerline, lane.ego_s_m + s, np.full_like(s, lane.ego_d_m))
    x, y = x - x[0], y - y[0]                 # start exactly at the rear axle
    return Trajectory(t, x, y, v, behavior="FALLBACK_BRAKE", notes=["simulator fallback: planner failed"])


def _sanitize_display_fields(traj: Trajectory) -> None:
    """``behavior``, ``notes`` and ``alternatives`` are display only.  Make
    them safe to draw so that a planner can never crash the renderer."""
    traj.behavior = str(traj.behavior)
    notes = traj.notes if isinstance(traj.notes, (list, tuple)) else [traj.notes]
    traj.notes = [str(n) for n in notes if n is not None]
    alternatives = traj.alternatives if isinstance(traj.alternatives, (list, tuple)) else []
    traj.alternatives = [a for a in alternatives if check_trajectory(a) is None]


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------
class Simulation:
    def __init__(self, scenario: Scenario, planner, planner_name: str = "planner",
                 verbose_errors: bool = True) -> None:
        self.scenario = scenario
        self.world = World(scenario)
        self.planner = planner
        self.planner_name = planner_name
        self.follower = TrajectoryFollower()
        self.evaluator = Evaluator(self.world, planner_name)
        self.cycle_dt_s = 1.0 / PLANNING_RATE_HZ
        self.history = deque(maxlen=4000)
        self.verbose_errors = verbose_errors
        self._error_printed = False

    @property
    def done(self) -> bool:
        return self.evaluator.finished

    @property
    def result(self) -> EvaluationResult:
        return self.evaluator.result

    def step(self) -> DisplayFrame:
        world = self.world
        obs = build_observation(world)

        # --- planner (only sees the observation) ---------------------------
        error = None
        started = time.perf_counter()
        try:
            traj = self.planner.plan(obs)
        except Exception as exc:                     # a crashing planner must not crash the simulator
            traj = None
            error = f"planner raised {type(exc).__name__}: {exc}"
            if self.verbose_errors and not self._error_printed:
                traceback.print_exc()
                self._error_printed = True
        planner_ms = (time.perf_counter() - started) * 1e3
        if error is None:
            problem = check_trajectory(traj)
            if problem is not None:
                error = f"invalid trajectory: {problem}"
                if self.verbose_errors and not self._error_printed:
                    print(f"[simulator] {error}")
                    self._error_printed = True
        if error is not None:
            self.evaluator.result.planner_errors += 1
            traj = braking_trajectory(obs)
        else:
            _sanitize_display_fields(traj)

        # --- controller (pure pursuit + speed control) -----------------------
        command = self.follower.compute(traj, world.ego.speed_mps)

        # --- display snapshot (state the plan was made in) ------------------
        ego = world.ego
        self.history.append((ego.x_m, ego.y_m))
        present = world.present_actors()
        global_view = GlobalView(
            world.network, (ego.x_m, ego.y_m, ego.yaw_rad), np.array(self.history),
            tuple((a.actor_id, a.class_id, a.x_m, a.y_m, a.heading_rad, a.length_m, a.width_m) for a in present),
            tuple((sig.position_m[0], sig.position_m[1], world.signal_state(sid).value)
                  for sid, sig in world.network.signals.items()))
        scene = scene_snapshot(world)
        frame_time = world.time_s

        # --- vehicle model and actors, independent evaluation -----------------
        sub_dt = self.cycle_dt_s / PLANT_SUBSTEPS
        for _ in range(PLANT_SUBSTEPS):
            world.advance(command.steering_angle_rad, command.acceleration_mps2, sub_dt)
            self.evaluator.update()
            if self.evaluator.finished:
                break
        return DisplayFrame(self.scenario.name, self.scenario.description, self.planner_name, frame_time,
                            obs, traj, command, scene, global_view, replace(self.evaluator.result),
                            planner_ms, error)

    def run(self, on_frame: Optional[Callable[[DisplayFrame], bool]] = None) -> EvaluationResult:
        """Run to the end.  ``on_frame(frame)`` may return False to stop early."""
        while not self.done:
            frame = self.step()
            if on_frame is not None and on_frame(frame) is False:
                if self.evaluator.result.end_reason == "running":
                    self.evaluator.result.end_reason = "stopped by user"
                break
        return self.evaluator.result
