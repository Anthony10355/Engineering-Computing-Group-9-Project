"""Baseline planner: a simplified version of the research "Planner 0".

Kept from Planner 0 (same ideas, much less code):
  1. Build a few CANDIDATE trajectories every cycle:
       * keep driving in the current lane (cruise, following the vehicle ahead);
       * change to the left / right lane if the lane marking allows it;
       * stop at the next stop line or yield line;
       * stop just before a predicted conflict (e.g. a crossing pedestrian).
     Longitudinal motion: cruise speed, curve speed limit, Intelligent Driver
     Model car following, stop law.  Lateral motion: quintic polynomial.
  2. Traffic controls: stop for red (and for yellow if comfortable), stop at
     stop signs for 2 s, then go when the junction is clear.
  3. CHECK every candidate against constant-velocity predictions of all
     observed road users for 8 s (the 4 s plan, then continuing at its final
     speed).  Lane changes also need enough space in the target lane.
  4. SELECT the valid candidate with the lowest cost (less progress, lane
     changes, leaving the route lane and harsh braking cost more).  If none
     is valid, brake harder; if that fails too, brake at the limit.

Removed from Planner 0: lane-aware multi-hypothesis prediction, the separate
safety certificate, junction commitment logic, nudging, lane-drop handling,
pedestrian signals and many tuning details.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import List, Optional

import numpy as np

from planners.planner_tools import (
    EGO_REAR_AXLE_TO_FRONT_M, SpeedLimits, Lead, actor_position_on_lane, build_trajectory, extend_trajectory,
    find_lead, first_collision, lateral_profile, speed_along_lane, speed_profile, time_grid,
)
from simulator.geometry import reference_state
from simulator.trajectory import Trajectory


NORMAL_LIMITS = SpeedLimits()
FIRM_LIMITS = SpeedLimits(max_decel_mps2=6.0, max_jerk_mps3=15.0)
CHECK_HORIZON_S = 8.0
STOP_SIGN_WAIT_S = 2.0
CROSSING_TIME_BUFFER_S = 1.0     # keep 1 s between us and crossing traffic


@dataclass
class Candidate:
    name: str
    lane_name: str
    trajectory: Trajectory
    lane: object
    lead: Optional[Lead] = None
    stop_reason: Optional[str] = None
    valid: bool = True
    reason: str = ""
    cost: float = math.inf
    conflict_s: Optional[float] = None        # lane s of the first predicted conflict


class BaselinePlanner:
    """Every planner has a ``plan(observation) -> Trajectory`` method."""

    def __init__(self) -> None:
        self.t = time_grid()
        self.target_lane_id: Optional[str] = None      # lane change in progress
        self.last_change_time_s = -math.inf
        self.stop_sign_done: set = set()
        self.stop_sign_wait_start: Optional[float] = None

    # ==================================================================
    def plan(self, obs) -> Trajectory:
        current = obs.lanes["current"]
        if self.target_lane_id is not None and current.lane_id == self.target_lane_id \
                and abs(current.ego_d_m) < 0.5:
            self.target_lane_id = None                 # lane change finished
            self.last_change_time_s = obs.time_s

        stop_s, stop_reason = self._traffic_control_stop(obs)
        candidates = self._make_candidates(obs, NORMAL_LIMITS, stop_s, stop_reason)
        for c in candidates:
            self._check(obs, c)
        # stop just before the first predicted conflict of the current-lane cruise plan
        cruise = next((c for c in candidates if c.name == "cruise" and c.lane_name == "current"), None)
        if cruise is not None and not cruise.valid and cruise.conflict_s is not None:
            extra = self._stop_candidate(obs, current, self._conflict_stop(obs, cruise), "conflict ahead",
                                         NORMAL_LIMITS)
            if extra is not None:
                self._check(obs, extra)
                candidates.append(extra)
        for c in candidates:
            self._cost(obs, c)

        valid = [c for c in candidates if c.valid]
        emergency = False
        if not valid:                                   # brake harder
            firm = self._make_candidates(obs, FIRM_LIMITS, stop_s, stop_reason, lanes=("current",))
            if cruise is not None and cruise.conflict_s is not None:
                extra = self._stop_candidate(obs, current, self._conflict_stop(obs, cruise), "conflict ahead",
                                             FIRM_LIMITS)
                if extra is not None:
                    firm.append(extra)
            for c in firm:
                self._check(obs, c)
                self._cost(obs, c)
            valid = [c for c in firm if c.valid]
            candidates += firm
            emergency = True
        if valid:
            best = min(valid, key=lambda c: c.cost)
        else:                                           # nothing is safe: brake at the limit
            best = self._full_brake(obs)
            candidates.append(best)
            emergency = True

        if best.lane_name != "current" and self.target_lane_id is None:
            self.target_lane_id = best.lane.lane_id            # a lane change starts
        elif best.lane_name == "current" and self.target_lane_id is not None \
                and best.lane.lane_id != self.target_lane_id:
            self.target_lane_id = None                           # lane change aborted
        traj = best.trajectory
        traj.behavior = self._behavior(obs, best, emergency)
        traj.notes = self._notes(best, candidates)
        traj.alternatives = []
        for c in candidates:
            if c is not best:
                c.trajectory.rejected = not c.valid
                traj.alternatives.append(c.trajectory)
        return traj

    # ==================================================================
    # Candidates
    # ==================================================================
    def _make_candidates(self, obs, limits, stop_s, stop_reason, lanes=("current", "left", "right")):
        out: List[Candidate] = []
        current = obs.lanes["current"]
        near_junction = any(j.entry_distance_m < 25.0 and j.exit_distance_m > 0.0 for j in obs.junctions)
        for name in lanes:
            lane = obs.lanes.get(name)
            if lane is None:
                continue
            if name != "current":
                if not lane.change_allowed or near_junction:
                    continue
                if obs.time_s - self.last_change_time_s < 3.0 and self.target_lane_id is None:
                    continue                                 # no ping-pong lane changes
            lead = find_lead(lane, obs.actors)
            if name != "current":
                # until we are in the new lane, the car ahead in our lane still matters
                own_lead = find_lead(current, obs.actors)
                if own_lead is not None and (lead is None or own_lead.gap_m < lead.gap_m):
                    lead = own_lead
            out.append(self._lane_candidate(obs, lane, "cruise", lead, stop_s, stop_reason, limits))
        # stop at the next stop / yield line (junction approach), current lane only
        line = next((sl for sl in obs.stop_lines if sl.distance_m - EGO_REAR_AXLE_TO_FRONT_M > -0.5), None)
        if line is not None and line.distance_m < 80.0 and stop_s is None:
            c = self._stop_candidate(obs, current, line.distance_m - EGO_REAR_AXLE_TO_FRONT_M - 1.0,
                                     "yield line" if line.control is None else "stop line", limits)
            if c is not None:
                out.append(c)
        return out

    @staticmethod
    def _conflict_stop(obs, cruise: Candidate) -> float:
        """Rear-axle stop distance 2 m before a predicted conflict, but never
        past a stop / yield line that is still ahead (do not wait inside a junction)."""
        stop = cruise.conflict_s - obs.lanes["current"].ego_s_m - 2.0
        for line in obs.stop_lines:
            line_stop = line.distance_m - EGO_REAR_AXLE_TO_FRONT_M - 1.0
            if line_stop > -0.5:
                return min(stop, line_stop)
        return stop

    def _lane_candidate(self, obs, lane, name, lead, stop_s, stop_reason, limits) -> Candidate:
        s_rel, v, a = speed_profile(self.t, obs.ego.speed_mps, obs.ego.acceleration_mps2, obs.desired_speed_mps,
                                    lane=lane, stop_s_m=stop_s, lead=lead, limits=limits)
        d = self._lateral(obs, lane, s_rel, changing=(lane.name != "current"))
        traj = build_trajectory(lane, self.t, s_rel, d, v, a)
        return Candidate(name, lane.name, traj, lane, lead, stop_reason if stop_s is not None else None)

    def _stop_candidate(self, obs, lane, stop_rear_axle, reason, limits) -> Optional[Candidate]:
        """Stop with the REAR AXLE at most ``stop_rear_axle`` metres ahead of where it is now."""
        if stop_rear_axle < -0.5:
            return None
        lead = find_lead(lane, obs.actors)
        s_rel, v, a = speed_profile(self.t, obs.ego.speed_mps, obs.ego.acceleration_mps2, obs.desired_speed_mps,
                                    lane=lane, stop_s_m=max(stop_rear_axle, 0.0), lead=lead, limits=limits)
        d = self._lateral(obs, lane, s_rel, changing=False)
        traj = build_trajectory(lane, self.t, s_rel, d, v, a)
        return Candidate("stop", lane.name, traj, lane, lead, reason)

    def _lateral(self, obs, lane, s_rel, changing: bool) -> np.ndarray:
        """Quintic from the current offset back to the lane centre.

        The time to get there grows with the REMAINING offset, so that the
        sideways acceleration stays bounded (the peak of a quintic over
        distance D in time T is about 5.77 D / T^2)."""
        heading_lane = float(reference_state(lane.centerline, lane.ego_s_m)[2])
        slope0 = math.tan(max(-1.0, min(1.0, -heading_lane)))   # ego heading is 0 in the ego frame
        v = max(obs.ego.speed_mps, 2.0)
        offset = abs(lane.ego_d_m)
        if changing:
            t_go = max(2.0, math.sqrt(5.77 * offset / 1.0))       # lane change: lateral accel <= 1.0 m/s^2
            length = max(20.0, v * t_go)                          # and never steeper than ~20 m per lane
        else:
            t_go = max(1.5, math.sqrt(5.77 * offset / 1.5))       # keep lane: lateral accel <= 1.5 m/s^2
            length = max(6.0, v * t_go)
        return lateral_profile(lane.ego_d_m, slope0, 0.0, length, s_rel)

    def _full_brake(self, obs) -> Candidate:
        lane = obs.lanes["current"]
        s_rel, v, a = speed_profile(self.t, obs.ego.speed_mps, obs.ego.acceleration_mps2, 0.0, lane=lane,
                                    stop_s_m=0.0, limits=SpeedLimits(max_decel_mps2=6.0, max_jerk_mps3=30.0))
        d = self._lateral(obs, lane, s_rel, changing=False)
        traj = build_trajectory(lane, self.t, s_rel, d, v, a)
        return Candidate("full_brake", "current", traj, lane, None, "no safe option", valid=True, cost=0.0)

    # ==================================================================
    # Traffic controls
    # ==================================================================
    def _traffic_control_stop(self, obs):
        """Rear-axle distance at which we must be stopped for a signal / stop sign."""
        for line in obs.stop_lines:
            front_dist = line.distance_m - EGO_REAR_AXLE_TO_FRONT_M
            if front_dist < -0.5:
                continue                                  # already past this line
            stop_at = front_dist - 1.0
            if line.control == "signal" and line.signal_state in ("RED", "YELLOW"):
                v = obs.ego.speed_mps
                needed = v * v / (2.0 * max(stop_at, 0.1))
                if line.signal_state == "YELLOW" and needed > NORMAL_LIMITS.max_decel_mps2:
                    continue                              # too close to stop for yellow: go
                if line.signal_state == "RED" and front_dist < 0.5 and v > 3.0:
                    continue                              # already at the line and moving: clear it
                return max(stop_at, 0.0), f"{line.signal_state.lower()} light"
            if line.control == "stop_sign" and line.stop_line_id not in self.stop_sign_done:
                if front_dist < 4.0 and obs.ego.speed_mps < 0.1:
                    if self.stop_sign_wait_start is None:
                        self.stop_sign_wait_start = obs.time_s
                    if obs.time_s - self.stop_sign_wait_start >= STOP_SIGN_WAIT_S:
                        self.stop_sign_done.add(line.stop_line_id)
                        self.stop_sign_wait_start = None
                        continue
                return max(stop_at, 0.0), "stop sign"
        return None, None

    # ==================================================================
    # Checking and cost
    # ==================================================================
    def _check(self, obs, c: Candidate) -> None:
        traj, lane = c.trajectory, c.lane
        t, x, y, h = extend_trajectory(lane, traj, CHECK_HORIZON_S)
        ignore = set()
        current = obs.lanes["current"]
        for a in obs.actors:
            # vehicles BEHIND us in our own lane, driving our way, must keep their distance
            s, d = actor_position_on_lane(current, a)
            if c.lane_name == "current" and s < current.ego_s_m and abs(d) < 0.5 * current.width_m \
                    and speed_along_lane(current, a, s) > 0.5:
                ignore.add(a.actor_id)
        hit = first_collision(t, x, y, h, obs.actors, ignore, crossing_time_buffer_s=CROSSING_TIME_BUFFER_S)
        if hit is not None:
            c.valid = False
            c.reason = f"collision #{hit[1]} @{hit[0]:.1f}s"
            # lane position of our rear axle when the conflict would happen
            c.conflict_s = lane.ego_s_m + self._distance_at(t, x, y, hit[0])
            return
        if c.lane_name != "current":
            gap_problem = self._lane_change_gaps(obs, c.lane)
            if gap_problem:
                c.valid, c.reason = False, gap_problem

    @staticmethod
    def _distance_at(t, x, y, t_hit) -> float:
        seg = np.hypot(np.diff(x), np.diff(y))
        travelled = np.concatenate(([0.0], np.cumsum(seg)))
        return float(np.interp(t_hit, t, travelled))

    @staticmethod
    def _lane_change_gaps(obs, lane) -> str:
        """Enough room in front of and behind us in the target lane?"""
        v_ego = obs.ego.speed_mps
        for a in obs.actors:
            s, d = actor_position_on_lane(lane, a)
            if abs(d) > 0.5 * lane.width_m + 0.5:
                continue
            ahead = s - lane.ego_s_m
            v_a = speed_along_lane(lane, a, s)
            if ahead >= 0.0:
                gap = ahead - 0.5 * a.length_m - EGO_REAR_AXLE_TO_FRONT_M
                needed = max(6.0, 1.0 * v_ego + 2.0 * max(v_ego - v_a, 0.0))
                if gap < needed:
                    return f"front gap {gap:.0f}/{needed:.0f} m"
            else:
                gap = -ahead - 0.5 * a.length_m - (EGO_REAR_AXLE_TO_FRONT_M - 0.5 * 4.66) - 0.8
                needed = max(6.0, 1.0 * v_a + 3.0 * max(v_a - v_ego, 0.0))
                if gap < needed:
                    return f"rear gap {gap:.0f}/{needed:.0f} m"
        return ""

    def _cost(self, obs, c: Candidate) -> None:
        traj = c.trajectory
        ideal = max(obs.desired_speed_mps * traj.t_s[-1], 1.0)
        travel = float(np.sum(np.hypot(np.diff(traj.x_m), np.diff(traj.y_m))))
        cost = max(0.0, (ideal - travel) / ideal)                 # lost progress
        if c.lane_name != "current":
            cost += 0.05                                          # a lane change costs a little
        if not c.lane.is_route_lane:
            cost += 0.10                                          # prefer the route lane (keep right)
        if self.target_lane_id is not None and c.lane.lane_id != self.target_lane_id:
            cost += 0.40                                          # do not abandon a lane change lightly
        cost += 0.05 * float(np.max(np.abs(traj.acceleration_mps2))) / 3.0
        c.cost = cost

    # ==================================================================
    # Labels (display only)
    # ==================================================================
    def _behavior(self, obs, best: Candidate, emergency: bool) -> str:
        if emergency and best.name in ("full_brake", "stop", "cruise") and best.trajectory.terminal_speed_mps < 0.5:
            return "EMERGENCY_STOP"
        if best.lane_name == "left":
            return "CHANGE_LEFT"
        if best.lane_name == "right":
            return "CHANGE_RIGHT"
        if best.stop_reason in ("red light", "yellow light"):
            return "STOP_SIGNAL"
        if best.stop_reason == "stop sign":
            return "STOP_SIGN"
        if best.name == "stop":
            return "YIELD"
        if best.lead is not None and best.lead.gap_m < 40.0 and best.trajectory.terminal_speed_mps \
                < obs.desired_speed_mps - 0.5:
            return "FOLLOW_LEAD"
        turn = next((j.movement for j in obs.junctions if j.entry_distance_m < 30.0 and j.exit_distance_m > 0.0), None)
        if turn in ("LEFT", "RIGHT"):
            return f"TURN_{turn}"
        return "KEEP_LANE"

    @staticmethod
    def _notes(best: Candidate, candidates) -> List[str]:
        notes = []
        if best.stop_reason:
            notes.append(f"stop: {best.stop_reason}")
        elif best.lead is not None:
            notes.append(f"lead #{best.lead.actor_id}  {best.lead.gap_m:.0f} m  {best.lead.speed_mps:.1f} m/s")
        else:
            notes.append("")
        valid = sum(c.valid for c in candidates)
        notes.append(f"{len(candidates)} candidates, {valid} valid -> {best.lane_name} {best.name}")
        for c in candidates:
            mark = "*" if c is best else " "
            info = f"{c.cost:.2f}" if c.valid else c.reason
            notes.append(f"{mark}{c.lane_name:7s} {c.name:6s} {info}")
        return notes
