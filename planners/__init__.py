"""Planner registry.  To add another planner, import its class and add it to PLANNERS."""

from __future__ import annotations

import importlib.util

from planners.baseline_planner import BaselinePlanner
from planners.student_planner import StudentPlanner

PLANNERS = {
    "baseline": BaselinePlanner,
    "student": StudentPlanner,
}


def _learned_example():
    """Example learned planner (planners/learned_example).  Imported only when
    used, because it needs PyTorch, which the rest of the project does not."""
    from planners.learned_example.planner import LearnedExamplePlanner
    return LearnedExamplePlanner()


if importlib.util.find_spec("torch") is not None:
    PLANNERS["learned_example"] = _learned_example


def make_planner(name: str):
    if name not in PLANNERS:
        raise KeyError(f"unknown planner '{name}'. Available: {', '.join(PLANNERS)}")
    return PLANNERS[name]()
