"""Scenario registry: example scenarios + the scenario team's scenarios."""

from __future__ import annotations

from scenarios.example_scenarios import EXAMPLE_SCENARIOS
from scenarios.student_scenario import STUDENT_SCENARIOS


def all_scenarios() -> dict:
    """name -> function that builds the Scenario."""
    clash = set(EXAMPLE_SCENARIOS) & set(STUDENT_SCENARIOS)
    if clash:
        raise ValueError(f"student scenario names must differ from the examples: {sorted(clash)}")
    return {**EXAMPLE_SCENARIOS, **STUDENT_SCENARIOS}


def get_scenario(name: str):
    scenarios = all_scenarios()
    if name not in scenarios:
        raise KeyError(f"unknown scenario '{name}'. Available: {', '.join(scenarios)}")
    scenario = scenarios[name]()
    if scenario.name != name:
        raise ValueError(f"scenario function '{name}' returned a Scenario named '{scenario.name}'")
    return scenario
