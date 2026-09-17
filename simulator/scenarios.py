"""Scenario loading and management."""

from pathlib import Path

import yaml


def load_scenario(path: str | Path, scenario_name: str | None = None) -> dict:
    """Load a scenario from scenarios YAML file.

    Args:
        path: Path to scenarios.yaml.
        scenario_name: Name of scenario to load. If None, returns first scenario.

    Returns:
        dict with keys: name, map, start, goal, mpc_config, description.
    """
    with open(path, "r", encoding="utf-8") as f:
        scenarios = yaml.safe_load(f)

    if scenario_name:
        if scenario_name not in scenarios:
            raise KeyError(
                f"Scenario '{scenario_name}' not found. "
                f"Available: {list(scenarios.keys())}"
            )
        s = scenarios[scenario_name]
    else:
        scenario_name = next(iter(scenarios))
        s = scenarios[scenario_name]

    return {
        "name": scenario_name,
        "map": s["map"],
        "start": tuple(s["start"]),
        "goal": tuple(s["goal"]),
        "mpc_config": s.get("mpc_config", "configs/mpc_baseline.yaml"),
        "description": s.get("description", ""),
    }


def list_scenarios(path: str | Path) -> list[str]:
    """List available scenario names."""
    with open(path, "r", encoding="utf-8") as f:
        scenarios = yaml.safe_load(f)
    return list(scenarios.keys())
