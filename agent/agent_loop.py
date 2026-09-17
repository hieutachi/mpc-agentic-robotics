"""AI Agent loop — calls MCP tools to optimize MPC configuration."""

import json
from pathlib import Path
from typing import Any

from .prompts import build_prompt, build_simple_suggestion
from mcp_server.server import MCPServer
from mcp_server.safety import get_safe_config_range


class AgentLoop:
    """Agent that iteratively searches for good MPC configurations.

    Uses a rule-based strategy (no LLM required) to suggest configs,
    calls MCP tools to simulate, and tracks the best result.
    """

    def __init__(self, server: MCPServer | None = None, max_trials: int = 10):
        self.server = server or MCPServer()
        self.max_trials = max_trials
        self.history: list[dict[str, Any]] = []
        self.best_result: dict[str, Any] | None = None

    def run(
        self,
        goal: dict[str, Any],
        scenario_name: str = "basic_circle",
    ) -> dict[str, Any]:
        """Run the agent loop.

        Args:
            goal: Target objectives, e.g.:
                {
                    "rmse_target": 0.5,       # Target position RMSE
                    "collision_target": 0,     # Target collision count
                    "max_trials": 10,          # Override max trials
                }
            scenario_name: Which scenario to optimize for.

        Returns:
            dict with keys: best_config, best_metrics, trials, history.
        """
        max_trials = goal.get("max_trials", self.max_trials)
        rmse_target = goal.get("rmse_target", float("inf"))
        collision_target = goal.get("collision_target", 0)

        safe_ranges = get_safe_config_range()
        self.history = []
        self.best_result = None

        print(f"\n{'='*60}")
        print(f"  AI Agent — Optimizing MPC for: {scenario_name}")
        print(f"  Goal: RMSE ≤ {rmse_target}, collisions ≤ {collision_target}")
        print(f"  Max trials: {max_trials}")
        print(f"{'='*60}\n")

        for trial in range(max_trials):
            # 1. Suggest config
            config = build_simple_suggestion(goal, self.history, safe_ranges)

            # 2. Validate
            valid, reason = self.server.call("validate_config", config=config)
            if not valid:
                print(f"  Trial {trial+1}: Config rejected — {reason}")
                self.history.append({
                    "trial": trial + 1,
                    "config": config,
                    "status": "rejected",
                    "error": reason,
                })
                continue

            # 3. Run simulation
            result = self.server.call(
                "run_mpc_sim",
                config=config,
                scenario_name=scenario_name,
                save_log=False,
            )

            if result.get("status") != "success":
                print(f"  Trial {trial+1}: Simulation error — {result.get('error')}")
                self.history.append({
                    "trial": trial + 1,
                    "config": config,
                    "status": "error",
                    "error": result.get("error"),
                })
                continue

            metrics = result["metrics"]
            self.history.append({
                "trial": trial + 1,
                "config": config,
                "status": "success",
                "metrics": metrics,
            })

            # 4. Check if best
            is_better = False
            if self.best_result is None:
                is_better = True
            else:
                best_m = self.best_result["metrics"]
                # Prefer lower collisions, then lower RMSE
                if metrics["collision_count"] < best_m["collision_count"]:
                    is_better = True
                elif (metrics["collision_count"] == best_m["collision_count"]
                      and metrics["position_rmse"] < best_m["position_rmse"]):
                    is_better = True

            if is_better:
                self.best_result = {"config": config, "metrics": metrics}

            rmse_val = metrics.get("position_rmse", "N/A")
            coll_val = metrics.get("collision_count", "N/A")
            best_flag = " ★ NEW BEST" if is_better else ""
            print(
                f"  Trial {trial+1}/{max_trials}: "
                f"RMSE={rmse_val}, collisions={coll_val}{best_flag}"
            )

            # 5. Early stopping if goal met
            if (metrics["position_rmse"] <= rmse_target
                    and metrics["collision_count"] <= collision_target):
                print(f"\n  Goal reached at trial {trial+1}!")
                break

        # Build prompt for LLM (informational)
        prompt = build_prompt(goal, self.history, safe_ranges)

        summary = {
            "best_config": self.best_result["config"] if self.best_result else None,
            "best_metrics": self.best_result["metrics"] if self.best_result else None,
            "trials": len(self.history),
            "history": self.history,
            "prompt_generated": prompt,
        }

        print(f"\n  Best config: {summary['best_config']}")
        print(f"  Best metrics: {summary['best_metrics']}")

        return summary
