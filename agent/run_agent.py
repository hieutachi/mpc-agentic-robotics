#!/usr/bin/env python3
"""
run_agent.py — Run the AI Agent to optimize MPC configuration.

Usage:
    python run_agent.py --scenario basic_circle --max-trials 10
    python run_agent.py --scenario obstacles --rmse-target 0.3
"""

import argparse
import json
import sys
from pathlib import Path

# Allow running this script directly: python agent/run_agent.py
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from agent.agent_loop import AgentLoop
from mcp_server.server import MCPServer


def main():
    parser = argparse.ArgumentParser(description="AI Agent for MPC Config Optimization")
    parser.add_argument("--scenario", type=str, default="basic_circle",
                        help="Scenario to optimize for")
    parser.add_argument("--max-trials", type=int, default=10,
                        help="Maximum number of trials")
    parser.add_argument("--rmse-target", type=float, default=0.5,
                        help="Target position RMSE")
    parser.add_argument("--collision-target", type=int, default=0,
                        help="Target collision count")
    parser.add_argument("--save-results", type=str, default=None,
                        help="Save results to JSON file")
    args = parser.parse_args()

    server = MCPServer()
    agent = AgentLoop(server=server, max_trials=args.max_trials)

    goal = {
        "rmse_target": args.rmse_target,
        "collision_target": args.collision_target,
        "max_trials": args.max_trials,
    }

    results = agent.run(goal=goal, scenario_name=args.scenario)

    if args.save_results:
        Path(args.save_results).parent.mkdir(parents=True, exist_ok=True)
        with open(args.save_results, "w") as f:
            json.dump(results, f, indent=2, default=str)
        print(f"\nResults saved to: {args.save_results}")


if __name__ == "__main__":
    main()
