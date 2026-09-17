"""MCP Tool Server — registers and dispatches tool calls."""

from typing import Any, Callable

from .tools import list_scenarios, run_mpc_sim, evaluate_experiment
from .safety import validate_config, get_safe_config_range


class MCPServer:
    """Simple MCP-compatible tool server.

    Registers a set of tools and dispatches calls by name.
    This is a local/Python implementation; in production it would
    use the MCP protocol over stdio or HTTP.
    """

    def __init__(self):
        self._tools: dict[str, dict[str, Any]] = {}
        self._register_defaults()

    def _register_defaults(self):
        """Register the built-in MPC tools."""
        self.register_tool(
            name="list_scenarios",
            func=list_scenarios,
            description="List available simulation scenarios.",
            parameters={},
        )
        self.register_tool(
            name="run_mpc_sim",
            func=run_mpc_sim,
            description="Run MPC simulation with given config. Returns metrics.",
            parameters={
                "config": "dict — MPC configuration (horizon, Q, R, etc.)",
                "scenario_name": "str — scenario to run (default: basic_circle)",
            },
        )
        self.register_tool(
            name="evaluate_experiment",
            func=evaluate_experiment,
            description="Evaluate metrics from an existing simulation log.",
            parameters={"log_path": "str — path to simulation log CSV"},
        )
        self.register_tool(
            name="get_safe_config_range",
            func=get_safe_config_range,
            description="Return safe parameter ranges for MPC config.",
            parameters={},
        )
        self.register_tool(
            name="validate_config",
            func=validate_config,
            description="Validate an MPC config for safety.",
            parameters={"config": "dict — MPC configuration"},
        )

    def register_tool(
        self,
        name: str,
        func: Callable,
        description: str,
        parameters: dict,
    ):
        """Register a new tool."""
        self._tools[name] = {
            "func": func,
            "description": description,
            "parameters": parameters,
        }

    def call(self, tool_name: str, **kwargs) -> dict[str, Any]:
        """Call a registered tool by name.

        Args:
            tool_name: Name of the tool.
            **kwargs: Arguments to pass to the tool function.

        Returns:
            Tool result as dict.
        """
        if tool_name not in self._tools:
            return {
                "status": "error",
                "error": f"Unknown tool: '{tool_name}'. "
                         f"Available: {list(self._tools.keys())}",
            }

        try:
            result = self._tools[tool_name]["func"](**kwargs)
            return result
        except Exception as e:
            return {"status": "error", "error": str(e)}

    def list_tools(self) -> list[dict[str, str]]:
        """List all registered tools with descriptions."""
        return [
            {
                "name": name,
                "description": info["description"],
                "parameters": info["parameters"],
            }
            for name, info in self._tools.items()
        ]
