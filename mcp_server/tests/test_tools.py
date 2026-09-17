"""Tests for MCP tools and safety validation."""

import pytest

from mcp_server.safety import validate_config, get_safe_config_range
from mcp_server.server import MCPServer


class TestSafety:
    def test_valid_config(self):
        config = {
            "dt": 0.1, "horizon": 10,
            "Q": [1, 1, 0.5, 0.1], "R": [0.1, 0.1],
            "max_steer": 0.5, "max_accel": 2.0,
            "max_speed": 3.0, "vehicle_length": 0.3,
        }
        ok, reason = validate_config(config)
        assert ok is True
        assert reason == ""

    def test_negative_horizon(self):
        config = {"horizon": -5}
        ok, reason = validate_config(config)
        assert ok is False
        assert "horizon" in reason

    def test_negative_Q(self):
        config = {"horizon": 10, "Q": [1, -1, 0.5, 0.1]}
        ok, reason = validate_config(config)
        assert ok is False
        assert "Q" in reason

    def test_excessive_steer(self):
        config = {"horizon": 10, "max_steer": 3.0}
        ok, reason = validate_config(config)
        assert ok is False
        assert "max_steer" in reason

    def test_excessive_horizon(self):
        config = {"horizon": 100}
        ok, reason = validate_config(config)
        assert ok is False

    def test_safe_range(self):
        ranges = get_safe_config_range()
        assert "horizon" in ranges
        assert ranges["horizon"] == (1, 30)


class TestMCPServer:
    def test_list_tools(self):
        server = MCPServer()
        tools = server.list_tools()
        names = [t["name"] for t in tools]
        assert "list_scenarios" in names
        assert "run_mpc_sim" in names
        assert "get_safe_config_range" in names

    def test_list_scenarios(self):
        server = MCPServer()
        result = server.call("list_scenarios")
        assert isinstance(result, list)
        assert len(result) > 0

    def test_call_unknown_tool(self):
        server = MCPServer()
        result = server.call("nonexistent_tool")
        assert result["status"] == "error"

    def test_validate_config_via_server(self):
        server = MCPServer()
        result = server.call("validate_config", config={"horizon": 10})
        assert isinstance(result, tuple)
        assert result[0] is True

    def test_safe_config_range(self):
        server = MCPServer()
        result = server.call("get_safe_config_range")
        assert "horizon" in result
