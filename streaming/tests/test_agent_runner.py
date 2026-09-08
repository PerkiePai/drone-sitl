"""agent_runner.load_agent -- find exactly one Agent subclass in a file."""
import os
import sys
import textwrap

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

import agent_runner  # noqa: E402


def _write(tmp_path, body):
    p = tmp_path / "script.py"
    p.write_text(textwrap.dedent(body))
    return str(p)


def test_loads_the_single_agent_subclass(tmp_path):
    path = _write(tmp_path, """
        from competition import Agent, Command, flight
        class Mine(Agent):
            def on_tick(self, state):
                return Command(flight=flight(0, 0, 0, 1.0))
    """)
    cls = agent_runner.load_agent(path)
    assert cls.__name__ == "Mine"


def test_rejects_a_file_with_no_agent(tmp_path):
    path = _write(tmp_path, "x = 1\n")
    with pytest.raises(RuntimeError, match="no Agent subclass"):
        agent_runner.load_agent(path)


def test_rejects_a_file_with_two_agents(tmp_path):
    path = _write(tmp_path, """
        from competition import Agent
        class A(Agent): pass
        class B(Agent): pass
    """)
    with pytest.raises(RuntimeError, match="more than one"):
        agent_runner.load_agent(path)


def test_a_syntax_error_in_the_script_is_reported(tmp_path):
    path = _write(tmp_path, "def broken(\n")
    with pytest.raises(RuntimeError):
        agent_runner.load_agent(path)
