"""Competitor-facing drone control API (sandbox subset).

    from competition import Agent, Command, Velocity, Route, Goto, Hold

    class MyAgent(Agent):
        def on_start(self, arena):
            return Command(flight=Velocity(forward=3.0))

Full surface: docs/competition-api.md. What this sandbox implements and what it
leaves out: docs/superpowers/specs/2026-08-27-website-agent-upload-design.md.
"""
from competition.agent import Agent
from competition.commands import (
    Command, Goto, Hold, Route, Velocity, VelocityWorld,
)

__all__ = [
    "Agent", "Command", "Velocity", "VelocityWorld", "Goto", "Route", "Hold",
]
