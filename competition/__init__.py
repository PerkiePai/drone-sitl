"""Competitor-facing drone control API (sandbox subset).

    from competition import Agent, Command, flight, Route

    class MyAgent(Agent):
        def on_start(self, arena):
            return Command(flight=flight(0, 0, 0, 1.0))

Full surface: docs/competition-api.md. What this sandbox implements and what it
leaves out: docs/superpowers/specs/2026-08-27-website-agent-upload-design.md.
`flight()` replaced Velocity/VelocityWorld/Goto/Hold -- see
docs/superpowers/specs/2026-09-07-competition-flight-primitive-design.md.
"""
from competition.agent import Agent
from competition.commands import Command, Route, flight

__all__ = ["Agent", "Command", "flight", "Route"]
