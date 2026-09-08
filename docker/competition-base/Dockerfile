# competition-base: what every competitor submission's Dockerfile builds on.
#
# Contains everything agent_runner.py needs to load and drive an uploaded
# Agent script: the competition package itself, agent_runner.py, and its
# runtime deps. Deliberately NOT pymavlink -- agent_runner.py never touches
# MAVLink (see its own docstring); all flight goes through /agent/control.
# See docs/superpowers/specs/2026-09-08-competitor-submission-docker-design.md
# Decision 1.
#
# Build from the repo root:
#   docker build -t competition-base:latest -f docker/competition-base/Dockerfile .
FROM python:3.11-slim

RUN pip install --no-cache-dir websockets numpy Pillow

WORKDIR /competition
COPY competition/ ./competition/
COPY agent_runner.py .

# A competitor's own Dockerfile does:
#   FROM competition-base:latest
#   COPY their_agent.py [detection.py weights.pt ...] .
#   ENTRYPOINT ["python", "agent_runner.py", "--file", "their_agent.py"]
#
# agent_runner.py's --host/--port default to 127.0.0.1:8090, which is
# correct here too: with `docker run --network host`, the container's
# localhost IS the organizer's box -- no flag needed, same as running it
# by hand for debugging.
