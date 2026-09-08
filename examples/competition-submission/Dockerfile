# Example competitor submission image.
#
# competition-base is published by the organizer and already has: the
# competition package (Agent/Command/State/flight()), agent_runner.py,
# websockets, numpy, Pillow. See ../../docker/competition-base/Dockerfile
# and ../../docs/superpowers/specs/2026-09-08-competitor-submission-docker-design.md
# Decision 1 for why a base image instead of a fully self-contained
# Dockerfile per submission.
FROM competition-base:latest

# No WORKDIR override -- stay in competition-base's /competition, where
# agent_runner.py lives. agent_runner.py resolves its own directory from
# __file__, not the cwd, but "python agent_runner.py" below needs it on
# the cwd (or PATH) to find it at all.

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# weights.pt is supplied by whoever builds this image -- not included in
# this example directory. Swap full_flight_agent.py for single_flight.py
# (and drop the COPY of weights.pt, and requirements.txt is then empty)
# to submit the no-detection example instead.
COPY detection.py full_flight_agent.py weights.pt ./

# agent_runner.py is what actually drives the Agent -- full_flight_agent.py
# only *defines* one, it doesn't run itself. Same contract as running
# agent_runner.py by hand for debugging (see its own docstring): it finds
# exactly one Agent subclass in the file it's given.
ENTRYPOINT ["python", "agent_runner.py", "--file", "full_flight_agent.py"]

# Run with:
#   docker build -t my-submission .
#   docker run --rm --network host --gpus all my-submission
# See the design doc's Decision 2 for why --network host / --gpus all.
# The aircraft must already be armed and in OFFBOARD -- this container
# attaches to /agent/control and takes over from there; it doesn't do the
# ARM -> TAKEOFF -> OFFBOARD sequencing the website's RUN SCRIPT button does.
