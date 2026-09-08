# Example competitor submission image.
#
# competition-base is published by the organizer and already has: the
# competition package (Agent/Command/State/flight(), once implemented),
# pymavlink, numpy, Pillow -- everything agent_runner.py's subprocess path
# assumes today. See ../../docs/superpowers/specs/
# 2026-09-08-competitor-submission-docker-design.md Decision 1 for why a
# base image instead of a fully self-contained Dockerfile per submission.
FROM competition-base:latest

WORKDIR /submission

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# weights.pt is supplied by whoever builds this image -- not included in
# this example directory. Swap full_flight_agent.py for single_flight.py
# (and drop the COPY of weights.pt) to submit the no-detection example
# instead.
COPY detection.py full_flight_agent.py weights.pt ./

# The organizer's harness expects exactly one Agent subclass in the
# entrypoint module -- same contract agent_runner.py already enforces for
# script uploads today (see load_agent() there).
ENTRYPOINT ["python", "full_flight_agent.py"]

# Run with:
#   docker build -t my-submission .
#   docker run --network host --gpus all my-submission
# See the design doc's Decision 2 for why --network host / --gpus all.
