# Mock competitor image #2 -- proves the agent.py/detection.py split and
# the camera-switch-on-detection path work end to end in a real container,
# using a mock Detector (docker/mock-detection-competitor/detection.py) so
# no torch/ultralytics/GPU/real weights are needed for this test. The
# flight logic is examples/competition-submission/full_flight_agent.py,
# unchanged.
#
# Build from the repo root, after competition-base:latest exists:
#   docker build -t mock-detection-competitor:latest -f docker/mock-detection-competitor/Dockerfile .
# Run against a site that's already ARMED + OFFBOARD:
#   docker run --rm --network host mock-detection-competitor:latest
FROM competition-base:latest

COPY docker/mock-detection-competitor/detection.py .
COPY docker/mock-detection-competitor/weights.pt .
COPY examples/competition-submission/full_flight_agent.py .

ENTRYPOINT ["python", "agent_runner.py", "--file", "full_flight_agent.py"]
