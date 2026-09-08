# Mock competitor image #1 -- proves the packaging/execution contract works
# at all, before dragging in a detection model. Wraps examples/flight.py
# unchanged (zero extra deps): fly forward 8s, then hold.
#
# Build from the repo root, after competition-base:latest exists:
#   docker build -t mock-flight-competitor:latest -f docker/mock-flight-competitor/Dockerfile .
# Run against a site that's already ARMED + OFFBOARD:
#   docker run --rm --network host mock-flight-competitor:latest
FROM competition-base:latest

COPY examples/flight.py .

ENTRYPOINT ["python", "agent_runner.py", "--file", "flight.py"]
