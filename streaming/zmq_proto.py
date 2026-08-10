"""ZMQ wire format shared by vio-streamer.py, pipeline-streaming.py and
joystick-server.py.

Two buses:
  :5556  Isaac  -> estimator      meta, imu, baro, frame, gt
  :5557  estimator -> web server  vio

msgpack rather than JSON because the frame topic carries raw JPEG bytes, and
because float64 survives exactly -- position is in metres and the origin is in
degrees, so a float32 round-trip would inject metre-scale georeference error.
"""
import msgpack

TOPIC_META = b"meta"
TOPIC_IMU = b"imu"
TOPIC_BARO = b"baro"
TOPIC_FRAME = b"frame"
TOPIC_GT = b"gt"
TOPIC_VIO = b"vio"

TOPICS = (TOPIC_META, TOPIC_IMU, TOPIC_BARO, TOPIC_FRAME, TOPIC_GT, TOPIC_VIO)

SENSOR_PORT = 5556      # Isaac -> estimator
VIO_PORT = 5557         # estimator -> web server


def pack(topic, payload):
    """-> [topic, msgpack] for socket.send_multipart()."""
    if topic not in TOPICS:
        raise ValueError(f"unknown topic {topic!r}; expected one of {TOPICS}")
    return [topic, msgpack.packb(payload, use_bin_type=True)]


def unpack(parts):
    """[topic, msgpack] -> (topic, payload)."""
    topic, blob = parts[0], parts[1]
    if topic not in TOPICS:
        raise ValueError(f"unknown topic {topic!r}; expected one of {TOPICS}")
    return topic, msgpack.unpackb(blob, raw=False)
