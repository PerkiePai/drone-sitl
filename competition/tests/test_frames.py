"""competition.frames -- decode the latest JPEG from a multipart MJPEG stream.

Runs against a local fake MJPEG server, no Isaac. The fake serves the exact
`multipart/x-mixed-replace; boundary=frame` shape
drone_setup_px4_cesium.py's camera server produces.
"""
import io
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from competition.frames import MjpegFrames  # noqa: E402


def _jpeg(color):
    buf = io.BytesIO()
    Image.new("RGB", (16, 12), color).save(buf, format="JPEG")
    return buf.getvalue()


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        name = self.path.strip("/")
        color = {"down": (255, 0, 0), "detect": (0, 255, 0)}.get(name)
        if color is None:
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            for _ in range(400):
                jpeg = _jpeg(color)
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError, ValueError, OSError):
            return


def _server():
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd, httpd.server_address[1]


def _wait(fn, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = fn()
        if v is not None:
            return v
        time.sleep(0.02)
    raise AssertionError("condition never held")


def test_latest_is_none_before_the_first_frame_then_an_rgb_array():
    httpd, port = _server()
    try:
        f = MjpegFrames("127.0.0.1", port, camera="nadir")
        assert f.latest() is None
        f.start()
        img = _wait(f.latest)
        assert isinstance(img, np.ndarray)
        assert img.shape == (12, 16, 3) and img.dtype == np.uint8
        # nadir -> /down -> red
        assert img[0, 0, 0] > 200 and img[0, 0, 1] < 60
        f.stop()
    finally:
        httpd.shutdown()


def test_select_switches_the_endpoint():
    httpd, port = _server()
    try:
        f = MjpegFrames("127.0.0.1", port, camera="nadir")
        f.start()
        _wait(f.latest)
        f.select("oblique")            # -> /detect -> green
        _wait(lambda: ((im := f.latest()) is not None
                       and im[0, 0, 1] > 200 and im[0, 0, 0] < 60) or None)
        f.stop()
    finally:
        httpd.shutdown()


def test_stop_ends_the_reader_thread():
    httpd, port = _server()
    try:
        f = MjpegFrames("127.0.0.1", port)
        f.start()
        _wait(f.latest)
        f.stop()
        time.sleep(0.3)
        assert not f._thread.is_alive()
    finally:
        httpd.shutdown()


def test_a_dead_endpoint_leaves_latest_none_without_raising():
    f = MjpegFrames("127.0.0.1", 1, camera="nadir")   # nothing listening
    f.start()
    time.sleep(0.5)
    assert f.latest() is None
    f.stop()
