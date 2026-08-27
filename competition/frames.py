"""One MJPEG connection to the Isaac camera server, latest frame decoded.

drone_setup_px4_cesium.py serves each camera at
`http://<host>:8080/<name>` as `multipart/x-mixed-replace; boundary=frame`.
This holds one connection to the selected camera, parses the multipart
stream, and keeps only the most recent JPEG decoded to a numpy RGB array --
`on_frame` wants the freshest frame, never a backlog.

Never raises to the caller: a missing stream just means `latest()` stays
None and the flight continues.
"""
import io
import threading
import urllib.request

import numpy as np
from PIL import Image


class MjpegFrames:
    CAMERA_PATHS = {"nadir": "down", "oblique": "detect"}

    def __init__(self, host, port, *, camera="nadir"):
        self.host = host
        self.port = port
        self._camera = camera
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._generation = 0            # bumped on select() to retire the reader
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def select(self, camera):
        if camera not in self.CAMERA_PATHS:
            return
        with self._lock:
            self._camera = camera
            self._latest = None
            self._generation += 1

    def latest(self):
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    # -- internals -------------------------------------------------------

    def _current_generation(self):
        with self._lock:
            return self._generation

    def _url(self):
        with self._lock:
            path = self.CAMERA_PATHS[self._camera]
            gen = self._generation
        return f"http://{self.host}:{self.port}/{path}", gen

    def _run(self):
        while not self._stop.is_set():
            url, gen = self._url()
            try:
                self._read_stream(url, gen)
            except Exception:            # noqa: BLE001 -- retry, never propagate
                pass
            self._stop.wait(0.5)         # backoff before reconnecting

    def _read_stream(self, url, gen):
        with urllib.request.urlopen(url, timeout=5.0) as resp:
            buf = b""
            while not self._stop.is_set():
                if self._current_generation() != gen:
                    return               # select() moved us elsewhere
                chunk = resp.read(4096)
                if not chunk:
                    return
                buf = self._drain(buf + chunk, gen)

    def _drain(self, buf, gen):
        """Pull complete Content-Length-delimited JPEG parts out of buf,
        decoding the last one. Returns the unconsumed tail."""
        while True:
            header_end = buf.find(b"\r\n\r\n")
            if header_end == -1:
                return buf
            header = buf[:header_end]
            length = None
            for line in header.split(b"\r\n"):
                if line.lower().startswith(b"content-length:"):
                    try:
                        length = int(line.split(b":", 1)[1])
                    except ValueError:
                        length = None
                    break
            if length is None:
                nxt = buf.find(b"--frame", header_end + 4)
                if nxt == -1:
                    return b""
                buf = buf[nxt:]
                continue
            start = header_end + 4
            if len(buf) < start + length:
                return buf
            self._decode(buf[start:start + length], gen)
            buf = buf[start + length:]

    def _decode(self, jpeg, gen):
        try:
            img = np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB"))
        except Exception:                # noqa: BLE001
            return
        with self._lock:
            if self._generation == gen:
                self._latest = img
