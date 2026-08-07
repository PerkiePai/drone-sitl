"""Start and stop vio-recorder-pai.py from outside Isaac Sim.

Two layers, split so the interesting one is testable:

  RecorderSession   pure state machine; every Kit coupling is injected
  RecorderControl   the Kit bindings -- HTTP server, command queue, update
                    callback

The recorder itself is NEVER modified. It is exec'd into a namespace we keep,
exactly as sim/bootstrap.py:90-98 runs the setup script, and driven through that
namespace afterwards -- its physics callback resolves module globals
dynamically, so post-exec mutation works.

Design: docs/superpowers/specs/2026-08-07-web-recorder-design.md
"""
import os
import shutil
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RECORDER_PATH = os.path.join(REPO_ROOT, "vio-recorder-pai.py")
DATASET_ROOT = os.path.expanduser("~/vio_dataset")

STATE_OFFLINE = "offline"       # only ever reported by the web server
STATE_IDLE = "idle"
STATE_RECORDING = "recording"
STATE_ERROR = "error"

# Runs measure 22-30 GB and the disk sat at 90% full when this was designed, so
# free space is a gate rather than a warning: a run that fills the disk halfway
# through corrupts its own dataset and destabilises the sim and PX4 with it.
MIN_FREE_BYTES = 50 * 1024 ** 3     # refuse to start below this
WARN_FREE_BYTES = 100 * 1024 ** 3   # amber on the page below this

# What TAKEOFF_ALT_M becomes once the button is the trigger (design R5). NOT
# 0.0: the recorder waits while `climbed < TAKEOFF_ALT_M` where climbed is
# measured against the drone's FIRST sampled altitude
# (vio-recorder-pai.py:361-372), and a drone settling into the terrain makes
# that negative. At 0.0 the gate would re-latch and write nothing, which is the
# silent failure this whole feature exists to kill.
GATE_DISABLED_ALT_M = -1e9


def _disk_free(path):
    probe = path if os.path.isdir(path) else os.path.dirname(path) or "/"
    return shutil.disk_usage(probe).free


def _dir_size(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass                # a file the writer threads are mid-rename on
    return total


class RecorderSession:
    """The recorder's lifecycle. No Kit, no HTTP, no threads.

    Callables are injected so this runs under pytest:
      exec_recorder()      -> the recorder's exec namespace
      stop_recorder(ns)    -> run the documented stop sequence
      free_bytes()         -> bytes free on the dataset volume
      dir_size(path)       -> bytes currently written
      now()                -> monotonic seconds
      is_playing()         -> whether the timeline is running
    """

    def __init__(self, dataset_root=DATASET_ROOT, exec_recorder=None,
                 stop_recorder=None, free_bytes=None, dir_size=None,
                 now=None, is_playing=None):
        self.dataset_root = dataset_root
        self._exec = exec_recorder
        self._stop = stop_recorder
        self._free = free_bytes or (lambda: _disk_free(dataset_root))
        self._size = dir_size or _dir_size
        self._now = now or time.monotonic
        self._playing = is_playing or (lambda: True)

        self.state = STATE_IDLE
        self.error = None
        self._ns = None
        self._started_at = None

    # --- gating ------------------------------------------------------------

    def _why_not_start(self):
        """(code, reason) if start must be refused, else None."""
        if self.state == STATE_RECORDING:
            return 409, "already recording"
        if not self._playing():
            return 503, ("the sim timeline is stopped -- press Play. Pegasus "
                         "streams sensor data only from the play event, so "
                         "nothing would be recorded")
        free = self._free()
        if free < MIN_FREE_BYTES:
            return 507, (f"only {free / 1024 ** 3:.0f} GB free; "
                         f"{MIN_FREE_BYTES / 1024 ** 3:.0f} GB required")
        return None

    # --- transitions -------------------------------------------------------

    def start(self):
        """-> (ok, http_code, reason). Main thread only."""
        refusal = self._why_not_start()
        if refusal is not None:
            return (False,) + refusal

        ns = self._exec()

        # The recorder returns WITHOUT installing its physics callback when
        # there is no drone, IMU or down_cam (vio-recorder-pai.py:147-152). It
        # prints its own diagnostic; no _VIO_REC means nothing is recording, and
        # calling that a success is exactly how an empty dataset gets made.
        if not ns.get("_VIO_REC"):
            self.state = STATE_ERROR
            self.error = ("the recorder did not install -- no drone, IMU or "
                          "down_cam on the stage. See the Isaac console.")
            self._ns = None
            return False, 500, self.error

        # Design R5: the button IS the trigger. _on_phys reads TAKEOFF_ALT_M as
        # a dynamic global (vio-recorder-pai.py:366), so clearing it here starts
        # data flowing immediately without touching the recorder's source.
        ns["TAKEOFF_ALT_M"] = GATE_DISABLED_ALT_M

        self._ns = ns
        self.state = STATE_RECORDING
        self.error = None
        self._started_at = self._now()
        return True, 202, "recording"

    def stop(self):
        """-> (ok, http_code, reason). Idempotent. Main thread only."""
        if self.state != STATE_RECORDING:
            return True, 200, "not recording"
        self._stop(self._ns)
        self._ns = None
        self.state = STATE_IDLE
        self._started_at = None
        return True, 200, "stopped"

    # --- status ------------------------------------------------------------

    def status(self):
        st = (self._ns or {}).get("st") or {}
        rec = (self._ns or {}).get("_VIO_REC") or {}
        imgq = (self._ns or {}).get("imgq")
        run_dir = rec.get("dir")
        free = self._free()
        return {
            "state": self.state,
            "run_dir": run_dir,
            "elapsed_s": (self._now() - self._started_at
                          if self._started_at is not None else 0.0),
            "frames": st.get("frame", 0),
            "images": st.get("n_frame", 0),
            "dropped": st.get("dropped", 0),
            "queue": imgq.qsize() if imgq is not None else 0,
            "bytes": self._size(run_dir) if run_dir else 0,
            "free_bytes": free,
            "min_free_bytes": MIN_FREE_BYTES,
            "warn_free_bytes": WARN_FREE_BYTES,
            "can_start": self._why_not_start() is None,
            "error": self.error,
        }
