# Implementation plan: record VIO datasets from the web UI

Design: `docs/superpowers/specs/2026-08-07-web-recorder-design.md`.
Runs **before** `2026-08-07-vio-gps-denied.md`, which develops its estimator
against datasets this feature captures.

## Goal

A RECORD button on the flight page that starts and stops `vio-recorder-pai.py`
inside Isaac Sim, writes `~/vio_dataset/<timestamp>/`, and reports live whether
data is actually landing on disk.

## Architecture summary

```
browser ──WS /ws──▶ joystick-server.py ──HTTP :8091──▶ sim/recorder_control.py (Kit)
 RECORD              :8090, 1 Hz poll                   HTTP thread: ENQUEUE only
 + status                                               main thread: EXEC recorder
                                                          → ~/vio_dataset/<ts>/
```

## Tech stack

Nothing new. Kit side is stdlib only (`http.server`, `queue`, `threading`,
`shutil.disk_usage`) — deliberately, since whether Kit's interpreter has `pyzmq`
is still unverified and this feature must not depend on finding out. The `drone`
env has no HTTP client (`httpx`/`aiohttp`/`requests` all absent), so the proxy
uses `urllib.request` in `asyncio.to_thread()`.

## Global constraints

- **`vio-recorder-pai.py` is not modified.** It stays byte-identical to
  `~/pai/drone-vio/vio-recorder-pai.py`. Everything is driven through its exec
  namespace (design R4).
- **The HTTP thread never execs anything** (design R1). It validates and
  enqueues; a Kit update callback does the work on the main thread.
- **Record commands never enter `SetpointLoop.submit()`** (design R2). Recording
  touches no MAVLink, and a blocking HTTP call on the 20 Hz setpoint thread would
  gap the setpoint stream and drop PX4 out of OFFBOARD.
- The pure state machine takes its Kit couplings as injected callables, so the
  bulk of this feature is testable without Isaac.

## Task list

| # | Task | Files | Needs Isaac |
|---|---|---|---|
| 0 | Branch | — | no |
| 1 | `RecorderSession` state machine | `sim/recorder_control.py` | no |
| 2 | HTTP layer | `sim/recorder_control.py` | no |
| 3 | Kit bindings + bootstrap wiring | `sim/recorder_control.py`, `sim/bootstrap.py` | **yes** |
| 4 | Web-server proxy | `joystick-server.py` | no |
| 5 | RECORD button and status | `web/` | no |
| 6 | Live bring-up | — | **yes** |

---

## Task 0 — Branch

- [ ] **0.1** The tree still carries the uncommitted web refactor
      (`joystick-server.py`, `web/index.html`, `web/css/`, `web/js/`) and a `+9`
      change to `vio-recorder-pai.py`. Commit or stash before branching.
- [ ] **0.2** Deal with `NvStreamer-20260807-111234.etli` (64 MB untracked) —
      delete or gitignore. It must not land in a commit.

```bash
cd ~/pai/drone-sitl
git status --short
git checkout -b feat/web-recorder
```

---

## Task 1 — `RecorderSession`, the state machine

**Consumes:** injected callables — `exec_recorder()`, `stop_recorder(ns)`,
`free_bytes()`, `dir_size(path)`, `now()`, `is_playing()`.
**Produces:** state transitions and the status dict from design R7.

All Kit coupling is injected, so this task runs entirely under pytest.

### Step 1.1 — Failing test

`sim/tests/test_recorder_control.py`:

```python
"""The recorder control state machine. No Isaac Sim: every Kit coupling is an
injected callable, and the recorder's exec namespace is a plain dict."""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import recorder_control as rc  # noqa: E402

GIB = 1024 ** 3


def make_ns(run_dir="/tmp/run", frame=0, n_frame=0, dropped=0, with_rec=True):
    """A stand-in for the recorder's exec namespace.

    Mirrors the real thing: `st` is the counters dict (vio-recorder-pai.py:340),
    `_VIO_REC` is the handle the stop sequence needs (:442), and TAKEOFF_ALT_M
    is the module global the physics callback reads dynamically (:366).
    """
    ns = {"TAKEOFF_ALT_M": 0.5,
          "st": {"frame": frame, "n_frame": n_frame, "dropped": dropped,
                 "img_every": 20, "armed": True},
          "imgq": type("Q", (), {"qsize": lambda self: 0})()}
    if with_rec:
        ns["_VIO_REC"] = {"cb": "vio_rec", "q": None, "files": [], "dir": run_dir}
    return ns


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make_session(free=200 * GIB, playing=True, ns=None, sizes=None):
    clock = FakeClock()
    state = {"stopped": 0, "ns": ns if ns is not None else make_ns()}

    def exec_recorder():
        return state["ns"]

    def stop_recorder(namespace):
        state["stopped"] += 1

    s = rc.RecorderSession(
        dataset_root="/tmp/vio_dataset",
        exec_recorder=exec_recorder,
        stop_recorder=stop_recorder,
        free_bytes=lambda: free,
        dir_size=lambda p: (sizes or {}).get(p, 0),
        now=clock,
        is_playing=lambda: playing)
    return s, state, clock


# --- start gating ----------------------------------------------------------

def test_starts_from_idle():
    s, _, _ = make_session()
    assert s.state == rc.STATE_IDLE
    ok, code, reason = s.start()
    assert ok and code == 202, reason
    assert s.state == rc.STATE_RECORDING


def test_second_start_is_rejected_and_does_not_make_a_second_run():
    """409, and critically the recorder is not exec'd again -- a second exec
    would create a second dataset directory and orphan the first."""
    s, state, _ = make_session()
    s.start()
    first_dir = s.status()["run_dir"]
    ok, code, _ = s.start()
    assert not ok and code == 409
    assert s.status()["run_dir"] == first_dir
    assert state["stopped"] == 0


def test_start_refused_below_the_free_space_floor():
    s, _, _ = make_session(free=49 * GIB)
    ok, code, reason = s.start()
    assert not ok and code == 507
    assert "free" in reason.lower()
    assert s.state == rc.STATE_IDLE


def test_start_allowed_just_above_the_floor():
    s, _, _ = make_session(free=51 * GIB)
    ok, code, _ = s.start()
    assert ok, code


def test_start_refused_when_the_timeline_is_stopped():
    """Pegasus streams sensor data only from the play event
    (sim/bootstrap.py:192-195), so physics callbacks never fire when stopped --
    recording would produce another zero-byte dataset."""
    s, _, _ = make_session(playing=False)
    ok, code, reason = s.start()
    assert not ok and code == 503
    assert "play" in reason.lower()


def test_recorder_that_failed_to_install_becomes_error_not_recording():
    """The recorder prints its own diagnostic and returns without installing a
    physics callback when there is no drone, IMU or down_cam
    (vio-recorder-pai.py:147-152). No _VIO_REC means nothing is recording, and
    reporting success there is how you get another empty dataset."""
    s, _, _ = make_session(ns=make_ns(with_rec=False))
    ok, code, _ = s.start()
    assert not ok and code == 500
    assert s.state == rc.STATE_ERROR
    assert s.status()["error"]


def test_error_state_can_be_cleared_by_starting_again():
    s, state, _ = make_session(ns=make_ns(with_rec=False))
    s.start()
    assert s.state == rc.STATE_ERROR
    state["ns"] = make_ns()
    ok, _, _ = s.start()
    assert ok and s.state == rc.STATE_RECORDING


# --- the takeoff gate ------------------------------------------------------

def test_start_disables_the_takeoff_gate():
    """Design R5: the button IS the trigger. The recorder reads TAKEOFF_ALT_M as
    a dynamic global inside its physics callback, so setting it after exec is
    what makes data flow immediately without editing the recorder."""
    ns = make_ns()
    s, _, _ = make_session(ns=ns)
    s.start()
    assert ns["TAKEOFF_ALT_M"] == 0.0


# --- stop ------------------------------------------------------------------

def test_stop_runs_the_documented_sequence_once():
    s, state, _ = make_session()
    s.start()
    ok, code, _ = s.stop()
    assert ok and code == 200
    assert state["stopped"] == 1
    assert s.state == rc.STATE_IDLE


def test_stop_while_idle_is_a_no_op_not_an_error():
    s, state, _ = make_session()
    ok, code, _ = s.stop()
    assert ok and code == 200
    assert state["stopped"] == 0


def test_stop_is_idempotent():
    s, state, _ = make_session()
    s.start()
    s.stop()
    s.stop()
    assert state["stopped"] == 1


# --- status ----------------------------------------------------------------

def test_idle_status_reports_no_run():
    s, _, _ = make_session()
    st = s.status()
    assert st["state"] == rc.STATE_IDLE
    assert st["run_dir"] is None
    assert st["images"] == 0


def test_status_reads_live_counters_out_of_the_namespace():
    ns = make_ns(frame=2480, n_frame=186, dropped=3)
    s, _, _ = make_session(ns=ns)
    s.start()
    st = s.status()
    assert (st["frames"], st["images"], st["dropped"]) == (2480, 186, 3)


def test_status_elapsed_tracks_the_clock():
    s, _, clock = make_session()
    s.start()
    clock.t += 12.5
    assert s.status()["elapsed_s"] == pytest.approx(12.5)


def test_status_reports_run_size_from_disk_not_an_accumulator():
    """Sampled rather than accumulated, so it stays honest if a writer thread
    dies mid-run."""
    s, _, _ = make_session(ns=make_ns(run_dir="/tmp/run"),
                           sizes={"/tmp/run": 18234567})
    s.start()
    assert s.status()["bytes"] == 18234567


def test_status_always_carries_free_space_for_the_button_gate():
    s, _, _ = make_session(free=193 * GIB)
    assert s.status()["free_bytes"] == 193 * GIB


def test_low_disk_is_visible_before_the_button_is_pressed():
    s, _, _ = make_session(free=10 * GIB)
    st = s.status()
    assert st["free_bytes"] < rc.MIN_FREE_BYTES
    assert not st["can_start"]


def test_can_start_is_false_while_recording():
    s, _, _ = make_session()
    s.start()
    assert not s.status()["can_start"]
```

```bash
conda run -n drone pytest sim/tests/test_recorder_control.py -q
```
Expect: `ModuleNotFoundError: No module named 'recorder_control'`.

### Step 1.2 — Implement the state machine

Create `sim/recorder_control.py`:

```python
"""Start and stop vio-recorder-pai.py from outside Isaac Sim.

Two layers, split so the interesting one is testable:

  RecorderSession   pure state machine; every Kit coupling is injected
  RecorderControl   the Kit bindings -- HTTP server, command queue, update
                    callback (Task 3)

The recorder itself is NEVER modified. It is exec'd into a namespace we keep,
exactly as sim/bootstrap.py:90-98 runs the setup script, and driven through that
namespace afterwards -- its physics callback resolves module globals
dynamically, so post-exec mutation works.
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
        ns["TAKEOFF_ALT_M"] = 0.0

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
```

```bash
conda run -n drone pytest sim/tests/test_recorder_control.py -q
```
Expect: `18 passed`.

**Commit:** `feat(recorder): add the recorder session state machine`

---

## Task 2 — HTTP layer

**Consumes:** HTTP requests on 8091. **Produces:** queued commands and status
JSON. Critically, the handler **enqueues and returns** — it never execs
(design R1).

### Step 2.1 — Failing test

Append to `sim/tests/test_recorder_control.py`:

```python
# --- HTTP layer ------------------------------------------------------------

class FakeQueue:
    def __init__(self):
        self.items = []

    def put_nowait(self, item):
        self.items.append(item)


def test_start_request_only_enqueues_and_never_execs():
    """Design R1, the load-bearing one. The HTTP thread must not exec the
    recorder: that installs a physics callback and touches USD and replicator,
    which Kit does not support off the main thread. The handler's whole job is
    to validate and enqueue."""
    executed = []
    s, _, _ = make_session()
    s._exec = lambda: executed.append(1) or make_ns()
    q = FakeQueue()
    code, body = rc.handle_request("POST", "/record/start", s, q)
    assert code == 202
    assert q.items == ["start"]
    assert executed == []               # nothing ran on this thread
    assert s.state == rc.STATE_IDLE     # still idle until the main thread acts


def test_start_request_is_refused_without_enqueueing_when_gated():
    s, _, _ = make_session(free=10 * GIB)
    q = FakeQueue()
    code, body = rc.handle_request("POST", "/record/start", s, q)
    assert code == 507
    assert q.items == []


def test_stop_request_enqueues():
    s, _, _ = make_session()
    q = FakeQueue()
    code, _ = rc.handle_request("POST", "/record/stop", s, q)
    assert code == 200
    assert q.items == ["stop"]


def test_status_is_always_200_so_offline_is_distinguishable_from_broken():
    s, _, _ = make_session()
    code, body = rc.handle_request("GET", "/record/status", s, FakeQueue())
    assert code == 200
    assert body["state"] == rc.STATE_IDLE


def test_status_never_enqueues():
    q = FakeQueue()
    s, _, _ = make_session()
    rc.handle_request("GET", "/record/status", s, q)
    assert q.items == []


def test_unknown_path_is_404():
    s, _, _ = make_session()
    code, _ = rc.handle_request("GET", "/nope", s, FakeQueue())
    assert code == 404


def test_wrong_method_is_405():
    s, _, _ = make_session()
    code, _ = rc.handle_request("GET", "/record/start", s, FakeQueue())
    assert code == 405
```

### Step 2.2 — Implement

Add to `sim/recorder_control.py`:

```python
# --- HTTP ------------------------------------------------------------------
#
# Kept as a pure function of (method, path, session, queue) so it is testable
# without binding a socket. The BaseHTTPRequestHandler below is a thin shell.

def handle_request(method, path, session, commands):
    """-> (http_code, body_dict). Runs on the HTTP thread: ENQUEUE ONLY.

    Never touches the recorder. Execing it here would install a physics callback
    from a worker thread, which Kit does not support (design R1).
    """
    if path == "/record/status":
        if method != "GET":
            return 405, {"error": "GET only"}
        return 200, session.status()

    if path in ("/record/start", "/record/stop"):
        if method != "POST":
            return 405, {"error": "POST only"}
        if path == "/record/stop":
            commands.put_nowait("stop")
            return 200, {"queued": "stop"}
        # Gate on the HTTP thread so a refusal is immediate and specific,
        # rather than queued and silently dropped a frame later.
        refusal = session._why_not_start()
        if refusal is not None:
            code, reason = refusal
            return code, {"error": reason}
        commands.put_nowait("start")
        return 202, {"queued": "start"}

    return 404, {"error": f"unknown path {path}"}
```

Plus the socket shell:

```python
def make_handler(session, commands):
    """BaseHTTPRequestHandler bound to one session/queue pair."""
    import json
    from http.server import BaseHTTPRequestHandler

    class Handler(BaseHTTPRequestHandler):
        def _respond(self, method):
            code, body = handle_request(method, self.path.split("?")[0],
                                        session, commands)
            blob = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(blob)))
            # The page is served from :8090 and this is :8091, so a browser
            # calling it directly would be a cross-origin request. Nothing does
            # today -- the web server proxies -- but allowing it keeps curl and
            # a future direct call working.
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(blob)

        def do_GET(self):
            self._respond("GET")

        def do_POST(self):
            self._respond("POST")

        def log_message(self, fmt, *args):
            pass                    # a 1 Hz status poll would flood the console

    return Handler
```

```bash
conda run -n drone pytest sim/tests/ -q
```
Expect: `25 passed`.

**Commit:** `feat(recorder): add the HTTP control layer`

---

## Task 3 — Kit bindings and bootstrap wiring

**Needs Isaac.** This is the only part that cannot be tested offline.

### Step 3.1 — `RecorderControl`

Add to `sim/recorder_control.py`. Responsibilities:

1. Build a `RecorderSession` with the **real** couplings:
   - `exec_recorder` — read `RECORDER_PATH`, `exec(compile(src, path, "exec"), ns)`
     with `ns = {"__name__": "__main__", "__file__": RECORDER_PATH}`, return `ns`.
   - `stop_recorder(ns)` — the sequence the recorder documents at
     `vio-recorder-pai.py:460-461`: `rec["q"].put(None)`, then
     `ns["world"].remove_physics_callback(rec["cb"])`, then close `rec["files"]`.
     Each step in its own `try` — a half-stopped recorder must still release the
     physics callback, or the next start collides with it.
   - `is_playing` — `omni.timeline.get_timeline_interface().is_playing()`.
2. `ThreadingHTTPServer(("0.0.0.0", port), make_handler(session, commands))` on a
   daemon thread.
3. A subscription on `omni.kit.app.get_app().get_update_event_stream()` that
   drains `commands` and calls `session.start()` / `session.stop()` **on the main
   thread**.

Port from `SITL_RECORDER_PORT`, default 8091.

Guard against double-start on module re-exec the same way the recorder does
(`vio-recorder-pai.py:159-169`): stash the server in a module global and shut the
previous one down first, or `launch-sitl.sh` on a warm Kit leaves a bound socket.

### Step 3.2 — Wire into `bootstrap.py`

After `_run_setup_script`, alongside the other post-setup steps, and **advisory**
like all of them (`bootstrap.py:108-124`) — a control server that fails to bind
must not cost you the sim:

```python
    _advisory("recorder control server", _start_recorder_control)
```

Place it **before** the `AUTOPLAY` block so the server is accepting requests by
the time PX4 comes up and the operator reaches for the page.

### Step 3.3 — Verify the Kit side alone, before any UI

```bash
./sim/launch-sitl.sh
```

Then from another terminal — this is the whole feature, without a browser:

```bash
curl -s localhost:8091/record/status | python3 -m json.tool
curl -s -X POST localhost:8091/record/start
sleep 10
curl -s localhost:8091/record/status | python3 -m json.tool
curl -s -X POST localhost:8091/record/stop
```

- [ ] Status before start: `state: "idle"`, `can_start: true`, real `free_bytes`.
- [ ] After start: `state: "recording"`, and **`images` climbing** — the number
      that was zero in the 2026-08-02 run.
- [ ] `ls ~/vio_dataset/<newest>/` shows non-zero `imu.csv`, `poses.csv`,
      `frames.csv` and JPEGs under `images/cam0/`.
- [ ] Stop, then confirm the dataset **loads**:

```bash
conda run -n drone python -c "
from flow_odometry import load_dataset
import glob
d = sorted(glob.glob('$HOME/vio_dataset/*'))[-1]
K, R, recs = load_dataset(d)
print(f'{d}: {len(recs)} records, K[0,0]={K[0,0]:.1f}')
"
```

This is the acceptance bar for the whole feature — a dataset the pipeline cannot
load is not a recording.

- [ ] Second `POST /record/start` while recording returns 409 and does **not**
      create a second directory.
- [ ] With the timeline stopped (`AUTOPLAY=0`), start returns 503.

**Commit:** `feat(recorder): drive the recorder from a control server inside Kit`

---

## Task 4 — Web-server proxy

### Step 4.1 — Failing test

Add to `streaming/tests/test_offboard_loop.py` (the fake-PX4 file, so the
setpoint-thread assertion has a real loop to make it against):

```python
def test_record_commands_never_reach_the_setpoint_queue():
    """Design R2. Every other command routes through SetpointLoop.submit()
    because it touches MAVLink; recording does not. An HTTP call to Kit can
    block for hundreds of milliseconds, and a gap in the setpoint stream drops
    PX4 out of OFFBOARD (joystick-server.py:216-219) -- so routing RECORD
    through that thread would mean pressing it could drop the aircraft."""


def test_unreachable_control_server_reports_offline_not_an_error():
    """Normal before Isaac is up. The page must show a disabled button with a
    reason, not a stack trace."""


def test_status_poll_timeout_does_not_stall_telemetry():
    """The 5 Hz telemetry push must keep going while a status poll hangs."""
```

### Step 4.2 — Implement

In `joystick-server.py`:

- `--recorder-url`, default `http://127.0.0.1:8091`.
- `RecorderProxy` using `urllib.request` inside `asyncio.to_thread()` with a
  short timeout (2 s), since the `drone` env has no async HTTP client.
- A 1 Hz background task caching `/record/status`; on any exception the cached
  value becomes `{"state": "offline", "can_start": False}`.
- The cached dict is folded into the telemetry push as `rec` — 1 Hz because the
  numbers do not change faster and Kit should not be polled at telemetry rate.
- WS: `{type: "record", action: "start"|"stop"}` handled **inline in the
  WebSocket handler**, never via `loop_thread.submit()`.

```bash
conda run -n drone pytest streaming/tests/ sim/tests/ -q
```

**Commit:** `feat(web): proxy recorder control and status to the flight page`

---

## Task 5 — RECORD button and status

`web/index.html` gains a RECORD button beside the existing command row
(`web/index.html:56-62`) and a status line. `web/js/telemetry.js` paints it in the
established `good`/`wait`/`bad` vocabulary (`telemetry.js:84-105`); a new
`web/js/recorder.js` owns the button, matching the existing per-concern module
split (`controls.js`, `route.js`, `map.js`).

Shown while recording: elapsed, images, size, free space.

- [ ] **Stalled** — `state == "recording"` but `images` has not advanced for
      >3 s — renders `bad`. This is the zero-byte-run detector and the reason
      the feature exists; it must be impossible to miss.
- [ ] `dropped > 0` renders `bad` (the writer queue is saturating).
- [ ] Free space below `warn_free_bytes` renders `wait`; below `min_free_bytes`
      the button is disabled with the number in the label.
- [ ] `state == "offline"` disables the button and says the sim is not running.
- [ ] Recording is visually unmistakable — a red dot, not just a word.

Extend `streaming/tests/test_web_ui.py` in the existing style.

**Commit:** `feat(web): add the RECORD button and recorder status`

---

## Task 6 — Live bring-up

- [ ] **6.1** `./sim/launch-sitl.sh`; page shows `idle`, real free space, RECORD
      enabled.
- [ ] **6.2** Press RECORD → red dot, `images` climbing within a second, size
      growing.
- [ ] **6.3** Take off and fly a short route from the map. Recording is
      unaffected by flight commands, and — the R2 assertion, live — the aircraft
      **does not drop out of OFFBOARD** when RECORD is pressed mid-flight.
- [ ] **6.4** STOP. Dataset loads via `load_dataset` (Step 3.3's check).
- [ ] **6.5** Run `pipeline.py` or `flow_odometry.run()` over the new dataset
      end-to-end. Datasets have been recorded that load but do not process.
- [ ] **6.6** Kill Isaac mid-recording → page shows `offline`, partial dataset
      survives on disk.
- [ ] **6.7** Record with the disk artificially near the floor (a large
      temporary file) → RECORD disabled with the reason shown. Delete the file
      afterwards.
- [ ] **6.8** Note the measured GB/minute in `SESSION.md` — at 180 GB free it
      determines how many flights fit before pruning.

---

## Plan validation

Design coverage: R1 (Task 2, asserted by `test_start_request_only_enqueues_and_never_execs`),
R2 (Task 4 + Step 6.3), R3 (Step 4.2), R4 (Task 3 Step 3.1 — the recorder is only
ever read and exec'd), R5 (Task 1, `test_start_disables_the_takeoff_gate`),
R6 (Task 1 gating + Task 5 + Step 6.7), R7 (Task 1 status + Task 5 stalled
detection).

Names are consistent across tasks: `RecorderSession`, `RecorderControl`,
`handle_request`, `make_handler`, `STATE_IDLE`/`STATE_RECORDING`/`STATE_ERROR`,
`MIN_FREE_BYTES`/`WARN_FREE_BYTES`, `can_start`.

**Known gaps, deliberate:** Tasks 3–5 give structure and interfaces rather than
complete code. Task 3 is Kit-bound and cannot be written blind against
`omni.timeline`/replicator behaviour; Tasks 4 and 5 depend on the telemetry shape
Task 3 produces. Tasks 1 and 2 — the state machine and the HTTP layer, which hold
all the logic worth getting wrong — carry complete code and 25 tests.

**Riskiest step:** 3.1's `stop_recorder`. If `remove_physics_callback` is skipped
because an earlier step raised, the next start collides with a live `vio_rec`
callback and the failure looks like a corrupt dataset rather than a stop bug —
hence each step in its own `try`.

**Sequencing note:** this plan lands before `2026-08-07-vio-gps-denied.md`, whose
Task 1 Step 1.3 needs a recorded dataset for the AHRS baseline and whose Task 5
Step 5.2 replays one offline. Run Task 6 here to completion first, so that plan
starts with a known-good dataset.
