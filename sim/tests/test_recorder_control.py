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
    what makes data flow immediately without editing the recorder.

    Asserted through the recorder's own predicate (`climbed < TAKEOFF_ALT_M`,
    vio-recorder-pai.py:366) rather than against a literal, because `climbed`
    goes NEGATIVE when the drone settles a millimetre below its first sample.
    A gate value of 0.0 would re-latch there and write nothing -- the exact
    silent failure this feature exists to kill.
    """
    ns = make_ns()
    s, _, _ = make_session(ns=ns)
    s.start()
    for climbed in (0.0, -0.001, -0.5, -5.0):
        assert not (climbed < ns["TAKEOFF_ALT_M"]), (
            f"a {climbed:+.3f} m settle would re-arm the takeoff gate")


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


def test_the_socket_shell_serves_status_and_start_over_real_http():
    """handle_request is pure, but make_handler is what Kit actually binds, and
    everything in it -- method dispatch, the JSON body, Content-Length -- is
    otherwise first exercised inside Isaac where a mistake costs a launch."""
    import json
    import threading
    import urllib.request
    from http.server import ThreadingHTTPServer

    s, _, _ = make_session()
    commands = FakeQueue()
    srv = ThreadingHTTPServer(("127.0.0.1", 0),
                              rc.make_handler(s, commands))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    port = srv.server_address[1]
    try:
        with urllib.request.urlopen(
                f"http://127.0.0.1:{port}/record/status", timeout=5) as r:
            assert r.status == 200
            body = json.loads(r.read())
        assert body["state"] == rc.STATE_IDLE
        assert "free_bytes" in body and "can_start" in body

        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/record/start", method="POST", data=b"")
        with urllib.request.urlopen(req, timeout=5) as r:
            assert r.status == 202
        assert commands.items == ["start"]

        # A query string must not defeat the route match.
        with urllib.request.urlopen(
                f"http://127.0.0.1:{port}/record/status?t=1", timeout=5) as r:
            assert r.status == 200
    finally:
        srv.shutdown()
        srv.server_close()
