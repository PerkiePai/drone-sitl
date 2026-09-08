# Docker Submissions in the Website Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let an operator upload a `.tar` bundle (Dockerfile + competitor code + weights) through the website's Agent panel, have the server build it, and RUN it exactly like today's single-`.py` scripts — same ARM→TAKEOFF→OFFBOARD sequencing, same STOP, same log pane pattern.

**Architecture:** A new `streaming/docker_build.py` module (`DockerBuild` class: `idle|building|built|error` + capped log, mirrors `AgentRun`'s shape) builds an uploaded bundle synchronously on upload via `docker build`. `AgentRun` gains a `kind` (`"script"` | `"docker"`): `run()`/`_spawn()`/`stop()` branch on it, but the ARM/TAKEOFF/OFFBOARD state machine, `/agent/control`, and the log pane are shared, unchanged code. The UI gets a second file picker + image dropdown + build-log pane next to the existing ones; RUN is one shared button.

**Tech Stack:** Python 3.11 (`drone` conda env), FastAPI, `asyncio.create_subprocess_exec`, Docker CLI (already installed, GPU runtime confirmed present), vanilla JS (no framework) in `web/js/`.

**Spec:** `docs/superpowers/specs/2026-09-08-docker-submission-website-integration-design.md`

## Global Constraints

- `AGENT_DOCKER_MAX_BYTES = 200 * 1024 * 1024` (200 MiB) — from the spec's Decision 1.
- Upload endpoint is raw bytes, **not multipart** (no `python-multipart` dependency) — matches `/agent/upload`'s existing pattern exactly.
- `competition-base:latest` must already exist on the host (built via `docker/competition-base/Dockerfile`, already done this session) — not built per-request.
- `docker run` is spawned **in the foreground** (no `-d`) via `subprocess.Popen`, so it has the same terminate/exit lifecycle as the existing `agent_runner.py` subprocess path — this is what lets `AgentRun` share code between the two kinds.
- `stop()` for a docker run must not rely solely on signal-forwarding — always issue an explicit `docker stop <name>` too (spec Decision 3).
- Every new server-side constant/endpoint follows the naming already established: `AGENT_UPLOAD_DIR` → `AGENT_DOCKER_UPLOAD_DIR`, `/agent/upload` → `/agent/upload-docker`, `/agent/list` → `/agent/docker-list`.

---

### Task 1: `DockerBuild` — the build state machine

**Files:**
- Create: `streaming/docker_build.py`
- Test: `streaming/tests/test_docker_build.py`

**Interfaces:**
- Produces: `class DockerBuild` with `.state` (`"idle"|"building"|"built"|"error"`), `.image_tag`, `async def build(self, tar_path: str, stem: str) -> tuple[bool, str | None]`, `.snapshot() -> dict`.
- Produces: `async def list_images() -> list[dict]` (module-level function), each dict `{"tag": str, "created_at": str}`.
- Produces: `IMAGE_PREFIX = "submission-"` (module constant).

- [ ] **Step 1: Write the failing tests**

```python
# streaming/tests/test_docker_build.py
"""streaming.docker_build.DockerBuild -- builds an uploaded bundle.

No real Docker in these tests -- asyncio.create_subprocess_exec is
monkeypatched. Task 5's manual pass is what exercises a real Docker
daemon end to end; pytest-asyncio is not a dependency of this repo (see
streaming/tests/test_web_ui.py's existing `async def exercise():` +
`asyncio.run(exercise())` pattern), so every test here follows that same
shape: a plain `def test_...():` that wraps its async body in
`asyncio.run(...)`, not `@pytest.mark.asyncio`.
"""
import asyncio
import io
import os
import sys
import tarfile

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))

import docker_build  # noqa: E402


def _make_bundle(tmp_path, with_dockerfile=True):
    src = tmp_path / "src"
    src.mkdir()
    if with_dockerfile:
        (src / "Dockerfile").write_text("FROM busybox\n")
    tar_path = tmp_path / "bundle.tar"
    with tarfile.open(tar_path, "w") as tar:
        for f in src.iterdir():
            tar.add(f, arcname=f.name)
    return str(tar_path)


class FakeProc:
    """Stands in for asyncio.subprocess.Process."""
    def __init__(self, lines, returncode):
        self._lines = [l.encode() + b"\n" for l in lines]
        self.returncode = returncode
        self.stdout = self

    def __aiter__(self):
        return self

    async def __anext__(self):
        if not self._lines:
            raise StopAsyncIteration
        return self._lines.pop(0)

    async def wait(self):
        return self.returncode


def test_build_succeeds_and_sets_state_built(tmp_path, monkeypatch):
    tar_path = _make_bundle(tmp_path)

    async def fake_exec(*args, **kwargs):
        assert args[0] == "docker" and args[1] == "build"
        return FakeProc(["Step 1/1 : FROM busybox", "Successfully built abc123"], 0)

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        db = docker_build.DockerBuild()
        ok, image_tag = await db.build(tar_path, "myagent")
        assert ok is True
        assert image_tag.startswith("submission-myagent-")
        assert db.state == "built"
        assert db.image_tag == image_tag
        assert "Successfully built abc123" in db.snapshot()["log"]

    asyncio.run(run())


def test_build_failure_sets_state_error(tmp_path, monkeypatch):
    tar_path = _make_bundle(tmp_path)

    async def fake_exec(*args, **kwargs):
        return FakeProc(["ERROR: something broke"], 1)

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        db = docker_build.DockerBuild()
        ok, image_tag = await db.build(tar_path, "myagent")
        assert ok is False
        assert image_tag is None
        assert db.state == "error"
        assert any("exited 1" in line for line in db.snapshot()["log"])

    asyncio.run(run())


def test_missing_dockerfile_fails_before_any_subprocess_call(tmp_path, monkeypatch):
    tar_path = _make_bundle(tmp_path, with_dockerfile=False)
    called = []

    async def fake_exec(*args, **kwargs):
        called.append(args)
        raise AssertionError("should never reach docker build")

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        db = docker_build.DockerBuild()
        ok, image_tag = await db.build(tar_path, "myagent")
        assert ok is False
        assert image_tag is None
        assert db.state == "error"
        assert not called
        assert any("no Dockerfile" in line for line in db.snapshot()["log"])

    asyncio.run(run())


def test_docker_not_installed_sets_state_error(tmp_path, monkeypatch):
    tar_path = _make_bundle(tmp_path)

    async def fake_exec(*args, **kwargs):
        raise FileNotFoundError("docker")

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        db = docker_build.DockerBuild()
        ok, image_tag = await db.build(tar_path, "myagent")
        assert ok is False
        assert db.state == "error"
        assert any("not found" in line for line in db.snapshot()["log"])

    asyncio.run(run())


def test_tar_member_escaping_the_context_is_rejected(tmp_path):
    """A malicious bundle with `../../etc/passwd`-style paths must not
    write outside the extraction directory."""
    tar_path = tmp_path / "evil.tar"
    with tarfile.open(tar_path, "w") as tar:
        info = tarfile.TarInfo(name="../../evil.txt")
        info.size = 4
        tar.addfile(info, io.BytesIO(b"evil"))

    dest = tmp_path / "dest"
    dest.mkdir()
    with tarfile.open(tar_path) as tar:
        with pytest.raises(ValueError, match="escapes"):
            docker_build._safe_extract(tar, str(dest))


def test_list_images_parses_docker_output(monkeypatch):
    async def fake_exec(*args, **kwargs):
        assert args[:3] == ("docker", "image", "ls")
        class P:
            async def communicate(self):
                return (b"submission-a-1:latest\t2026-09-08 10:00:00\n"
                        b"submission-b-2:latest\t2026-09-08 11:00:00\n", b"")
        return P()

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        images = await docker_build.list_images()
        assert images[0]["tag"] == "submission-b-2:latest"    # newest first
        assert images[1]["tag"] == "submission-a-1:latest"

    asyncio.run(run())
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `conda run -n drone python -m pytest streaming/tests/test_docker_build.py -v`
Expected: collection error / `ModuleNotFoundError: No module named 'docker_build'`

- [ ] **Step 3: Write the implementation**

```python
# streaming/docker_build.py
"""Builds a competitor's uploaded Docker bundle. Mirrors AgentRun's shape
(state machine + capped log) but for `docker build` instead of flying a
script. See
docs/superpowers/specs/2026-09-08-docker-submission-website-integration-design.md.
"""
import asyncio
import collections
import os
import shutil
import tarfile
import tempfile
import time

IMAGE_PREFIX = "submission-"


def _safe_extract(tar, dest):
    """Extract `tar` into `dest`, refusing any member whose resolved path
    would land outside `dest` (path traversal via `../` or an absolute
    path). Python 3.11's tarfile.extractall has no `filter=` kwarg
    (that's 3.12+), so this is done by hand rather than relying on one."""
    dest_real = os.path.realpath(dest)
    for member in tar.getmembers():
        member_path = os.path.realpath(os.path.join(dest, member.name))
        if member_path != dest_real and not member_path.startswith(dest_real + os.sep):
            raise ValueError(f"tar member escapes the build context: {member.name!r}")
    tar.extractall(dest)


class DockerBuild:
    def __init__(self, base_image="competition-base:latest"):
        self.base_image = base_image
        self.state = "idle"          # idle | building | built | error
        self.image_tag = None
        self._log = collections.deque(maxlen=40)

    def snapshot(self):
        return {"state": self.state, "image_tag": self.image_tag,
                "log": list(self._log)}

    def _note(self, line):
        self._log.append(line)

    async def build(self, tar_path, stem):
        """Extract `tar_path`, docker build it, tag submission-<stem>-<ts>.
        Returns (ok, image_tag | None)."""
        self.state = "building"
        self._log.clear()
        ts = time.strftime("%Y%m%d-%H%M%S")
        image_tag = f"{IMAGE_PREFIX}{stem}-{ts}"
        ctx = tempfile.mkdtemp(prefix="docker-build-")
        try:
            with tarfile.open(tar_path) as tar:
                _safe_extract(tar, ctx)
            dockerfile = os.path.join(ctx, "Dockerfile")
            if not os.path.isfile(dockerfile):
                self.state = "error"
                self._note("no Dockerfile at the bundle's root")
                return False, None

            self._note(f"building {image_tag} ...")
            proc = await asyncio.create_subprocess_exec(
                "docker", "build", "-t", image_tag, "-f", dockerfile, ctx,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT)
            async for line in proc.stdout:
                self._note(line.decode(errors="replace").rstrip())
            code = await proc.wait()

            if code != 0:
                self.state = "error"
                self._note(f"docker build exited {code}")
                return False, None

            self.state = "built"
            self.image_tag = image_tag
            self._note(f"built {image_tag}")
            return True, image_tag
        except FileNotFoundError:
            self.state = "error"
            self._note("docker not found -- is it installed on this host?")
            return False, None
        except Exception as exc:                      # noqa: BLE001
            self.state = "error"
            self._note(f"build failed: {exc!r}")
            return False, None
        finally:
            shutil.rmtree(ctx, ignore_errors=True)


async def list_images():
    """Built submission images, newest first: [{"tag":..., "created_at":...}]."""
    proc = await asyncio.create_subprocess_exec(
        "docker", "image", "ls",
        "--filter", f"reference={IMAGE_PREFIX}*",
        "--format", "{{.Repository}}:{{.Tag}}\t{{.CreatedAt}}",
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
    out, _ = await proc.communicate()
    rows = []
    for line in out.decode(errors="replace").splitlines():
        if not line.strip():
            continue
        tag, _, created = line.partition("\t")
        rows.append({"tag": tag, "created_at": created})
    rows.sort(key=lambda r: r["created_at"], reverse=True)
    return rows
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest streaming/tests/test_docker_build.py -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add streaming/docker_build.py streaming/tests/test_docker_build.py
git commit -m "feat(docker): DockerBuild state machine + list_images()

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011WEp4YtTLi1h6yxhc92J3P"
```

---

### Task 2: `/agent/upload-docker` and `/agent/docker-list` endpoints

**Files:**
- Modify: `joystick-server.py` (constants near line 39-46, `_safe_agent_name` near line 61-68, endpoints near line 475-539, `build_app` signature line 450, `_push_telemetry` line 434-447, call sites lines 546/608/690)
- Test: `streaming/tests/test_web_ui.py`

**Interfaces:**
- Consumes: `docker_build.DockerBuild`, `docker_build.list_images()` (Task 1).
- Produces: `_safe_docker_bundle_name(name) -> str | None`. `AGENT_DOCKER_UPLOAD_DIR`, `AGENT_DOCKER_MAX_BYTES` module constants. `build_app(loop_thread, state, video_port, mission_speed, agent_run, agent_docker_build)` (one new positional param, appended last so every existing call site needs exactly one addition).

- [ ] **Step 1: Write the failing tests**

Append to `streaming/tests/test_web_ui.py` (find the existing `# --- /agent/control socket ---` section and add a new one near it, following the file's existing `server` fixture and `websockets.connect`/`urllib` patterns already used throughout that file):

```python
# --- /agent/upload-docker, /agent/docker-list ------------------------

def _make_tar_bytes(with_dockerfile=True):
    import io
    import tarfile
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w") as tar:
        if with_dockerfile:
            data = b"FROM busybox\n"
            info = tarfile.TarInfo(name="Dockerfile")
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return buf.getvalue()


def test_docker_upload_rejects_non_tar_name(server):
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload-docker?name=bundle.zip",
        data=_make_tar_bytes(), method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_docker_upload_rejects_oversize_body(server):
    big = b"x" * (256 * 1024 * 1024)   # over AGENT_DOCKER_MAX_BYTES (200 MiB)
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload-docker?name=bundle.tar",
        data=big, method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_docker_upload_without_dockerfile_in_bundle_fails(server):
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/upload-docker?name=bundle.tar",
        data=_make_tar_bytes(with_dockerfile=False), method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400
        body = json.loads(e.read())
        assert "Dockerfile" in body["detail"] or "log" in body


def test_agent_docker_list_returns_json_list(server):
    r = urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/agent/docker-list")
    body = json.loads(r.read())
    assert "images" in body and isinstance(body["images"], list)
```

Check the top of `streaming/tests/test_web_ui.py` for its existing imports (`urllib.request`, `urllib.error`, `json`, the `server` fixture, `WEB_PORT`) before adding this — reuse them, don't re-import.

- [ ] **Step 2: Run tests to verify they fail**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -k docker -v`
Expected: FAIL with 404 (route doesn't exist yet) on all four.

- [ ] **Step 3: Add the constants and helper**

In `joystick-server.py`, right after the existing `AGENT_TIME_LIMIT_S = None` block (around line 42-46, next to the `AGENT_FLIGHT_SPEED`/`AGENT_FLIGHT_YAW_RATE_DPS` constants added for `flight()`):

```python
AGENT_DOCKER_UPLOAD_DIR = os.path.join(ROOT, "logs", "docker-agents")
AGENT_DOCKER_MAX_BYTES = 200 * 1024 * 1024
os.makedirs(AGENT_DOCKER_UPLOAD_DIR, exist_ok=True)
```

Right after `_safe_agent_name` (around line 68):

```python
def _safe_docker_bundle_name(name):
    """A base filename ending .tar with no path parts, or None."""
    if not name or not name.endswith(".tar"):
        return None
    if name != os.path.basename(name) or "/" in name or "\\" in name \
            or ".." in name:
        return None
    return name
```

Add the import near the other `streaming/` imports (after `import agent_control as agentctl`):

```python
import docker_build                                  # noqa: E402
```

- [ ] **Step 4: Add the endpoints**

In `joystick-server.py`, right after the existing `/agent/list` endpoint (ends around line 539):

```python
    @app.post("/agent/upload-docker")
    async def agent_upload_docker(request: Request):
        # Raw body, not multipart -- same reasoning as /agent/upload.
        name = request.query_params.get("name", "")
        safe = _safe_docker_bundle_name(name)
        if safe is None:
            return JSONResponse({"detail": "name must be a bare *.tar filename"},
                                status_code=400)
        body = await request.body()
        if len(body) > AGENT_DOCKER_MAX_BYTES:
            return JSONResponse(
                {"detail": f"bundle over {AGENT_DOCKER_MAX_BYTES} bytes"},
                status_code=400)
        stem = safe[:-4]
        stored = f"{stem}-{time.strftime('%Y%m%d-%H%M%S')}.tar"
        tar_path = os.path.join(AGENT_DOCKER_UPLOAD_DIR, stored)
        with open(tar_path, "wb") as fh:
            fh.write(body)
        ok, image_tag = await agent_docker_build.build(tar_path, stem)
        if not ok:
            return JSONResponse(
                {"detail": "build failed", "log": agent_docker_build.snapshot()["log"]},
                status_code=400)
        return JSONResponse({"stored": stored, "image_tag": image_tag})

    @app.get("/agent/docker-list")
    async def agent_docker_list():
        images = await docker_build.list_images()
        return JSONResponse({"images": images})
```

Note `agent_docker_build` here is a closure variable, not yet defined -- Step 5 adds it as a `build_app` parameter, same way `agent_run` already works in this function.

- [ ] **Step 5: Wire `agent_docker_build` through `build_app` and telemetry**

Change the `build_app` signature (line 450):

```python
def build_app(loop_thread, state, video_port, mission_speed, agent_run,
              agent_docker_build):
```

Change `_push_telemetry` (line 434) to also snapshot the build state:

```python
async def _push_telemetry(sock, loop_thread, agent_run=None,
                          agent_docker_build=None, hz=5.0):
    try:
        while True:
            t = loop_thread.telemetry()
            if agent_run is not None:
                agent_run.tick()
                t["agent"] = agent_run.snapshot()
            if agent_docker_build is not None:
                t["docker_build"] = agent_docker_build.snapshot()
            await sock.send_text(json.dumps(t))
            await asyncio.sleep(1.0 / hz)
    except Exception:
        pass          # socket closed; the /ws handler cleans up
```

Update the `/ws` call site (line 546) to pass it through:

```python
            _push_telemetry(sock, loop_thread, agent_run, agent_docker_build))
```

(The `/agent/control` call site at line 608 stays `_push_telemetry(sock, loop_thread)` unchanged -- that socket is for the child process, not the browser.)

Update the `main()` construction (around line 682-691) to build and pass it:

```python
    agent_run = AgentRun(loop_thread, sys.executable, "127.0.0.1", args.port,
                         args.video_port)
    agent_docker_build = docker_build.DockerBuild()
    ...
    uvicorn.run(build_app(loop_thread, state, args.video_port,
                          args.mission_speed, agent_run, agent_docker_build),
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -k docker -v`
Expected: all PASS. Then run the full file to confirm nothing else broke:
Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -v`
Expected: all PASS

- [ ] **Step 7: Commit**

```bash
git add joystick-server.py streaming/tests/test_web_ui.py
git commit -m "feat(docker): /agent/upload-docker and /agent/docker-list endpoints

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011WEp4YtTLi1h6yxhc92J3P"
```

---

### Task 3: `AgentRun` gains a `kind` — script vs. docker execution

**Files:**
- Modify: `joystick-server.py` (`class AgentRun`, lines ~319-431: `__init__`, `run`, `stop`, `_spawn`)
- Test: `streaming/tests/test_web_ui.py`

**Interfaces:**
- Consumes: nothing new from other tasks.
- Produces: `AgentRun.run(kind: str, identifier: str)` (was `run(filename)`), `AgentRun.snapshot()` gains `"kind"` key.

- [ ] **Step 1: Write the failing tests**

Add near any existing `AgentRun` tests in `streaming/tests/test_web_ui.py` (search the file for `AgentRun(` to find its existing construction pattern and reuse it):

```python
def test_agent_run_docker_spawns_docker_run_with_network_host_and_name(monkeypatch):
    import joystick_server as js   # however the existing AgentRun tests import it -- match that file's pattern
    calls = []

    class FakePopen:
        def __init__(self, argv, **kwargs):
            calls.append(argv)
            self.stdout = iter([])
        def poll(self):
            return None

    monkeypatch.setattr(js.subprocess, "Popen", FakePopen)

    run = js.AgentRun(loop_thread=_fake_loop_thread_connected(), python_exe="python",
                      host="127.0.0.1", port=8090, video_port=8080)
    run.run("docker", "submission-foo-20260908-120000")
    run.loop_thread._telem["armed"] = True
    run.tick()
    run.loop_thread._telem["alt_m"] = 5.0
    run.loop_thread._telem["vz"] = 0.0
    run.loop_thread._telem["ready_for_offboard"] = True
    run.tick()
    run.loop_thread._telem["mode"] = "OFFBOARD"
    run.tick()

    argv = calls[0]
    assert argv[:6] == ["docker", "run", "--rm", "--network", "host", "--gpus"]
    assert "submission-foo-20260908-120000" in argv
    assert "--name" in argv


def test_agent_run_stop_issues_docker_stop_for_docker_kind(monkeypatch):
    import joystick_server as js
    stop_calls = []

    class FakePopen:
        def __init__(self, argv, **kwargs):
            self.stdout = iter([])
        def poll(self):
            return None
        def terminate(self):
            pass
        def wait(self, timeout=None):
            pass

    def fake_run(argv, **kwargs):
        stop_calls.append(argv)
        class R:
            returncode = 0
        return R()

    monkeypatch.setattr(js.subprocess, "Popen", FakePopen)
    monkeypatch.setattr(js.subprocess, "run", fake_run)

    run = js.AgentRun(loop_thread=_fake_loop_thread_connected(), python_exe="python",
                      host="127.0.0.1", port=8090, video_port=8080)
    run.run("docker", "submission-foo-20260908-120000")
    for key, val in (("armed", True), (), ()):
        pass
    run.loop_thread._telem["armed"] = True
    run.tick()
    run.loop_thread._telem["alt_m"] = 5.0
    run.loop_thread._telem["vz"] = 0.0
    run.loop_thread._telem["ready_for_offboard"] = True
    run.tick()
    run.loop_thread._telem["mode"] = "OFFBOARD"
    run.tick()

    run.stop("test")

    assert any(a[:2] == ["docker", "stop"] for a in stop_calls)
```

These two tests reference a `_fake_loop_thread_connected()` helper -- **before writing them**, search `streaming/tests/test_web_ui.py` for how existing `AgentRun`/preflight tests fake `loop_thread` (it needs `.telemetry()`, `.submit()`, `.agent_control`, and a mutable `._telem` dict the test pokes at directly) and reuse that exact fixture/helper instead of inventing a new one -- match whatever name and shape the file already has. If the file constructs `AgentRun` differently (e.g. via a fixture), follow that pattern instead of the constructor call shown above.

- [ ] **Step 2: Run tests to verify they fail**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -k "docker_spawns or docker_stop" -v`
Expected: FAIL — `run()` doesn't accept two positional args yet (`TypeError`).

- [ ] **Step 3: Implement**

In `joystick-server.py`, `AgentRun.__init__` (line ~328-337), add two attributes:

```python
    def __init__(self, loop_thread, python_exe, host, port, video_port):
        self.loop_thread = loop_thread
        self.python_exe = python_exe
        self.host, self.port, self.video_port = host, port, video_port
        self.state = "idle"        # idle | arming | running | stopped | error
        self.kind = None           # script | docker
        self.file = None
        self._proc = None
        self._container_name = None
        self._phase = None         # arm | takeoff | offboard  (while arming)
        self._log = collections.deque(maxlen=40)
        self._lock = threading.Lock()
```

Add `"kind": self.kind` to `snapshot()`'s returned dict (line ~340-344):

```python
    def snapshot(self):
        with self._lock:
            running = self.state in ("arming", "running")
            return {"state": self.state, "kind": self.kind, "file": self.file,
                    "camera": self.loop_thread.agent_camera if running else None,
                    "log": list(self._log)}
```

Replace `run()` (line ~346-363):

```python
    def run(self, kind, identifier):
        if not self.loop_thread.telemetry().get("connected"):
            self._note("RUN refused: no MAVLink link")
            self.state = "error"
            return
        if self.state in ("arming", "running"):
            self._note("RUN refused: an agent is already running -- STOP first")
            return
        if kind == "script":
            path = os.path.join(AGENT_UPLOAD_DIR, os.path.basename(identifier or ""))
            if not identifier or not os.path.isfile(path):
                self._note(f"RUN refused: {identifier!r} not found")
                self.state = "error"
                return
        elif kind == "docker":
            if not identifier:
                self._note("RUN refused: no docker image selected")
                self.state = "error"
                return
        else:
            self._note(f"RUN refused: unknown kind {kind!r}")
            self.state = "error"
            return
        self.kind = kind
        self.file = identifier
        self.state = "arming"
        self._phase = "arm"
        self._note(f"arming for {identifier}")
        self.loop_thread.submit("arm")
```

Replace `stop()` (line ~365-380):

```python
    def stop(self, why="stopped"):
        if self.state not in ("arming", "running"):
            return
        self._note(f"stop: {why}")
        if self._container_name:
            subprocess.run(["docker", "stop", self._container_name],
                           capture_output=True, timeout=5)
        p = self._proc
        if p and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                p.kill()
        self._proc = None
        self._container_name = None
        self.loop_thread.submit("mission_clear")
        self.loop_thread.agent_control.clear()
        self.state = "stopped"
        self._phase = None
```

Replace `_spawn()` (line ~412-422):

```python
    def _spawn(self):
        if self.kind == "docker":
            self._container_name = f"submission-run-{time.strftime('%Y%m%d-%H%M%S')}"
            self._proc = subprocess.Popen(
                ["docker", "run", "--rm", "--network", "host", "--gpus", "all",
                 "--name", self._container_name, self.file],
                cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1)
        else:
            self._container_name = None
            path = os.path.join(AGENT_UPLOAD_DIR, os.path.basename(self.file))
            self._proc = subprocess.Popen(
                [self.python_exe, "-u", os.path.join(ROOT, "agent_runner.py"),
                 "--file", path, "--host", self.host, "--port", str(self.port),
                 "--video-port", str(self.video_port)],
                cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1)
        threading.Thread(target=self._drain_child, daemon=True).start()
        self.state = "running"
        self._note("agent running")
```

Update the `/ws` handler's `action == "run"` branch (line ~567-568) to pass both fields:

```python
                    if action == "run":
                        agent_run.run(msg.get("kind", "script"),
                                      msg.get("identifier", ""))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -v`
Expected: all PASS, including the pre-existing tests that call `agent_run.run(filename)` the old way — **check for any such call sites first** (`grep -n "agent_run.run(\|\.run(msg\|action.*run" streaming/tests/test_web_ui.py`) and update each to the new two-arg `run("script", filename)` form before running.

- [ ] **Step 5: Commit**

```bash
git add joystick-server.py streaming/tests/test_web_ui.py
git commit -m "feat(docker): AgentRun gains a kind, runs docker containers via docker run

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011WEp4YtTLi1h6yxhc92J3P"
```

---

### Task 4: Web UI — second picker, build log, shared RUN

**Files:**
- Modify: `web/index.html` (the `#agent` block, lines 54-63)
- Modify: `web/css/app.css` (lines 86-89)
- Modify: `web/js/agent.js` (whole file)

**Interfaces:**
- Consumes: `/agent/upload-docker`, `/agent/docker-list` (Task 2); `t.docker_build`, `t.agent.kind` in telemetry (Tasks 2-3); wire message `{type:"agent", action:"run", kind, identifier}` (Task 3).
- Produces: nothing consumed by a later task — this is the last task.

- [ ] **Step 1: Update `web/index.html`**

Replace the `#agent` block (lines 54-63):

```html
  <div id="agent">
    <div class="a-row">
      <select id="a-file" title="uploaded scripts"></select>
      <label class="a-file-btn">upload<input id="a-upload" type="file" accept=".py"></label>
    </div>
    <div class="a-row">
      <select id="a-docker-image" title="built docker images"></select>
      <label class="a-file-btn">upload bundle<input id="a-docker-upload" type="file" accept=".tar"></label>
    </div>
    <div class="a-row">
      <button id="a-run" disabled>RUN</button>
      <button id="a-stop" disabled>STOP</button>
      <span id="a-state">idle</span>
    </div>
    <pre id="a-build-log"></pre>
    <pre id="a-log"></pre>
  </div>
```

- [ ] **Step 2: Update `web/css/app.css`**

Change (lines 86-89):

```css
#a-log { margin-top:6px; height:9em; overflow-y:auto; white-space:pre-wrap;
```

to:

```css
#a-log, #a-build-log { margin-top:6px; height:9em; overflow-y:auto; white-space:pre-wrap;
```

and the matching `:empty` rule from `#a-log:empty { display:none; }` to `#a-log:empty, #a-build-log:empty { display:none; }`.

- [ ] **Step 3: Rewrite `web/js/agent.js`**

```javascript
// The Agent panel: upload a .py script or a Docker bundle, RUN either
// one, watch its log. Flight goes through the same /ws socket every
// other control uses.
import { send } from './ws.js';

const el = id => document.getElementById(id);
let lastLogLen = 0;
let lastBuildLogLen = 0;
let wasRunning = false;
let activeKind = 'script';   // last thing the operator picked or uploaded

async function refreshList(selected) {
  try {
    const { files } = await (await fetch('/agent/list')).json();
    const sel = el('a-file');
    sel.innerHTML = '';
    for (const f of files) {
      const o = document.createElement('option');
      o.value = o.textContent = f;
      sel.appendChild(o);
    }
    if (selected) sel.value = selected;
  } catch (e) { /* server not up yet */ }
}

async function refreshDockerList(selected) {
  try {
    const { images } = await (await fetch('/agent/docker-list')).json();
    const sel = el('a-docker-image');
    sel.innerHTML = '';
    for (const img of images) {
      const o = document.createElement('option');
      o.value = o.textContent = img.tag;
      sel.appendChild(o);
    }
    if (selected) sel.value = selected;
  } catch (e) { /* server not up yet */ }
}

async function upload(file) {
  const text = await file.text();
  const r = await fetch(
    `/agent/upload?name=${encodeURIComponent(file.name)}`,
    { method: 'POST', headers: { 'Content-Type': 'text/x-python' }, body: text });
  if (!r.ok) {
    const { detail } = await r.json().catch(() => ({ detail: r.statusText }));
    el('a-state').textContent = `upload failed: ${detail}`;
    return;
  }
  const { stored } = await r.json();
  activeKind = 'script';
  await refreshList(stored);
}

async function uploadDocker(file) {
  const bytes = await file.arrayBuffer();
  el('a-build-log').textContent = 'building...';
  const r = await fetch(
    `/agent/upload-docker?name=${encodeURIComponent(file.name)}`,
    { method: 'POST', body: bytes });
  if (!r.ok) {
    const body = await r.json().catch(() => ({}));
    el('a-state').textContent = `build failed: ${body.detail || r.statusText}`;
    if (body.log) el('a-build-log').textContent = body.log.join('\n');
    return;
  }
  const { image_tag } = await r.json();
  activeKind = 'docker';
  await refreshDockerList(image_tag);
}

export function initAgent() {
  refreshList();
  refreshDockerList();
  el('a-upload').addEventListener('change', e => {
    if (e.target.files[0]) upload(e.target.files[0]);
    e.target.value = '';
  });
  el('a-docker-upload').addEventListener('change', e => {
    if (e.target.files[0]) uploadDocker(e.target.files[0]);
    e.target.value = '';
  });
  el('a-file').addEventListener('change', () => { activeKind = 'script'; });
  el('a-docker-image').addEventListener('change', () => { activeKind = 'docker'; });
  el('a-run').addEventListener('click', () => {
    const identifier = activeKind === 'docker'
      ? el('a-docker-image').value : el('a-file').value;
    if (identifier) {
      el('a-log').textContent = '';
      lastLogLen = 0;
      send({ type: 'agent', action: 'run', kind: activeKind, identifier });
    }
  });
  el('a-stop').addEventListener('click',
    () => send({ type: 'agent', action: 'stop' }));
}

export function paintAgent(t) {
  const a = t.agent;
  if (a) {
    el('a-state').textContent = a.state + (a.camera ? ` · ${a.camera}` : '');
    const running = a.state === 'arming' || a.state === 'running';
    el('a-run').disabled = !t.connected || running;
    el('a-stop').disabled = !running;

    if (a.log.length !== lastLogLen) {
      el('a-log').textContent = a.log.join('\n');
      el('a-log').scrollTop = el('a-log').scrollHeight;
      lastLogLen = a.log.length;
    }

    // Swap the main video feed to the agent's selected camera while it
    // flies, and restore the default forward feed when it stops.
    const v = el('video');
    const port = v.dataset.port || 8080;
    if (a.camera) {
      const path = a.camera === 'oblique' ? 'detect' : 'down';
      const want = `http://${location.hostname}:${port}/${path}`;
      if (v.src !== want) v.src = want;
    } else if (wasRunning) {
      v.src = `http://${location.hostname}:${port}/detect`;
    }
    wasRunning = running;
  }

  const b = t.docker_build;
  if (b && b.log.length !== lastBuildLogLen) {
    el('a-build-log').textContent = b.log.join('\n');
    el('a-build-log').scrollTop = el('a-build-log').scrollHeight;
    lastBuildLogLen = b.log.length;
  }
}
```

- [ ] **Step 4: Manual check — no Isaac/PX4 needed for this step**

Run: `conda run -n drone python joystick-server.py` (or `SKIP_SIM=1 ./sim/run-website.sh` if that env var is supported — check `sim/run-website.sh` first) and load `http://127.0.0.1:8090/` in a browser. Confirm: both pickers render, no console errors, `#a-build-log` and `#a-log` are both empty and hidden (the `:empty` CSS rule).

- [ ] **Step 5: Commit**

```bash
git add web/index.html web/css/app.css web/js/agent.js
git commit -m "feat(docker): website UI for uploading and running Docker bundles

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011WEp4YtTLi1h6yxhc92J3P"
```

---

### Task 5: Full live verification against the running site

**Files:** none created — this is the manual acceptance pass the user asked to see.

**Interfaces:**
- Consumes: everything from Tasks 1-4, running together.

- [ ] **Step 1: Restart the website with the new code**

Run: `./sim/stop-website.sh` (via the `stop-website` skill or directly) then `./sim/run-website.sh` (via the `run-website` skill). Confirm `curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/` → `200` and `grep -q '>>> params:' logs/joystick-server.console`.

- [ ] **Step 2: Build a real upload bundle from the already-verified mock**

```bash
cd docker/mock-flight-competitor
tar -cf /tmp/mock-flight-bundle.tar Dockerfile
```

Note this Dockerfile's `COPY examples/flight.py .` line references a path relative to the repo root, not this bundle directory — **before tarring**, copy `examples/flight.py` alongside the Dockerfile into a scratch dir so the bundle is self-contained (the whole point of the bundle format is that it doesn't depend on repo-root-relative paths the way the CLI-only verification did):

```bash
mkdir -p /tmp/flight-bundle && cd /tmp/flight-bundle
cp /home/innovation/pai/drone-sitl/docker/mock-flight-competitor/Dockerfile .
sed -i 's#examples/flight.py#flight.py#' Dockerfile
cp /home/innovation/pai/drone-sitl/examples/flight.py .
tar -cf /tmp/mock-flight-bundle.tar Dockerfile flight.py
```

- [ ] **Step 3: Upload it through the real endpoint (not the browser yet — confirm the pipe works first)**

```bash
curl -s -X POST --data-binary @/tmp/mock-flight-bundle.tar \
  "http://127.0.0.1:8090/agent/upload-docker?name=mock-flight-bundle.tar"
```

Expected: JSON with `"image_tag": "submission-mock-flight-bundle-..."`. If it returns `"detail": "build failed"`, read the accompanying `"log"` array — it's the real `docker build` output.

- [ ] **Step 4: Confirm it's listed**

```bash
curl -s http://127.0.0.1:8090/agent/docker-list
```

Expected: the tag from Step 3 present.

- [ ] **Step 5: Open the actual browser, upload through the UI, RUN it**

Browse to `http://<box-ip>:8090/`. Under the Agent panel: use **upload bundle** to pick `/tmp/mock-flight-bundle.tar`, confirm `#a-build-log` fills in and the new image appears in the docker dropdown. Select it, press **RUN**. Confirm: `#a-state` goes `arming` → `running`, the drone arms/takes off/goes OFFBOARD (same as any script run), and it flies forward — same behavior already confirmed via the CLI path. Press **STOP**, confirm it returns to `stopped` and the drone holds/can be landed manually.

- [ ] **Step 6: Land and confirm clean state**

Land via the pad or `{"type":"cmd","name":"land"}` over `/ws` (same as done earlier this session). Confirm `armed: false` once down.

- [ ] **Step 7: Note the result** (no commit — this task produces no file changes, only confirms Tasks 1-4 work together against the real site)

---

### Task 6: Step-by-step usage guide

**Files:**
- Create: `docs/docker-submission-quickstart.md`

**Interfaces:** none — pure documentation, read by a human, not imported by any code.

- [ ] **Step 1: Write the guide**

```markdown
# Docker submission quickstart

How to package a competitor script as a Docker bundle and fly it through
the website's Agent panel, instead of uploading a single `.py` file.

## 1. Write your files

At minimum: a `Dockerfile` and an agent script. Optionally `detection.py`
+ `weights.pt` if you're running a detection model — see
`examples/competition-submission/` for a full worked example with
detection, or `docker/mock-flight-competitor/` for the simplest possible
one (no detection).

Your `Dockerfile`:

```dockerfile
FROM competition-base:latest
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY agent.py .
ENTRYPOINT ["python", "agent_runner.py", "--file", "agent.py"]
```

**`ENTRYPOINT` must run `agent_runner.py --file <your script>`, not your
script directly** — `agent.py` only *defines* an `Agent` subclass, it
doesn't run itself. `agent_runner.py` (baked into `competition-base`) is
what loads it and drives it.

No `requirements.txt` needed if your script has no extra dependencies
beyond what `competition-base` already provides (the `competition`
package, `websockets`, `numpy`, `Pillow`) — just drop that `COPY`/`RUN`
pair.

## 2. Bundle it into a tar

```bash
tar -cf bundle.tar Dockerfile agent.py requirements.txt   # + detection.py, weights.pt if used
```

Everything your Dockerfile `COPY`s must be in this tar, at its root,
alongside the `Dockerfile` itself.

## 3. Upload it

Open the site (`http://<box-ip>:8090/`). In the Agent panel, use the
**upload bundle** button (next to the plain `.py` upload) and pick your
`bundle.tar`.

The server builds it immediately — watch the build log pane fill in. A
build failure shows the `docker build` error right there; fix your
Dockerfile and upload again.

## 4. Run it

Once built, your image appears in the dropdown next to **upload bundle**.
Select it and press **RUN** — same button the `.py` scripts use. The site
arms, takes off, goes OFFBOARD, then starts your container. **STOP**
works the same way regardless of which kind is running.

## Troubleshooting

| Symptom | Cause |
|---|---|
| "name must be a bare *.tar filename" | Upload a `.tar`, not `.zip` or a bare `Dockerfile` |
| "bundle over 209715200 bytes" | Bundle is over 200 MiB — trim your weights file or the upload cap needs raising (`AGENT_DOCKER_MAX_BYTES` in `joystick-server.py`) |
| Build log ends with "no Dockerfile at the bundle's root" | Your tar's `Dockerfile` isn't at the top level, or it's misspelled/mis-cased |
| Build log ends with "docker build exited 1" (or similar) | Read the lines above it — it's the real `docker build` stderr |
| RUN does nothing / drone doesn't move | Check `ENTRYPOINT` — it must invoke `agent_runner.py --file <script>`, not the script directly (see step 1) |
```

- [ ] **Step 2: Commit**

```bash
git add docs/docker-submission-quickstart.md
git commit -m "docs: step-by-step Docker submission quickstart guide

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011WEp4YtTLi1h6yxhc92J3P"
```
