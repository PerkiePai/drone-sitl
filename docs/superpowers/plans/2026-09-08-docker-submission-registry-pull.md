# Docker Submission Registry Pull Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let an operator submit a public registry image reference (`ghcr.io/user/image:tag`) instead of a `.tar` bundle, and have the server `docker pull` + re-tag it into the exact same `submission-*` namespace the tar-build path already produces — so the docker-image dropdown, RUN, and STOP need no changes at all.

**Architecture:** `DockerBuild` (already shipped) gains a `pull(image_ref, stem)` method alongside its existing `build()`, reusing the same `state`/`_log`/`image_tag` fields. One new endpoint (`POST /agent/pull-docker?ref=...`) and one new UI row (text input + pull button) are the only new surface — `AgentRun`, `_spawn()`, `/agent/docker-list`, and the RUN/STOP buttons are untouched.

**Tech Stack:** Same as the parent feature — Python 3.11 (`drone` conda env), FastAPI, `asyncio.create_subprocess_exec`, Docker CLI, vanilla JS.

**Spec:** `docs/superpowers/specs/2026-09-08-docker-submission-registry-pull-design.md`

## Global Constraints

- Public registries only — no `docker login`, no stored credentials, anywhere.
- `docker pull` then `docker tag <ref> submission-<stem>-<ts>` — the pulled image must land in the exact same naming scheme `docker_build.IMAGE_PREFIX` already produces, so `docker image ls --filter reference=submission-*` (Task 2 of the parent plan) picks it up unchanged.
- Endpoint takes a query param, not a request body (there's no file).
- `_safe_image_ref` is a light guard (empty/whitespace/length), not a full Docker reference-spec validator — Docker itself is the source of truth for whether a ref is real.

---

### Task 1: `DockerBuild.pull()`

**Files:**
- Modify: `streaming/docker_build.py`
- Test: `streaming/tests/test_docker_build.py`

**Interfaces:**
- Produces: `DockerBuild.pull(self, image_ref: str, stem: str) -> tuple[bool, str | None]` (async), same return shape as `build()`.

- [ ] **Step 1: Write the failing tests**

Append to `streaming/tests/test_docker_build.py` (same `asyncio.run(run())`-wrapped-sync-test shape every other test in that file already uses, and the same `FakeProc` class already defined there):

```python
def test_pull_succeeds_tags_and_sets_state_built(monkeypatch):
    calls = []

    async def fake_exec(*args, **kwargs):
        calls.append(args)
        if args[1] == "pull":
            return FakeProc(["latest: Pulling from user/image", "Status: Downloaded"], 0)
        return FakeProc([], 0)   # docker tag

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        db = docker_build.DockerBuild()
        ok, image_tag = await db.pull("ghcr.io/user/image:latest", "myagent")
        assert ok is True
        assert image_tag.startswith("submission-myagent-")
        assert db.state == "built"
        assert db.image_tag == image_tag
        assert calls[0] == ("docker", "pull", "ghcr.io/user/image:latest")
        assert calls[1] == ("docker", "tag", "ghcr.io/user/image:latest", image_tag)

    asyncio.run(run())


def test_pull_failure_sets_state_error_and_does_not_tag(monkeypatch):
    calls = []

    async def fake_exec(*args, **kwargs):
        calls.append(args)
        return FakeProc(["Error response from daemon: pull access denied"], 1)

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        db = docker_build.DockerBuild()
        ok, image_tag = await db.pull("ghcr.io/user/private:latest", "myagent")
        assert ok is False
        assert image_tag is None
        assert db.state == "error"
        assert len(calls) == 1, "must not attempt docker tag after a failed pull"
        assert any("exited 1" in line for line in db.snapshot()["log"])

    asyncio.run(run())


def test_pull_tag_failure_sets_state_error(monkeypatch):
    async def fake_exec(*args, **kwargs):
        if args[1] == "pull":
            return FakeProc(["Status: Downloaded"], 0)
        return FakeProc(["Error: No such image"], 1)   # docker tag fails

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        db = docker_build.DockerBuild()
        ok, image_tag = await db.pull("ghcr.io/user/image:latest", "myagent")
        assert ok is False
        assert image_tag is None
        assert db.state == "error"

    asyncio.run(run())
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `conda run -n drone python -m pytest streaming/tests/test_docker_build.py -k pull -v`
Expected: FAIL — `AttributeError: 'DockerBuild' object has no attribute 'pull'`

- [ ] **Step 3: Implement**

In `streaming/docker_build.py`, add this method to `DockerBuild`, right after `build()`:

```python
    async def pull(self, image_ref, stem):
        """docker pull image_ref, then tag it submission-<stem>-<ts> so
        it's indistinguishable from a locally-built image everywhere
        downstream (list_images(), AgentRun, the docker-image dropdown).
        Returns (ok, image_tag | None)."""
        self.state = "building"
        self._log.clear()
        ts = time.strftime("%Y%m%d-%H%M%S")
        image_tag = f"{IMAGE_PREFIX}{stem}-{ts}"
        try:
            self._note(f"pulling {image_ref} ...")
            proc = await asyncio.create_subprocess_exec(
                "docker", "pull", image_ref,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT)
            async for line in proc.stdout:
                self._note(line.decode(errors="replace").rstrip())
            code = await proc.wait()
            if code != 0:
                self.state = "error"
                self._note(f"docker pull exited {code}")
                return False, None

            self._note(f"tagging as {image_tag}")
            proc = await asyncio.create_subprocess_exec(
                "docker", "tag", image_ref, image_tag,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT)
            async for line in proc.stdout:
                self._note(line.decode(errors="replace").rstrip())
            code = await proc.wait()
            if code != 0:
                self.state = "error"
                self._note(f"docker tag exited {code}")
                return False, None

            self.state = "built"
            self.image_tag = image_tag
            self._note(f"pulled and tagged {image_tag}")
            return True, image_tag
        except FileNotFoundError:
            self.state = "error"
            self._note("docker not found -- is it installed on this host?")
            return False, None
        except Exception as exc:                      # noqa: BLE001
            self.state = "error"
            self._note(f"pull failed: {exc!r}")
            return False, None
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest streaming/tests/test_docker_build.py -v`
Expected: all PASS (existing `build()`/`list_images()` tests plus the three new `pull()` tests)

- [ ] **Step 5: Commit**

```bash
git add streaming/docker_build.py streaming/tests/test_docker_build.py
git commit -m "feat(docker): DockerBuild.pull() -- docker pull + re-tag as submission-*

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011WEp4YtTLi1h6yxhc92J3P"
```

---

### Task 2: `POST /agent/pull-docker` endpoint

**Files:**
- Modify: `joystick-server.py` (near `_safe_docker_bundle_name` at line ~79, and the endpoints block near line ~604-632)
- Test: `streaming/tests/test_web_ui.py`

**Interfaces:**
- Consumes: `DockerBuild.pull()` (Task 1).
- Produces: `_safe_image_ref(ref) -> str | None`.

- [ ] **Step 1: Write the failing tests**

Add near the existing `# --- docker upload / list ---` tests in `streaming/tests/test_web_ui.py` (same file, same `server` fixture / `urllib.request` pattern already used there):

```python
def test_pull_docker_rejects_empty_ref(server):
    import urllib.request
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/pull-docker?ref=",
        data=b"", method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_pull_docker_rejects_ref_with_whitespace(server):
    import urllib.request
    req = urllib.request.Request(
        f"http://127.0.0.1:{WEB_PORT}/agent/pull-docker?ref="
        + urllib.parse.quote("ghcr.io/user/image with spaces:latest"),
        data=b"", method="POST")
    try:
        urllib.request.urlopen(req)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400
```

The second test needs `import urllib.parse` -- add it as a local `import urllib.parse` inside that test function, matching the file's existing per-test local-import style. These two tests deliberately never reach a real `docker pull` (both fail validation first), so no live Docker/network dependency here — a third, real-pull test is covered manually in Task 3's live verification instead, matching how the parent plan's Task 5 covered `build()` live rather than as an automated pytest case.

- [ ] **Step 2: Run tests to verify they fail**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -k pull_docker -v`
Expected: FAIL with 404 (route doesn't exist yet)

- [ ] **Step 3: Add `_safe_image_ref` and the endpoint**

In `joystick-server.py`, right after `_safe_docker_bundle_name` (ends around line 86):

```python
def _safe_image_ref(ref):
    """A non-empty, single-line, reasonably-sized image reference, or
    None. Not a full Docker reference-spec validator -- Docker itself
    rejects a genuinely malformed ref; this just guards the obviously-bad
    cases (empty, pasted whitespace) before spawning a subprocess."""
    if not ref or len(ref) > 512:
        return None
    if ref != ref.strip() or any(c.isspace() for c in ref):
        return None
    return ref
```

Right after the existing `/agent/docker-list` endpoint (ends around line 632):

```python
    @app.post("/agent/pull-docker")
    async def agent_pull_docker(request: Request):
        ref = request.query_params.get("ref", "")
        safe = _safe_image_ref(ref)
        if safe is None:
            return JSONResponse({"detail": "ref must be a non-empty, single-line image reference"},
                                status_code=400)
        # Same stem-from-name shape /agent/upload-docker uses, just derived
        # from the last path segment of the ref instead of a filename.
        stem = safe.rsplit("/", 1)[-1].split(":")[0].split("@")[0] or "pulled"
        ok, image_tag = await agent_docker_build.pull(safe, stem)
        if not ok:
            return JSONResponse(
                {"detail": "pull failed", "log": agent_docker_build.snapshot()["log"]},
                status_code=400)
        return JSONResponse({"image_tag": image_tag})
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -v`
Expected: all PASS (full file, to catch any regression)

- [ ] **Step 5: Commit**

```bash
git add joystick-server.py streaming/tests/test_web_ui.py
git commit -m "feat(docker): POST /agent/pull-docker -- submit a public registry ref

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011WEp4YtTLi1h6yxhc92J3P"
```

---

### Task 3: UI row + live verification

**Files:**
- Modify: `web/index.html` (the `#agent` block)
- Modify: `web/js/agent.js`

**Interfaces:**
- Consumes: `POST /agent/pull-docker` (Task 2); refreshes the same `#a-docker-image` dropdown `refreshDockerList()` already manages.

- [ ] **Step 1: Add the UI row**

In `web/index.html`, add a new `.a-row` right after the existing docker-upload row (which currently reads `<select id="a-docker-image" ...></select><label class="a-file-btn">upload bundle...</label>`):

```html
    <div class="a-row">
      <input id="a-docker-ref" type="text" placeholder="ghcr.io/user/image:tag">
      <button id="a-docker-pull">pull</button>
    </div>
```

- [ ] **Step 2: Wire it up in `web/js/agent.js`**

Add this function (near `uploadDocker`):

```javascript
async function pullDocker(ref) {
  el('a-build-log').textContent = 'pulling...';
  const r = await fetch(`/agent/pull-docker?ref=${encodeURIComponent(ref)}`,
    { method: 'POST' });
  if (!r.ok) {
    const body = await r.json().catch(() => ({}));
    el('a-state').textContent = `pull failed: ${body.detail || r.statusText}`;
    if (body.log) el('a-build-log').textContent = body.log.join('\n');
    return;
  }
  const { image_tag } = await r.json();
  activeKind = 'docker';
  await refreshDockerList(image_tag);
}
```

In `initAgent()`, add next to the existing `el('a-docker-upload').addEventListener(...)`:

```javascript
  el('a-docker-pull').addEventListener('click', () => {
    const ref = el('a-docker-ref').value.trim();
    if (ref) pullDocker(ref);
  });
```

- [ ] **Step 3: Regression check**

Run: `conda run -n drone python -m pytest streaming/tests/test_web_ui.py -v`
Expected: all PASS, including `test_agent_js_and_panel_are_served` (unaffected — it only checks for `initAgent` and `id="agent"`, both still present).

- [ ] **Step 4: Commit**

```bash
git add web/index.html web/js/agent.js
git commit -m "feat(docker): UI row to pull a submission from a public registry

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011WEp4YtTLi1h6yxhc92J3P"
```

- [ ] **Step 5: Live verification against the running site**

Restart `joystick-server.py` (kill the running PID, relaunch — same as the parent plan's Task 5, no need to touch Isaac/PX4). Confirm the aircraft is armed + OFFBOARD (or arm/takeoff/offboard it via the pad first). Then, exactly as Task 5 of the parent plan did with `curl`, hit the new endpoint directly with a small, fast-pulling public image that will exit immediately and harmlessly if actually run (this step only proves *pull*, not flight — `hello-world` is not a competition agent and will exit non-zero when `agent_runner.py`-shaped RUN is attempted against it, which is fine and expected):

```bash
curl -s -X POST "http://127.0.0.1:8090/agent/pull-docker?ref=hello-world:latest"
```

Expected: `{"image_tag": "submission-hello-world-<timestamp>"}`. Confirm it with:

```bash
curl -s http://127.0.0.1:8090/agent/docker-list
```

Expected: the new tag present alongside any earlier `submission-*` images. This confirms the pull+tag+list pipeline works through the real endpoint; RUNning `hello-world` itself is out of scope (it's not a competition agent, it has no `agent_runner.py` inside it) — Task 5 of the parent plan already proved RUN works for any `submission-*` tag regardless of how it got there, so re-proving that here would be redundant. No commit for this step (verification only, no file changes).
