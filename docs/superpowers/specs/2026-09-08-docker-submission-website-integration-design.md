# Design: Docker submissions in the website — upload, build, run

**Status:** approved (design settled interactively), spec written before implementation plan
**Date:** 2026-09-08

## Relationship to existing docs

Wires the CLI-only Docker submission flow from
`docs/superpowers/specs/2026-09-08-competitor-submission-docker-design.md`
(verified working by hand that day — `docker build` + `docker run
--network host` against a live site) into the website itself, so an
operator uploads a bundle through the browser instead of running `docker
build`/`docker run` on the box by hand. Depends on `competition-base:latest`
already existing on the host (`docker/competition-base/Dockerfile`, built
that same day) and on `flight()` being implemented
(`docs/superpowers/specs/2026-09-07-competition-flight-primitive-design.md`,
commit `7eede71`) — both already true.

Extends `docs/superpowers/specs/2026-08-27-website-agent-upload-design.md`'s
`AgentRun`/`/agent/upload` machinery rather than replacing it — see
Decision 3.

---

## Decision 1 — Upload unit: a `.tar` bundle

`POST /agent/upload-docker?name=<bundle>.tar` — raw tar bytes in the
request body (no multipart, matching `/agent/upload`'s existing
non-multipart pattern). Reject a `name` not ending `.tar`, containing a
path separator, or a body over `AGENT_DOCKER_MAX_BYTES` (200 MiB —
weights files are much bigger than a `.py`; this number is a guess, see
Open Items). Saved to `logs/docker-agents/<stem>-<timestamp>.tar` (mirrors
`AGENT_UPLOAD_DIR`'s `<stem>-<timestamp>.py` convention).

The tar is extracted into a fresh temp directory
(`tempfile.mkdtemp(prefix="docker-build-")`); if there's no `Dockerfile`
at its root, the request fails with a clear 400 before any `docker build`
is attempted.

## Decision 2 — Build happens synchronously with upload

The same request that uploads the bundle also builds it:
`docker build -t submission-<stem>-<timestamp> -f <ctx>/Dockerfile <ctx>`,
run as an async subprocess (`asyncio.create_subprocess_exec`, not blocking
the event loop) with stdout/stderr streamed into a capped log (matching
`AgentRun._log`'s `deque(maxlen=40)`). The temp build context is removed
after the build finishes either way — the image itself is what persists
(in Docker's own store), not the extracted files.

A new `DockerBuild` class holds this state — `state: idle|building|built|error`,
`image_tag`, `log` — one instance on `loop_thread`, same ownership shape
as `AgentRun`. `GET /agent/docker-list` shells out to
`docker image ls --filter reference='submission-*' --format ...`, newest
first — stateless and always accurate, rather than an in-memory list a
server restart could desync from what Docker's store actually has.

**Precondition, not handled here:** `competition-base:latest` must
already exist on the host. Not built automatically per-request (too slow
to be worth it inside a user-facing upload call) — an operational fact
like the `drone` conda env already being present.

## Decision 3 — Execution: `AgentRun` gains a `kind`, not a parallel class

`AgentRun.run(kind, identifier)`: `kind="script"` is today's path,
byte-for-byte unchanged (`identifier` = filename,
`[python_exe, "agent_runner.py", "--file", path]`). `kind="docker"` spawns

```python
["docker", "run", "--rm", "--network", "host", "--gpus", "all",
 "--name", f"submission-run-{timestamp}", image_tag]
```

Everything else — the ARM→TAKEOFF→OFFBOARD auto-sequencing state machine,
`/agent/control`, the `#a-log` pane, the `stopped`/`error` states — is
shared and untouched, because `docker run` in the foreground (no `-d`) is
an ordinary child process with the same terminate/exit lifecycle
`subprocess.Popen` already has.

One deliberate strengthening over reusing that lifecycle as-is: `stop()`
must not depend solely on the assumption that killing the `docker run`
CLI process also stops the *container* — that's very likely true (the CLI
forwards signals to an attached container) but not something worth
betting a flying drone on unverified. So `stop()` for `kind="docker"`
explicitly runs `docker stop submission-run-<timestamp>` (the `--name`
above makes this targetable) in addition to the existing
terminate/wait/kill fallback.

**Rejected alternative:** a parallel `DockerAgentRun` class duplicating
the whole state machine. The two kinds differ in exactly one thing (the
`Popen` argv) and share every other transition, log, and error path —
duplicating the class would mean two places to fix every future bug in
that shared machinery.

## Decision 4 — UI: one panel, two pickers, one RUN

The existing Agent panel keeps `#a-file` (`.py`) unchanged and gains
`#a-docker-upload` (`accept=".tar"`) + `#a-docker-image` (a dropdown
populated from `/agent/docker-list`) + `#a-build-log` (separate from
`#a-log`, so a build failure's stderr doesn't get mixed into runtime log
lines). **RUN is shared** — it runs whichever of `#a-file`/`#a-docker-image`
was most recently selected; `STOP` is shared already (kills whatever
`AgentRun` is currently doing, regardless of kind).

---

## Error handling

| Failure | Behaviour |
|---|---|
| Bundle not `.tar` / too big | `POST /agent/upload-docker` → 400, panel shows the message |
| No `Dockerfile` at the bundle's root | 400 before any `docker build` runs |
| `docker build` fails | `DockerBuild.state = error`, `#a-build-log` shows the tail |
| `docker` daemon unreachable | `DockerBuild.state = error`, a clear message (not a raw traceback) |
| RUN pressed with no built image selected | Refused, same shape as today's "no file selected" |
| Container exits non-zero | Same as today's script path: `agent.state = error`, drone holds, operator takes over |
| STOP mid-flight | `docker stop <name>` + existing terminate/kill fallback; `agent_control.clear()`, `mission_clear` unchanged |

## Testing

- Unit tests (`streaming/tests/test_docker_upload.py`, new): bundle
  accepted/rejected (bad extension, oversize, missing `Dockerfile`);
  `/agent/docker-list` reflects `docker image ls` output (subprocess
  calls stubbed); `AgentRun.run(kind="docker", ...)` builds the expected
  argv (subprocess spawning stubbed — no real Docker needed for most
  cases).
- One real end-to-end test, skipped if `docker` isn't on the test box:
  build and run a trivial fast-building image (e.g. `FROM busybox`,
  `ENTRYPOINT ["true"]`) through the actual endpoints, confirm the state
  transitions.
- Manual, against the live site (this is the deliverable the user asked
  to see): upload one of the mock bundles from
  `docker/mock-flight-competitor/` or `docker/mock-detection-competitor/`
  through the browser, confirm build log, RUN, confirm the drone flies,
  STOP, confirm clean shutdown.

## Security note (carried forward, not re-decided)

Same stance the CLI-only Docker design already made: sandboxing untrusted
uploads is out of scope, deferred. Worth restating explicitly here because
this *is* a bigger surface than before — a `docker build` runs arbitrary
`RUN` commands on the host at build time, and `--network host --gpus all`
gives the resulting container full host-network and full GPU access at
run time, now reachable from a browser upload rather than only from
someone with a terminal on the box. Not re-decided now; flagged so it's
on record.

## Open items

1. `AGENT_DOCKER_MAX_BYTES = 200 MiB` is a guess — no real weights file
   has been measured against it yet.
2. Whether killing the `docker run` CLI process reliably stops the
   container (assumed yes, mitigated with an explicit `docker stop`
   regardless — see Decision 3) is worth confirming directly during
   implementation, not just assumed.
3. No image cleanup/retention policy — `logs/docker-agents/*.tar` and
   built `submission-*` images accumulate indefinitely, same as
   `logs/agents/*.py` does today, but images are much larger. Deferred;
   flagged as a real disk-usage risk over time.
