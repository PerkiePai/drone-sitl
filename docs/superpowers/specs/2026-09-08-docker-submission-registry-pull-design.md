# Design: pull a Docker submission from a public registry

**Status:** approved
**Date:** 2026-09-08

## Relationship to existing docs

Small extension of
`docs/superpowers/specs/2026-09-08-docker-submission-website-integration-design.md`
(implemented, verified live same day). That doc's `DockerBuild`,
`AgentRun`, and the docker-image dropdown are reused unchanged — see
Decision 1 below for why.

## Decision 1 — Pull, then re-tag as `submission-*`; touch nothing downstream

`docker pull <ref>` followed by `docker tag <ref> submission-<stem>-<ts>`
gives a pulled image the exact same naming a locally-built one gets.
Consequence: `/agent/docker-list` (already filters on `reference=submission-*`),
`AgentRun`/`_spawn()`/`stop()`, and the UI's existing docker dropdown +
RUN button need **zero changes** — they already just run whatever tag
shows up in the list, indifferent to whether `docker build` or `docker
pull` produced it.

## Decision 2 — Public registries only, no credentials anywhere

`docker pull <ref>` runs exactly as given, no `docker login` step, no
stored token/secret. A private/unauthenticated ref fails with Docker's
own "pull access denied" — same as it would from a terminal. Explicit
choice: storing registry credentials server-side, reachable from a
website upload, is a new secret-management surface not worth opening for
this.

## Decision 3 — `DockerBuild.pull()`, not a new class

New method on the existing `DockerBuild` (`streaming/docker_build.py`),
reusing its `state`/`_log`/`image_tag` fields and the `"building"→"built"`
state names — to the UI, a pull and a build look identical: something's
happening, then an image is ready.

```python
async def pull(self, image_ref, stem):
    """docker pull image_ref, then tag it submission-<stem>-<ts> so it's
    indistinguishable from a locally-built image everywhere downstream.
    Returns (ok, image_tag | None)."""
```

## Decision 4 — New endpoint, no file body

`POST /agent/pull-docker?ref=<image_ref>` — a query param, not a body
(there's no file to upload). `_safe_image_ref(ref)`: non-empty, no
whitespace/newlines, under a length cap (512) — not a full Docker
reference-spec validator; Docker itself rejects a genuinely malformed
ref. `create_subprocess_exec` (argv list, no shell) means there's no
injection risk from special characters either way, so this validation is
purely about giving a fast, clear 400 rather than a raw Docker CLI error
for the obviously-wrong cases (empty string, pasted whitespace).

## Decision 5 — UI: a third row, same dropdown

A text input (`ghcr.io/user/image:tag`) + a **pull** button, next to the
existing **upload bundle** row. On success, refreshes the same
`#a-docker-image` dropdown Decision 1 keeps populated identically either
way — no new dropdown, no new RUN path.

## Error handling

| Failure | Behaviour |
|---|---|
| Empty/malformed ref | 400 before any `docker pull` runs |
| `docker pull` fails (not found, private, network) | `DockerBuild.state = error`, log shows Docker's own message |
| `docker tag` fails (shouldn't, if pull succeeded) | `DockerBuild.state = error`, logged |

## Testing

Same shape as the existing `test_docker_build.py`/`test_web_ui.py` docker
tests: `asyncio.create_subprocess_exec` monkeypatched for the unit-level
`pull()` tests, real endpoint tests (400 cases) against the live-subprocess
`server` fixture, one manual pass against the running site pulling a real
small public image.

## Open items

None — this reuses enough already-verified machinery that there's little
new to flag.
