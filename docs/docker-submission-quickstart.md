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
| Run log shows `docker: Error response from daemon: could not select device driver "" with capabilities: [[gpu]]` | The host's `nvidia-container-toolkit` isn't actually installed (a stale `/etc/docker/daemon.json` runtime entry can list "nvidia" without it). The server detects this automatically and omits `--gpus all`, so this specific error shouldn't reach a submission — if you see it, the organizer's box needs `nvidia-container-toolkit` installed for GPU-using submissions to get GPU access; CPU-only submissions are unaffected. |

## Verified

Built and flown through this exact pipeline (upload → build → RUN → STOP
→ land) on 2026-09-08 using `docker/mock-flight-competitor/`'s
`flight.py` — climbed, cruised forward, held, stopped cleanly, and
landed. See `docs/superpowers/specs/2026-09-08-docker-submission-website-integration-design.md`
for the full design and
`docs/superpowers/plans/2026-09-08-docker-submission-website-integration.md`
for how it was built.
