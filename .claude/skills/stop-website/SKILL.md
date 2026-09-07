---
name: stop-website
description: Use when the user says "stop website" / "stop the website" / "shut down the website", or asks to close, kill, or stop the joystick server, the drone web page, or the SITL launched for it. Stops joystick-server.py, any agent_runner.py child, and this repo's Isaac Sim / PX4 instance.
---

# stop website

```bash
./sim/stop-website.sh
```

Stops, in order: `agent_runner.py` -> `joystick-server.py` (:8090) **or
`joystick-server-descriptive.py`** (:8091, the `--descriptive` fork) -> the
Isaac Sim + PX4 instance running **this repo's** `sim/bootstrap.py`. SIGTERM
first, SIGKILL after ~10s.

It matches this repo's own process signatures only, so an unrelated Isaac /
SITL on the box (e.g. a `screen -S friend` session) keeps running.

## After it returns

Each line ends in `stopped` or `not running`. A `WARNING still running` line
means a PID ignored SIGKILL (rare -- usually a kernel-blocked syscall); report
the PID.

Quick confirm:

```bash
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/   # expect 000
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8091/   # expect 000 (--descriptive)
pgrep -af "sim/bootstrap.py|joystick-server"                      # expect nothing
```

If `kill` is blocked by the harness when you run the script, ask the user to
run `! ./sim/stop-website.sh` themselves.

Bring it back with the `run-website` skill / `./sim/run-website.sh`.
