---
name: run-website
description: Use when the user says "run website" / "run the website", or asks to start, launch, or bring up the drone web page, joystick UI, or web control panel. Brings up Isaac Sim + PX4 SITL and the joystick server, starting only what is not already running.
---

# run website

One command. It is idempotent -- it checks what is already running and starts
only what is missing (Isaac Sim + PX4 via `sim/launch-sitl.sh`, then
`joystick-server.py` on port 8090).

```bash
./sim/run-website.sh
```

First Isaac boot takes 2-5 minutes; the script waits for it. Pass-through env
works: `SITE=bangkok-survey-040 ./sim/run-website.sh`, `SKIP_SIM=1` for web only.

## After it returns

1. `curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/detect` -> expect `200` (camera).
2. `grep -q '>>> params:' logs/joystick-server.console` -> PX4 answered. If it
   never prints, the MAVLink link is down -- RUN-WEBSITE.md section 9.1.
3. Give the user the URL: `http://<box-ip>:8090/` (`hostname -I | awk '{print $1}'`).

## If the sim will not stay up

`logs/launch-sitl.console` full of `poll timeout` and kit exits = PX4 lockstep
is starving. Check for a non-Isaac CPU hog:

```bash
ps -eo pcpu,cmd --sort=-pcpu | head
```

See memory `px4_sitl_cpu_contention.md`. The web server still runs; `link`
stays red until the sim holds.

Full operator handbook, tuning, and troubleshooting: `RUN-WEBSITE.md`.
