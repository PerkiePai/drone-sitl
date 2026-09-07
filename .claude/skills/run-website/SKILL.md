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

### `--descriptive`

```bash
./sim/run-website.sh --descriptive
```

Runs `joystick-server-descriptive.py` on **:8091** instead of the default
server: labelled camera/map panels, a map legend, and a second marker for
simulator ground truth (green = PX4's estimate, cyan = truth) fed from
`/tmp/drone_truth.json`, which `drone_setup_px4_cesium.py` writes. Isaac is
shared, so both servers can run at once (`./sim/run-website.sh` then
`./sim/run-website.sh --descriptive`). Any other args are still passed through
to `launch-sitl.sh`. `stop-website` stops whichever server is up.
See memory `descriptive-web-ui-fork.md`.

Browsing from another machine needs port **8091** opened in `ufw` (the box
only allows 8080/8090 by default; a blocked port shows as a browser timeout):
`sudo ufw allow from 192.168.20.0/24 to any port 8091 proto tcp`.

## After it returns

1. `curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/` -> expect `200` (camera server;
   hit `/`, not a feed -- `/detect` is an endless MJPEG stream and the curl will hang).
2. `grep -q '>>> params:' logs/joystick-server.console` -> PX4 answered. If it
   never prints, the MAVLink link is down -- RUN-WEBSITE.md section 9.1.
3. Give the user the URL: `http://<box-ip>:8090/` (`hostname -I | awk '{print $1}'`).
   With `--descriptive` it is `:8091`, and the log is
   `logs/joystick-server-descriptive.console`.

## If the sim will not stay up

`logs/launch-sitl.console` full of `poll timeout` and kit exits = PX4 lockstep
is starving. Check for a non-Isaac CPU hog:

```bash
ps -eo pcpu,cmd --sort=-pcpu | head
```

See memory `px4_sitl_cpu_contention.md`. The web server still runs; `link`
stays red until the sim holds.

Full operator handbook, tuning, and troubleshooting: `RUN-WEBSITE.md`.
