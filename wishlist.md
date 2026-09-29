# Wishlist

- Change recording resolution/framerate to Full HD: `REC_W, REC_H = 1280, 800` -> `1920, 1080`, `REC_FPS = 30` -> maybe `60`. Location: `drone_setup_px4_cesium.py` recording constants.
  - Applies to all three recorded cameras (`down`, `detect`, `chase`) since they share the same global `REC_W/REC_H/REC_FPS` constants -- no per-camera resolution today.
  - Web stream (`STREAM_W, STREAM_H = 640, 400`, `STREAM_FPS = 20`) is a separate render product/constants and can stay lower-res (e.g. 720p) independently of the Full HD recording.

- Run Isaac with RTX - Real-Time 2.0 (`RealTimePathTracing`) + DLSS Quality. Location: `sim/bootstrap.py` (`RENDER_MODE`, `DLSS_MODE`, `_apply_render_settings()`, called from `_bring_up()`).
  - The committed version already does this; the working tree currently has it stripped out (uncommitted, looks like a CPU-saturation diagnostic alongside `SITL_SKIP_TILESET` / `SITL_SKIP_STREAM`). Restore with `git checkout -- sim/bootstrap.py`, then restart Isaac.
  - Heavier than the stock preset -- if PX4 lockstep stalls (`poll timeout` in `logs/launch-sitl.console`), suspect this first. Override with `SITL_RENDER_MODE` / `SITL_DLSS_MODE`.

- Docker submissions from the website (discussed 2026-09-29): confirmed the web UI on :8090 can upload/build/run a Docker submission end to end (arm -> takeoff -> offboard -> agent running).
  - Test image for a quick flight check: `mock-flight-competitor:latest` (detection view: `mock-detection-competitor:latest`; real detector: `perkiepai/drone-rtdetr-submission:latest`, GPU). `competition-base:latest` is only a base image, don't run it.
  - Run containers are named `submission-run-<timestamp>`; follow with `docker logs -f <name>`. The container printed nothing in its first ~minute, so check whether that is expected for the mock bundle.
  - `--gpus all` needs `nvidia-container-toolkit` on the host.
