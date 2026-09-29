# Wishlist

- Change recording resolution/framerate to Full HD: `REC_W, REC_H = 1280, 800` -> `1920, 1080`, `REC_FPS = 30` -> maybe `60`. Location: `drone_setup_px4_cesium.py` recording constants.
  - Applies to all three recorded cameras (`down`, `detect`, `chase`) since they share the same global `REC_W/REC_H/REC_FPS` constants -- no per-camera resolution today.
  - Web stream (`STREAM_W, STREAM_H = 640, 400`, `STREAM_FPS = 20`) is a separate render product/constants and can stay lower-res (e.g. 720p) independently of the Full HD recording.
