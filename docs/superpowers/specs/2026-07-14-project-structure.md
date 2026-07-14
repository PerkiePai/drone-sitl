# Project structure: streaming VIO + commanding (v1 → V2b)

Grounded in the existing flat-root convention: runnable entry-point scripts live
at repo root (matching `pipeline.py`, `vio-recorder-pai.py`); shared importable
code lives in the `streaming/` package. Isaac-Sim Script-Editor scripts stay at
root (they can't cleanly import a package from the Script Editor).

Phase tags: untagged = **v1** (flow-odom → VISION_POSITION_ESTIMATE);
`[Layer 1]` = DSMAC ported to streaming; `[V2a]` = commanding link;
`[V2b]` = climb-and-search on repeated DSMAC failure.

```
drone-sitl/
├── pipeline.py                     # existing batch pipeline (has DSMAC)
├── flow_odometry.py                # existing; +MahonyState, +load_calib
├── vio-recorder-pai.py             # existing (unchanged)
├── vio-streamer-pai.py             # NEW  Isaac Sim ZMQ publisher
├── pipeline-streaming.py           # NEW  streaming estimator + MAVLink loop (entry)
├── drone_setup_px4_cesium-pai.py   # existing
│
├── streaming/                      # NEW  shared, non-Isaac-Sim modules
│   ├── zmq_proto.py                #   wire format (imu/baro/frame)
│   ├── mavlink_bridge.py           #   ENU→NED, VisionPositionSender,
│   │                               #   SetpointSender, origin        [V2a]
│   ├── dsmac_live.py               #   live DSMAC + ortho management  [Layer 1]
│   ├── commander.py                #   climb-and-search FSM           [V2b]
│   ├── ekf2_ev_params.params       #   PX4 EKF2 param set
│   └── tests/
│       ├── test_mahony_state.py
│       ├── test_zmq_proto.py
│       ├── test_mavlink_bridge.py
│       ├── test_load_calib.py
│       ├── test_dsmac_live.py      #                                  [Layer 1]
│       └── test_commander.py       #                                  [V2b]
│
└── docs/superpowers/{plans,specs}/ # existing
```

## Rules

- **Entry-point scripts at root** — `pipeline-streaming.py`, `vio-streamer-pai.py`
  (matches the existing flat convention).
- **Importable code in `streaming/`** — anything shared/unit-testable.
- **Isaac-Sim code stays at root** — `vio-streamer-pai.py` can't import a package
  cleanly from the Script Editor; it uses an explicit `sys.path` insert instead.
- **Reuse, don't fork** — `dsmac_live.py` imports DSMAC functions from `pipeline.py`
  / `flow_odometry.py` unchanged, same as the estimator reuses `_detect`/`_track_lk`/
  `_solve_translation`.

## File count by phase

- **v1**: 9 new files (4 modules, 4 tests, 1 param) + edits to `flow_odometry.py`.
- **Layer 1 / V2a / V2b**: `dsmac_live.py`, `commander.py`, their tests, and the
  `SetpointSender` addition to `mavlink_bridge.py`.
