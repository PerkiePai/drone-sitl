# Graph Report - drone-sitl  (2026-09-10)

## Corpus Check
- 101 files · ~149,907 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1305 nodes · 2310 edges · 96 communities (65 shown, 30 thin omitted)
- Extraction: 95% EXTRACTED · 5% INFERRED · 0% AMBIGUOUS · INFERRED: 104 edges (avg confidence: 0.89)
- Token cost: 460,733 input · 0 output

## Community Hubs (Navigation)
- Competition Agent Harness
- Docker Submission Pipeline
- Descriptive Web UI (web-v2)
- Main Web UI (web)
- Agent Setpoint Control
- OFFBOARD Unit Tests
- Descriptive Joystick Server
- Isaac Recorder Control
- Dual-Stick OFFBOARD Layer
- MJPEG Camera Frames
- HIL Freeze Tap
- Competition Command Types
- GPS-Denied Vision Pipeline
- Leaflet Vendor (web-v2)
- Leaflet Vendor (web)
- Setpoint Loop Thread
- Isaac Stage Setup Script
- Waypoint Sequencing Tests
- Waypoint Mission State Machine
- USD Stage Builder
- Agent Callback Base Class
- Site Configuration
- Docker Build Tests
- flight() Primitive
- MAVLink Write Wrapper
- Web Layer Tests
- Joystick Server Entry Point
- ZMQ VIO Streaming Design
- Optical-Flow Odometry
- Isaac Bootstrap Sequence
- Scripted Autopilot PoC
- Stage From Config Decisions
- Agent Upload Decisions
- Lawnmower Search Example
- Docker Run Lifecycle Tests
- Leaflet Internals A (web-v2)
- Leaflet Internals A (web)
- SITL Launch Script
- Competitor API Surface
- Leaflet Internals B (web-v2)
- Leaflet Internals B (web)
- Detection Model Split
- Single-Waypoint Flight Example
- Shared Setpoint Stream
- Cartesian Command Vocabulary
- Post-v1 Streaming Roadmap
- Descriptive UI Tests
- Leaflet Internals C (web-v2)
- Leaflet Internals D (web-v2)
- Leaflet Internals C (web)
- Leaflet Internals D (web)
- Lockstep and Heartbeat Incident
- Route vs ATE Question
- Waypoint Geometry Helpers
- Website Launch Skills
- Mock Detector Submission
- Perception-Not-Airmanship Premise
- Inversion of Control Design
- Streaming VIO Bring-Up
- HIL Freeze Brief Deck
- Leaflet Internals E (web-v2)
- Leaflet Internals E (web)
- Idempotent Website Launcher
- Camera Toggle Example
- Box Survey Route Example
- Website Stop Script
- Leaflet Internals F (web-v2)
- Leaflet Internals F (web)
- Agent Control Test Fake
- Test Server Fixture
- ROI Image Delivery Tiers
- Watchdog Stops Yaw
- Stick Keepalive Ping
- MAVLink Constant Parity
- Yaw Sign Convention
- Idempotent Mission Pause
- Paused Mission Falls Through
- No Fix Means No Steering
- Nose Points Along Leg
- Waypoint Arrival Radius
- Metres Not Degrees
- Consume Waypoints At Once
- DONE Holds Last Waypoint
- Held Yaw Does Not Wander
- Skip Waypoint Already At
- Great-Circle Bearing
- Planning Never Flies
- WebSocket Support Warning
- Mission Speed Drives ETA
- Full Mission Socket Round-Trip
- Manual Takeover Edge
- Centering Does Not Pause
- Uvicorn WebSocket Guard
- Async Telemetry Wait Helper
- Live WebSocket Telemetry Test

## God Nodes (most connected - your core abstractions)
1. `Command` - 48 edges
2. `flight` - 39 edges
3. `Harness` - 31 edges
4. `SetpointLoop` - 31 edges
5. `AgentControl` - 31 edges
6. `Agent` - 27 edges
7. `CommandState` - 22 edges
8. `Website Agent Upload Design Spec` - 22 edges
9. `Arena` - 21 edges
10. `AgentRun` - 21 edges

## Surprising Connections (you probably didn't know these)
- `OffboardLink` --semantically_similar_to--> `VisionPositionSender`  [INFERRED] [semantically similar]
  streaming/offboard.py → docs/superpowers/plans/2026-07-13-pipeline-streaming.md
- `flight(left_x, left_y, right_x, right_y)` --semantically_similar_to--> `CommandState`  [EXTRACTED] [semantically similar]
  docs/superpowers/specs/2026-09-07-competition-flight-primitive-design.md → streaming/offboard.py
- `examples/: one Agent per flight primitive` --references--> `Agent`  [EXTRACTED]
  docs/superpowers/specs/2026-08-27-website-agent-upload-design.md → competition/agent.py
- `SetpointLoop` --implements--> `Setpoint Source Priority (mission > agent > manual)`  [EXTRACTED]
  joystick-server.py → docs/superpowers/plans/2026-08-27-website-agent-upload.md
- `SetpointLoop` --implements--> `Setpoint source priority: mission > agent > manual`  [EXTRACTED]
  joystick-server.py → docs/superpowers/specs/2026-08-27-website-agent-upload-design.md

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **flight() call path from agent to PX4 setpoint** — docs_competition_api_flight, run_website_agent_runner, run_website_joystick_server, drone_sitl_brief_agentcontrol_set_flight, drone_sitl_brief_axes_to_body_velocity, run_website_input_watchdog [EXTRACTED 1.00]
- **Docker submission pipeline (upload/pull, build, run)** — docs_docker_submission_quickstart_bundle_upload, docs_docker_submission_quickstart_competition_base, drone_sitl_brief_docker_pull, drone_sitl_brief_agentrun_kind, web_index_docker_agent_rows, run_website_agent_runner [INFERRED 0.85]
- **HIL freeze tap over PX4/Isaac lockstep** — sim_hil_tap_hil_tap_py, sim_hil_tap_hil_stream, sim_hil_tap_max_freeze_guard, sim_hil_tap_hil_endpoints, web_index_freeze_sim_button, run_website_px4_isaac_lockstep [EXTRACTED 1.00]
- **Web joystick to PX4 OFFBOARD control stack** — web_index, joystick_server_build_app, streaming_offboard_commandstate, joystick_server_setpointloop, streaming_offboard_offboardlink, streaming_waypoints_mission [EXTRACTED 1.00]
- **Streaming VIO data path (Isaac -> ZMQ -> flow-odom -> VISION_POSITION_ESTIMATE)** — vio_streamer_pai, streaming_zmq_proto, pipeline_streaming_main, flow_odometry_mahonystate, pipeline_detect, streaming_mavlink_bridge_visionpositionsender [EXTRACTED 1.00]
- **SITL stage bring-up from site config** — sim_launch_sitl, sim_bootstrap_bring_up, sim_sites_site, sim_stage_builder_build_stage, sim_bootstrap_run_setup_script, drone_setup_px4_cesium [EXTRACTED 1.00]
- **Uploaded agent command path: script -> harness -> WebSocket -> AgentControl -> SetpointLoop -> PX4** — agent_runner, competition_harness_harness, docs_superpowers_plans_2026_08_27_website_agent_upload_agent_control_websocket, streaming_agent_control_agentcontrol, joystick_server_setpointloop [EXTRACTED 1.00]
- **Docker submission pipeline: upload/pull, build into submission-*, run via AgentRun** — streaming_docker_build_dockerbuild, streaming_docker_build_dockerbuild_pull, streaming_docker_build_image_prefix, joystick_server_agentrun, docker_competition_base_dockerfile [EXTRACTED 1.00]
- **Proportional dual-stick control across UI, servers and CommandState** — web_js_controls, web_v2_js_controls, joystick_server, joystick_server_descriptive, streaming_offboard_commandstate [EXTRACTED 1.00]
- **One-setpoint-per-tick source dispatch (mission > agent > manual)** — joystick_server_setpointloop, streaming_waypoints_mission, streaming_agent_control_agentcontrol, streaming_offboard_commandstate, streaming_offboard_offboardlink [EXTRACTED 1.00]
- **Docker submission pipeline: upload/pull -> build/tag -> run** — docs_superpowers_specs_2026_09_08_docker_submission_website_integration_design_tar_bundle_upload, docs_superpowers_specs_2026_09_08_docker_submission_registry_pull_design_pull_docker_endpoint, streaming_docker_build_dockerbuild, joystick_server_agentrun, docs_superpowers_specs_2026_09_08_competitor_submission_docker_design_competition_base_image, agent_runner [EXTRACTED 1.00]
- **Keep blocking/unsafe work off the MAVLink setpoint thread** — docs_superpowers_specs_2026_07_30_joystick_offboard_design_single_mavlink_owner_thread, docs_superpowers_specs_2026_07_30_joystick_offboard_design_setpoint_continuity, docs_superpowers_specs_2026_08_07_web_recorder_design_r1_enqueue_main_thread, docs_superpowers_specs_2026_08_07_web_recorder_design_r2_proxy_off_setpoint_queue, docs_superpowers_specs_2026_08_27_website_agent_upload_design_ad2_child_no_mavlink [INFERRED 0.85]

## Communities (96 total, 30 thin omitted)

### Community 0 - "Competition Agent Harness"
Cohesion: 0.06
Nodes (52): load_agent(), main(), Child process that flies an uploaded competition Agent. Spawned by joystick-…, Import `path` and return its single Agent subclass., Adapts the /agent/control WebSocket to the Harness channel contract., _run(), WsChannel, _call_with_timeout() (+44 more)

### Community 1 - "Docker Submission Pipeline"
Cohesion: 0.05
Nodes (41): competition-base Docker image, Docker submission quickstart, ARM -> TAKEOFF -> OFFBOARD Run State Machine, Raw-Body Upload (No Multipart), Docker Submission Registry Pull Plan, Public Registries Only (No docker login), Docker Submissions in the Website Plan, AgentRun kind: script vs docker (+33 more)

### Community 2 - "Descriptive Web UI (web-v2)"
Cohesion: 0.08
Nodes (47): Up-Positive Convention, Single NED Flip, RC Mode 2 Stick Layout, Stick Wire Message {type: stick}, el(), initAgent(), loadSource(), paintAgent(), refreshList() (+39 more)

### Community 3 - "Main Web UI (web)"
Cohesion: 0.08
Nodes (46): el(), initAgent(), paintAgent(), pullDocker(), refreshDockerList(), refreshList(), upload(), uploadDocker() (+38 more)

### Community 4 - "Agent Setpoint Control"
Cohesion: 0.07
Nodes (39): AgentControl, The agent's velocity setpoint holder. Mirrors offboard.CommandState. The…, flight()'s four stick axes -> a body velocity, reusing exactly the conversion…, streaming.agent_control.AgentControl -- the agent's setpoint holder. Mirrors…, right_y is pitch/forward -- full deflection hits speed_fwd exactly, same shape…, test_body_velocity_flips_up_to_ned_down_and_converts_yaw_to_radians(), test_clear_drops_back_to_no_command(), test_flight_decays_to_zero_after_the_watchdog() (+31 more)

### Community 6 - "Descriptive Joystick Server"
Cohesion: 0.07
Nodes (17): AgentRun, build_app(), main(), Current (lat, lon), either may be None. For the setpoint thread., GLOBAL_POSITION_INT -> map position and heading. Heading comes from here rather…, Poll TRUTH_FILE (~5 Hz) for simulator ground truth. Any problem -- missing,…, Web joystick -> PX4 OFFBOARD velocity control -- descriptive UI variant.…, ATTITUDE carries radians; the agent API and any UI want degrees. (+9 more)

### Community 7 - "Isaac Recorder Control"
Cohesion: 0.09
Nodes (28): Plan: VIO GPS-Denied (supersedes streaming plan), Web Recorder Implementation Plan, HTTP Thread Enqueues, Main Thread Execs (R1), Recorder Free-Space Gate, Recorder Takeoff Gate, Single MAVLink-owner setpoint thread + command queue, VIO GPS-denied design (2026-08-07), Web Recorder Design Spec (+20 more)

### Community 8 - "Dual-Stick OFFBOARD Layer"
Cohesion: 0.09
Nodes (24): Setpoint Source Priority (mission > agent > manual), Proportional Dual-Stick Joystick Plan, Manual Takeover Edge (rest -> active), Input watchdog + 150 ms keepalive ping, Mission pause as edge (axis pressed), not level, AD5: first manual input kills agent, no resume, Joystick Analog Sticks Design Spec, set_stick became_active edge (deadzone crossing) (+16 more)

### Community 9 - "MJPEG Camera Frames"
Cohesion: 0.12
Nodes (13): BaseHTTPRequestHandler, MjpegFrames, One MJPEG connection to the Isaac camera server, latest frame decoded.…, Pull complete Content-Length-delimited JPEG parts out of buf, decoding the last…, _Handler, _jpeg(), competition.frames -- decode the latest JPEG from a multipart MJPEG stream.…, _server() (+5 more)

### Community 10 - "HIL Freeze Tap"
Cohesion: 0.11
Nodes (10): Counter, main(), PX4 -> Isaac. This is the direction we withhold when frozen., Isaac -> PX4. Always forwarded; when Isaac halts there is simply nothing here…, Print exact message counts seen in each window -- not a smoothed rate, the…, One JSON snapshot per display tick, for anything polling this tap without…, Poll a plain text file for freeze/thaw commands -- lets anything (a shell,…, Exact message count, plus a per-window count for the display line. No smoothing… (+2 more)

### Community 11 - "Competition Command Types"
Cohesion: 0.13
Nodes (19): _check_numbers(), _check_unit_range(), Command, The flight and camera commands an Agent returns from its callbacks. Mirrors…, Fly a list of (lat, lon) in order at one altitude. Stateful. Progress arrives…, What every callback returns (or None to keep doing what it was doing). Both…, Route, Velocity (+11 more)

### Community 12 - "GPS-Denied Vision Pipeline"
Cohesion: 0.11
Nodes (26): build_ortho(), _deg2tile(), _detect(), _ensure_geo_georef(), green_dominance(), is_canopy_nonviable(), main(), make_sift_lg() (+18 more)

### Community 13 - "Leaflet Vendor (web-v2)"
Cohesion: 0.08
Nodes (6): a(), Ci(), l(), me(), x(), ze()

### Community 14 - "Leaflet Vendor (web)"
Cohesion: 0.08
Nodes (6): a(), Ci(), l(), me(), x(), ze()

### Community 15 - "Setpoint Loop Thread"
Cohesion: 0.12
Nodes (12): advance()-Based Autonomous/Manual Dispatch, Record Commands Bypass SetpointLoop (R2), Sole owner of the MAVLink connection. Sends a velocity setpoint every tick…, Current (lat, lon), either may be None. For the setpoint thread., GLOBAL_POSITION_INT -> map position and heading. Heading comes from here rather…, ATTITUDE carries radians; the agent API and any UI want degrees., Called from the web thread. Mission carries its own lock and touches no…, Record PX4's actual mode, auto-pausing a mission that has lost its only means… (+4 more)

### Community 16 - "Isaac Stage Setup Script"
Cohesion: 0.11
Nodes (21): _env_overrides(), FIX_GROUND_FLIP = False, fix_ground_plane(), Return {tunable: value} parsed from the DRONE_SETUP_* environment., Return (lat, lon, height) from the CesiumGeoreference prim, or None., Attach the DOWN camera (real ZED X One GS, hard-mounted + anti-vibration soft…, Grab RGB from each camera's render product and serve both as MJPEG over HTTP.…, Find the ground plane on the stage and rotate it back to flat. A plane created… (+13 more)

### Community 18 - "Waypoint Mission State Machine"
Cohesion: 0.11
Nodes (16): Joystick Guide, Plan: Web Joystick to PX4 OFFBOARD Velocity Control, Plan: Satellite Map + Autonomous Waypoint Missions, Setpoint continuity invariant (never stop the stream), Spec: Map Waypoints Design, advance() return contract as loop dispatch rule, FLY gating (home_valid, OFFBOARD, position), SET_POSITION_TARGET_GLOBAL_INT (frame 6, mask 2552) (+8 more)

### Community 19 - "USD Stage Builder"
Cohesion: 0.14
Nodes (20): _add_lighting(), _add_model(), apply_model_overrides(), _describe_xform(), geodetic_to_ecef(), Assemble the SITL stage from a Site definition. Pure stage authoring -- no…, Author a sun and an ambient dome. Not optional: a stage created with…, Put the ion token on the server prim, defining it if Cesium has not yet.… (+12 more)

### Community 20 - "Agent Callback Base Class"
Cohesion: 0.11
Nodes (11): Agent, The base class every uploaded script subclasses. The harness owns the loop and…, Once, before anything flies. Return the first Command, or None., ~5 Hz. `image` is a numpy array from the selected camera, or None until the…, ~20 Hz. No image. Cheap steering only., A single-waypoint Route finished., Route waypoint `index` (0-based) was reached., The last Route waypoint was reached. (+3 more)

### Community 21 - "Site Configuration"
Cohesion: 0.11
Nodes (11): Path, Site bangkok-survey-040, get_site(), ModelAnchor, Site definitions for the SITL stage builder. A "site" is everything a hand-…, Look up a site by name, raising with the valid options if it is unknown., A Cesium ion tileset streamed over the site., A reconstructed map model, placed by lat/lon rather than by dragging. Cesium's… (+3 more)

### Community 22 - "Docker Build Tests"
Cohesion: 0.13
Nodes (10): FakeProc, _make_bundle(), streaming.docker_build.DockerBuild -- builds an uploaded bundle. No real Docker…, A malicious bundle with `../../etc/passwd`-style paths must not write outside…, Stands in for asyncio.subprocess.Process., test_build_failure_sets_state_error(), test_build_succeeds_and_sets_state_built(), test_docker_not_installed_sets_state_error() (+2 more)

### Community 23 - "flight() Primitive"
Cohesion: 0.17
Nodes (10): flight, The one flight primitive: two virtual joysticks, body-frame, normalized.…, test_flight_defaults_to_all_zero_hover(), test_flight_is_frozen(), test_flight_rejects_non_numeric(), test_flight_rejects_out_of_range(), FullSortie, _metres() (+2 more)

### Community 24 - "MAVLink Write Wrapper"
Cohesion: 0.15
Nodes (6): FLY Gating that Names the Blocker, OffboardLink, Every MAVLink write for the joystick PoC. Single-threaded by contract:…, Adopt sysid/compid from a received HEARTBEAT., Fly to a lat/lon at an altitude relative to home, nose on yaw_deg. Same…, AUTO.TAKEOFF climbs to MIS_TAKEOFF_ALT, set at startup.

### Community 25 - "Web Layer Tests"
Cohesion: 0.12
Nodes (4): _make_tar_bytes(), Tests for the web layer of joystick-server.py. These exist because of a bug…, test_docker_upload_rejects_non_tar_name(), test_docker_upload_without_dockerfile_in_bundle_fails()

### Community 26 - "Joystick Server Entry Point"
Cohesion: 0.16
Nodes (14): Axis-Press Pause Edge (manual takeover), Spec: Joystick OFFBOARD Design, BODY_NED velocity setpoint (type_mask 1479, yaw_rate), COM_RCL_EXCEPT=4 (offboard RC-loss exception), udpin:0.0.0.0:14540, not udpout, Wind disabled for bring-up (ADD_WIND=False), Vendored Leaflet + Esri World Imagery map panel, build_app() (+6 more)

### Community 27 - "ZMQ VIO Streaming Design"
Cohesion: 0.24
Nodes (16): Spec: Pipeline Streaming Design, EKF2 external-vision param set (EKF2_EV_CTRL, GPS off), MahonyState incremental AHRS, Middleware decision: stay on pymavlink, SET_GPS_GLOBAL_ORIGIN georef anchor, V2a: fly-to-lat/lon commanding, V2b: climb-and-search on repeated DSMAC failure, VISION_POSITION_ESTIMATE to EKF2 (+8 more)

### Community 28 - "Optical-Flow Odometry"
Cohesion: 0.18
Nodes (13): build_dem_raster(), compute_true_agl(), load_calib(), load_dataset(), quat_xyzw_to_R(), Closest 3D point to two world rays (origin C, unit dir d). None if degenerate., Altitude-scaled optical-flow odometry for the nadir drone camera. Why this…, Per-frame true AGL = camera_altitude - terrain_elevation, the honest sim stand-… (+5 more)

### Community 29 - "Isaac Bootstrap Sequence"
Cohesion: 0.23
Nodes (15): _advisory(), _banner(), _bring_up(), _check_georeference(), _frames(), _guarded(), Run a diagnostic without ever letting it stop the bring-up. Every check below…, Manual step 7 -- 'drone position is the same for both QGC and Isaac'.… (+7 more)

### Community 30 - "Scripted Autopilot PoC"
Cohesion: 0.20
Nodes (7): Autopilot, main(), _main_async(), Scripted autonomous flight over joystick-server.py's existing /ws protocol.…, One flight, one WebSocket. Owns the telemetry-driven wait/hold logic that a…, Poll telemetry until predicate(telemetry) is true or time out. See…, Press `direction`, keep it alive for `seconds` of WALL time, release. Wall time…

### Community 31 - "Stage From Config Decisions"
Cohesion: 0.21
Nodes (13): Plan: Build the SITL Stage from Config, Spec: SITL Stage from Config Design, Build stage from config instead of baked .usd, fix_ground_plane / GROUND_FLIP_DEG compensation, Georeference read-back verification before Play, Cesium globe-anchored map model, Baked Y-up stage rotated 90 deg by set_stage_up_axis('z'), Absolute spawn height. Derived so it cannot drift from the ground plane. (+5 more)

### Community 32 - "Agent Upload Decisions"
Cohesion: 0.15
Nodes (14): Website Agent Upload Implementation Plan, Website Agent Upload Design Spec, AD1: agent in child process, not thread, AD3: reuse waypoints.Mission for Route and Goto, AD4: RUN auto-sequences ARM -> TAKEOFF -> OFFBOARD, AD6: +up in API, NED inside, flip in one place, AD9: no new flags on joystick-server.py, /agent/control WebSocket (+6 more)

### Community 33 - "Lawnmower Search Example"
Cohesion: 0.21
Nodes (8): _bearing_body(), FullFlightAgent, _lawnmower(), _offset_latlon(), full_flight_agent.py -- REFERENCE EXAMPLE, not runnable against this repo's…, Toy boustrophedon pattern as (north_m, east_m) offsets from launch. A real…, Same Decision-5 rotation as single_flight.py -- duplicated rather than shared…, Sweep for a target on the nadir camera; on a confident hit, switch to the…

### Community 34 - "Docker Run Lifecycle Tests"
Cohesion: 0.23
Nodes (11): _FakeLoopThread, _fly_to_offboard(), _load_server(), Just enough of SetpointLoop's surface for AgentRun's preflight state machine:…, Drive AgentRun's arm->takeoff->offboard preflight to completion by ticking it…, Caught live: docker info can list an "nvidia" runtime in…, Caught live: docker stop's default 10s grace period can outlast a short…, test_agent_run_docker_omits_gpus_flag_when_toolkit_unavailable() (+3 more)

### Community 35 - "Leaflet Internals A (web-v2)"
Cohesion: 0.19
Nodes (13): bi(), c(), e(), hi(), m(), Pi(), Qe(), Ti() (+5 more)

### Community 36 - "Leaflet Internals A (web)"
Cohesion: 0.19
Nodes (13): bi(), c(), e(), hi(), m(), Pi(), Qe(), Ti() (+5 more)

### Community 37 - "SITL Launch Script"
Cohesion: 0.18
Nodes (11): CESIUM_ION_TOKEN via Environment (No Secrets in Tracked Files), CESIUM_ION_TOKEN env-only secret, Cesium ion token (sim/secrets.env), sim/sites.py site config, Z-up stage authored fresh on every launch, die(), launch-sitl.sh script, SITL_AUTOPLAY (+3 more)

### Community 38 - "Competitor API Surface"
Cohesion: 0.21
Nodes (12): Design: competitor-facing drone control API, Drone Control API competitor reference, bundle.tar upload and build (upload bundle), competition-base:latest image, --gpus all omitted without nvidia-container-toolkit, AgentRun kind (script | docker), POST /agent/pull-docker (public registry pull), Competitor submission examples README (+4 more)

### Community 39 - "Leaflet Internals B (web-v2)"
Cohesion: 0.24
Nodes (12): F(), G(), h(), j(), k(), ke(), ne(), e() (+4 more)

### Community 40 - "Leaflet Internals B (web)"
Cohesion: 0.24
Nodes (12): F(), G(), h(), j(), k(), ke(), ne(), e() (+4 more)

### Community 41 - "Detection Model Split"
Cohesion: 0.24
Nodes (7): Design: Docker submission format + flight()/detection examples, agent.py / detection.py split, one Detector per model, Detection, Detector, detection.py -- everything about the detection model, nothing about flying.…, One model. Competitors using more than one model (e.g. a car detector and a…, image: RGB numpy array, exactly what on_frame receives. Returns a list of…

### Community 42 - "Single-Waypoint Flight Example"
Cohesion: 0.25
Nodes (6): _bearing_body(), _offset_latlon(), single_flight.py -- REFERENCE EXAMPLE, not runnable against this repo's sandbox…, World bearing/distance to (tgt_lat, tgt_lon), rotated into flight()'s body…, hover -> fly to a 47 m offset waypoint at 15 m alt -> spin -> hold., SingleFlightAgent

### Community 43 - "Shared Setpoint Stream"
Cohesion: 0.20
Nodes (10): D7 command vocabulary (4 flight DOF + camera), Geometry helpers (pixel_to_ground, ground_to_pixel, is_visible), AgentControl.set_flight(), offboard.axes_to_body_velocity(), Proportional dual-stick control (replaces WASD held-set), joystick-server.py (web deck on :8090), --mission-speed (sets MPC_XY_VEL_MAX), Shared OFFBOARD setpoint stream (+2 more)

### Community 44 - "Cartesian Command Vocabulary"
Cohesion: 0.24
Nodes (10): D8 hybrid statefulness split on hazard, flight(left_x, left_y, right_x, right_y) primitive, Cartesian axes, not polar (bearing, push), flight() validation spike (15-line guidance loop), Four commands collapsed to flight() (commit 7eede71), Detector / Detection (detection.py), full_flight_agent.py example, single_flight.py example (+2 more)

### Community 45 - "Post-v1 Streaming Roadmap"
Cohesion: 0.31
Nodes (10): Roadmap: After v1 Streaming Lands, Validation Gate: DSMAC Accept-Rate vs Altitude (Exp10), Reuse, don't fork (import pipeline.py functions unchanged), streaming/commander.py (V2b climb-and-search FSM), streaming/dsmac_live.py (Layer 1: DSMAC to streaming), streaming/ekf2_ev_params.params (EKF2 external-vision params), enu_to_ned() / yaw_enu_to_ned(), SetpointSender (planned V2a) (+2 more)

### Community 46 - "Descriptive UI Tests"
Cohesion: 0.22
Nodes (4): fixture, Tests for joystick-server-descriptive.py -- the labelled-UI / ground-truth fork…, server(), _wait_for_port()

### Community 47 - "Leaflet Internals C (web-v2)"
Cohesion: 0.22
Nodes (9): Ae(), be(), De(), ei(), Ie(), ii(), p(), pe() (+1 more)

### Community 48 - "Leaflet Internals D (web-v2)"
Cohesion: 0.25
Nodes (9): at(), d(), ht(), i(), Li(), Mi(), v(), W() (+1 more)

### Community 49 - "Leaflet Internals C (web)"
Cohesion: 0.22
Nodes (9): Ae(), be(), De(), ei(), Ie(), ii(), p(), pe() (+1 more)

### Community 50 - "Leaflet Internals D (web)"
Cohesion: 0.25
Nodes (9): at(), d(), ht(), i(), Li(), Mi(), v(), W() (+1 more)

### Community 51 - "Lockstep and Heartbeat Incident"
Cohesion: 0.25
Nodes (8): D3 free-running real-time sim (not lockstep), D1/D2 Jetson edge box, FC co-located with Isaac, D4 setpoint-level API (no attitude/rate/motors), PX4 / Isaac Sim lockstep, Command confirmation deadline in sim time, Pegasus px4_mavlink_backend HITL defaults patch, PX4 SITL never sent heartbeat (2026-08-07), PX4MavlinkBackendConfig explicit SITL values (tcpin, localhost, lockstep)

### Community 52 - "Route vs ATE Question"
Cohesion: 0.36
Nodes (8): Original competition control API design (2026-08-09), Design: flight() dual-stick primitive (PAUSED), ATE trajectory-following score, flight(left_x, left_y, right_x, right_y), Inspect(lat, lon) standoff command, Unresolved: Route vs ATE score, flight_waypoint_test.py spike validation, Decision 5: competitor-owned world->body rotation

### Community 53 - "Waypoint Geometry Helpers"
Cohesion: 0.29
Nodes (6): bearing_deg(), haversine_m(), Waypoint sequencing for autonomous missions. Pure logic: no MAVLink, no I/O.…, The target for this tick, or None if the loop should fly manually. Returning…, Great-circle distance between two lat/lon points, in metres., Initial great-circle bearing, in compass degrees (0 = N, 90 = E). "Initial"…

### Community 54 - "Website Launch Skills"
Cohesion: 0.43
Nodes (7): run-website skill, --descriptive mode (joystick-server-descriptive.py on :8091), stop-website skill, RUN-WEBSITE operator handbook, sim/run-website.sh (idempotent launcher), Descriptive web UI (web-v2/index.html), Ground-truth map marker and GT err readout

### Community 55 - "Mock Detector Submission"
Cohesion: 0.33
Nodes (4): Detection, Detector, Mock Detector -- no real model, no torch/ultralytics. Proves the…, Fakes a detector: "loads" weights (just proves the file made it into the image)…

### Community 56 - "Perception-Not-Airmanship Premise"
Cohesion: 0.33
Nodes (7): D10 control-style freedom and follow_route asymmetry, Perception, not airmanship (competition premise), Route(waypoints, alt, speed), ATE score (absolute trajectory error), Open question: Route vs ATE, FLY button gates (link, position, home_valid, OFFBOARD, waypoints), Waypoint mission state machine (IDLE/RUNNING/PAUSED/DONE)

### Community 57 - "Inversion of Control Design"
Cohesion: 0.29
Nodes (7): D5 inversion of control (harness owns the loop), Legibility and coverage budget (arena/camera physics), D6 two fixed cameras (nadir search / oblique inspect), Agent (callback class: on_start/on_frame/on_tick/events), Callback timing budgets (on_frame 200 ms, on_tick 50 ms), Command(flight, camera), Inspect(lat, lon)

### Community 58 - "Streaming VIO Bring-Up"
Cohesion: 0.33
Nodes (7): Plan: Streaming VIO Pipeline to PX4/QGC, Initial-State Hardening (start_alt/yaw, GPS origin, Mahony seed), compute_ahrs_attitude(), MahonyState (incremental Mahony AHRS), Per-frame body attitude from the IMU ALONE (deployment-realistic; no GT).…, pipeline-streaming.py (streaming estimator + MAVLink loop), Use udpin, not udpout, for PX4 link (14540)

### Community 59 - "HIL Freeze Brief Deck"
Cohesion: 0.29
Nodes (7): Drone SITL & flight() brief deck, Drone SITL & flight() brief deck (artifact variant), /hil/freeze, /hil/thaw, /hil/status endpoints, HIL_ACTUATOR_CONTROLS / HIL_SENSOR / HIL_GPS stream, sim/hil_tap.py (HIL MAVLink MITM freeze tap), --max-freeze auto-thaw guard (20 s), FREEZE SIM button and hil-stream readout

### Community 60 - "Leaflet Internals E (web-v2)"
Cohesion: 0.29
Nodes (7): Jt(), Le(), O(), Qt(), Re(), $t(), te()

### Community 61 - "Leaflet Internals E (web)"
Cohesion: 0.29
Nodes (7): Jt(), Le(), O(), Qt(), Re(), $t(), te()

### Community 62 - "Idempotent Website Launcher"
Cohesion: 0.60
Nodes (5): isaac_running(), isaac_up(), say(), run-website.sh script, web_up()

### Community 65 - "Website Stop Script"
Cohesion: 1.00
Nodes (3): say(), stop-website.sh script, stop_pat()

### Community 66 - "Leaflet Internals F (web-v2)"
Cohesion: 0.67
Nodes (4): Je(), ni(), oi(), si()

### Community 67 - "Leaflet Internals F (web)"
Cohesion: 0.67
Nodes (4): Je(), ni(), oi(), si()

### Community 69 - "Test Server Fixture"
Cohesion: 0.67
Nodes (3): fixture, server(), _wait_for_port()

## Ambiguous Edges - Review These
- `flight(left_x, left_y, right_x, right_y) primitive` → `Competitor submission examples README`  [AMBIGUOUS]
  examples/competition-submission/README.md · relation: conceptually_related_to
- `Roadmap: After v1 Streaming Lands` → `Spec: Project Structure File Map`  [AMBIGUOUS]
  docs/superpowers/plans/2026-07-14-roadmap.md · relation: cites

## Knowledge Gaps
- **49 isolated node(s):** `SITL_SITE`, `SITL_SETUP_SCRIPT`, `SITL_AUTOPLAY`, `SITL_TILE_SETTLE_FRAMES`, `STICKS` (+44 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 487 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **30 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **What is the exact relationship between `flight(left_x, left_y, right_x, right_y) primitive` and `Competitor submission examples README`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._
- **What is the exact relationship between `Roadmap: After v1 Streaming Lands` and `Spec: Project Structure File Map`?**
  _Edge tagged AMBIGUOUS (relation: cites) - confidence is low._
- **Why does `Website Agent Upload Design Spec` connect `Agent Upload Decisions` to `Competition Agent Harness`, `Docker Submission Pipeline`, `Agent Setpoint Control`, `Competitor API Surface`, `Isaac Recorder Control`, `Dual-Stick OFFBOARD Layer`, `Competition Command Types`, `Waypoint Mission State Machine`, `Agent Callback Base Class`, `Joystick Server Entry Point`?**
  _High betweenness centrality (0.149) - this node is a cross-community bridge._
- **Why does `Joystick web UI (web/index.html)` connect `Joystick Server Entry Point` to `Competitor API Surface`, `Isaac Recorder Control`, `Dual-Stick OFFBOARD Layer`, `Shared Setpoint Stream`, `Isaac Stage Setup Script`, `Site Configuration`, `Website Launch Skills`, `MAVLink Write Wrapper`, `HIL Freeze Brief Deck`?**
  _High betweenness centrality (0.123) - this node is a cross-community bridge._
- **Why does `AgentControl` connect `Agent Setpoint Control` to `Agent Upload Decisions`, `Competition Agent Harness`, `Descriptive Web UI (web-v2)`, `Dual-Stick OFFBOARD Layer`, `Setpoint Loop Thread`?**
  _High betweenness centrality (0.111) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `Harness` (e.g. with `Command` and `flight`) actually correct?**
  _`Harness` has 4 INFERRED edges - model-reasoned connections that need verification._
- **What connects `SITL_SITE`, `SITL_SETUP_SCRIPT`, `SITL_AUTOPLAY` to the rest of the system?**
  _49 weakly-connected nodes found - possible documentation gaps or missing edges._