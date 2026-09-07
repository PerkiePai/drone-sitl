// Entry point: config fetch, button wiring, and the WebSocket message
// dispatch that fans a telemetry frame out to the other modules.
import { connect } from './ws.js';
import { setStatus, paint as paintTelemetry, sendCmd, clearPending } from './telemetry.js';
import { paintDrone } from './map.js';
import { updateMission, syncAltDefault, setMissionSpeed } from './route.js';
import { resetSticks } from './controls.js';
import { initAgent, paintAgent } from './agent.js';
import { initHil } from './hil.js';

// The video comes from the Isaac MJPEG server on a different port, so derive
// the host from the page rather than hard-coding an IP -- this has to work
// from a phone as well as from the box itself.
fetch('/config').then(r => r.json()).then(c => {
  const video = document.getElementById('video');
  video.dataset.port = c.video_port;   // agent.js swaps the feed by camera
  video.src = `http://${location.hostname}:${c.video_port}/detect`;
  document.getElementById('video-chase').src =
    `http://${location.hostname}:${c.video_port}/chase`;
  setMissionSpeed(c.mission_speed);
});

['arm', 'takeoff', 'offboard', 'land', 'disarm'].forEach(name =>
  document.getElementById(`c-${name}`).addEventListener(
    'click', () => sendCmd(name)));

initAgent();
initHil();

connect({
  onOpen: () => setStatus('connected to server', 'good'),
  onMessage: t => {
    paintTelemetry(t);
    syncAltDefault(t);
    paintDrone(t);
    updateMission(t);
    paintAgent(t);
  },
  onClose: () => {
    resetSticks();
    clearPending();
    setStatus('lost connection to server — reconnecting…', 'bad');
  },
});
