// The telemetry bar, the status line, and command confirmation: whether PX4
// actually ACTED on a press rather than only that the click was registered.
import { send, isConnected } from './ws.js';

const el = id => document.getElementById(id);

export function setStatus(text, cls) {
  el('status').textContent = text;
  el('status').className = cls || '';
}

function flash(btn) {
  btn.classList.add('sent');
  setTimeout(() => btn.classList.remove('sent'), 250);
}

export function sendCmd(name) {
  if (!isConnected()) {
    setStatus(`${name.toUpperCase()} NOT sent — no connection to server`, 'bad');
    return;
  }
  send({type: 'cmd', name: name});
  flash(el(`c-${name}`));
  pending = {name: name, sentAt: Date.now()};
  setStatus(`${name.toUpperCase()} sent, waiting for PX4…`, 'wait');
}

// A queued confirmation can never arrive once the socket is gone.
export function clearPending() { pending = null; }

// What each command should do to the telemetry, so the page can report whether
// PX4 actually ACTED on a press rather than only that the click was registered.
// PX4 silently ignores commands it won't accept (arming refused, OFFBOARD
// rejected), so "sent" and "worked" are genuinely different states.
const EXPECT = {
  arm:      t => t.armed,
  disarm:   t => !t.armed,
  takeoff:  t => t.mode === 'AUTO.TAKEOFF',
  offboard: t => t.mode === 'OFFBOARD',
  land:     t => t.mode === 'AUTO.LAND',
};
// Budget for PX4 to act, expressed in SIM seconds -- because PX4 SITL runs in
// lockstep with Isaac, so sim time is the only clock it experiences.
const CONFIRM_TIMEOUT_MS = 3000;
// Floor on the rate used for scaling. A sim crawling at 5% would otherwise turn
// the deadline into a minute-long wait that never usefully fires.
const MIN_SIM_RATE = 0.15;
let pending = null;          // {name, sentAt} awaiting confirmation

// Wall-clock deadline equivalent to CONFIRM_TIMEOUT_MS of sim time.
//
// Date.now() measures wall clock, but PX4 only advances when Isaac does. At a
// measured 0.3 -- routine under render load -- a fixed 3 s wall deadline
// expires after PX4 has had less than one second of its own to respond, so a
// command that worked perfectly gets reported as "refused". That false alarm is
// indistinguishable from a real refusal, which defeats the point of confirming
// at all.
//
// sim_rate is 0 until the server's first 3 s measurement window closes; treat
// that as "unknown" and fall back to wall clock rather than scaling by zero.
function confirmBudgetMs(t) {
  const rate = t.sim_rate > 0 ? Math.max(t.sim_rate, MIN_SIM_RATE) : 1.0;
  return CONFIRM_TIMEOUT_MS / rate;
}

function checkPending(t) {
  if (!pending) return;
  const expect = EXPECT[pending.name];
  if (expect && expect(t)) {
    setStatus(`${pending.name.toUpperCase()} confirmed by PX4`, 'good');
    pending = null;
  } else {
    const budget = confirmBudgetMs(t);
    if (Date.now() - pending.sentAt > budget) {
      // Report the wall-clock figure actually waited, not the nominal 3 s --
      // otherwise the message contradicts the operator's own stopwatch.
      setStatus(`${pending.name.toUpperCase()} — no response from PX4 after `
                + `${(budget / 1000).toFixed(1)} s (command refused?)`, 'bad');
      pending = null;
    }
  }
}

export function paint(t) {
  el('t-link').textContent = t.connected ? 'up' : 'down';
  el('t-link').className = t.connected ? 'good' : 'bad';
  el('t-mode').textContent = t.mode;
  el('t-mode').className = t.mode === 'OFFBOARD' ? 'good' : 'bad';
  el('t-armed').textContent = t.armed ? 'yes' : 'no';
  el('t-alt').textContent = t.alt_m.toFixed(1);
  el('t-hdg').textContent = t.heading_deg.toFixed(0);

  // Measured ground speed against what we commanded. A large persistent gap
  // means PX4 has the setpoint but is not achieving it -- usually because the
  // sim is running slow, not because control is broken.
  el('t-gs').textContent = t.gs.toFixed(1);
  el('t-cmd').textContent = Math.abs(t.cmd_vx).toFixed(1);

  // Horizontal gap between PX4's fused position and simulator ground truth,
  // in metres -- the estimator error, shown numerically alongside the two
  // map markers. Blank when the sim is not publishing truth.
  if (t.lat_gt !== null && t.lat_gt !== undefined && t.lat !== null) {
    const mlat = 111320;
    const mlon = 111320 * Math.cos(t.lat * Math.PI / 180);
    const dn = (t.lat_gt - t.lat) * mlat;
    const de = (t.lon_gt - t.lon) * mlon;
    el('t-gterr').textContent = Math.hypot(dn, de).toFixed(1) + ' m';
  } else {
    el('t-gterr').textContent = '--';
  }

  // Sim time vs wall clock. Below ~0.8 the drone will look sluggish no matter
  // how correct the commands are, because physics itself is running slow.
  if (t.sim_rate > 0) {
    el('t-sim').textContent = `${Math.round(t.sim_rate * 100)}%`;
    el('t-sim').className = t.sim_rate > 0.8 ? 'good'
                          : t.sim_rate > 0.4 ? 'wait' : 'bad';
  }

  el('c-offboard').disabled = !t.ready_for_offboard;
  ['c-arm','c-takeoff','c-land','c-disarm'].forEach(
    id => el(id).disabled = !t.connected);

  checkPending(t);
}
