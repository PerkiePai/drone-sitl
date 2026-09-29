// REC button: start/stop MP4 recording of all sim cameras. Isaac does the
// recording (files land in ~/flight_recordings/ on the sim machine); this
// only talks to the server's /record/* routes. The status poll is the source
// of truth for on/off, so the R key in the Isaac viewport or a second tab
// keeps the button honest. Sim-side tool, NOT a flight command.
import { setStatus } from './telemetry.js';

export function initRecord() {
  const btn = document.getElementById('c-rec');
  if (!btn) return;

  let on = false;
  let startedAt = null;   // sim-machine epoch seconds
  let clockSkew = 0;      // browser-now minus server-now, set from the poll

  function paint() {
    btn.classList.toggle('recording', on);
    if (!on) { btn.textContent = 'REC'; return; }
    const secs = Math.max(0, Math.floor(Date.now() / 1000 - clockSkew - (startedAt || 0)));
    const mm = String(Math.floor(secs / 60)).padStart(2, '0');
    const ss = String(secs % 60).padStart(2, '0');
    btn.textContent = `● REC ${mm}:${ss}`;
  }

  function apply(s) {
    on = !!s.on;
    startedAt = s.started_at || null;
    if (s.now) clockSkew = Date.now() / 1000 - s.now;
    paint();
  }

  btn.addEventListener('click', async () => {
    const action = on ? 'stop' : 'start';
    btn.disabled = true;
    try {
      const r = await fetch(`/record/${action}`, { method: 'POST' });
      const body = await r.json();
      if (!r.ok || !body.ok) throw new Error(body.detail || `HTTP ${r.status}`);
      apply(body);
      setStatus(on ? 'recording started' : 'recording saved: ' +
        (lastPaths.length ? lastPaths.join(', ') : '~/flight_recordings/'),
        on ? 'good' : 'wait');
    } catch (e) {
      setStatus(`record ${action} failed: ${e.message}`, 'bad');
    } finally {
      btn.disabled = false;
    }
  });

  let lastPaths = [];
  async function poll() {
    try {
      const r = await fetch('/record/status');
      const s = await r.json();
      if (r.ok && s.ok) {
        if (s.paths && s.paths.length) lastPaths = s.paths;
        apply(s);
      }
    } catch { /* server down: leave the button as is */ }
  }
  poll();
  setInterval(poll, 1000);
  setInterval(paint, 500);
}
