// FREEZE SIM button + live thrust-stream readout. Talks to sim/hil_tap.py
// (a separate, optional diagnostic process -- see sim/HIL_TAP.md) only
// through the server's /hil/* routes, which read/write plain files
// (hil_tap.ctl, hil_tap.status.json). This is sim-debug, NOT a flight
// command: plain HTTP, never the MAVLink command queue.
import { setStatus } from './telemetry.js';

export function initHil() {
  const btn = document.getElementById('c-freeze');
  const readout = document.getElementById('hil-stream');
  if (!btn && !readout) return;

  // The server's /hil/status poll is the source of truth for frozen state,
  // not the button click -- that way an auto-thaw (hil_tap.py's own
  // --max-freeze safety) or a second browser tab toggling it still shows
  // up here within one poll, instead of the button silently going stale.
  let frozen = false;

  function paintButton() {
    if (!btn) return;
    btn.textContent = frozen ? 'THAW SIM' : 'FREEZE SIM';
    btn.classList.toggle('frozen', frozen);
  }

  if (btn) {
    btn.addEventListener('click', async () => {
      const next = !frozen;
      btn.disabled = true;
      try {
        const r = await fetch(`/hil/${next ? 'freeze' : 'thaw'}`, { method: 'POST' });
        const body = await r.json();
        if (!r.ok || !body.ok) throw new Error(body.detail || `HTTP ${r.status}`);
        frozen = next;
        paintButton();
        setStatus(frozen
          ? 'sim FROZEN (hil_tap.py must be running -- see sim/HIL_TAP.md)'
          : 'sim thawed', frozen ? 'wait' : 'good');
      } catch (e) {
        setStatus(`freeze/thaw failed: ${e.message}`, 'bad');
      } finally {
        btn.disabled = false;
      }
    });
  }

  if (!readout) return;

  async function poll() {
    try {
      const r = await fetch('/hil/status');
      const s = await r.json();
      if (!s.up) {
        readout.textContent = 'tap: not running (start sim/hil_tap.py)';
        readout.classList.remove('live', 'frozen');
        return;
      }
      frozen = s.frozen;
      paintButton();
      const m = Array.isArray(s.controls)
        ? 'm=[' + s.controls.slice(0, 4).map(c => c.toFixed(2)).join(' ') + ']'
        : 'm=[--]';
      readout.textContent =
        `tap: ${s.frozen ? 'FROZEN' : 'live'}  t=${s.sim_t.toFixed(3)}s  ${m}  ` +
        `acts=${s.acts_per_interval}/${s.interval_s}s  ` +
        `sensor=${s.sensor_per_interval}/${s.interval_s}s  ` +
        `gps=${s.gps_per_interval}/${s.interval_s}s`;
      readout.classList.toggle('frozen', s.frozen);
      readout.classList.toggle('live', !s.frozen);
    } catch {
      readout.textContent = 'tap: status unreachable';
      readout.classList.remove('live', 'frozen');
    }
  }
  poll();
  setInterval(poll, 1000);
}
