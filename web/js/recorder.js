// The RECORD button and the recorder status line.
//
// This page has recorded a dataset of zero bytes while showing nothing wrong
// (~/vio_dataset/20260802_151016), so the job here is not "show the state" but
// "show whether data is actually landing". The image count, the run size and
// the stalled detector below are the readout that matters; the word
// "recording" is the part that lied last time.
import { send, isConnected } from './ws.js';

const el = id => document.getElementById(id);

const GIB = 1024 ** 3;

// How long the image count may sit still before recording counts as stalled.
// Scaled by sim rate for the same reason telemetry.js scales its command
// confirmations: PX4 and the recorder both live on Isaac's clock, so at a
// measured 0.3 the recorder genuinely produces images three times slower in
// wall-clock terms and a fixed deadline would cry stall on a healthy run.
const STALL_MS = 3000;
const MIN_SIM_RATE = 0.15;

let images = null;          // last image count seen
let imagesAt = 0;           // when it last CHANGED (wall clock)
let recording = false;      // what the button currently means

function fmtBytes(n) {
  if (n === null || n === undefined) return '--';
  if (n >= GIB) return `${(n / GIB).toFixed(1)} GB`;
  return `${Math.round(n / 1024 ** 2)} MB`;
}

function fmtElapsed(s) {
  const total = Math.floor(s || 0);
  return `${Math.floor(total / 60)}:${String(total % 60).padStart(2, '0')}`;
}

// Track the image counter against the wall clock. A change resets the timer; a
// state that is not `recording` clears it, so a stall cannot be inherited from
// the previous run.
function stalledFor(r) {
  if (r.state !== 'recording') {
    images = null;
    return 0;
  }
  const now = Date.now();
  if (images === null || r.images !== images) {
    images = r.images;
    imagesAt = now;
    return 0;
  }
  return now - imagesAt;
}

// (text, class, buttonEnabled) for the current recorder status. Ordered by
// severity: anything that means "you are not getting a dataset" outranks the
// running counters.
function describe(r, simRate) {
  const free = r.free_bytes;
  const stalledMs = stalledFor(r);
  const budget = STALL_MS / Math.max(simRate > 0 ? simRate : 1.0, MIN_SIM_RATE);

  if (r.cmd_error) return [r.cmd_error, 'bad', r.can_start];
  if (r.state === 'offline')
    return ['recorder offline — Isaac Sim is not running', '', false];
  if (r.state === 'error')
    return [r.error || 'recorder error', 'bad', true];

  if (r.state === 'recording') {
    const body = `${fmtElapsed(r.elapsed_s)} · ${r.images} images · `
               + `${fmtBytes(r.bytes)} · ${fmtBytes(free)} free`;
    if (stalledMs > budget)
      return [`STALLED — no image for ${(stalledMs / 1000).toFixed(0)} s. `
              + `${body}`, 'bad', false];
    if (r.dropped > 0)
      return [`DROPPING FRAMES (${r.dropped}) — the writers cannot keep up. `
              + `${body}`, 'bad', false];
    if (free !== null && free < r.min_free_bytes)
      return [`DISK NEARLY FULL — ${body}`, 'bad', false];
    if (free !== null && free < r.warn_free_bytes)
      return [body, 'wait', false];
    return [body, 'good', false];
  }

  // idle
  if (free !== null && r.min_free_bytes !== null && free < r.min_free_bytes)
    return [`only ${fmtBytes(free)} free — need `
            + `${fmtBytes(r.min_free_bytes)} to record`, 'bad', false];
  if (free !== null && r.warn_free_bytes !== null && free < r.warn_free_bytes)
    return [`ready — ${fmtBytes(free)} free (running low)`, 'wait', r.can_start];
  return [`ready — ${fmtBytes(free)} free`, '', r.can_start];
}

export function paintRecorder(t) {
  const r = t.rec;
  if (!r) return;
  const [text, cls, enabled] = describe(r, t.sim_rate);

  recording = r.state === 'recording';
  const btn = el('c-record');
  btn.textContent = recording ? 'STOP REC' : 'RECORD';
  btn.classList.toggle('recording', recording);
  // While recording, STOP must always be available -- a stalled or
  // disk-starved run is precisely when the operator needs to end it.
  btn.disabled = recording ? false : !enabled;

  const dot = el('rec-dot');
  dot.classList.toggle('live', recording && cls !== 'bad');
  dot.classList.toggle('stalled', recording && cls === 'bad');

  el('rec-stat').textContent = text;
  el('rec-stat').className = cls;
}

el('c-record').addEventListener('click', () => {
  if (!isConnected()) return;
  send({type: 'record', action: recording ? 'stop' : 'start'});
  // Optimistic local feedback only; the truth arrives with the next telemetry
  // frame, which is where every other number on this page comes from too.
  el('rec-stat').textContent = recording ? 'stopping…' : 'starting…';
  el('rec-stat').className = 'wait';
});
