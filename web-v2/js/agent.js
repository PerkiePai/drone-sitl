// The Agent panel: upload a control script, Run it, watch its log.
// Flight goes through the same /ws socket every other control uses.
import { send } from './ws.js';

const el = id => document.getElementById(id);
let lastLogLen = 0;
let wasRunning = false;
let srcShown = null;              // filename currently rendered in #a-src

async function refreshList(selected) {
  try {
    const { files } = await (await fetch('/agent/list')).json();
    const sel = el('a-file');
    sel.innerHTML = '';
    for (const f of files) {
      const o = document.createElement('option');
      o.value = o.textContent = f;
      sel.appendChild(o);
    }
    if (selected) sel.value = selected;
    loadSource();
  } catch (e) { /* server not up yet */ }
}

// Show the source of whichever script is selected -- the same text that is
// running the aircraft, next to its live log.
async function loadSource() {
  const file = el('a-file').value;
  if (!file) { el('a-src').textContent = 'no scripts uploaded'; srcShown = null; return; }
  if (file === srcShown) return;
  try {
    const r = await fetch(`/agent/source?file=${encodeURIComponent(file)}`);
    el('a-src').textContent = r.ok ? await r.text()
      : `(could not load ${file})`;
    srcShown = file;
  } catch (e) { el('a-src').textContent = `(could not load ${file})`; }
}

async function upload(file) {
  const text = await file.text();
  const r = await fetch(
    `/agent/upload?name=${encodeURIComponent(file.name)}`,
    { method: 'POST', headers: { 'Content-Type': 'text/x-python' }, body: text });
  if (!r.ok) {
    const { detail } = await r.json().catch(() => ({ detail: r.statusText }));
    el('a-state').textContent = `upload failed: ${detail}`;
    return;
  }
  const { stored } = await r.json();
  await refreshList(stored);
}

export function initAgent() {
  refreshList();
  el('a-upload').addEventListener('change', e => {
    if (e.target.files[0]) upload(e.target.files[0]);
    e.target.value = '';
  });
  el('a-file').addEventListener('change', loadSource);
  el('a-run').addEventListener('click', () => {
    const file = el('a-file').value;
    if (file) {
      el('a-log').textContent = '';
      lastLogLen = 0;
      send({ type: 'agent', action: 'run', file });
    }
  });
  el('a-stop').addEventListener('click',
    () => send({ type: 'agent', action: 'stop' }));
}

export function paintAgent(t) {
  const a = t.agent;
  if (!a) return;
  el('a-state').textContent = a.state + (a.camera ? ` · ${a.camera}` : '');
  const running = a.state === 'arming' || a.state === 'running';
  el('a-run').disabled = !t.connected || running;
  el('a-stop').disabled = !running;

  // While an agent is actually flying, pin the source view to it (this also
  // fixes it up after a reload mid-run). Once it stops, the server still
  // reports the last a.file forever -- do NOT touch the <select> then, or the
  // operator can never pick a different script.
  if (running && a.file && a.file !== srcShown) {
    const sel = el('a-file');
    if ([...sel.options].some(o => o.value === a.file)) sel.value = a.file;
    loadSource();
  }

  if (a.log.length !== lastLogLen) {
    el('a-log').textContent = a.log.join('\n');
    el('a-log').scrollTop = el('a-log').scrollHeight;
    lastLogLen = a.log.length;
  }

  // Swap the main video feed to the agent's selected camera while it flies,
  // and restore the default forward feed when it stops. Keep the ONBOARD
  // CAMERA header's subtitle in step with whichever feed is showing.
  const v = el('video');
  const port = v.dataset.port || 8080;
  const sub = el('cam-sub');
  if (a.camera) {
    const path = a.camera === 'oblique' ? 'detect' : 'down';
    const want = `http://${location.hostname}:${port}/${path}`;
    if (v.src !== want) v.src = want;
    if (sub) sub.textContent = a.camera === 'oblique'
      ? 'oblique · forward-down gimbal'
      : 'nadir · straight down';
  } else {
    if (wasRunning) v.src = `http://${location.hostname}:${port}/detect`;
    if (sub) sub.textContent = 'oblique · forward-down gimbal';
  }
  wasRunning = running;
}
