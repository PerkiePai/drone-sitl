// The Agent panel: upload a control script, Run it, watch its log.
// Flight goes through the same /ws socket every other control uses.
import { send } from './ws.js';

const el = id => document.getElementById(id);
let lastLogLen = 0;
let wasRunning = false;

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
  } catch (e) { /* server not up yet */ }
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

  if (a.log.length !== lastLogLen) {
    el('a-log').textContent = a.log.join('\n');
    el('a-log').scrollTop = el('a-log').scrollHeight;
    lastLogLen = a.log.length;
  }

  // Swap the main video feed to the agent's selected camera while it flies,
  // and restore the default forward feed when it stops.
  const v = el('video');
  const port = v.dataset.port || 8080;
  if (a.camera) {
    const path = a.camera === 'oblique' ? 'detect' : 'down';
    const want = `http://${location.hostname}:${port}/${path}`;
    if (v.src !== want) v.src = want;
  } else if (wasRunning) {
    v.src = `http://${location.hostname}:${port}/detect`;
  }
  wasRunning = running;
}
