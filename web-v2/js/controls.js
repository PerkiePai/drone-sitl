// The two virtual sticks and keyboard fallback: Mode 2 layout --
// left stick x=yaw, y=thrust; right stick x=roll, y=pitch. Screen
// coordinates only (right/down positive); the server owns every flight
// sign convention.
import { send } from './ws.js';

const STICKS = ['left', 'right'];
const drag = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 } };
const keys = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 } };
const dragging = { left: false, right: false };

function current(name) {
  // A live pointer drag on this stick wins over the keyboard fallback.
  return dragging[name] ? drag[name] : keys[name];
}

function paint(name) {
  const el = document.querySelector(`#stick-${name} .knob`);
  const r = document.getElementById(`stick-${name}`).getBoundingClientRect();
  const { x, y } = current(name);
  el.style.transform =
    `translate(calc(-50% + ${x * r.width / 2}px), calc(-50% + ${y * r.height / 2}px))`;
}

function sendStick(name) {
  const { x, y } = current(name);
  send({ type: 'stick', stick: name, x, y });
  paint(name);
}

STICKS.forEach(name => {
  const el = document.getElementById(`stick-${name}`);
  const setFromEvent = e => {
    const r = el.getBoundingClientRect();
    const cx = r.left + r.width / 2, cy = r.top + r.height / 2;
    let x = (e.clientX - cx) / (r.width / 2);
    let y = (e.clientY - cy) / (r.height / 2);
    const mag = Math.hypot(x, y);
    if (mag > 1) { x /= mag; y /= mag; }
    drag[name] = { x, y };
    sendStick(name);
  };
  el.addEventListener('pointerdown', e => {
    e.preventDefault(); el.setPointerCapture(e.pointerId);
    dragging[name] = true; setFromEvent(e);
  });
  el.addEventListener('pointermove', e => { if (dragging[name]) setFromEvent(e); });
  const release = () => {
    dragging[name] = false;
    drag[name] = { x: 0, y: 0 };
    sendStick(name);
  };
  el.addEventListener('pointerup', release);
  el.addEventListener('pointercancel', release);
  el.addEventListener('contextmenu', e => e.preventDefault());
});

// Arrows drive the left stick (yaw/thrust), WASD drives the right stick
// (roll/pitch), each key at full deflection on one axis. A live pointer
// drag on that stick still wins (see current()).
const KEYS = {
  ArrowUp:    ['left', 'y', -1],  ArrowDown:  ['left', 'y', 1],
  ArrowLeft:  ['left', 'x', -1],  ArrowRight: ['left', 'x', 1],
  w: ['right', 'y', -1], W: ['right', 'y', -1],
  s: ['right', 'y', 1],  S: ['right', 'y', 1],
  a: ['right', 'x', -1], A: ['right', 'x', -1],
  d: ['right', 'x', 1],  D: ['right', 'x', 1],
};

function setKey(e, active) {
  const mapped = KEYS[e.key];
  if (!mapped) return;
  e.preventDefault();
  const [name, axis, sign] = mapped;
  const next = active ? sign : 0;
  if (keys[name][axis] === next) return;   // ignore key auto-repeat
  keys[name][axis] = next;
  if (!dragging[name]) sendStick(name);
}
addEventListener('keydown', e => setKey(e, true));
addEventListener('keyup', e => setKey(e, false));

// Releasing on blur stops a stuck key or drag from latching a velocity
// the operator has tabbed away from and can no longer see or cancel.
addEventListener('blur', () => {
  STICKS.forEach(name => {
    dragging[name] = false;
    drag[name] = { x: 0, y: 0 };
    keys[name] = { x: 0, y: 0 };
    sendStick(name);
  });
});

// Keepalive: the server zeroes velocity after 0.5 s of silence, and a
// stick sends a message only when it changes, so an off-center stick
// must be refreshed.
setInterval(() => {
  if (STICKS.some(name => current(name).x || current(name).y)) send({ type: 'ping' });
}, 150);

// A dead socket must not latch the last commanded velocity.
export function resetSticks() {
  STICKS.forEach(name => {
    dragging[name] = false;
    drag[name] = { x: 0, y: 0 };
    keys[name] = { x: 0, y: 0 };
    paint(name);
  });
}
