// The d-pad and keyboard: movement/turn/altitude axes and the held-key set.
import { send } from './ws.js';

const held = new Set();

function paintPad() {
  document.querySelectorAll('#controls button').forEach(
    b => b.classList.toggle('on', held.has(b.dataset.dir)));
}

function press(dir, on) {
  if (on === held.has(dir)) return;          // ignore key auto-repeat
  on ? held.add(dir) : held.delete(dir);
  send({type: 'axis', dir: dir, pressed: on});
  paintPad();
}

document.querySelectorAll('#controls button').forEach(b => {
  const d = b.dataset.dir;
  b.addEventListener('pointerdown', e => {
    e.preventDefault(); b.setPointerCapture(e.pointerId); press(d, true); });
  b.addEventListener('pointerup', e => { e.preventDefault(); press(d, false); });
  b.addEventListener('pointercancel', () => press(d, false));
  b.addEventListener('contextmenu', e => e.preventDefault());
});

// Arrows/WASD mirror the pad: up-down = move, left-right = turn.
// Q/E take altitude, matching the separate column on screen.
const KEYS = {ArrowUp:'fwd', ArrowDown:'back',
              ArrowLeft:'yaw_left', ArrowRight:'yaw_right',
              w:'fwd', s:'back', a:'yaw_left', d:'yaw_right',
              q:'up', e:'down',
              W:'fwd', S:'back', A:'yaw_left', D:'yaw_right',
              Q:'up', E:'down'};
addEventListener('keydown', e => {
  if (KEYS[e.key]) { e.preventDefault(); press(KEYS[e.key], true); } });
addEventListener('keyup', e => {
  if (KEYS[e.key]) { e.preventDefault(); press(KEYS[e.key], false); } });

// Releasing on blur stops a stuck key from latching a velocity the operator
// has tabbed away from and can no longer see or cancel.
addEventListener('blur', () => { [...held].forEach(d => press(d, false)); });

// Keepalive: the server zeroes velocity after 0.5 s of silence, and holding a
// button sends exactly one message, so a held direction must be refreshed.
setInterval(() => { if (held.size) send({type: 'ping'}); }, 150);

// A dead socket must not latch the last commanded velocity.
export function resetHeld() {
  held.clear();
  paintPad();
}
