// Waypoint route planning and the FLY/PAUSE/CLEAR mission bar.
// Client-side until FLY: planning must never move the aircraft.
import { map } from './map.js';
import { send } from './ws.js';

const el = id => document.getElementById(id);

let route = [];                  // [[lat, lon], ...]
let wpMarkers = [];
let routeLine = null;
let missionSpeed = 3.0;          // overwritten from /config
let mission = { state: 'IDLE', index: 0, count: 0, dist_m: null };
let altTouched = false;
el('m-alt').addEventListener('input', () => { altTouched = true; });

export function setMissionSpeed(v) { missionSpeed = v; }

// Climb manually to a height that looks right on camera, then plan: the
// route defaults to where you already are, with no typing.
export function syncAltDefault(t) {
  if (!altTouched && t.alt_m > 0.5) el('m-alt').value = t.alt_m.toFixed(0);
}

function drawRoute() {
  wpMarkers.forEach(m => map.removeLayer(m));
  wpMarkers = route.map((p, i) => {
    const active = mission.state === 'RUNNING' && i === mission.index;
    const m = L.marker(p, {icon: L.divIcon({
      className: '', iconSize: [22, 22], iconAnchor: [11, 11],
      html: `<div class="wp-icon${active ? ' active' : ''}">${i + 1}</div>`})});
    m.on('click', ev => {
      L.DomEvent.stop(ev);       // do not also drop a new waypoint here
      route.splice(i, 1);
      drawRoute();
      paintMission();
    });
    return m.addTo(map);
  });
  if (routeLine) map.removeLayer(routeLine);
  routeLine = route.length > 1
    ? L.polyline(route, {color: '#fc6', weight: 2, dashArray: '6 5'}).addTo(map)
    : null;
}

map.on('click', e => {
  route.push([e.latlng.lat, e.latlng.lng]);
  drawRoute();
  paintMission();
});

// Why FLY is disabled, in the order the operator would hit them. Two of these
// fail SILENTLY inside PX4 -- a mission with no home altitude, or one flown
// outside OFFBOARD, is accepted and discarded -- so naming them here is the
// difference between a diagnosis and a wasted afternoon.
function flyBlocker(t) {
  if (!t.connected) return 'no MAVLink link';
  if (t.lat === null) return 'waiting for position fix';
  if (!t.home_valid) return 'PX4 home position not set yet';
  if (t.mode !== 'OFFBOARD') return `not in OFFBOARD (mode is ${t.mode})`;
  if (!route.length && !mission.count) return 'no waypoints — click the map';
  return null;
}

function paintMission(t) {
  const running = mission.state === 'RUNNING';
  const paused = mission.state === 'PAUSED';
  el('m-fly').textContent = paused ? 'RESUME' : 'FLY';
  el('m-pause').disabled = !running;
  el('m-clear').disabled = !route.length && !mission.count;

  if (!t) return;                // called from a map click; no fresh telemetry
  const blocker = flyBlocker(t);
  el('m-fly').disabled = running || blocker !== null;
  el('m-fly').title = blocker || '';

  if (running || mission.state === 'DONE') {
    const eta = mission.dist_m !== null
      ? ` · ${Math.round(mission.dist_m / missionSpeed)} s` : '';
    el('mstat').textContent = mission.state === 'DONE'
      ? `route complete — holding waypoint ${mission.count}`
      : `flying waypoint ${mission.index + 1}/${mission.count} · `
        + `${mission.dist_m === null ? '--' : mission.dist_m.toFixed(0)} m${eta}`;
  } else if (paused) {
    el('mstat').textContent =
      `paused at waypoint ${mission.index + 1}/${mission.count} — RESUME to continue`;
  } else if (route.length) {
    el('mstat').textContent = blocker
      ? `${route.length} waypoint${route.length > 1 ? 's' : ''} — ${blocker}`
      : `${route.length} waypoint${route.length > 1 ? 's' : ''} — ready to fly`;
  } else {
    el('mstat').textContent = 'click the map to add waypoints';
  }
}

el('m-fly').addEventListener('click', () => {
  if (mission.state === 'PAUSED') {
    send({type: 'mission', action: 'fly'});          // resume; route retained
  } else {
    send({type: 'mission', action: 'fly', points: route,
          alt: parseFloat(el('m-alt').value)});
  }
});
el('m-pause').addEventListener('click',
  () => send({type: 'mission', action: 'pause'}));
el('m-clear').addEventListener('click', () => {
  send({type: 'mission', action: 'clear'});
  route = [];
  drawRoute();
  paintMission();
});

// Called with fresh telemetry on every WebSocket message.
export function updateMission(t) {
  const wasIndex = mission.index, wasState = mission.state;
  mission = t.mission;
  // Only redraw when the highlight actually moves; every frame would fight
  // the operator's clicks.
  if (mission.index !== wasIndex || mission.state !== wasState) drawRoute();
  paintMission(t);
}
