// Leaflet map init and the live drone marker. L is a global from the classic
// (non-module) <script src="/vendor/leaflet.js">, loaded before this module.

// Esri World Imagery: no API key, and the same place on Earth as the Cesium
// tiles the drone is flying over, because drone_setup_px4_cesium.py sets the
// PX4 GPS origin from the Cesium georeference.
export const map = L.map('map', { zoomControl: true }).setView([0, 0], 2);
L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { maxZoom: 19, attribution: 'Imagery &copy; Esri' }).addTo(map);

// Three positions are drawn, and they are three different numbers:
//
//   green arrow  ground truth, straight from Isaac's physics
//   amber trace  PX4/EKF2's own estimate -- what the aircraft is flying on
//   cyan trace   the raw vision estimate, before EKF2 fuses it
//
// Reading any one alone tells the wrong story. At 98 m the vision drift stays
// ~2 m (the estimator is fine) while the aircraft leaves by 25 m, and PX4's
// estimate tracks the vision, so the amber trace looks calm the whole way.
// Seeing amber and cyan agree while green walks away IS the failure.
//
// The server projects all three through ONE latched origin, so their positions
// on screen are directly comparable and no alignment is applied on the way.
const TRACE_LEN_M = 100;
// Metres of PATH, not samples and not seconds. The trail is then its own scale
// bar: a tight tangle around the hold point when the hold is good, a 100 m
// streak when it is not.

const droneIcon = L.divIcon({
  className: 'drone-icon', iconSize: [24, 24], iconAnchor: [12, 12],
  html: '<div class="drone-arrow"></div>' });
const droneMarker = L.marker([0, 0], { icon: droneIcon, interactive: false });
let droneOnMap = false;
let centredOnce = false;      // first fix centres; after that the operator pans

const truthIcon = L.divIcon({
  className: 'drone-icon', iconSize: [24, 24], iconAnchor: [12, 12],
  html: '<div class="gt-arrow"></div>' });
const truthMarker = L.marker([0, 0], { icon: truthIcon, interactive: false });
let truthOnMap = false;

// One trace each for PX4 and the vision estimate. Ground truth gets the arrow
// rather than a trail: it is the reference, and a third trail would clutter the
// picture the other two exist to make.
function makeTrace(colour) {
  return { line: L.polyline([], { color: colour, weight: 2, opacity: 0.9 }),
           pts: [], len_m: 0, onMap: false };
}
const px4Trace = makeTrace('#fc6');
const vioTrace = makeTrace('#6cf');
let lastRealigned = null;     // vio.realigned at the last frame painted

function reset(trace) {
  trace.pts = [];
  trace.len_m = 0;
  trace.line.setLatLngs([]);
}

// Append a point and drop the oldest until at most TRACE_LEN_M of path remains.
// Distances come from map.distance() -- real metres on the ellipsoid, because
// a degree of longitude is not a degree of latitude and a trail trimmed in
// degrees would be a different length depending on which way the drone flew.
function extend(trace, p) {
  const last = trace.pts[trace.pts.length - 1];
  if (last) {
    const step = map.distance(last, p);
    if (step < 0.05) return;      // a hovering aircraft must not fill the buffer
    trace.len_m += step;
  }
  trace.pts.push(p);
  while (trace.pts.length > 2 && trace.len_m > TRACE_LEN_M) {
    trace.len_m -= map.distance(trace.pts[0], trace.pts[1]);
    trace.pts.shift();
  }
  if (!trace.onMap) { trace.line.addTo(map); trace.onMap = true; }
  trace.line.setLatLngs(trace.pts);
}

function point(marker, on, p, heading) {
  marker.setLatLng(p);
  const arrow = marker.getElement() && marker.getElement().firstElementChild;
  // The triangles are drawn pointing north, so heading is a plain rotation.
  if (arrow && heading !== null && heading !== undefined) {
    arrow.style.transform = `rotate(${heading}deg)`;
  }
}

export function paintDrone(t) {
  if (t.lat === null || t.lon === null) return;
  const p = [t.lat, t.lon];
  if (!droneOnMap) { droneMarker.addTo(map); droneOnMap = true; }
  point(droneMarker, droneOnMap, p, t.heading_deg);
  extend(px4Trace, p);
  if (!centredOnce) { map.setView(p, 18); centredOnce = true; }
}

// Ground truth and the raw vision estimate. `v` is telemetry.vio, which is null
// on a flight started without --vision, and whose position fields are null
// until PX4 has a fix to site the local frame with.
export function paintTruth(v) {
  const gt = v && v.gt_lat !== null && v.gt_lat !== undefined;
  if (gt) {
    if (!truthOnMap) { truthMarker.addTo(map); truthOnMap = true; }
    point(truthMarker, truthOnMap, [v.gt_lat, v.gt_lon], v.gt_heading_deg);
  } else if (truthOnMap) {
    // Off the map, not frozen in place. A green arrow left at the last known
    // spot is indistinguishable from an aircraft holding station there.
    map.removeLayer(truthMarker);
    truthOnMap = false;
  }

  // The legend names a green arrow and a cyan trace, so it appears only once
  // there is a vision source to draw them from.
  const legend = document.getElementById('maplegend');
  if (legend) legend.hidden = !v;
  if (!v) return;

  // A realignment REDEFINES the vision frame -- once when fusion starts, again
  // at the GNSS cut. Points either side of one are in different coordinate
  // systems, so a trace spanning it would draw a jump the aircraft never made.
  // PX4's trace is deliberately NOT reset for EKF2's own position reset at the
  // cut: that jump happens within one frame, and showing it is the point.
  if (lastRealigned !== null && v.realigned !== lastRealigned) reset(vioTrace);
  lastRealigned = v.realigned;

  if (v.vio_lat !== null && v.vio_lat !== undefined) {
    extend(vioTrace, [v.vio_lat, v.vio_lon]);
  }
}
