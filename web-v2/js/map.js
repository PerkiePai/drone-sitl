// Leaflet map init and the live drone markers. L is a global from the classic
// (non-module) <script src="/vendor/leaflet.js">, loaded before this module.
//
// TWO markers share the map:
//   green (droneMarker)  -- PX4's own fused position (GLOBAL_POSITION_INT).
//                           This is what the autopilot BELIEVES; it can drift.
//   cyan  (gtMarker)     -- ground truth straight from the simulator, fed via
//                           /tmp/drone_truth.json. Absent when the sim is not
//                           publishing it. The offset between the two is the
//                           estimator error.

// Esri World Imagery: no API key, and the same place on Earth as the Cesium
// tiles the drone is flying over, because drone_setup_px4_cesium.py sets the
// PX4 GPS origin from the Cesium georeference.
export const map = L.map('map', { zoomControl: true }).setView([0, 0], 2);
L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { maxZoom: 19, attribution: 'Imagery &copy; Esri' }).addTo(map);

const droneIcon = L.divIcon({
  className: 'drone-icon', iconSize: [24, 24], iconAnchor: [12, 12],
  html: '<div class="drone-arrow"></div>' });
const droneMarker = L.marker([0, 0], { icon: droneIcon, interactive: false });
let droneOnMap = false;
let centredOnce = false;      // first fix centres; after that the operator pans

const gtIcon = L.divIcon({
  className: 'drone-icon', iconSize: [28, 28], iconAnchor: [14, 14],
  html: '<div class="gt-arrow"></div>' });
// zIndexOffset negative so the green estimate always draws on top of it.
const gtMarker = L.marker([0, 0],
  { icon: gtIcon, interactive: false, zIndexOffset: -1000 });
let gtOnMap = false;

function rotate(marker, cls, deg) {
  const el = marker.getElement() && marker.getElement().querySelector(cls);
  // The triangle is drawn pointing north, so heading is a plain rotation.
  if (el) el.style.transform = `rotate(${deg}deg)`;
}

export function paintDrone(t) {
  if (t.lat !== null && t.lon !== null) {
    const p = [t.lat, t.lon];
    if (!droneOnMap) { droneMarker.addTo(map); droneOnMap = true; }
    droneMarker.setLatLng(p);
    if (!centredOnce) { map.setView(p, 18); centredOnce = true; }
    rotate(droneMarker, '.drone-arrow', t.heading_deg);
  }

  // Ground truth is optional -- only shown while the sim publishes it.
  if (t.lat_gt !== null && t.lat_gt !== undefined && t.lon_gt !== null) {
    const g = [t.lat_gt, t.lon_gt];
    if (!gtOnMap) { gtMarker.addTo(map); gtOnMap = true; }
    gtMarker.setLatLng(g);
    rotate(gtMarker, '.gt-arrow', t.heading_gt);
    // If PX4 has no position yet, still centre the map on truth.
    if (!centredOnce) { map.setView(g, 18); centredOnce = true; }
  } else if (gtOnMap) {
    map.removeLayer(gtMarker); gtOnMap = false;
  }
}
