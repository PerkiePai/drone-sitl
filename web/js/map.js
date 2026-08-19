// Leaflet map init and the live drone marker. L is a global from the classic
// (non-module) <script src="/vendor/leaflet.js">, loaded before this module.

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

export function paintDrone(t) {
  if (t.lat === null || t.lon === null) return;
  const p = [t.lat, t.lon];
  if (!droneOnMap) { droneMarker.addTo(map); droneOnMap = true; }
  droneMarker.setLatLng(p);
  if (!centredOnce) { map.setView(p, 18); centredOnce = true; }
  const arrow = droneMarker.getElement()
    && droneMarker.getElement().querySelector('.drone-arrow');
  // The triangle is drawn pointing north, so heading is a plain rotation.
  if (arrow) arrow.style.transform = `rotate(${t.heading_deg}deg)`;
}
