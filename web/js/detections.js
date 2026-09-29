// Detection debug view: mirrors #video's current src into a hidden #det-img
// and redraws #det-canvas from it every frame with the agent's live
// bounding boxes on top. See harness.py's docstring for the wire format
// ({"type": "detections", "boxes": [...]}) and app.css for why the canvas
// (not object-fit) is what keeps image and boxes aligned.
const img = document.getElementById('det-img');
const canvas = document.getElementById('det-canvas');
const ctx = canvas.getContext('2d');
let boxes = [];

export function paintDetections(t) {
  const src = document.getElementById('video').src;
  if (src && img.src !== src) img.src = src;
  // An agent's own boxes win; otherwise show the operator-side detector's
  // (detector/web_feed.py), so hand-flying gets boxes too.
  const agentBoxes = (t.agent && t.agent.detections) || [];
  boxes = agentBoxes.length ? agentBoxes : (t.detector || []);
}

function draw() {
  requestAnimationFrame(draw);
  if (!img.naturalWidth) return;
  if (canvas.width !== img.naturalWidth) canvas.width = img.naturalWidth;
  if (canvas.height !== img.naturalHeight) canvas.height = img.naturalHeight;
  ctx.drawImage(img, 0, 0);
  if (!boxes.length) return;
  const lineWidth = Math.max(2, canvas.width / 320);
  ctx.lineWidth = lineWidth;
  ctx.strokeStyle = '#3f6';
  ctx.fillStyle = '#3f6';
  ctx.font = `${Math.max(14, Math.round(canvas.width / 40))}px system-ui, sans-serif`;
  ctx.textBaseline = 'bottom';
  for (const b of boxes) {
    const [x1, y1, x2, y2] = b.bbox;
    ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
    ctx.fillText(`${b.class_name} ${Math.round(b.confidence * 100)}%`,
      x1 + lineWidth, Math.max(y1, 12));
  }
}
draw();
