// The WebSocket lifecycle: connect/reconnect and outbound send. Carries no
// opinion about telemetry, mission state, or the DOM -- callers wire that up
// via the callbacks passed to connect().

let socket = null;

export function connect({ onOpen, onMessage, onClose }) {
  socket = new WebSocket(`ws://${location.host}/ws`);
  socket.onopen = () => onOpen();
  socket.onmessage = e => onMessage(JSON.parse(e.data));
  socket.onclose = () => {
    onClose();
    setTimeout(() => connect({ onOpen, onMessage, onClose }), 1000);
  };
}

export function isConnected() {
  return !!socket && socket.readyState === 1;
}

export function send(o) {
  if (isConnected()) socket.send(JSON.stringify(o));
}
