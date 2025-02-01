export function getRosBridgeAddress() {
  const rosbridge_ws = localStorage.getItem("rosbridge_ws");

  if (!rosbridge_ws) {
    return "localhost:9090";
  }

  return rosbridge_ws;
}

export function setRosBridgeAddress(address) {
  localStorage.setItem("rosbridge_ws", address);
}
