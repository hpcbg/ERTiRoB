export function formatTime(time) {
  return `${Math.round(time)} s`;
}

export function formatUnixTimestamp(timestamp) {
  return new Date(timestamp * 1000).toLocaleString();
}
