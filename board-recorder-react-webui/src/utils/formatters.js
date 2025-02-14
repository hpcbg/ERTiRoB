export function formatTime(time) {
  return `${Math.round(time)}`;
}

export function formatUnixTimestamp(timestamp) {
  return new Date(timestamp * 1000).toLocaleString();
}
