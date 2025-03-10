export function formatTime(time) {
  const format = new Intl.NumberFormat("en-US", {
    minimumFractionDigits: 2,
  });
  return `${format.format(Math.round(time * 100) / 100)}`;
}

export function formatUnixTimestamp(timestamp) {
  return new Date(timestamp * 1000).toLocaleString();
}
