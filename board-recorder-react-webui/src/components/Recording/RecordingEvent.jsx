export default function RecordingEvent({ children }) {
  return (
    <tr>
      {children.map((field, i) => (
        <td key={`td_${i}`}>{field}</td>
      ))}
    </tr>
  );
}
