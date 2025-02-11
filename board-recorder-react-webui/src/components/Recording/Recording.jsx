import RecordingEvents from "./RecordingEvents";

import { formatUnixTimestamp } from "../../utils/formatters";

export default function Recording({ recording }) {
  return (
    <div>
      <p>
        <strong>Recording Details</strong>
      </p>
      <p>Recording id: {recording.id}</p>
      <p>Task board id: {recording.task_board_id}</p>
      {recording.status != "Not Found" && (
        <p>Recording time: {formatUnixTimestamp(recording.start_time)}</p>
      )}
      {recording.status != "Not Found" && <p>Protocol: {recording.protocol}</p>}
      <p>Recording status: {recording.status}</p>
      {recording.status != "Not Found" && (
        <RecordingEvents>{recording.events}</RecordingEvents>
      )}
    </div>
  );
}
