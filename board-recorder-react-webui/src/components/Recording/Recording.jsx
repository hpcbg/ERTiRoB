import { useSelector } from "react-redux";

import RecordingEvents from "./RecordingEvents";

import { formatUnixTimestamp } from "../../utils/formatters";

export default function Recording() {
  const recording = useSelector((state) => state.recordings.data);

  return (
    <>
      {recording && (
        <div>
          <p>
            <strong>Recording Details</strong>
          </p>
          <p>Recording id: {recording.id}</p>
          <p>Recording time: {formatUnixTimestamp(recording.start_time)}</p>
          <p>Recording name: {recording.name}</p>
          <p>Recording status: {recording.status}</p>
          {recording.status != "Not Found" && (
            <RecordingEvents>{recording.events}</RecordingEvents>
          )}
        </div>
      )}
    </>
  );
}
