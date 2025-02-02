import { useSelector } from "react-redux";

import RecordingEvents from "./RecordingEvents";

export default function Recording() {
  const recording = useSelector((state) => state.recordings.data);

  return (
    <>
      {recording && (
        <div>
          <p>Recording id: {recording.id}</p>
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
