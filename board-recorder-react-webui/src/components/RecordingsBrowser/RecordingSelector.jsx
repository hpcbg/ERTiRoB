import { useState } from "react";
import { useDispatch, useSelector } from "react-redux";

import { recordingActions } from "../../recordings/recordings";
import { formatUnixTimestamp } from "../../utils/formatters";

export default function RecordingSelector() {
  const [recordingIds, setRecordingIds] = useState([]);

  const taskBoardId = useSelector(
    (state) => state.recordings.uploaded.taskBoardId
  );
  const protocols = useSelector((state) => state.recordings.uploaded.protocols);
  const recordings = useSelector(
    (state) => state.recordings.uploaded.recordings
  );

  const dispatch = useDispatch();

  function handleProtocolChange(protocol) {
    setRecordingIds(
      recordings
        .filter((recording) => protocol == "" || recording.protocol == protocol)
        .map((recording) => ({
          id: recording.id,
          startTime: recording.start_time,
        }))
    );
  }

  return (
    <div>
      <table>
        <tbody>
          <tr>
            <td>Task board ID:</td>
            <td>{taskBoardId}</td>
          </tr>
          <tr>
            <td>Protocol:</td>
            <td>
              <select onChange={(e) => handleProtocolChange(e.target.value)}>
                <option value={""}>any protocol</option>
                {protocols.map((protocol) => (
                  <option key={protocol} value={protocol}>
                    {protocol}
                  </option>
                ))}
              </select>
            </td>
          </tr>
          <tr>
            <td>Recording:</td>
            <td>
              <select
                onChange={(e) =>
                  dispatch(
                    recordingActions.setUploadedSelectedRecording({
                      recordingId: e.target.value,
                    })
                  )
                }
              >
                <option value={0}>Select recording</option>
                {recordingIds.map((recording) => (
                  <option key={recording.id} value={recording.id}>
                    {recording.id} from{" "}
                    {formatUnixTimestamp(recording.startTime)}
                  </option>
                ))}
              </select>
            </td>
          </tr>
        </tbody>
      </table>
    </div>
  );
}
