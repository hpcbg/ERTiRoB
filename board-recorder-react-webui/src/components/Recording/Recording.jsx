import RecordingEvents from "./RecordingEvents";

import { formatUnixTimestamp } from "../../utils/formatters";

export default function Recording({ recording }) {
  return (
    <div>
      <p>
        <strong>Recording Details</strong>
      </p>
      <table>
        <tbody>
          <tr>
            <td>Recording id:</td>
            <td>{recording.id}</td>
          </tr>
          <tr>
            <td>Task board id:</td>
            <td>{recording.task_board_id}</td>
          </tr>
          <tr>
            <td>Recording status:</td>
            <td>{recording.status}</td>
          </tr>
          {recording.status != "Not Found" && (
            <tr>
              <td>Recording time:</td>
              <td>{formatUnixTimestamp(recording.start_time)}</td>
            </tr>
          )}
          {recording.status != "Not Found" && (
            <tr>
              <td>Protocol:</td>
              <td>{recording.protocol}</td>
            </tr>
          )}

          {recording.status != "Not Found" && (
            <tr>
              <td style={{ verticalAlign: "top" }}>Events:</td>
              <td>
                <RecordingEvents>{recording.events}</RecordingEvents>
              </td>
            </tr>
          )}
        </tbody>
      </table>
    </div>
  );
}
