import RecordingEvent from "./RecordingEvent";

import { formatTime } from "../../utils/formatters";

import styles from "./RecordingEvents.module.css";

export default function RecordingEvents({ children }) {
  return (
    <table className={styles.table}>
      <thead>
        <tr>
          <th>Time</th>
          <th>Name</th>
          <th>Value</th>
        </tr>
      </thead>
      <tbody>
        {children.map((event, i) => (
          <RecordingEvent key={`event_${i}`}>
            {[formatTime(event.time), event.name, event.data]}
          </RecordingEvent>
        ))}
      </tbody>
    </table>
  );
}
