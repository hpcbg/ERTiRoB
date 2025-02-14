import RecordingEvent from "./RecordingEvent";

import { formatTime } from "../../utils/formatters";

import styles from "./RecordingEvents.module.css";

export default function RecordingEvents({ children }) {
  return (
    <table className={styles.table}>
      <thead>
        <tr>
          <th>Name</th>
          <th>Time [s]</th>
          <th>Value</th>
        </tr>
      </thead>
      <tbody>
        {children.map((event, i) => (
          <RecordingEvent key={`event_${i}`}>
            {[event.name, formatTime(event.time), event.data]}
          </RecordingEvent>
        ))}
      </tbody>
    </table>
  );
}
