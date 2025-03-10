import { useEffect, useState } from "react";
import { useDispatch, useSelector } from "react-redux";

import ROSLIB from "roslib";

import Button from "../UI/Button";
import { recordingActions } from "../../recordings/recordings";
import { formatUnixTimestamp } from "../../utils/formatters";

export default function RosTaskBoardFetchRecordings({ rosRef }) {
  const taskBoardId = useSelector((state) => state.recordings.taskBoardId);
  const [latestRecordings, setLatestRecordings] = useState([]);
  const [recordingIdInput, setRecordingIdInput] = useState(0);
  const [protocol, setProtocol] = useState("");
  const [protocols, setProtocols] = useState([]);
  const [count, setCount] = useState(5);
  const [autoUpdateSeconds, setAutoUpdateSeconds] = useState(10);
  const [autoUpdateChecked, setAutoUpdateChecked] = useState(false);

  const recording = useSelector((state) => state.recordings.data);
  const recordingId = useSelector(
    (state) => state.recordings.currentRecordingId
  );
  const dispatch = useDispatch();

  function fetchTaskBoardProtocols() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_task_board_protocols",
      serviceType: "board_recorder_interfaces/srv/FetchTaskBoardProtocols",
    });

    service.callService(
      { task_board_id: taskBoardId },
      (response) => {
        setProtocols(JSON.parse(response.task_board_protocols_json));
        console.log(response);
      },
      (error) => console.log(error)
    );
  }

  function fetchNewestRecordings() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_newest_recordings",
      serviceType: "board_recorder_interfaces/srv/FetchNewestRecordings",
    });

    service.callService(
      { task_board_id: taskBoardId, protocol, count },
      (response) => {
        setLatestRecordings(JSON.parse(response.recordings_list_json));
        console.log(response);
      },
      (error) => console.log(error)
    );
  }

  function fetchRecording() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_recording",
      serviceType: "board_recorder_interfaces/srv/FetchRecording",
    });

    service.callService(
      { task_board_id: taskBoardId, recording_id: parseInt(recordingIdInput) },
      (response) => {
        dispatch(
          recordingActions.setData({
            data: JSON.parse(response.recording_json),
          })
        );
        console.log(response);
      },
      (error) => console.log(error)
    );
  }

  function downloadRecording() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_recording",
      serviceType: "board_recorder_interfaces/srv/FetchRecording",
    });

    service.callService(
      { task_board_id: taskBoardId, recording_id: parseInt(recordingIdInput) },
      (response) => {
        const blob = new Blob([response.recording_json], {
          type: "application/json",
        });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `recording_${parseInt(
          recordingIdInput
        )}_${taskBoardId}_${Math.round(+new Date() / 1000)}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        console.log(response);
      },
      (error) => console.log(error)
    );
  }

  function fetchNewRecordingEvents() {
    if (!recording || !("events" in recording)) return;

    let time = 0;
    if (recording.events.length > 0)
      time = recording.events[recording.events.length - 1].time + 0.01;

    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_recording_events",
      serviceType: "board_recorder_interfaces/srv/FetchRecordingEvents",
    });

    service.callService(
      {
        task_board_id: taskBoardId,
        recording_id: recording.id,
        from_time: time,
      },
      (response) => {
        dispatch(
          recordingActions.addNewEventsData({
            recording_id: recording.id,
            events: JSON.parse(response.events_json),
          })
        );
        console.log(response);
      },
      (error) => console.log(error)
    );
  }

  useEffect(() => {
    let autoUpdateInterval = null;
    if (autoUpdateChecked) {
      autoUpdateInterval = setInterval(
        fetchNewRecordingEvents,
        autoUpdateSeconds * 1000
      );
    }

    return () => {
      if (autoUpdateInterval) {
        clearInterval(autoUpdateInterval);
      }
    };
  }, [autoUpdateChecked, autoUpdateSeconds, recording]);

  return (
    <div>
      <p>
        Protocol:{" "}
        <select onChange={(e) => setProtocol(e.target.value)}>
          {protocols.length == 0 ? (
            <option value={""}>Press reload button</option>
          ) : (
            <>
              <option value={""}>any protocol</option>
              {protocols.map((protocol) => (
                <option key={protocol} value={protocol}>
                  {protocol}
                </option>
              ))}
            </>
          )}
        </select>{" "}
        <Button type="button" style="button" onClick={fetchTaskBoardProtocols}>
          Reload protocols
        </Button>
      </p>
      <p>
        <Button type="button" style="button" onClick={fetchNewestRecordings}>
          Load
        </Button>
        {" newest "}
        <input
          type="text"
          placeholder="5"
          name="count"
          value={count}
          onChange={(e) => setCount(parseInt(e.target.value))}
        />
        {" recordings"}
      </p>
      <p>
        Recordings:{" "}
        <select onChange={(e) => setRecordingIdInput(e.target.value)}>
          {latestRecordings.length == 0 ? (
            <option value={0}>Press load button</option>
          ) : (
            <>
              <option value={0}>Select recording</option>
              {latestRecordings.map((recording) => (
                <option key={recording.id} value={recording.id}>
                  {recording.id} from{" "}
                  {formatUnixTimestamp(recording.start_time)}
                </option>
              ))}
            </>
          )}
        </select>
      </p>
      <p>
        Recording ID:{" "}
        <input
          type="text"
          placeholder="1"
          name="recordingIdInput"
          value={recordingIdInput}
          onChange={(e) => setRecordingIdInput(e.target.value)}
        />
        {recordingId && (
          <>
            {" "}
            <Button
              type="button"
              style="button"
              onClick={() => setRecordingIdInput(recordingId)}
            >
              Use current
            </Button>
          </>
        )}
      </p>
      <p>
        <Button type="button" style="button" onClick={fetchRecording}>
          Load recording
        </Button>{" "}
        <Button type="button" style="button" onClick={downloadRecording}>
          Download recording
        </Button>
      </p>
      <p>
        <label>
          <input
            type="checkbox"
            name="autoUpdate"
            checked={autoUpdateChecked}
            onChange={() => setAutoUpdateChecked(!autoUpdateChecked)}
          />{" "}
          Auto update
        </label>{" "}
        every{" "}
        <input
          type="input"
          name="autoUpdate"
          value={autoUpdateSeconds}
          onChange={(e) => setAutoUpdateSeconds(parseInt(e.target.value))}
        />{" "}
        seconds
      </p>
    </div>
  );
}
