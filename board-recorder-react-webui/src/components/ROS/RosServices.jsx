import { useEffect, useState } from "react";
import { useDispatch, useSelector } from "react-redux";

import ROSLIB from "roslib";

import Button from "../UI/Button";
import { recordingActions } from "../../recordings/recordings";
import { formatUnixTimestamp } from "../../utils/formatters";

export default function RosServices({ rosRef }) {
  const [recordingId, setRecordingId] = useState(1);
  const [sensorNames, setSensorNames] = useState([]);
  const [sensorName, setSensorName] = useState("");
  const [sensorData, setSensorData] = useState("");
  const [latestRecordings, setLatestRecordings] = useState([]);
  const [autoUpdateChecked, setAutoUpdateChecked] = useState(false);

  const recording = useSelector((state) => state.recordings.data);

  const dispatch = useDispatch();

  function fetchCurrentRecordingId() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_current_recording_id",
      serviceType: "board_recorder_interfaces/srv/FetchCurrentRecordingId",
    });

    service.callService(
      {},
      (response) => {
        if (response.recording_id >= 0) {
          dispatch(
            recordingActions.record({
              recording_id: response.recording_id,
            })
          );
        } else {
          dispatch(recordingActions.stop());
        }
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
      { recording_id: parseInt(recordingId) },
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

  function fetchLatestRecordings() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_latest_recordings",
      serviceType: "board_recorder_interfaces/srv/FetchLatestRecordings",
    });

    service.callService(
      { count: 10 },
      (response) => {
        setLatestRecordings(JSON.parse(response.recordings_list_json));
        console.log(response);
      },
      (error) => console.log(error)
    );
  }

  function fetchNewRecordingEvents() {
    if (!recording || !("events" in recording) || recording.events.length == 0)
      return;

    let time = recording.events[recording.events.length - 1].time + 0.01;

    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_recording_events",
      serviceType: "board_recorder_interfaces/srv/FetchRecordingEvents",
    });

    service.callService(
      { recording_id: recording.id, from_time: time },
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

  function fetchSensorNames() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_sensor_names",
      serviceType: "board_recorder_interfaces/srv/FetchSensorNames",
    });

    service.callService(
      {},
      (response) => {
        let json = JSON.parse(response.sensor_names_json);
        setSensorNames(json);
        console.log(response);
      },
      (error) => console.log(error)
    );
  }

  function fetchSensorData(sensorName) {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_sensor_data",
      serviceType: "board_recorder_interfaces/srv/FetchSensorData",
    });

    service.callService(
      { sensor_name: sensorName },
      (response) => {
        if (response.data_json == "") {
          setSensorData("");
          setSensorName("");
        } else {
          let json = JSON.parse(response.data_json);
          setSensorData(json.data);
          setSensorName(sensorName);
        }
        console.log(response);
      },
      (error) => console.log(error)
    );
  }

  useEffect(() => {
    fetchSensorNames();
    fetchLatestRecordings();
    fetchCurrentRecordingId();
    const fetchCurrentRecordingIdInterval = setInterval(
      fetchCurrentRecordingId,
      10000
    );

    return () => {
      clearInterval(fetchCurrentRecordingIdInterval);
    };
  }, []);

  useEffect(() => {
    let autoUpdateInterval = null;
    if (autoUpdateChecked) {
      autoUpdateInterval = setInterval(fetchNewRecordingEvents, 10000);
    }

    return () => {
      if (autoUpdateInterval) {
        clearInterval(autoUpdateInterval);
      }
    };
  }, [autoUpdateChecked, recording]);

  return (
    <div>
      <p>
        <select onChange={(e) => fetchSensorData(e.target.value)}>
          {sensorNames.length == 0 ? (
            <option>Press load button</option>
          ) : (
            <>
              <option>Select sensor</option>
              {sensorNames.map((sensorName, i) => (
                <option key={i}>{sensorName}</option>
              ))}
            </>
          )}
        </select>{" "}
        current value:{" "}
        {sensorName && (
          <>
            {sensorData}{" "}
            <Button
              type="button"
              style="button"
              onClick={() => fetchSensorData(sensorName)}
            >
              Update
            </Button>
          </>
        )}
      </p>
      <hr />
      <p>
        <Button type="button" style="button" onClick={fetchLatestRecordings}>
          Reload Last 10 Recordings
        </Button>{" "}
        <select onChange={(e) => setRecordingId(e.target.value)}>
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
        Recording id:{" "}
        <input
          type="text"
          placeholder="1"
          name="recordingId"
          value={recordingId}
          onChange={(e) => setRecordingId(e.target.value)}
        />{" "}
        <Button type="button" style="button" onClick={fetchRecording}>
          Load Recording
        </Button>
        <label>
          <input
            type="checkbox"
            name="autoUpdate"
            checked={autoUpdateChecked}
            onChange={(e) => setAutoUpdateChecked(!autoUpdateChecked)}
          />{" "}
          Auto update
        </label>
      </p>
    </div>
  );
}
