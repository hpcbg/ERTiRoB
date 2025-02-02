import { useState } from "react";
import { useDispatch } from "react-redux";

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

  const dispatch = useDispatch();

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
      { count: 5 },
      (response) => {
        setLatestRecordings(JSON.parse(response.recordings_list_json));
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
        console.log(response.sensor_names_json);
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

  return (
    <div>
      <p>
        <Button type="button" style="button" onClick={fetchSensorNames}>
          Load Sensors
        </Button>{" "}
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
        {sensorName && (
          <>
            Sensor value: {sensorData}{" "}
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
          Load Latest 5 Recordings
        </Button>{" "}
        <select onChange={(e) => setRecordingId(e.target.value)}>
          {latestRecordings.length == 0 ? (
            <option value={0}>Press load button</option>
          ) : (
            <>
              <option value={0}>Select recording</option>
              {latestRecordings.map((recording, i) => (
                <option key={i} value={recording.id}>
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
      </p>
    </div>
  );
}
