import { useEffect, useState } from "react";
import { useSelector } from "react-redux";

import ROSLIB from "roslib";

import Button from "../UI/Button";

export default function RosTaskBoardSensors({ rosRef }) {
  const [sensorNames, setSensorNames] = useState([]);
  const [sensorName, setSensorName] = useState("");
  const [sensorData, setSensorData] = useState("");

  const taskBoardId = useSelector((state) => state.recordings.taskBoardId);

  function fetchSensorNames() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_sensor_names",
      serviceType: "board_recorder_interfaces/srv/FetchSensorNames",
    });

    service.callService(
      { task_board_id: taskBoardId },
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
      { task_board_id: taskBoardId, sensor_name: sensorName },
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

    return () => {};
  }, []);

  return (
    <div>
      <p>
        Board sensors:{" "}
        {sensorNames.length == 0 && (
          <>
            <Button type="button" style="button" onClick={fetchSensorNames}>
              Load
            </Button>{" "}
          </>
        )}
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
    </div>
  );
}
