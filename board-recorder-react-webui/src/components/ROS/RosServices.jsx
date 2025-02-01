import { useState } from "react";
import { useDispatch } from "react-redux";

import ROSLIB from "roslib";

import Button from "../UI/Button";
import { recordingActions } from "../../store/recordings";

export default function RosServices({ rosRef }) {
  const [recordingId, setRecordingId] = useState(1);

  const dispatch = useDispatch();

  function fetch() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_recording",
      serviceType: "board_recorder_interfaces/srv/FetchRecording",
    });

    service.callService(
      { recording_id: parseInt(recordingId) },

      function (response) {
        let json = JSON.parse(response.recording_json);
        console.log(json);
        let data = json.data || {};
        data.status = json.status;
        data.id = json.recording_id;
        console.log(data);
        dispatch(
          recordingActions.setData({
            data,
          })
        );
        console.log(response);
      },
      function (error) {
        console.log(error);
      }
    );
  }

  return (
    <div>
      Recording id:
      <input
        type="text"
        placeholder="1"
        name="recordingId"
        value={recordingId}
        onChange={(e) => setRecordingId(e.target.value)}
      />{" "}
      <Button type="button" style="button" onClick={fetch}>
        Load
      </Button>
    </div>
  );
}
