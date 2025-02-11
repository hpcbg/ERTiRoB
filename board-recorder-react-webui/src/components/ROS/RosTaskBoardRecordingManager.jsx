import { useEffect, useState } from "react";
import { useDispatch, useSelector } from "react-redux";

import ROSLIB from "roslib";

import Button from "../UI/Button";
import { recordingActions } from "../../recordings/recordings";

export default function RosTaskBoardRecordingManager({ rosRef }) {
  const [protocol, setProtocol] = useState("protocol_1");

  const taskBoardId = useSelector((state) => state.recordings.taskBoardId);
  const isRecording = useSelector((state) => state.recordings.isRecording);
  const recordingId = useSelector(
    (state) => state.recordings.currentRecordingId
  );

  const dispatch = useDispatch();

  function record() {
    const action = new ROSLIB.Action({
      ros: rosRef.current,
      name: "/record",
      actionType: "board_recorder_interfaces/action/Record",
    });

    const goal = new ROSLIB.ActionGoal({
      task_board_id: taskBoardId,
      protocol,
    });

    action.sendGoal(
      goal,
      (result) => {
        console.log("result", result);
        if (result.recording_id > 0) {
          dispatch(
            recordingActions.record({
              recording_id: result.recording_id,
            })
          );
        }
      },
      (feedback) => console.log("feedback", feedback),
      (error) => console.log("error", error)
    );
  }

  function stop() {
    const action = new ROSLIB.Action({
      ros: rosRef.current,
      name: "/stop",
      actionType: "board_recorder_interfaces/action/Stop",
    });

    const goal = new ROSLIB.ActionGoal({
      task_board_id: taskBoardId,
      recording_id: recordingId,
    });

    action.sendGoal(
      goal,
      (result) => {
        console.log("result", result);
        dispatch(recordingActions.stop());
      },
      (feedback) => console.log("feedback", feedback),
      (error) => console.log("error", error)
    );
  }

  function fetchCurrentRecordingId() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_current_recording_id",
      serviceType: "board_recorder_interfaces/srv/FetchCurrentRecordingId",
    });

    service.callService(
      { task_board_id: taskBoardId },
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

  useEffect(() => {
    fetchCurrentRecordingId();
    const fetchCurrentRecordingIdInterval = setInterval(
      fetchCurrentRecordingId,
      10000
    );

    return () => {
      clearInterval(fetchCurrentRecordingIdInterval);
    };
  }, [taskBoardId]);

  return (
    <div>
      {!isRecording ? (
        <>
          <label>
            Protocol:{" "}
            <input
              type="text"
              placeholder="protocol_1"
              name="protocol"
              value={protocol}
              onChange={(e) => setProtocol(e.target.value)}
            />
          </label>{" "}
          <Button type="button" style="button" onClick={record}>
            Record
          </Button>
        </>
      ) : (
        <>
          Current recording ID: {recordingId}{" "}
          <Button type="button" style="button" onClick={stop}>
            Stop
          </Button>
        </>
      )}
    </div>
  );
}
