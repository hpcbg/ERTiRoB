import { useSelector, useDispatch } from "react-redux";

import ROSLIB from "roslib";

import Button from "../UI/Button";
import { recordingActions } from "../../recordings/recordings";

export default function RosActions({ rosRef }) {
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
      recording_name: "untitled",
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

  return (
    <div>
      {!isRecording ? (
        <Button type="button" style="button" onClick={record}>
          Record
        </Button>
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
