import { useState } from "react";
import { useSelector } from "react-redux";

import ROSLIB from "roslib";

import Modal from "../UI/Modal";
import { FormActions } from "../UI/Form";
import Button from "../UI/Button";

export default function RosTaskBoardManager({ rosRef }) {
  const [isRemoving, setIsRemoving] = useState(false);

  const taskBoardId = useSelector((state) => state.recordings.taskBoardId);

  function removeTaskBoard() {
    const action = new ROSLIB.Action({
      ros: rosRef.current,
      name: "/remove_task_board",
      actionType: "board_recorder_interfaces/action/RemoveTaskBoard",
    });

    const goal = new ROSLIB.ActionGoal({
      task_board_id: taskBoardId,
    });

    action.sendGoal(
      goal,
      (result) => {
        console.log("result", result);
        setIsRemoving(false);
      },
      (feedback) => console.log("feedback", feedback),
      (error) => console.log("error", error)
    );
  }

  function fetchTaskBoardRecordings() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "fetch_task_board_recordings",
      serviceType: "board_recorder_interfaces/srv/FetchTaskBoardRecordings",
    });

    service.callService(
      { task_board_id: taskBoardId },
      (response) => {
        const blob = new Blob([response.task_board_recordings_json], {
          type: "application/json",
        });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `task_board_${taskBoardId}_${Math.round(
          +new Date() / 1000
        )}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        console.log(response);
      },
      (error) => console.log(error)
    );
  }

  return (
    <div>
      {isRemoving && (
        <Modal onClose={() => setIsRemoving(false)}>
          <h2>Are you sure?</h2>
          <p>
            Do you really want to remove from the local database the data for
            board with id {taskBoardId}?<br />
            <br />
            This action cannot be undone!
          </p>
          <FormActions>
            <Button
              type="button"
              style="text"
              onClick={() => setIsRemoving(false)}
            >
              Cancel
            </Button>
            <Button type="button" style="text" onClick={removeTaskBoard}>
              Confirm removal
            </Button>
          </FormActions>
        </Modal>
      )}
      <p>
        <Button type="button" style="button" onClick={fetchTaskBoardRecordings}>
          Download all recordings
        </Button>{" "}
        <Button
          type="button"
          style="button"
          onClick={() => setIsRemoving(true)}
        >
          Remove task board data
        </Button>
      </p>
    </div>
  );
}
