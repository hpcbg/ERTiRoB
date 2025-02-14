import { useEffect, useState } from "react";
import { useDispatch, useSelector } from "react-redux";

import ROSLIB from "roslib";

import Button from "../UI/Button";
import { recordingActions } from "../../recordings/recordings";
import RosTaskBoard from "./RosTaskBoard";

export default function RosTaskBoards({ rosRef }) {
  const [taskBoards, setTaskBoards] = useState([]);

  const taskBoardId = useSelector((state) => state.recordings.taskBoardId);

  const dispatch = useDispatch();

  function fetchTaskBoards() {
    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: "/fetch_task_boards",
      serviceType: "board_recorder_interfaces/srv/FetchTaskBoards",
    });

    service.callService(
      {},
      (response) => {
        setTaskBoards(JSON.parse(response.task_boards_json));
        console.log(response);
      },
      (error) => console.log(error)
    );
  }

  useEffect(() => {
    fetchTaskBoards();

    return () => {};
  }, []);

  return (
    <div>
      <div>
        <p>
          <Button type="button" style="button" onClick={fetchTaskBoards}>
            Refresh task boards
          </Button>
          {" Task board: "}
          <select
            onChange={(e) =>
              dispatch(
                recordingActions.setTaskBoardId({
                  taskBoardId: e.target.value,
                })
              )
            }
          >
            {taskBoards.length == 0 ? (
              <option value={""}>Press refresh button</option>
            ) : (
              <>
                <option value={""}>Select task board</option>
                {taskBoards.map((taskBoard) => (
                  <option key={taskBoard.id} value={taskBoard.id}>
                    {taskBoard.id} ({taskBoard.status})
                  </option>
                ))}
              </>
            )}
          </select>
        </p>
      </div>
      {taskBoardId != "" && (
        <>
          <hr />
          <RosTaskBoard rosRef={rosRef}></RosTaskBoard>
        </>
      )}
    </div>
  );
}
