import { useSelector } from "react-redux";

import RosTaskBoardManager from "./RosTaskBoardManager";
import RosTaskBoardSensors from "./RosTaskBoardSensors";
import RosTaskBoardRecordingManager from "./RosTaskBoardRecordingManager";
import RosTaskBoardFetchRecordings from "./RosTaskBoardFetchRecordings";
import Recording from "../Recording/Recording";

export default function RosTaskBoard({ rosRef }) {
  const recording = useSelector((state) => state.recordings.data);

  return (
    <div>
      <RosTaskBoardManager rosRef={rosRef}></RosTaskBoardManager>
      <hr />
      <RosTaskBoardSensors rosRef={rosRef}></RosTaskBoardSensors>
      <hr />
      <RosTaskBoardRecordingManager
        rosRef={rosRef}
      ></RosTaskBoardRecordingManager>
      <hr />
      <RosTaskBoardFetchRecordings
        rosRef={rosRef}
      ></RosTaskBoardFetchRecordings>
      {recording && (
        <>
          <hr />
          <Recording recording={recording}></Recording>
        </>
      )}
    </div>
  );
}
