import { Outlet } from "react-router-dom";
import { useSelector } from "react-redux";

import Header from "../components/UI/Header.jsx";

import styles from "./Recordings.module.css";

import FileUploader from "../components/RecordingsBrowser/FileUploader.jsx";
import RecordingSelector from "../components/RecordingsBrowser/RecordingSelector.jsx";
import Recording from "../components/Recording/Recording.jsx";

export default function Browser() {
  const taskBoardId = useSelector(
    (state) => state.recordings.uploaded.taskBoardId
  );
  const recording = useSelector(
    (state) => state.recordings.uploaded.selectedRecording
  );
  return (
    <>
      <Outlet />
      <Header></Header>
      <main>
        <div className={styles.content}>
          <FileUploader></FileUploader>
          {taskBoardId && (
            <>
              <hr />
              <RecordingSelector></RecordingSelector>
              {recording && (
                <>
                  <hr />
                  <Recording recording={recording}></Recording>
                </>
              )}
            </>
          )}
        </div>
      </main>
    </>
  );
}
