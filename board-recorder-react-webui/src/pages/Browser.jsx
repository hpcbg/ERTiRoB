import { useEffect } from "react";
import { Outlet } from "react-router-dom";
import { useDispatch, useSelector } from "react-redux";

import Header from "../components/UI/Header.jsx";
import FileUploader from "../components/RecordingsBrowser/FileUploader.jsx";
import RecordingSelector from "../components/RecordingsBrowser/RecordingSelector.jsx";
import Recording from "../components/Recording/Recording.jsx";

import { recordingActions } from "../recordings/recordings.js";

import styles from "./Browser.module.css";

export default function Browser() {
  const taskBoardId = useSelector(
    (state) => state.recordings.uploaded.taskBoardId
  );
  const recording = useSelector(
    (state) => state.recordings.uploaded.selectedRecording
  );

  const dispatch = useDispatch();

  useEffect(() => {
    return () => dispatch(recordingActions.clearUploadedData());
  }, []);

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
