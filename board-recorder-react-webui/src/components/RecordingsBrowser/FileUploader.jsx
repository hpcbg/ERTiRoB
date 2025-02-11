import { useState } from "react";
import { useDispatch } from "react-redux";

import { recordingActions } from "../../recordings/recordings";

export default function FileUploader() {
  const [jsonData, setJsonData] = useState(null);
  const [fileName, setFileName] = useState("");
  const [error, setError] = useState("");

  const dispatch = useDispatch();

  const handleFileChange = (event) => {
    const file = event.target.files[0];

    if (file && file.type === "application/json") {
      setFileName(file.name);
      const reader = new FileReader();

      reader.onload = (e) => {
        try {
          const parsedData = JSON.parse(e.target.result);
          setJsonData(parsedData);
          dispatch(
            recordingActions.setUploadedData({
              taskBoardId: parsedData.task_board_id,
              recordings: parsedData.recordings,
            })
          );
          setError("");
        } catch (error) {
          setError("Error reading JSON file!");
        }
      };
      reader.readAsText(file);
    } else {
      setError("Please, upload a valid JSON!");
    }
  };

  return (
    <div>
      {!jsonData && (
        <>
          <p>
            Select task board database JSON file:{" "}
            <input type="file" accept=".json" onChange={handleFileChange} />
          </p>
          {error != "" && <p>Error: {error}</p>}
        </>
      )}
      {jsonData && (
        <p>
          Uploaded file: {fileName}{" "}
          <button
            onClick={() => {
              setJsonData(null);
              setFileName("");
              dispatch(recordingActions.clearUploadedData());
            }}
          >
            Clear
          </button>
        </p>
      )}
    </div>
  );
}
