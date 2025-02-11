import { createSlice } from "@reduxjs/toolkit";

const initialRecordingsState = {
  isRecording: false,
  currentRecordingId: null,
  lastRecordingId: null,
  rosStatus: "connecting...",
  taskBoardId: "",
  data: null,
  uploaded: {
    taskBoardId: null,
    protocols: [],
    recordings: [],
    selectedRecording: null,
  },
};

const recordingsSlice = createSlice({
  name: "cart",
  initialState: initialRecordingsState,
  reducers: {
    record(state, action) {
      state.isRecording = true;
      state.currentRecordingId = action.payload.recording_id;
    },
    stop(state) {
      state.isRecording = false;
      state.lastRecordingId = state.currentRecordingId;
      state.currentRecordingId = null;
    },
    setRosStatus(state, action) {
      state.rosStatus = action.payload.status;
    },
    setTaskBoardId(state, action) {
      state.taskBoardId = action.payload.taskBoardId;
    },
    setData(state, action) {
      state.data = action.payload.data;
    },
    addNewEventsData(state, action) {
      if (
        !state.data ||
        state.data.recording_id != action.payload.id ||
        action.payload.events.length == 0
      )
        return;
      state.data.events = state.data.events.concat(action.payload.events);
    },
    setUploadedData(state, action) {
      state.uploaded = {
        taskBoardId: action.payload.taskBoardId,
        protocols: [
          ...new Set(
            action.payload.recordings.map((recording) => recording.protocol)
          ),
        ].sort(),
        recordings: action.payload.recordings,
        recordings_index: Object.fromEntries(
          action.payload.recordings.map((recording, i) => [recording.id, i])
        ),
        selectedRecording: null,
      };
    },
    setUploadedSelectedRecording(state, action) {
      state.uploaded.selectedRecording =
        state.uploaded.recordings[
          state.uploaded.recordings_index[action.payload.recordingId]
        ];
    },
    clearUploadedData(state) {
      state.uploaded = {
        taskBoardId: null,
        protocols: [],
        recordings: [],
        selectedRecording: null,
      };
    },
  },
});

export const recordingActions = recordingsSlice.actions;

export default recordingsSlice.reducer;
