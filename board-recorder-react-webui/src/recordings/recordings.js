import { createSlice } from "@reduxjs/toolkit";

const initialRecordingsState = {
  isRecording: false,
  currentRecordingId: null,
  lastRecordingId: null,
  rosStatus: "connecting...",
  taskBoardId: "",
  data: null,
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
  },
});

export const recordingActions = recordingsSlice.actions;

export default recordingsSlice.reducer;
