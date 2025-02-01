import { createSlice } from "@reduxjs/toolkit";

const initialRecordingsState = {
  isRecording: false,
  currentRecordingId: null,
  lastRecordingId: null,
  rosStatus: "connecting...",
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
    setData(state, action) {
      state.data = action.payload.data;
    },
  },
});

export const recordingActions = recordingsSlice.actions;

export default recordingsSlice.reducer;
