import { configureStore } from "@reduxjs/toolkit";

import recordingsReducer from "./recordings.js";

const recordings = configureStore({
  reducer: { recordings: recordingsReducer },
});

export default recordings;
