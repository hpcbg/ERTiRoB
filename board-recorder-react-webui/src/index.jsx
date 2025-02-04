import ReactDOM from "react-dom/client";
import { Provider } from "react-redux";

import App from "./App.jsx";
import "./App.css";
import recordings from "./recordings/index.js";

ReactDOM.createRoot(document.getElementById("root")).render(
  <Provider store={recordings}>
    <App />
  </Provider>
);
