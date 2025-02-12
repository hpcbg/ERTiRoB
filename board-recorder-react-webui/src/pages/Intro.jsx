import { useState } from "react";

import {
  getRosBridgeAddress,
  setRosBridgeAddress,
} from "../query_utils/auth.js";

import Button from "../components/UI/Button.jsx";

import introImage from "../assets/intro.jpg";

import styles from "./Intro.module.css";

export default function IntroPage() {
  const [value, setValue] = useState(getRosBridgeAddress());
  function handleChange(e) {
    setValue(e.target.value);
    setRosBridgeAddress(e.target.value);
  }

  return (
    <section
      className={styles.contentSection}
      id="overview-section"
      style={{
        backgroundImage: `url(${introImage})`,
        backgroundRepeat: "no-repeat",
        backgroundSize: "cover",
      }}
    >
      <div>
        <h2>Welcome to the task board recorder!</h2>
        <p>To connect to ROS 2, please set your rosbridge websocket address:</p>
        <p>
          wss://
          <input
            type="text"
            placeholder="localhost:9090"
            name="websocket"
            value={value}
            onChange={handleChange}
          />{" "}
          <Button type="link" style="button" to="/ros">
            Connect to ROS
          </Button>
        </p>
        <p>or</p>
        <p>you can browse offline a downloaded task board recorder database</p>
        <p>
          <Button type="link" style="button" to="/browser">
            Open database browser
          </Button>
        </p>
      </div>
    </section>
  );
}
