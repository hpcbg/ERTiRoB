import { useRouteLoaderData } from "react-router-dom";
import { useState } from "react";

import {
  getRosBridgeAddress,
  setRosBridgeAddress,
} from "../query_utils/auth.js";

import Button from "../components/UI/Button.jsx";

import introImage from "../assets/intro.jpg";

import styles from "./Intro.module.css";

export default function IntroPage() {
  console.log(useRouteLoaderData("root"));
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
        <p>Please set the ROS 2 bridge websocket address of your task board:</p>
        <p>
          wss://
          <input
            type="text"
            placeholder="localhost:9090"
            name="websocket"
            value={value}
            onChange={handleChange}
          />
        </p>
        <p>
          <Button type="link" style="button" to="/recordings">
            CONNECT
          </Button>
        </p>
      </div>
    </section>
  );
}
