import { Outlet } from "react-router-dom";
import { useEffect, useRef, useState } from "react";

import ROSLIB from "roslib";

import Header from "../components/UI/Header.jsx";

import RosStatus from "../components/ROS/RosStatus.jsx";
import RosConnect from "../components/ROS/RosConnect.jsx";
import RosActions from "../components/ROS/RosActions.jsx";
import RosServices from "../components/ROS/RosServices.jsx";
import Recording from "../components/Recording/Recording.jsx";

import styles from "./Recordings.module.css";

import { getRosBridgeAddress } from "../query_utils/auth.js";

export default function Recordings() {
  const rosRef = useRef(new ROSLIB.Ros());

  console.log(getRosBridgeAddress());
  rosRef.current.on("error", function (error) {
    console.log("???", error);
    setRosError("ERROR: Cannot connect!");
    setRosStatus("CLOSED");
  });

  rosRef.current.on("connection", function () {
    console.log("Connected!");
    setRosError("");
    setRosStatus("OK");
  });

  rosRef.current.on("close", function () {
    console.log("Connection closed");
    setRosStatus("CLOSED");
  });

  const [rosStatus, setRosStatus] = useState("CLOSED");
  const [rosError, setRosError] = useState("");

  function connect() {
    try {
      setRosError("");
      rosRef.current.connect("wss://" + getRosBridgeAddress());
    } catch (error) {
      setRosError("ERROR: Cannot connect!");
    }
  }

  function close() {
    try {
      setRosError("");
      setRosStatus("CLOSED");
      rosRef.current.close();
    } catch (error) {}
  }
  useEffect(() => {
    connect();
    return () => close();
  }, []);

  return (
    <>
      <Outlet />
      <Header>
        <div className={styles.button}>
          <RosStatus close={close} rosStatus={rosStatus}></RosStatus>
        </div>
      </Header>
      <main>
        <div className={styles.content}>
          {rosStatus != "OK" && (
            <RosConnect
              reconnect={() => {
                close();
                connect();
              }}
              rosError={rosError}
            ></RosConnect>
          )}
          {rosStatus == "OK" && (
            <>
              <RosActions rosRef={rosRef}></RosActions>
              <hr />
              <RosServices rosRef={rosRef}></RosServices>
              <hr />
              <Recording></Recording>
            </>
          )}
        </div>
      </main>
    </>
  );
}
