import { Outlet } from "react-router-dom";
import { useEffect, useRef, useState } from "react";
import { useDispatch } from "react-redux";

import ROSLIB from "roslib";

import Header from "../components/UI/Header.jsx";

import RosTaskBoards from "../components/ROS/RosTaskBoards.jsx";
import RosStatus from "../components/ROS/RosStatus.jsx";
import RosConnect from "../components/ROS/RosConnect.jsx";

import { recordingActions } from "../recordings/recordings.js";

import styles from "./Ros.module.css";

import { getRosBridgeAddress } from "../query_utils/auth.js";

export default function Ros() {
  const dispatch = useDispatch();

  const rosRef = useRef(new ROSLIB.Ros());

  rosRef.current.on("error", function (error) {
    console.log("ROS connection error", error);
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
    dispatch(recordingActions.setTaskBoardId({ taskBoardId: "" }));
    dispatch(recordingActions.setData({ data: null }));
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
          {rosStatus == "OK" && <RosTaskBoards rosRef={rosRef}></RosTaskBoards>}
        </div>
      </main>
    </>
  );
}
