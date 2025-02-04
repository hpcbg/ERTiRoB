import Button from "../UI/Button";

export default function RosStatus({ close, rosStatus }) {
  return (
    <>
      ROS 2 {rosStatus}
      {" "}
      {rosStatus == "OK" && (
        <Button type="button" style="button" onClick={close}>
          Close
        </Button>
      )}
    </>
  );
}
