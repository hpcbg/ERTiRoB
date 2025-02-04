import { getRosBridgeAddress } from "../../query_utils/auth";

import Button from "../UI/Button";
import ContentSection from "../UI/ContentSection";

export default function RosConnect({ reconnect, rosError }) {
  return (
    <ContentSection header="ROS 2 Connection">
      <p>
        Websocket is set to wss://{getRosBridgeAddress()}{" "}
        <Button type="link" style="button" to="/intro">
          Edit
        </Button>
      </p>
      <p>
        <Button type="button" style="button" onClick={reconnect}>
          Reconnect
        </Button>
      </p>
      {rosError && (
        <p>
          {rosError} If the ROS bridge server is using a self-signed certificate
          you need to accept it by opening https://{getRosBridgeAddress()} in
          your browser to add it as an exception.
        </p>
      )}
    </ContentSection>
  );
}
