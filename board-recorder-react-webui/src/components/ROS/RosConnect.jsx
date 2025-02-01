import { getRosBridgeAddress } from "../../query_utils/auth";

import Button from "../UI/Button";
import ContentSection from "../UI/ContentSection";

export default function RosConnect({ reconnect, rosError }) {
  return (
    <ContentSection header="ROS 2 Connection">
      <p>
        Websocket is set to ws://{getRosBridgeAddress()}{" "}
        <Button type="link" style="button" to="/intro">
          Edit
        </Button>
      </p>
      <p>
        <Button type="button" style="button" onClick={reconnect}>
          Reconnect
        </Button>
      </p>
      {rosError && <p>{rosError}</p>}
    </ContentSection>
  );
}
