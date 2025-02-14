import { getRosBridgeAddress } from "../../query_utils/auth";

import ErrorBlock from "../UI/ErrorBlock";
import Button from "../UI/Button";
import ContentSection from "../UI/ContentSection";

export default function RosConnect({ reconnect, rosError }) {
  return (
    <>
      <ContentSection header="Connect to ROS 2">
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
      </ContentSection>
      {rosError && (
        <ErrorBlock
          title="Cannot connect to ROS"
          message={
            <>
              If rosbridge is using a self-signed certificate
              <br />
              you need to add it as an exception in your browser
              <br />
              by loading{" "}
              <a href={`https://${getRosBridgeAddress()}`} target="_blank">
                https://
                {getRosBridgeAddress()}
              </a>
              <br />
              and then reconnect to ROS.
            </>
          }
        ></ErrorBlock>
      )}
    </>
  );
}
