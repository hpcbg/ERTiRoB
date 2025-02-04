import { useNavigate } from "react-router-dom";

import Modal from "../UI/Modal.jsx";
import ErrorBlock from "../UI/ErrorBlock.jsx";
import Button from "../UI/Button.jsx";

export default function NotFoundBoundary() {
  const navigate = useNavigate();

  return (
    <Modal
      onClose={() => {
        navigate("/");
        return true;
      }}
    >
      <ErrorBlock title="An error occured!" message={"Page not found!"} />
      <Button type="link" style="button" to="/">
        Go to Home
      </Button>
    </Modal>
  );
}
