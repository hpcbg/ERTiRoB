import { useNavigate, useRouteError } from "react-router-dom";

import Modal from "../UI/Modal.jsx";
import ErrorBlock from "../UI/ErrorBlock.jsx";
import Button from "../UI/Button.jsx";

export default function AuthErrorBoundary() {
  let error = useRouteError();
  const navigate = useNavigate();

  return (
    <Modal
      onClose={() => {
        navigate("/");
        return true;
      }}
    >
      <ErrorBlock
        title="An error occured!"
        message={
          error.info?.error || error.info?.message
            ? "Invalid credentials!"
            : "Connection failed! Please, try again later."
        }
      />
      <Button type="link" style="button" to="/">
        Dismiss
      </Button>
    </Modal>
  );
}
