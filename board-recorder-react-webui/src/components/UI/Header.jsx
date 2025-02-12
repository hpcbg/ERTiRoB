import { useState } from "react";
import { useIsFetching } from "@tanstack/react-query";
import { useNavigate } from "react-router-dom";

import Modal from "./Modal";
import Button from "./Button";
import { FormActions } from "./Form";

import styles from "./Header.module.css";

export default function Header({ children }) {
  const [isExiting, setIsExiting] = useState(false);

  const fetching = useIsFetching();

  const navigate = useNavigate();

  return (
    <>
      {isExiting && (
        <Modal onClose={() => setIsExiting(false)}>
          <h2>Are you sure?</h2>
          <p>Do you really want to exit this page?</p>
          <FormActions>
            <Button
              type="button"
              style="text"
              onClick={() => setIsExiting(false)}
            >
              Cancel
            </Button>
            <Button type="button" style="text" onClick={() => navigate("/")}>
              Go to Home
            </Button>
          </FormActions>
        </Modal>
      )}
      <div className={styles.loading}>{fetching > 0 && <progress />}</div>
      <header className={styles.header}>
        <div className={styles.title}>
          <h1 onClick={() => setIsExiting(true)}>Home</h1>
        </div>
        <nav>{children}</nav>
      </header>
    </>
  );
}
