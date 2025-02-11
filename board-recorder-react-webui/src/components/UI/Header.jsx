import { useIsFetching } from "@tanstack/react-query";
import { useNavigate } from "react-router-dom";

import styles from "./Header.module.css";

export default function Header({ children }) {
  const fetching = useIsFetching();

  const navigate = useNavigate();

  return (
    <>
      <div className={styles.loading}>{fetching > 0 && <progress />}</div>
      <header className={styles.header}>
        <div className={styles.title}>
          <h1 onClick={() => navigate("/")}>Board Recorder</h1>
        </div>
        <nav>{children}</nav>
      </header>
    </>
  );
}
