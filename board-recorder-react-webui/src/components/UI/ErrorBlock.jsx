import styles from "./ErrorBlock.module.css";

export default function ErrorBlock({ title, message }) {
  return (
    <div className={styles.errorBlock}>
      <div className={styles.errorIcon}>!</div>
      <div className={styles.errorText}>
        <h2>{title}</h2>
        <p>{message}</p>
      </div>
    </div>
  );
}
