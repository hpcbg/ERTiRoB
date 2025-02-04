import styles from "./ContentSection.module.css";

export default function ContentSection({ header, children }) {
  return (
    <section className={styles.section}>
      <header>
        <h2>{header}</h2>
      </header>
      {children}
    </section>
  );
}
