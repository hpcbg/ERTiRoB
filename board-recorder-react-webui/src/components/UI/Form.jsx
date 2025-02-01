import styles from "./Form.module.css";

export default function Form({ children, ...props }) {
  return (
    <form className={styles.form} {...props}>
      {children}
    </form>
  );
}

export function FormActions({ children }) {
  return <p className={styles.actions}>{children}</p>;
}
