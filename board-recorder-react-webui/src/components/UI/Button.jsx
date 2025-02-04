import { Link } from "react-router-dom";
import { motion } from "framer-motion";

import styles from "./Button.module.css";

export default function Button({ type, style, children, ...props }) {
  let content = "";

  let className = styles[style];
  if (props.className) className += " " + props.className;

  if (type == "button") {
    content = (
      <motion.button className={className} {...props}>
        {children}
      </motion.button>
    );
  } else if (type == "submit") {
    content = (
      <button type="submit" className={className} {...props}>
        {children}
      </button>
    );
  } else if (type == "link") {
    content = (
      <Link className={className} {...props}>
        {children}
      </Link>
    );
  }

  return content;
}
