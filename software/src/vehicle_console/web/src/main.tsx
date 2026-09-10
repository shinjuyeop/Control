import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { App } from "./app/App";
import "./styles/base.css";
import "./styles/instruments.css";
import "./styles/layout.css";
import "./styles/pages.css";
import "./styles/remote.css";
import "./styles/runtime.css";
import "./styles/signals.css";
import "./styles/vehicle.css";
createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
