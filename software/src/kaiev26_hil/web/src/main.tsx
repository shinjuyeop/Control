import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { App } from "./app/App";
import { DesignPreview } from "./preview/DesignPreview";
import "./styles/base.css";
import "./styles/instruments.css";
import "./styles/layout.css";
import "./styles/pages.css";
import "./styles/remote.css";
createRoot(document.getElementById("root")!).render(
  <StrictMode>
    {new URLSearchParams(location.search).has("preview") ? (
      <DesignPreview />
    ) : (
      <App />
    )}
  </StrictMode>,
);
