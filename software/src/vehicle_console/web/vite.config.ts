import { defineConfig } from "vite";
export default defineConfig({
  server: {
    proxy: {
      "/ws": { target: "ws://127.0.0.1:8767", ws: true },
      "/api": "http://127.0.0.1:8767",
    },
  },
});
