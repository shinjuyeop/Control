import { createContext, useContext, useState, type ReactNode } from "react";
import type { Point } from "../domain/types";
import { useConsole } from "./useConsole";

function useTelemetryState() {
  const console = useConsole();
  const [paused, setPaused] = useState<Point[] | null>(null);
  return { ...console, paused, setPaused, points: paused ?? console.history };
}
const Context = createContext<ReturnType<typeof useTelemetryState> | null>(null);

export function TelemetryProvider({ children }: { children: ReactNode }) {
  const value = useTelemetryState();
  return <Context.Provider value={value}>{children}</Context.Provider>;
}

export function useTelemetry() {
  const value = useContext(Context);
  if (!value) throw new Error("TelemetryProvider is required");
  return value;
}
