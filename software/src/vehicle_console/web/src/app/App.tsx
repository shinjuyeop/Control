import { TelemetryProvider, useTelemetry } from "../hooks/useTelemetry";
import { HilApp } from "./HilApp";
import { VehicleApp } from "./VehicleApp";

function ProfileConsole() {
  const { state } = useTelemetry();
  if (state.profile === "hil") return <HilApp />;
  if (state.profile === "vehicle") return <VehicleApp />;
  return <main className="console-waiting"><h1>KAI Vehicle Console</h1><p>서버 연결 대기 · NO DATA</p></main>;
}

export function App() {
  return <TelemetryProvider><ProfileConsole /></TelemetryProvider>;
}
