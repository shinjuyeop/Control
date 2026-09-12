import { Activity, CircleGauge, Cpu, Gauge, LayoutDashboard, Pause, Play, ShieldCheck, Terminal } from "lucide-react";
import { useState } from "react";
import { Badge } from "../components/instruments";
import { reception } from "../domain/presentation";
import { pageAllowed } from "../domain/profile";
import { useTelemetry } from "../hooks/useTelemetry";
import { RuntimePage } from "../pages/RuntimePage";
import { VehicleDiagnostics, VehicleDrive, VehicleOverview, VehicleSafety, VehicleSteering } from "../pages/VehiclePages";

const pages = [
  { id: "overview", name: "Overview", icon: LayoutDashboard, component: VehicleOverview },
  { id: "drive", name: "Drive", icon: Gauge, component: VehicleDrive },
  { id: "steering", name: "Steering", icon: CircleGauge, component: VehicleSteering },
  { id: "safety", name: "Safety / Brake", icon: ShieldCheck, component: VehicleSafety },
  { id: "runtime", name: "ECU Runtime", icon: Cpu, component: RuntimePage },
  { id: "diagnostics", name: "Diagnostics", icon: Terminal, component: VehicleDiagnostics },
];

export function VehicleApp() {
  const { state, online, paused, setPaused, history } = useTelemetry();
  const [page, setPage] = useState("overview");
  const available = pages.filter(item => pageAllowed(state, item.id));
  const selected = available.find(item => item.id === page) ?? available[0];
  const Page = selected.component;
  return <div className="workstation vehicle-workstation">
    <header className="global-bar">
      <button className="wordmark" onClick={() => setPage("overview")} aria-label="KAI Vehicle Console Overview"><b>KAI</b><span>VEHICLE CONSOLE<strong className="profile-label">VEHICLE · READ ONLY</strong></span></button>
      <div className="vehicle-link-status" role="status"><Badge tone={online ? "good" : "warning"}>{online ? "WEB LIVE" : "WEB OFFLINE"}</Badge>
        {[["STM_A", "/drive/status"], ["STM_B", "/brake/status"]].map(([name, topic]) => {
          const status = reception(state, online, topic);
          return <Badge key={name} tone={status.tone}>{name} {status.label}</Badge>;
        })}
      </div>
    </header>
    <aside className="navigation" aria-label="페이지 탐색"><span className="nav-caption">VEHICLE OBSERVATION</span>
      <nav aria-label="주 메뉴">{available.map(item => <button key={item.id} className={item.id === selected.id ? "active" : ""} aria-current={item.id === selected.id ? "page" : undefined} onClick={() => { setPage(item.id); window.scrollTo(0, 0); }}><item.icon size={17} /><span>{item.name}</span></button>)}</nav>
    </aside>
    <main className="workspace-main">
      <div className="page-heading"><div><span className="eyebrow">KAI VEHICLE CONSOLE</span><h1>{selected.name}</h1></div>
        {page !== "overview" && <div className="page-tools"><button onClick={() => setPaused(paused ? null : [...history])}>{paused ? <Play size={14} /> : <Pause size={14} />}그래프 {paused ? "재개" : "일시정지"}</button></div>}
      </div>
      {!online && <div className="connection-notice" role="status"><Activity size={16} /><div><strong>WEB OFFLINE · 현재 상태 확인 불가</strong><p>자동 재연결 중 · 마지막 수신 이후 경과시간이 계속 증가합니다.</p></div></div>}
      <div className="page-content"><Page /></div>
      <footer className="workspace-footer"><span>VEHICLE · READ ONLY</span><span>상세 분석: Foxglove · 그래프 이력 60 s</span><span>{paused ? "HISTORY PAUSED" : "LIVE TELEMETRY"}</span></footer>
    </main>
  </div>;
}
