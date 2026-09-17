import {
  Activity,
  Check,
  ChevronRight,
  CircleGauge,
  Cpu,
  FlaskConical,
  Gauge,
  LayoutDashboard,
  LockKeyhole,
  Menu,
  Monitor,
  Pause,
  Play,
  RefreshCw,
  RotateCcw,
  ShieldCheck,
  SlidersHorizontal,
  Terminal,
  TriangleAlert,
  X,
  Zap,
} from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { CommandDialog } from "../components/CommandDialog";
import { Badge } from "../components/instruments";
import { reception } from "../domain/presentation";
import { OperatorRemote } from "../components/OperatorRemote";
import { ProductionCommand } from "../components/ProductionCommand";
import { pageAllowed } from "../domain/profile";
import { WorkbenchProvider, useWorkbench } from "../hooks/useWorkbench";
import { BenchPage } from "../pages/BenchPage";
import { DiagnosticsPage } from "../pages/DiagnosticsPage";
import { RuntimePage } from "../pages/RuntimePage";
import { DrivePage } from "../pages/DrivePage";
import { FaultsPage } from "../pages/FaultsPage";
import { SafetyPage } from "../pages/SafetyPage";
import { ScenariosPage } from "../pages/ScenariosPage";
import { SteeringPage } from "../pages/SteeringPage";
import { VirtualDisplay } from "../pages/VirtualDisplayPage";

const pages = [
  {
    id: "bench",
    name: "Bench",
    label: "BENCH OVERVIEW",
    icon: LayoutDashboard,
    description: "연결 상태와 주요 입력 → 응답",
    component: BenchPage,
  },
  {
    id: "drive",
    name: "Drive",
    label: "DRIVE",
    icon: Gauge,
    description: "주행 목표 · EZ command · 모터 피드백",
    component: DrivePage,
  },
  {
    id: "steering",
    name: "Steering",
    label: "STEERING",
    icon: CircleGauge,
    description: "로드휠 명령 · KEYA 모터축 응답",
    component: SteeringPage,
  },
  {
    id: "safety",
    name: "Safety / Brake",
    label: "BRAKE & SAFETY",
    icon: ShieldCheck,
    description: "제동 요청 · PWM · 구동 차단 관련 관측",
    component: SafetyPage,
  },
  {
    id: "display",
    name: "Virtual LCD",
    label: "VIRTUAL DISPLAY",
    icon: Monitor,
    description: "UNO가 적용한 표시 상태",
    component: null,
  },
  {
    id: "faults",
    name: "Fault Injection",
    label: "FAULT INJECTION",
    icon: Zap,
    description: "CAN enable · TX DROP · controller fault",
    component: FaultsPage,
  },
  {
    id: "scenarios",
    name: "Scenarios",
    label: "SCENARIO",
    icon: FlaskConical,
    description: "시험 단계와 실제 관측 결과",
    component: ScenariosPage,
  },
  {
    id: "diagnostics",
    name: "Diagnostics",
    label: "DIAGNOSTICS",
    icon: Terminal,
    description: "수신 주기 · CAN diagnostics · 원본 명령 로그",
    component: DiagnosticsPage,
  },
  {
    id: "runtime",
    name: "ECU Runtime",
    label: "ECU RUNTIME",
    icon: Cpu,
    description: "STM_A / STM_B · 실행 타이밍과 메모리 여유",
    component: RuntimePage,
  },
];
function Workstation() {
  const {
    state,
    online,
    page,
    navigate,
    fresh,
    hil,
    mine,
    busy,
    running,
    commandState,
    act,
    paused,
    setPaused,
    history,
    toast,
    setToast,
    setModal,
    setTakeoverRevision,
  } = useWorkbench();
  const [dockCollapsed, setDockCollapsed] = useState(false);
  const [drawer, setDrawer] = useState<"navigation" | "remote" | null>(null);
  const [compact, setCompact] = useState(
    () => window.matchMedia("(max-width:1100px)").matches,
  );
  const navRef = useRef<HTMLElement>(null),
    remoteRef = useRef<HTMLElement>(null);
  const visiblePages = pages.filter(item => pageAllowed(state, item.id));
  const selected = visiblePages.find((item) => item.id === page) ?? visiblePages[0];
  const Page = selected.component;
  const estopInputs = [hil.estop_dash, hil.estop_side, hil.estop_rf];
  const estopState = !fresh
    ? "UNKNOWN"
    : estopInputs.some((value) => value === true)
      ? "ACTIVE"
      : estopInputs.every((value) => value === false)
        ? "CLEAR"
        : "UNKNOWN";
  const dockOpen = compact ? drawer === "remote" : !dockCollapsed;
  useEffect(() => {
    const query = window.matchMedia("(max-width:1100px)");
    const resize = () => {
      setCompact(query.matches);
      if (!query.matches) setDrawer(null);
    };
    query.addEventListener("change", resize);
    return () => query.removeEventListener("change", resize);
  }, []);
  useEffect(() => {
    if (!drawer || !compact) return;
    const root = drawer === "remote" ? remoteRef.current : navRef.current;
    const previous = document.activeElement as HTMLElement | null;
    const previousOverflow = document.body.style.overflow;
    document.body.style.overflow = "hidden";
    const focusable = () =>
      Array.from(
        root?.querySelectorAll<HTMLElement>(
          "button:not(:disabled), input:not(:disabled), a[href]",
        ) ?? [],
      );
    focusable()[0]?.focus();
    const keyboard = (event: KeyboardEvent) => {
      // The ARM dialog owns keyboard focus while it is above the Remote drawer.
      if ((event.target as HTMLElement).closest("dialog[open]")) return;
      if (event.key === "Escape") {
        event.preventDefault();
        setDrawer(null);
      }
      if (event.key === "Tab") {
        const items = focusable(),
          first = items[0],
          last = items.at(-1);
        if (event.shiftKey && document.activeElement === first) {
          event.preventDefault();
          last?.focus();
        } else if (!event.shiftKey && document.activeElement === last) {
          event.preventDefault();
          first?.focus();
        }
      }
    };
    document.addEventListener("keydown", keyboard);
    return () => {
      document.body.style.overflow = previousOverflow;
      document.removeEventListener("keydown", keyboard);
      previous?.focus();
    };
  }, [drawer, compact]);
  const go = (id: string) => {
    navigate(id);
    setDrawer(null);
  };
  return (
    <div
      className={`workstation ${!compact && dockCollapsed ? "dock-collapsed" : ""}`}
    >
      <header className="global-bar">
        <button
          className="nav-toggle icon-button"
          aria-label="메뉴 열기"
          onClick={() => setDrawer("navigation")}
        >
          <Menu size={18} />
        </button>
        <button
          className="wordmark"
          onClick={() => go("bench")}
          aria-label="KAI Vehicle Console HIL 벤치"
        >
          <b>KAI</b>
          <span>VEHICLE CONSOLE <strong className="profile-label">HIL</strong></span>
        </button>
        <div className="global-status">
          <div
            className="connection-status"
            role="status"
            aria-label="연결 수신 상태"
          >
            <Badge tone={online ? "good" : "muted"}>
              {online ? "WEB LIVE" : "WEB OFFLINE"}
            </Badge>
            {[
              { name: "STM_A", topic: "/drive/status" },
              { name: "STM_B", topic: "/brake/status" },
            ].map(({ name, topic }) => {
              const status = reception(state, online, topic);
              return (
                <span key={name} title={`${topic} 수신 상태`}>
                  <Badge tone={status.tone}>
                    {name} {status.label}
                  </Badge>
                </span>
              );
            })}
            <Badge tone={fresh ? "good" : "warning"}>
              {fresh ? "STM_TEST LIVE" : "STM_TEST STALE"}
            </Badge>
          </div>
          <span
            className={`command-state ${commandState.tone}`}
            role="status"
            aria-label="주행 명령 상태"
          >
            <span>
              COMMAND<span className="command-source-suffix"> SOURCE</span>
            </span>
            <strong>{commandState.text}</strong>
          </span>
          <span
            className={`global-estop ${estopState.toLowerCase()}`}
            role="status"
            aria-label="E-stop 입력 상태"
            title="STM_TEST 보고 DASH / SIDE / RF 입력 합산 · 물리 비상정지 상태의 증명이 아닙니다."
          >
            E-STOP {estopState}
          </span>
          <span
            className={`scenario-status ${running ? "running" : ""}`}
            role="status"
            aria-label="시나리오 상태"
          >
            {running
              ? `RUNNING · ${state.scenario?.scenario}`
              : "SCENARIO IDLE"}
          </span>
        </div>
        <div className="global-actions">
          <button
            className={mine ? "control-owned" : ""}
            disabled={
              !online ||
              busy ||
              state.control.transitioning ||
              (state.control.occupied &&
                !mine &&
                state.control.revision === undefined)
            }
            onClick={() => {
              if (!mine && state.control.occupied) {
                setTakeoverRevision(state.control.revision);
                setModal("takeover");
              } else {
                act(mine ? "release" : "claim");
              }
            }}
          >
            <LockKeyhole size={13} />
            {state.control.transitioning
              ? "조작권 인계·정리 중"
              : mine
                ? "조작권 해제"
                : state.control.occupied
                  ? state.control.revision === undefined
                    ? "다른 브라우저 조작 중"
                    : "조작권 강제 인계"
                  : "조작권 가져오기"}
          </button>
          <button
            className="reset-button"
            title="주행 명령 OFF와 기본 입력 재적용. E-stop 해제, Plant ON을 포함합니다. 유효한 ECU snapshot의 deadline miss 이력도 새 기준으로 시작합니다."
            disabled={!mine || busy}
            onClick={() => act("safe")}
          >
            <RotateCcw size={13} />
            Bench Reset
          </button>
          <button
            className="control-dock-toggle"
            aria-label={
              compact
                ? "Control Dock 열기"
                : dockCollapsed
                  ? "Control Dock 펼치기"
                  : "Control Dock 접기"
            }
            aria-controls="control-dock"
            aria-expanded={dockOpen}
            onClick={() =>
              compact
                ? setDrawer("remote")
                : setDockCollapsed((value) => !value)
            }
          >
            <SlidersHorizontal size={15} />
            Controls
          </button>
        </div>
      </header>
      <aside
        ref={navRef}
        className={`navigation ${drawer === "navigation" ? "is-open" : ""}`}
        aria-label="페이지 탐색"
        role={compact && drawer === "navigation" ? "dialog" : undefined}
        aria-modal={compact && drawer === "navigation" ? true : undefined}
      >
        <div className="drawer-heading">
          PAGES
          <button
            className="icon-button"
            aria-label="메뉴 닫기"
            onClick={() => setDrawer(null)}
          >
            <X size={18} />
          </button>
        </div>
        <span className="nav-caption">VALIDATION WORKSPACE</span>
        <nav aria-label="주 메뉴">
          {visiblePages.map((item) => (
            <button
              key={item.id}
              aria-current={page === item.id ? "page" : undefined}
              className={page === item.id ? "active" : ""}
              onClick={() => go(item.id)}
            >
              <item.icon size={17} />
              <span>{item.name}</span>
              {item.id === "scenarios" && running && (
                <i className="running-dot" />
              )}
            </button>
          ))}
        </nav>
        <div className="navigation-footer">
          <span className={`status-dot ${online ? "live" : ""}`} />
          <span>{online ? "Web bridge 연결됨" : "Web bridge 연결 대기"}</span>
          <small>ROS 2 / LOCAL BENCH</small>
        </div>
      </aside>
      <main className="workspace-main">
        <div className="page-heading">
          <div>
            <span className="eyebrow">{selected.label}</span>
            <h1>{selected.name}</h1>
            <p>{selected.description}</p>
          </div>
          <div className="page-tools">
            <button
              className={paused ? "warning-text" : ""}
              onClick={() => setPaused(paused ? null : [...history])}
            >
              {paused ? <Play size={13} /> : <Pause size={13} />}그래프{" "}
              {paused ? "재개" : "일시정지"}
            </button>
            <button
              className="icon-button"
              aria-label="보드 상태 새로고침"
              disabled={!mine || busy}
              onClick={() => act("refresh")}
            >
              <RefreshCw size={15} />
            </button>
          </div>
        </div>
        {(!online || !fresh) && (
          <div className="connection-notice">
            <Activity size={16} />
            <div>
              <strong>
                {!online
                  ? "Web 연결 대기"
                  : "STM_TEST의 최신 상태를 기다립니다"}
              </strong>
              <p>
                STM_TEST 입력은 잠기며, 각 관측값은 해당 데이터의 수신 상태에
                따라 표시됩니다.
              </p>
            </div>
          </div>
        )}
        {running && page !== "scenarios" && (
          <button className="running-banner" onClick={() => go("scenarios")}>
            <FlaskConical size={15} />
            {state.scenario?.scenario} · 시험 실행 중 / 수동 입력 잠김
            <ChevronRight size={14} />
          </button>
        )}
        <div className="page-content">
          {Page ? (
            <Page />
          ) : (
            <VirtualDisplay display={state.display} online={online} />
          )}
        </div>
        <footer className="workspace-footer">
          <span>
            {fresh
              ? `STM_TEST protocol ${hil.protocol}`
              : "STM_TEST NO FRESH DATA"}
          </span>
          <span>관측 5 Hz · 그래프 60 s</span>
          <span>{mine ? "TEST CONTROL" : "VIEW ONLY"}</span>
        </footer>
      </main>
      {compact && drawer && (
        <button
          className="drawer-backdrop"
          aria-label="패널 닫기"
          tabIndex={-1}
          onClick={() => setDrawer(null)}
        />
      )}
      <aside
        id="control-dock"
        ref={remoteRef}
        className={`operator-dock ${drawer === "remote" ? "is-open" : ""}`}
        role={compact && drawer === "remote" ? "dialog" : "complementary"}
        aria-modal={compact && drawer === "remote" ? true : undefined}
        aria-label="Control Dock"
      >
        <div className="dock-heading">
          <strong>CONTROL DOCK</strong>
          <button
            aria-label="Control Dock 닫기"
            className="icon-button dock-close"
            onClick={() => setDrawer(null)}
          >
            <X size={18} />
          </button>
        </div>
        <OperatorRemote />
        <ProductionCommand />
      </aside>
      {busy && (
        <div className="pending-strip" role="status">
          <span className="spinner" />
          명령 응답 대기 중 · 아직 적용 확인 전입니다
        </div>
      )}
      {toast && (
        <div
          className={`toast ${toast.error ? "error" : ""}`}
          role={toast.error ? "alert" : "status"}
        >
          {toast.error ? <TriangleAlert size={16} /> : <Check size={16} />}
          <span>{toast.text}</span>
          <button
            className="icon-button"
            aria-label="알림 닫기"
            onClick={() => setToast(null)}
          >
            <X size={15} />
          </button>
        </div>
      )}
      <CommandDialog />
    </div>
  );
}
export function HilApp() {
  return (
    <WorkbenchProvider>
      <Workstation />
    </WorkbenchProvider>
  );
}
