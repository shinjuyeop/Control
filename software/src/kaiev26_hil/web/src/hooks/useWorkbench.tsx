import {
  createContext,
  useContext,
  useEffect,
  useState,
  type ReactNode,
} from "react";
import { Badge } from "../components/instruments";
import { canLink } from "../domain/format";
import type { Data, Point } from "../domain/types";
import { degToRad, kmhToMps, mpsToKmh, radToDeg } from "../domain/units";
import { useConsole } from "./useConsole";

function useWorkbenchState() {
  const { state, online, history, send } = useConsole();
  const [page, setPage] = useState("bench");
  const [busy, setBusy] = useState(false);
  const [toast, setToast] = useState<{ text: string; error: boolean } | null>(
    null,
  );
  const [paused, setPaused] = useState<Point[] | null>(null);
  const [scenarioId, setScenarioId] = useState("smoke_test");
  const [scenarioArm, setScenarioArm] = useState(false);
  const [past, setPast] = useState<Data[]>([]);
  const [logFilter, setLogFilter] = useState("all");
  const [pedal, setPedal] = useState(0);
  const [pedalEditing, setPedalEditing] = useState(false);
  const [production, setProduction] = useState({
    speed_kmh: 0,
    steering_deg: 0,
    brake_engage: false,
  });
  const [productionEdited, setProductionEdited] = useState(false);
  const [modal, setModal] = useState<"production" | "scenario" | null>(null);
  const observed = (topic: string): Data =>
    online && state.topics[topic]?.fresh ? state.topics[topic].data : {};
  const hil = observed("/hil/status"),
    drive = observed("/drive/status"),
    brake = observed("/brake/status"),
    steering = observed("/steering/status");
  const fresh =
    online && state.connected && !!state.topics["/hil/status"]?.fresh;
  const mine = online && state.control.mine;
  const running = state.scenario?.result === "RUNNING";
  // Same control gate as the original bench: no UI path can bypass it.
  const disabled = !fresh || !mine || busy || running;
  const generator = online && state.generator_fresh ? state.generator : {};
  const commandState =
    typeof generator.enabled === "boolean"
      ? {
          text: generator.enabled ? "ARMED" : "DISARMED",
          tone: generator.enabled ? "warning" : "muted",
        }
      : { text: "UNKNOWN", tone: "muted" };
  const points = paused ?? history;
  useEffect(() => {
    if (!pedalEditing && typeof hil.pedal === "number") setPedal(hil.pedal);
  }, [hil.pedal, pedalEditing]);
  useEffect(() => {
    if (!productionEdited && state.generator_fresh)
      setProduction({
        speed_kmh: mpsToKmh(generator.speed_target_mps ?? 0),
        steering_deg: radToDeg(generator.steering_target_rad ?? 0),
        brake_engage: generator.brake_engage ?? false,
      });
  }, [state.generator, productionEdited]);
  useEffect(() => {
    if (!toast) return;
    const timer = setTimeout(() => setToast(null), 7000);
    return () => clearTimeout(timer);
  }, [toast]);
  useEffect(() => {
    if (page === "scenarios" && online)
      fetch("/api/history")
        .then((r) => r.json())
        .then(setPast)
        .catch(() =>
          setToast({ text: "이전 결과를 불러오지 못했습니다.", error: true }),
        );
  }, [page, online, state.scenario?.result]);
  useEffect(() => {
    if (!mine || !fresh) {
      setModal(null);
      setScenarioArm(false);
    }
  }, [mine, fresh]);
  async function act(action: string, args: Data = {}) {
    setBusy(true);
    try {
      const result = await send(action, args);
      setToast({ text: result, error: false });
      return true;
    } catch (error) {
      setToast({ text: (error as Error).message, error: true });
      return false;
    } finally {
      setBusy(false);
    }
  }
  const command = (value: string) => act("command", { command: value });
  async function commands(values: string[]) {
    setBusy(true);
    try {
      for (const value of values) await send("command", { command: value });
      setToast({
        text: "명령 응답을 수신했습니다. 관측값을 확인하세요.",
        error: false,
      });
    } catch (error) {
      setToast({ text: (error as Error).message, error: true });
    } finally {
      setBusy(false);
    }
  }
  const navigate = (id: string) => {
    setPage(id);
    window.scrollTo(0, 0);
  };
  const link = (bus: "ez" | "keya") => canLink(hil, fresh, bus);
  const linkBadge = (bus: "ez" | "keya") => (
    <Badge tone={link(bus).tone}>{link(bus).text}</Badge>
  );
  const productionValues = (enabled: boolean) => ({
    enabled,
    speed_target_mps: kmhToMps(production.speed_kmh),
    steering_target_rad: degToRad(production.steering_deg),
    brake_engage: production.brake_engage,
  });
  const lockReason = !online
    ? "Web 연결 없음"
    : !fresh
      ? "STM_TEST 상태 미수신 / 지연"
      : !mine
        ? state.control.occupied
          ? "다른 브라우저가 조작 중"
          : "조작권이 없습니다"
        : running
          ? "시나리오 실행 중 · 수동 입력 잠김"
          : busy
            ? "명령 응답 대기 중"
            : "";
  return {
    state,
    online,
    history,
    points,
    paused,
    setPaused,
    page,
    navigate,
    busy,
    toast,
    setToast,
    scenarioId,
    setScenarioId,
    scenarioArm,
    setScenarioArm,
    past,
    logFilter,
    setLogFilter,
    pedal,
    setPedal,
    pedalEditing,
    setPedalEditing,
    production,
    setProduction,
    setProductionEdited,
    modal,
    setModal,
    observed,
    hil,
    drive,
    brake,
    steering,
    fresh,
    mine,
    running,
    disabled,
    generator,
    commandState,
    lockReason,
    act,
    command,
    commands,
    link,
    linkBadge,
    productionValues,
  };
}
const WorkbenchContext = createContext<ReturnType<
  typeof useWorkbenchState
> | null>(null);
export function WorkbenchProvider({ children }: { children: ReactNode }) {
  const workbench = useWorkbenchState();
  return (
    <WorkbenchContext.Provider value={workbench}>
      {children}
    </WorkbenchContext.Provider>
  );
}
export function useWorkbench() {
  const workbench = useContext(WorkbenchContext);
  if (!workbench) throw new Error("WorkbenchProvider is required");
  return workbench;
}
