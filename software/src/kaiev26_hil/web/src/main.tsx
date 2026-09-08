import { StrictMode, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import {
  Activity,
  ArrowRight,
  Cable,
  Check,
  ChevronRight,
  CircuitBoard,
  ClipboardList,
  Download,
  FlaskConical,
  Gauge,
  LockKeyhole,
  Menu,
  Monitor,
  Pause,
  Play,
  PlugZap,
  RefreshCw,
  RotateCcw,
  Square,
  Terminal,
  TriangleAlert,
  Unplug,
  X,
  Zap,
} from "lucide-react";
import { useConsole, type Data, type Point } from "./connection";
import {
  Badge,
  Chart,
  Choice,
  Details,
  Fact,
  fmt,
  Notice,
  Numeric,
  Panel,
  Source,
  Switch,
} from "./components";
import {
  VehicleSummary,
  VehicleTelemetry,
  FrameHealth,
  ReceptionTable,
  brakeReasons,
  boolText,
} from "./VehicleTelemetry";
import { mpsToKmh, kmhToMps, radToDeg, degToRad, scenarioValue } from "./units";
import { VirtualDisplay } from "./VirtualDisplay";
import "./style.css";
const sections = [
  {
    id: "bench",
    name: "벤치",
    en: "Bench",
    icon: Gauge,
    description:
      "구동 · 조향 · 브레이크를 함께 조작하고, 요청과 보드 반응을 한 화면에서 비교하세요.",
  },
  {
    id: "display",
    name: "가상 LCD",
    en: "Virtual display",
    icon: Monitor,
    description:
      "UNO가 실제로 수신·적용한 표시 상태를 LCD 화면으로 확인하세요.",
  },
  {
    id: "faults",
    name: "고장 주입",
    en: "Fault injection",
    icon: Zap,
    description:
      "통신 단절과 컨트롤러 고장을 주입하고 시스템의 반응을 확인하세요.",
  },
  {
    id: "scenarios",
    name: "시나리오",
    en: "Scenario runner",
    icon: FlaskConical,
    description: "반복 시험을 실행하고 각 조건의 관측 결과를 확인하세요.",
  },
  {
    id: "diagnostics",
    name: "진단 · 로그",
    en: "Diagnostics",
    icon: Terminal,
    description: "통신 품질과 명령 응답을 확인하고 문제의 원인을 좁혀보세요.",
  },
];
const cyan = "#57ddcb",
  blue = "#82aaff",
  amber = "#efbd6b";
const modes = [
  { value: "MANUAL", label: "MANUAL" },
  { value: "AUTO", label: "AUTO" },
];
const gears = [
  { value: "FWD", label: "전진 FWD" },
  { value: "REV", label: "후진 REV" },
];
const scenarioDescriptions: Record<string, string> = {
  smoke_test: "구동·조향 기본 통신과 피드백",
  estop_test: "정상 제동 해제 → E-stop → 제동 요청",
  ez_can_timeout: "정상 EZ 피드백 → TX 차단 → 유효성 해제",
  keya_can_timeout: "정상 KEYA 통신 → TX 차단 → timeout",
};
function App() {
  const { state, online, history, send } = useConsole();
  const [page, setPage] = useState("bench");
  const [sidebar, setSidebar] = useState(false);
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
  const section = sections.find((s) => s.id === page)!;
  const observed = (topic: string): Data =>
    online && state.topics[topic]?.fresh ? state.topics[topic].data : {};
  const hil = observed("/hil/status"),
    drive = observed("/drive/status"),
    brake = observed("/brake/status");
  const fresh =
    online && state.connected && !!state.topics["/hil/status"]?.fresh;
  const mine = online && state.control.mine;
  const running = state.scenario?.result === "RUNNING";
  const disabled = !fresh || !mine || busy || running;
  const generator = online && state.generator_fresh ? state.generator : {};
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
    } catch (e) {
      setToast({ text: (e as Error).message, error: true });
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
    } catch (e) {
      setToast({ text: (e as Error).message, error: true });
    } finally {
      setBusy(false);
    }
  }
  const changePage = (id: string) => {
    setPage(id);
    setSidebar(false);
    window.scrollTo(0, 0);
  };
  const link = (bus: "ez" | "keya") => {
    if (!fresh) return { text: "수신 대기", tone: "muted" };
    if ((hil.protocol ?? 1) < 2)
      return { text: "RX age 미지원", tone: "muted" };
    if (hil[`${bus}_busoff`]) return { text: "BUS OFF", tone: "danger" };
    if (!hil[`${bus}_can`]) return { text: "CAN OFF", tone: "warning" };
    if (!hil[`${bus}_rx`] || hil[`${bus}_rx_age_ms`] === 4294967295)
      return { text: "NO RX", tone: "muted" };
    return hil[`${bus}_rx_age_ms`] < 500
      ? { text: "ACTIVE", tone: "good" }
      : { text: "STALE RX", tone: "warning" };
  };
  const boolean = (v: unknown, yes = "ON", no = "OFF") =>
    typeof v === "boolean" ? (v ? yes : no) : "—";
  const age = (v: unknown) =>
    typeof v === "number" && v !== 4294967295 ? `${fmt(v, 0)} ms` : "—";
  const source = (topic: string) => (
    <Source name={topic} sample={state.topics[topic]} online={online} />
  );
  const linkBadge = (bus: "ez" | "keya") => (
    <Badge tone={link(bus).tone}>{link(bus).text}</Badge>
  );
  const estop = hil.estop_dash || hil.estop_side || hil.estop_rf;
  const numeric = (
    field: string,
    label: string,
    key: string,
    min: number,
    max: number,
    unit: string,
    step = 0.1,
  ) => (
    <Numeric
      key={field}
      label={label}
      value={hil[key]}
      min={min}
      max={max}
      step={step}
      unit={unit}
      disabled={disabled}
      onApply={(v) => command(`SET ${field} ${v}`)}
    />
  );
  function download(value: Data, name: string) {
    const url = URL.createObjectURL(
      new Blob([JSON.stringify(value, null, 2)], { type: "application/json" }),
    );
    const a = document.createElement("a");
    a.href = url;
    a.download = name;
    a.click();
    URL.revokeObjectURL(url);
  }
  const productionValues = (enabled: boolean) => ({
    enabled,
    speed_target_mps: kmhToMps(production.speed_kmh),
    steering_target_rad: degToRad(production.steering_deg),
    brake_engage: production.brake_engage,
  });
  const productionPanel = (
    <Panel
      title="주행 명령"
      eyebrow="HOST → GOVERNOR → CONTROLLERS"
      aside={
        <Badge tone={generator.enabled ? "warning" : "muted"}>
          {generator.enabled ? "발행 중" : "DISARMED"}
        </Badge>
      }
    >
      <p className="panel-intro">
        실제 제어 경로에 주행 목표를 공급합니다. 페달 주입과 별도의 입력입니다.
      </p>
      <form
        onSubmit={(e) => {
          e.preventDefault();
          setModal("production");
        }}
      >
        <div className="production-fields">
          <label>
            목표 속도
            <div className="input-with-unit">
              <input
                aria-label="목표 속도"
                type="number"
                min="-180"
                max="180"
                step="0.1"
                required
                disabled={disabled}
                value={production.speed_kmh}
                onChange={(e) => {
                  setProductionEdited(true);
                  setProduction({
                    ...production,
                    speed_kmh: Number(e.target.value),
                  });
                }}
              />
              <span>km/h</span>
            </div>
          </label>
          <label>
            목표 조향각
            <div className="input-with-unit">
              <input
                aria-label="목표 조향각"
                type="number"
                min={-radToDeg(1)}
                max={radToDeg(1)}
                step="any"
                required
                disabled={disabled}
                value={production.steering_deg}
                onChange={(e) => {
                  setProductionEdited(true);
                  setProduction({
                    ...production,
                    steering_deg: Number(e.target.value),
                  });
                }}
              />
              <span>deg</span>
            </div>
          </label>
        </div>
        <Switch
          label="브레이크 요청"
          value={production.brake_engage}
          disabled={disabled}
          onChange={(v) => {
            setProductionEdited(true);
            setProduction({ ...production, brake_engage: v });
          }}
        />
        <div className="actions">
          <button className="primary" disabled={disabled}>
            <Play size={15} />{" "}
            {generator.enabled ? "목표 변경" : "ARM · 명령 발행"}
          </button>
          <button
            type="button"
            disabled={!mine || busy || running}
            onClick={() =>
              act("production", { values: productionValues(false) })
            }
          >
            <Square size={14} /> 발행 해제
          </button>
        </div>
      </form>
      <div className="path-strip">
        Planning 대역
        <ChevronRight size={13} />
        Governor
        <ChevronRight size={13} />
        STM_A / STM_B
      </div>
    </Panel>
  );
  function benchPage() {
    return (
      <>
        <div className="connection-grid">
          {[
            {
              name: "STM_TEST",
              sub: "HIL bridge",
              icon: PlugZap,
              active: fresh,
              text: fresh ? "STATUS 수신" : "수신 대기",
            },
            {
              name: "STM_A",
              sub: "Drive status",
              icon: CircuitBoard,
              active: !!drive.header,
              text: drive.header ? "상태 수신" : "수신 대기",
            },
            {
              name: "STM_B",
              sub: "Brake status",
              icon: CircuitBoard,
              active: !!brake.header,
              text: brake.header ? "상태 수신" : "수신 대기",
            },
            {
              name: "KEYA",
              sub: "CAN command RX",
              icon: Cable,
              active: link("keya").tone === "good",
              text: link("keya").text,
            },
          ].map((item) => (
            <div className="connection-card" key={item.name}>
              <item.icon size={20} />
              <div>
                <strong>{item.name}</strong>
                <small>{item.sub}</small>
              </div>
              <Badge tone={item.active ? "good" : "muted"}>{item.text}</Badge>
            </div>
          ))}
        </div>

        <div className="workbench-links" aria-label="벤치 바로가기">
          <a href="#operator-inputs">입력 · 안전</a>
          <a href="#response-plots">응답 그래프</a>
          <a href="#vehicle-telemetry">차량 보드 관측</a>
          <a href="#feedback-inputs">피드백 설정</a>
          <a href="#reception-timing">수신 주기</a>
        </div>
        <VehicleSummary state={state} online={online} />
        <div id="operator-inputs" className="operator-grid">
          <Panel title="운전자 입력" eyebrow="STM_TEST → GPIO / DAC">
            <Choice
              label="운전 모드"
              options={modes}
              value={hil.mode}
              disabled={disabled}
              onChange={(v) => command(`SET MODE ${v}`)}
            />
            <Choice
              label="주행 방향"
              options={gears}
              value={hil.gear}
              disabled={disabled}
              onChange={(v) =>
                command(`SET GEAR ${v === "FWD" ? "FORWARD" : "REVERSE"}`)
              }
            />
            <div className="pedal-heading">
              <label htmlFor="pedal">가속 페달</label>
              <strong>
                {fmt(pedal, 0)}
                <small>%</small>
              </strong>
            </div>
            <input
              id="pedal"
              aria-label="가속 페달"
              className="pedal-slider"
              type="range"
              min="0"
              max="100"
              step="1"
              value={pedal}
              disabled={disabled}
              onPointerDown={() => setPedalEditing(true)}
              onChange={(e) => {
                setPedalEditing(true);
                setPedal(Number(e.target.value));
              }}
              onPointerUp={(e) => {
                command(`SET PEDAL ${e.currentTarget.value}`);
                setPedalEditing(false);
              }}
              onKeyUp={(e) => {
                if (
                  [
                    "ArrowLeft",
                    "ArrowRight",
                    "Home",
                    "End",
                    "PageUp",
                    "PageDown",
                    "ArrowUp",
                    "ArrowDown",
                  ].includes(e.key)
                ) {
                  command(`SET PEDAL ${e.currentTarget.value}`);
                  setPedalEditing(false);
                }
              }}
              onPointerCancel={() => setPedalEditing(false)}
            />
            <div className="range-labels">
              <span>0%</span>
              <span>놓으면 한 번 전송</span>
              <span>100%</span>
            </div>
            <div className="preset-row">
              {[0, 10, 30].map((v) => (
                <button
                  key={v}
                  disabled={disabled}
                  onClick={() => command(`SET PEDAL ${v}`)}
                >
                  {v === 0 ? "페달 해제" : `${v}%`}
                </button>
              ))}
              <button
                disabled={disabled}
                onClick={() => commands(["SET MODE AUTO", "SET GEAR FORWARD"])}
              >
                AUTO + 전진
              </button>
            </div>
            {source("/hil/status")}
          </Panel>
          {productionPanel}
          <Panel
            title="E-stop · 브레이크"
            eyebrow="INJECTION / PWM OBSERVATION"
            aside={
              <Badge tone={!fresh ? "muted" : estop ? "danger" : "good"}>
                {!fresh ? "UNKNOWN" : estop ? "E-STOP" : "CLEAR"}
              </Badge>
            }
          >
            {[
              ["DASH", "대시보드", "estop_dash"],
              ["SIDE", "차체 측면", "estop_side"],
              ["RF", "무선 리모컨", "estop_rf"],
            ].map(([key, label, field]) => (
              <Switch
                key={key}
                label={`${label} E-stop`}
                value={hil[field]}
                disabled={disabled}
                danger
                onChange={(value) =>
                  command(`SET ESTOP ${key} ${value ? "ON" : "OFF"}`)
                }
              />
            ))}
            <div className="telemetry-group">
              <Fact
                label="STM_B 제동 요청"
                value={boolean(brake.brake_commanded, "제동", "해제")}
              />
              <Fact label="관측한 PWM 판정" value={hil.brake ?? "—"} />
              <Fact
                label="PWM 펄스 폭 / 주기"
                value={`${fmt(hil.brake_us, 0)} / ${fmt(hil.brake_period_us, 0)}`}
                unit="μs"
              />
              <Fact
                label="PWM 신호"
                value={boolean(hil.pwm_lost, "LOST", "수신 중")}
              />
              <Fact
                label="정지 사유"
                value={brakeReasons(brake.stop_reason_bits)}
              />
            </div>
            <Details title="Safe State와 실제 피드백">
              <p className="muted">
                Safe State는 명령 발행을 끄고 기본 입력을 큐에 넣습니다. E-stop
                해제도 포함하며 물리 비상정지 버튼과 용도가 다릅니다.
              </p>
              <p className="muted">
                브레이크에는 실제 위치 피드백이 없습니다. PWM은 전기 신호의
                관측값입니다.
              </p>
              <button
                className="safe-button"
                disabled={!mine || busy}
                onClick={() => act("safe")}
              >
                <RotateCcw size={16} /> Safe State 적용
              </button>
            </Details>
          </Panel>
        </div>
        <FrameHealth state={state} online={online} />
        <div id="response-plots" className="grid-two">
          <Panel
            title="구동 · 속도 응답"
            eyebrow="GOVERNOR TARGET / STM_A FEEDBACK"
            aside={linkBadge("ez")}
          >
            <Chart
              points={points}
              traces={[
                { key: "targetSpeed", label: "Governor 출력", color: blue },
                { key: "speed", label: "STM_A 피드백", color: cyan },
              ]}
              unit="km/h"
            />
            {source("/drive/status")}
          </Panel>
          <Panel
            title="조향 · 로드휠 응답"
            eyebrow="LATERAL CONTROLLER"
            aside={linkBadge("keya")}
          >
            <Chart
              points={points}
              traces={[
                { key: "targetWheel", label: "제한 적용 명령각", color: blue },
                { key: "wheel", label: "피드백 조향각", color: cyan },
              ]}
              unit="deg"
            />
            {source("/steering/status")}
          </Panel>
        </div>
        <div id="vehicle-telemetry" className="section-label">
          <h2>차량 보드 관측</h2>
          <span>vehicle_dashboard의 명령 · 구동 · 조향 · 브레이크 진단</span>
        </div>
        <VehicleTelemetry state={state} online={online} />
        <div id="feedback-inputs" className="section-label">
          <h2>STM_TEST 에뮬레이터</h2>
          <span>보드로 보내는 피드백과 보드에서 받은 명령을 구분합니다</span>
        </div>
        <div className="grid-two">
          <Panel title="에뮬레이터 응답 설정" eyebrow="STM_TEST → CAN FEEDBACK">
            <Choice
              label="응답 방식"
              options={[
                { value: "closed", label: "폐루프 시뮬레이션" },
                { value: "direct", label: "직접 피드백" },
              ]}
              value={
                typeof hil.plant === "boolean"
                  ? hil.plant
                    ? "closed"
                    : "direct"
                  : null
              }
              disabled={disabled || hil.protocol < 2}
              onChange={(v) =>
                v === "direct"
                  ? commands(["SET PLANT OFF", `SET RPM ${hil.rpm ?? 0}`])
                  : command("SET PLANT ON")
              }
            />
            <p className="muted">
              폐루프에서는 STM_A 명령에 따라 모터 응답이 계산됩니다.
            </p>
            <fieldset disabled={disabled || hil.plant !== false}>
              <Numeric
                label="속도 피드백"
                value={hil.speed}
                min={-200}
                max={200}
                step={0.1}
                unit="km/h"
                disabled={disabled || hil.plant !== false}
                onApply={(v) => command(`SET SPEED ${v}`)}
              />
              <Numeric
                label="RPM 피드백"
                value={hil.rpm}
                min={-32000}
                max={32000}
                step={1}
                unit="RPM"
                disabled={disabled || hil.plant !== false}
                onApply={(v) => command(`SET RPM ${v}`)}
              />
            </fieldset>
            <Details title="전압 · 전류 · 온도 주입">
              {numeric("VOLTAGE", "DC 전압", "voltage", 0, 300, "V")}
              {numeric(
                "CURRENT",
                "모터 전류 피드백",
                "current",
                -3200,
                3200,
                "A",
              )}
              {numeric("MOTOR_TEMP", "모터 온도", "motor_temp", -40, 210, "°C")}
            </Details>
          </Panel>
          <Panel title="KEYA 모터축 피드백" eyebrow="STM_TEST / MOTOR SHAFT">
            {numeric(
              "STEER_POS",
              "모터축 초기 위치",
              "steer",
              -1800,
              1800,
              "deg",
              0.01,
            )}
            <Notice>
              모터축 각도와 로드휠 조향각은 다른 물리량입니다. Enable 상태에서는
              수신한 위치 명령을 다시 추종합니다.
            </Notice>
            <Fact label="KEYA Enable 명령" value={boolText(hil.keya_enabled)} />
            <Fact
              label="모터축 관측 / 수신 목표"
              value={`${fmt(hil.steer)} / ${fmt(hil.steer_target)}`}
              unit="deg"
            />
            <Chart
              points={points}
              traces={[
                { key: "targetSteer", label: "수신 모터축 목표", color: blue },
                { key: "steer", label: "에뮬레이터 응답", color: cyan },
              ]}
              unit="motor deg"
            />
            {source("/hil/status")}
          </Panel>
        </div>
        <div className="grid-two">
          <Panel title="EZ 명령 · RPM 응답" eyebrow="STM_A → STM_TEST">
            <Chart
              points={points}
              traces={[
                { key: "targetRpm", label: "Plant 목표 RPM", color: blue },
                { key: "rpm", label: "STM_TEST 출력 RPM", color: cyan },
              ]}
              unit="RPM"
            />
            <div className="inline-facts">
              <Fact label="RUN 명령" value={boolText(hil.ez_running)} />
              <Fact
                label="제어 방식"
                value={boolean(hil.ez_speed_control, "속도 제어", "토크 제어")}
              />
              <Fact
                label="수신 RPM 명령"
                value={fmt(hil.ez_command_rpm, 0)}
                unit="RPM"
              />
              <Fact
                label="수신 상전류 명령"
                value={fmt(hil.ez_command_current)}
                unit="A"
              />
              <Fact label="EZ RX age" value={age(hil.ez_rx_age_ms)} />
              <Fact
                label="EZ RX / TX"
                value={`${fmt(hil.ez_rx, 0)} / ${fmt(hil.ez_tx, 0)}`}
              />
              <Fact label="KEYA RX age" value={age(hil.keya_rx_age_ms)} />
              <Fact
                label="KEYA RX / TX"
                value={`${fmt(hil.keya_rx, 0)} / ${fmt(hil.keya_tx, 0)}`}
              />
            </div>
          </Panel>
          <Panel title="전류 응답" eyebrow="STM_A / CAN PART I">
            <Chart
              points={points}
              traces={[
                { key: "bus", label: "배터리 버스 전류", color: amber },
                { key: "phase", label: "모터 상전류", color: cyan },
              ]}
              unit="A"
            />
            {source("/drive/status")}
          </Panel>
        </div>
        <Panel title="ROS 수신 주기" eyebrow="HOST RECEPTION">
          <div id="reception-timing">
            <ReceptionTable state={state} online={online} />
          </div>
        </Panel>
        <Panel
          title="최근 이벤트"
          aside={
            <button
              className="text-button"
              onClick={() => changePage("diagnostics")}
            >
              모두 보기
              <ArrowRight size={15} />
            </button>
          }
        >
          <EventList events={state.events.slice(0, 4)} />
        </Panel>
      </>
    );
  }
  function faultsPage() {
    return (
      <>
        <Notice>
          CAN OFF는 인터페이스 동작을 끄고, TX DROP은 에뮬레이터의 송신을
          차단합니다. 초기화 성공과 실제 프레임 수신은 별개입니다.
        </Notice>
        <div className="grid-two">
          {(["ez", "keya"] as const).map((bus) => (
            <Panel
              key={bus}
              title={bus === "ez" ? "EZkontrol · 구동" : "KEYA · 조향"}
              eyebrow={bus === "ez" ? "CAN 1" : "CAN 2"}
              aside={linkBadge(bus)}
            >
              <Switch
                label={`${bus.toUpperCase()} CAN 활성화`}
                description="에뮬레이터 CAN 인터페이스"
                value={hil[`${bus}_can`]}
                disabled={disabled}
                onChange={(v) =>
                  command(`SET ${bus.toUpperCase()}_CAN ${v ? "ON" : "OFF"}`)
                }
              />
              <Switch
                label={`${bus.toUpperCase()} TX DROP`}
                description="송신 프레임 차단 · 아래 값은 bridge 요청 상태"
                value={state.desired[`${bus}_drop_tx`]}
                disabled={disabled}
                danger
                onChange={(v) =>
                  command(
                    `FAULT ${bus.toUpperCase()} DROP_TX ${v ? "ON" : "OFF"}`,
                  )
                }
              />
              <Switch
                label={`${bus.toUpperCase()} 컨트롤러 고장`}
                description="피드백의 fault 상태 주입"
                value={hil[`${bus}_fault`]}
                disabled={disabled}
                danger
                onChange={(v) =>
                  command(`FAULT ${bus.toUpperCase()} ${v ? "ON" : "OFF"}`)
                }
              />
              <div className="subsection">
                <span className="eyebrow">관측한 통신 상태</span>
                <Fact
                  label="수신 / 송신 프레임"
                  value={`${fmt(hil[`${bus}_rx`], 0)} / ${fmt(hil[`${bus}_tx`], 0)}`}
                />
                <Fact label="마지막 RX" value={age(hil[`${bus}_rx_age_ms`])} />
                <Fact
                  label="Bus-off"
                  value={
                    hil.protocol >= 3
                      ? boolean(hil[`${bus}_busoff`])
                      : "Protocol 3부터 지원"
                  }
                />
              </div>
            </Panel>
          ))}
        </div>
        <div className="action-panel">
          <div>
            <h3>주입한 고장 해제</h3>
            <p>양쪽 TX DROP과 fault를 해제하고 CAN을 다시 활성화합니다.</p>
          </div>
          <button
            disabled={disabled}
            onClick={() =>
              commands([
                "FAULT EZ DROP_TX OFF",
                "FAULT KEYA DROP_TX OFF",
                "FAULT EZ OFF",
                "FAULT KEYA OFF",
                "SET EZ_CAN ON",
                "SET KEYA_CAN ON",
              ])
            }
          >
            <RotateCcw size={16} /> 고장 주입 해제
          </button>
        </div>
        <Panel title="고장 전후 주행 응답">
          <Chart
            points={points}
            traces={[
              { key: "targetSpeed", label: "Governor 출력", color: blue },
              { key: "speed", label: "STM_A 피드백", color: cyan },
            ]}
            unit="km/h"
          />
          <Notice>
            피드백이 무효해지면 그래프가 끊깁니다. EZ 피드백 timeout 자체가
            구동을 차단하는지는 production 정책을 따릅니다.
          </Notice>
        </Panel>
      </>
    );
  }
  function scenariosPage() {
    const definition = state.catalog.find((s) => s.id === scenarioId);
    const run = state.scenario;
    return (
      <>
        <div className="grid-two scenario-layout">
          <Panel title="시험 라이브러리" eyebrow="HIL SCENARIOS">
            <div className="scenario-list">
              {state.catalog.length === 0 ? (
                <div className="empty">서버 연결 후 시나리오를 불러옵니다.</div>
              ) : (
                state.catalog.map((item) => (
                  <button
                    key={item.id}
                    className={scenarioId === item.id ? "active" : ""}
                    onClick={() => {
                      setScenarioId(item.id);
                      setScenarioArm(false);
                    }}
                  >
                    <FlaskConical size={19} />
                    <div>
                      <strong>{item.name}</strong>
                      <p>{scenarioDescriptions[item.id] ?? "HIL scenario"}</p>
                    </div>
                    <ChevronRight size={16} />
                  </button>
                ))
              )}
            </div>
          </Panel>
          <Panel
            title={definition?.name ?? "시나리오 선택"}
            eyebrow="TEST SEQUENCE"
            aside={<Badge>{definition?.steps.length ?? 0} STEPS</Badge>}
          >
            <div className="timeline">
              {definition?.steps.map((step: Data, i: number) => (
                <div className="timeline-step" key={i}>
                  <span className="timeline-index">
                    {String(i + 1).padStart(2, "0")}
                  </span>
                  <div>
                    <strong>{fmt(step.at, 1)}초</strong>
                    {[
                      ["hil", "입력"],
                      ["fault", "고장"],
                      ["command", "주행 명령"],
                    ].map(([key, label]) =>
                      Object.keys(step[key]).length > 0 ? (
                        <p key={key}>
                          <span>{label}</span>{" "}
                          {Object.entries(step[key])
                            .map(([k, v]) => `${k}=${scenarioValue(k, v)}`)
                            .join(", ")}
                        </p>
                      ) : null,
                    )}
                  </div>
                </div>
              ))}
            </div>
            <Details
              title={`관측 조건 ${definition?.expectations.length ?? 0}개`}
            >
              {definition?.expectations.map((e: Data, i: number) => (
                <Fact
                  key={i}
                  label={`${e.topic} · ${e.field}`}
                  value={
                    e.equals !== null
                      ? scenarioValue(e.field, e.equals)
                      : e.bitmask !== null
                        ? `bits 0x${e.bitmask.toString(16)}`
                        : `${e.minimum === null ? "−∞" : scenarioValue(e.field, e.minimum)} ~ ${e.maximum === null ? "∞" : scenarioValue(e.field, e.maximum)}`
                  }
                />
              ))}
            </Details>
            <div className="scenario-arm">
              <label>
                <input
                  type="checkbox"
                  disabled={disabled}
                  checked={scenarioArm}
                  onChange={(e) => setScenarioArm(e.target.checked)}
                />{" "}
                이 시험의 주행 명령 발행을 허용합니다
              </label>
              <div className="actions">
                <button
                  className="primary"
                  disabled={disabled || !scenarioArm || !definition}
                  onClick={() => setModal("scenario")}
                >
                  <Play size={15} /> 시나리오 실행
                </button>
                <button
                  disabled={!mine || !running || busy}
                  onClick={() => act("stop_scenario")}
                >
                  <Square size={14} /> 중단 · Safe State
                </button>
              </div>
            </div>
          </Panel>
        </div>
        <Panel
          title="실행 결과"
          eyebrow="OBSERVED RESULTS"
          aside={
            run && (
              <div className="actions">
                <Badge
                  tone={
                    run.result === "PASS"
                      ? "good"
                      : run.result === "FAIL"
                        ? "danger"
                        : run.result === "RUNNING"
                          ? "warning"
                          : "muted"
                  }
                >
                  {run.result}
                </Badge>
                <button
                  className="icon-button"
                  aria-label="현재 결과 다운로드"
                  onClick={() => download(run, `${run.scenario}.json`)}
                >
                  <Download size={16} />
                </button>
              </div>
            )
          }
        >
          {!run ? (
            <div className="empty">
              <ClipboardList size={32} />
              <h3>아직 실행한 시험이 없습니다</h3>
              <p>
                실행하면 실제 관측값을 기준으로 각 조건의 통과 여부를
                표시합니다.
              </p>
            </div>
          ) : (
            <>
              <div className="run-heading">
                <strong>{run.scenario}</strong>
                <span>
                  {fmt(run.duration_sec, 1)}s{" "}
                  {running
                    ? `· ${run.step_index} / ${run.step_count} 단계`
                    : ""}
                </span>
              </div>
              {running && (
                <div className="progress">
                  <span
                    style={{
                      width: `${(run.step_index / Math.max(1, run.step_count)) * 100}%`,
                    }}
                  />
                </div>
              )}
              <div className="result-list">
                {run.records.map((record: Data, i: number) => (
                  <div className="result-row" key={i}>
                    <span
                      className={`result-icon ${record.result === "FAIL" ? "danger" : "good"}`}
                    >
                      {record.result === "FAIL" ? (
                        <X size={15} />
                      ) : (
                        <Check size={15} />
                      )}
                    </span>
                    <div>
                      <strong>
                        {record.kind === "step"
                          ? `Step ${record.index + 1} · 명령 전송`
                          : `${record.topic} · ${record.field}`}
                      </strong>
                      <p>
                        {record.detail ??
                          record.hil_commands?.join(" / ") ??
                          ""}
                      </p>
                    </div>
                    <span>
                      {record.kind === "step" ? "전송됨" : record.result}
                    </span>
                    <small>{fmt(record.latency_ms, 0)} ms</small>
                  </div>
                ))}
              </div>
              {run.infrastructure_failures?.map((e: string, i: number) => (
                <div className="inline-error" key={i}>
                  {e}
                </div>
              ))}
              {running && (
                <p className="muted">
                  관측 조건의 실제 응답을 기다리고 있습니다…
                </p>
              )}
            </>
          )}
        </Panel>
        <Panel title="이전 시험 기록" eyebrow="SAVED RESULTS">
          {past.length === 0 ? (
            <p className="muted">저장된 결과가 없습니다.</p>
          ) : (
            past.map((result, i) => (
              <div className="history-row" key={i}>
                <div>
                  <strong>{result.scenario}</strong>
                  <small>
                    {result.started_at
                      ? new Date(result.started_at).toLocaleString("ko-KR")
                      : ""}
                  </small>
                </div>
                <Badge tone={result.result === "PASS" ? "good" : "danger"}>
                  {result.result}
                </Badge>
                <span>{fmt(result.duration_sec, 1)}s</span>
                <button
                  className="icon-button"
                  aria-label={`${result.scenario} 결과 다운로드`}
                  onClick={() => download(result, `${result.scenario}.json`)}
                >
                  <Download size={16} />
                </button>
              </div>
            ))
          )}
        </Panel>
      </>
    );
  }
  function diagnosticsPage() {
    return (
      <>
        <div className="grid-two">
          {(["ez", "keya"] as const).map((bus) => (
            <Panel
              key={bus}
              title={`${bus.toUpperCase()} CAN 진단`}
              eyebrow={bus === "ez" ? "TRACTION" : "STEERING"}
              aside={linkBadge(bus)}
            >
              <Fact
                label="Peripheral 초기화"
                value={boolean(hil[`${bus}_ok`], "OK", "실패")}
              />
              <Fact
                label="RX / TX"
                value={`${fmt(hil[`${bus}_rx`], 0)} / ${fmt(hil[`${bus}_tx`], 0)}`}
              />
              <Fact label="RX age" value={age(hil[`${bus}_rx_age_ms`])} />
              <Fact
                label="TEC / REC"
                value={
                  hil.protocol >= 3
                    ? `${hil[`${bus}_tec`]} / ${hil[`${bus}_rec`]}`
                    : "미지원"
                }
              />
              <Fact
                label="Bus-off / Error passive"
                value={
                  hil.protocol >= 3
                    ? `${boolean(hil[`${bus}_busoff`])} / ${boolean(hil[`${bus}_errpass`])}`
                    : "미지원"
                }
              />
              <Fact
                label="Error warning"
                value={
                  hil.protocol >= 3 ? boolean(hil[`${bus}_warn`]) : "미지원"
                }
              />
            </Panel>
          ))}
        </div>
        <Panel title="ROS 수신 상태" eyebrow="TELEMETRY SOURCES">
          <ReceptionTable state={state} online={online} />
          <Details title="Bridge와 제어기 diagnostics">
            {observed("/diagnostics").status?.map((item: Data, i: number) => (
              <div className="diagnostic-item" key={i}>
                <h3>{item.name}</h3>
                <p>{item.message}</p>
                {item.values?.map((v: Data) => (
                  <Fact key={v.key} label={v.key} value={v.value} />
                ))}
              </div>
            )) ?? <p className="muted">diagnostics 수신 대기</p>}
          </Details>
        </Panel>
        <Panel
          title="명령 · 응답 로그"
          eyebrow="EVENT STREAM"
          aside={
            <div className="actions">
              <select
                aria-label="로그 필터"
                value={logFilter}
                onChange={(e) => setLogFilter(e.target.value)}
              >
                <option value="all">전체</option>
                <option value="command">명령</option>
                <option value="response">응답</option>
                <option value="error">오류</option>
              </select>
              <button
                className="icon-button"
                aria-label="로그 다운로드"
                onClick={() =>
                  download({ events: state.events }, "hil-events.json")
                }
              >
                <Download size={16} />
              </button>
            </div>
          }
        >
          <EventList
            events={state.events.filter(
              (e) => logFilter === "all" || e.kind === logFilter,
            )}
          />
        </Panel>
      </>
    );
  }
  return (
    <div className="app-shell">
      <aside className={`sidebar ${sidebar ? "open" : ""}`}>
        <a
          className="brand"
          href="#"
          onClick={(e) => {
            e.preventDefault();
            changePage("bench");
          }}
        >
          <div className="brand-mark">K</div>
          <div>
            <strong>
              KAI<span>EV26</span>
            </strong>
            <small>HIL OPERATOR CONSOLE</small>
          </div>
        </a>
        <div className="workspace-label">
          <span className="tiny-square" />
          HARDWARE IN THE LOOP
        </div>
        <nav aria-label="주 메뉴">
          {sections.map((item) => (
            <button
              key={item.id}
              className={page === item.id ? "active" : ""}
              onClick={() => changePage(item.id)}
            >
              <item.icon size={19} />
              <span>{item.name}</span>
              {page === item.id && <i />}
              {item.id === "scenarios" && running && (
                <span className="running-dot" />
              )}
            </button>
          ))}
        </nav>
        <div className="sidebar-bottom">
          <div className="bench-label">
            <CircuitBoard size={19} />
            <div>
              <strong>KAIEV26 Bench</strong>
              <small>ROS 2 · STM_TEST</small>
            </div>
          </div>
          <div className="sidebar-connection">
            <span className={`status-dot ${online ? "live" : ""}`} />
            {online ? "Web bridge 연결됨" : "Web bridge 연결 대기"}
          </div>
          <span className="version">
            LOCAL CONSOLE <span>v0.1</span>
          </span>
        </div>
      </aside>
      {sidebar && (
        <button
          aria-label="메뉴 닫기"
          className="sidebar-backdrop"
          onClick={() => setSidebar(false)}
        />
      )}
      <div className="main-shell">
        <header className="topbar">
          <div className="breadcrumb">
            <button
              className="icon-button mobile-menu"
              aria-label="메뉴 열기"
              onClick={() => setSidebar(true)}
            >
              <Menu size={20} />
            </button>
            <span>Workspace</span>
            <ChevronRight size={13} />
            <strong>HIL Bench</strong>
          </div>
          <div className="topbar-actions">
            <Badge tone={online ? "good" : "muted"}>
              {online ? "LIVE CONNECTION" : "OFFLINE"}
            </Badge>
            <span className="divider" />
            <button
              disabled={!online || busy || (state.control.occupied && !mine)}
              className={mine ? "control-owned" : ""}
              onClick={() => act(mine ? "release" : "claim")}
            >
              <LockKeyhole size={14} />
              {mine
                ? "조작권 해제"
                : state.control.occupied
                  ? "다른 브라우저 조작 중"
                  : "조작권 가져오기"}
            </button>
            <button
              className="safe-button"
              disabled={!mine || busy}
              onClick={() => act("safe")}
            >
              <RotateCcw size={15} /> Safe State
            </button>
          </div>
        </header>
        <main>
          <div className="page-heading">
            <div>
              <span className="eyebrow">{section.en.toUpperCase()}</span>
              <h1>{section.name}</h1>
              <p>{section.description}</p>
            </div>
            <div className="page-tools">
              <button
                className={paused ? "warning-text" : ""}
                onClick={() => setPaused(paused ? null : [...history])}
              >
                {paused ? <Play size={14} /> : <Pause size={14} />}그래프{" "}
                {paused ? "재개" : "일시정지"}
              </button>
              <button
                className="icon-button"
                aria-label="보드 상태 새로고침"
                disabled={!mine || busy}
                onClick={() => act("refresh")}
              >
                <RefreshCw size={16} />
              </button>
            </div>
          </div>
          {(!online || !fresh) && (
            <div className="connection-notice">
              <Unplug size={18} />
              <div>
                <strong>
                  {!online
                    ? "Web 서버에 연결하고 있습니다"
                    : "STM_TEST 상태를 기다리고 있습니다"}
                </strong>
                <p>
                  {!online
                    ? "서버 주소와 실행 상태를 확인하세요. 연결되면 자동으로 갱신됩니다."
                    : "HIL bridge와 보드 연결을 확인하세요. 수신하지 않은 값은 — 로 표시합니다."}
                </p>
              </div>
              <span>{!online ? "RECONNECTING" : "NO TELEMETRY"}</span>
            </div>
          )}
          {online && fresh && !mine && (
            <div className="view-notice">
              <LockKeyhole size={15} />
              현재 모니터링 모드입니다. 입력을 변경하려면 상단에서 조작권을
              가져오세요.
            </div>
          )}
          {running && page !== "scenarios" && (
            <button
              className="running-banner"
              onClick={() => changePage("scenarios")}
            >
              <FlaskConical size={18} />
              <strong>{state.scenario?.scenario}</strong> 시험 실행 중 · 수동
              입력 잠김
              <ArrowRight size={16} />
            </button>
          )}
          <div className="page-content">
            {page === "bench" ? (
              benchPage()
            ) : page === "display" ? (
              <VirtualDisplay display={state.display} online={online} />
            ) : page === "faults" ? (
              faultsPage()
            ) : page === "scenarios" ? (
              scenariosPage()
            ) : (
              diagnosticsPage()
            )}
          </div>
          <footer>
            <span>
              <span className={`status-dot ${fresh ? "live" : ""}`} />
              {fresh
                ? `STM_TEST protocol ${hil.protocol}`
                : "STM_TEST 상태 미수신"}
            </span>
            <span>관측값 기준 표시 · 5 Hz 화면 갱신</span>
            <span>{mine ? "TEST CONTROL" : "VIEW ONLY"}</span>
          </footer>
        </main>
      </div>
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
          {toast.error ? <TriangleAlert size={19} /> : <Check size={19} />}
          <span>{toast.text}</span>
          <button
            className="icon-button"
            aria-label="알림 닫기"
            onClick={() => setToast(null)}
          >
            <X size={16} />
          </button>
        </div>
      )}
      {modal && (
        <div className="modal-backdrop" onClick={() => setModal(null)}>
          <section
            className="modal"
            role="dialog"
            aria-modal="true"
            aria-labelledby="arm-title"
            onClick={(e) => e.stopPropagation()}
            onKeyDown={(e) => {
              if (e.key === "Escape") setModal(null);
            }}
          >
            <div className="modal-icon">
              <Play size={24} />
            </div>
            <span className="eyebrow">COMMAND SOURCE</span>
            <h2 id="arm-title">
              {modal === "scenario"
                ? "시나리오를 실행합니다"
                : "주행 명령을 발행합니다"}
            </h2>
            <p>
              {modal === "scenario"
                ? scenarioId
                : `속도 ${production.speed_kmh} km/h · 조향 ${production.steering_deg}° · 브레이크 ${production.brake_engage ? "ON" : "OFF"}`}
            </p>
            <Notice>
              연결된 HIL 벤치에 입력을 보냅니다. 브라우저 조작 연결이 끊기면
              명령 발행이 해제됩니다.
            </Notice>
            <div className="actions">
              <button autoFocus onClick={() => setModal(null)}>
                취소
              </button>
              <button
                className="primary"
                onClick={() => {
                  const kind = modal;
                  setModal(null);
                  if (kind === "scenario") {
                    setScenarioArm(false);
                    act("scenario", { name: scenarioId, arm: true });
                  } else
                    act("production", {
                      values: productionValues(true),
                    });
                }}
              >
                <Play size={15} />{" "}
                {modal === "scenario" ? "ARM · 시험 시작" : "ARM · 발행"}
              </button>
            </div>
          </section>
        </div>
      )}
    </div>
  );
}
function EventList({
  events,
}: {
  events: { id: number; time: number; kind: string; text: string }[];
}) {
  return events.length ? (
    <div className="event-list">
      {events.map((event) => (
        <div
          className={`event-row ${event.kind === "error" ? "error" : ""}`}
          key={event.id}
        >
          <span className="event-time">
            {new Date(event.time * 1000).toLocaleTimeString("ko-KR", {
              hour12: false,
            })}
          </span>
          <span className="event-kind">{event.kind}</span>
          <span>{event.text}</span>
        </div>
      ))}
    </div>
  ) : (
    <div className="empty compact">
      <Activity size={21} />
      <p>연결 및 명령 이벤트가 여기에 표시됩니다.</p>
    </div>
  );
}

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
