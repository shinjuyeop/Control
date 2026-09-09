import {
  Activity,
  ArrowDownRight,
  ArrowLeft,
  ArrowUpRight,
  Check,
  ChevronRight,
  CircleGauge,
  Cpu,
  FlaskConical,
  Gauge,
  Radio,
  ShieldCheck,
  SlidersHorizontal,
  WifiOff,
} from "lucide-react";
import { useState, type ReactNode } from "react";
import { Chart } from "../components/instruments";
import type { Point } from "../domain/types";
import "./preview.css";

type Scene = "tracking" | "estop" | "canlost";
type Theme = "pitwall" | "rack";
const scenes: { value: Scene; label: string }[] = [
  { value: "tracking", label: "정상 주행" },
  { value: "estop", label: "E-stop" },
  { value: "canlost", label: "CAN 지연" },
];

function Module({
  id,
  title,
  aside,
  children,
  className = "",
}: {
  id: string;
  title: string;
  aside?: ReactNode;
  children: ReactNode;
  className?: string;
}) {
  return (
    <section className={`concept-module ${className}`} id={`concept-${id}`}>
      <header>
        <h2>
          <span>{id}</span>
          {title}
        </h2>
        {aside}
        <i className="module-fastener" />
      </header>
      {children}
    </section>
  );
}

function VehicleDiagram({ scene }: { scene: Scene }) {
  return (
    <svg
      className={`concept-vehicle ${scene}`}
      viewBox="0 0 300 340"
      role="img"
      aria-label="차량 상면도 예시: 앞바퀴 조향, 후륜 구동, 제동 경로"
    >
      <defs>
        <pattern
          id="concept-grid"
          width="20"
          height="20"
          patternUnits="userSpaceOnUse"
        >
          <path
            d="M20 0H0V20"
            fill="none"
            stroke="currentColor"
            strokeWidth=".5"
          />
        </pattern>
      </defs>
      <rect width="300" height="340" fill="url(#concept-grid)" opacity=".12" />
      <path
        d="M150 5V334M5 170H295"
        stroke="currentColor"
        strokeDasharray="3 7"
        opacity=".2"
      />
      <g className="vehicle-outline" fill="none" strokeWidth="1.5">
        <path d="M118 26Q150 16 182 26L198 67L203 260Q200 284 184 300H116Q100 284 97 260L102 67Z" />
        <path d="M119 110L127 78H173L181 110V216L170 239H130L119 216Z" />
        <path d="M124 124L134 114H166L177 124M122 206H179M127 140H173V189H127Z" />
        <path d="M76 76H224M76 260H224M150 80V256" opacity=".6" />
      </g>
      <g className="vehicle-wheels" strokeWidth="2">
        <rect
          x="65"
          y="50"
          width="23"
          height="54"
          rx="4"
          transform="rotate(-8 76 77)"
        />
        <rect
          x="212"
          y="50"
          width="23"
          height="54"
          rx="4"
          transform="rotate(-8 223 77)"
        />
        <rect x="65" y="234" width="23" height="54" rx="4" />
        <rect x="212" y="234" width="23" height="54" rx="4" />
      </g>
      <g className="vehicle-circuit" fill="none" strokeWidth="3">
        <path d="M91 76H210M89 260H212" />
        <rect x="132" y="248" width="36" height="24" rx="2" />
      </g>
      <circle cx="150" cy="166" r="13" fill="var(--p-accent)" opacity=".12" />
      <circle cx="150" cy="166" r="4" fill="var(--p-accent)" />
      <text x="150" y="43" textAnchor="middle">
        FRONT
      </text>
      <text x="150" y="325" textAnchor="middle">
        REAR DRIVE
      </text>
      <path
        d="M39 137V199M33 143L39 137L45 143"
        fill="none"
        stroke="var(--p-accent)"
      />
    </svg>
  );
}

export function DesignPreview() {
  const [theme, setTheme] = useState<Theme>(() =>
    new URLSearchParams(location.search).get("preview") === "rack"
      ? "rack"
      : "pitwall",
  );
  const [scene, setScene] = useState<Scene>("tracking");
  const [mode, setMode] = useState("AUTO");
  const [gear, setGear] = useState("FWD");
  const [pedal, setPedal] = useState(32);
  const stopped = scene === "estop",
    lost = scene === "canlost";
  const speed = lost ? "—" : stopped ? "0.0" : "35.8";
  const points: Point[] = Array.from({ length: 61 }, (_, i) => {
    const ramp = Math.min(1, i / 15);
    const stopRamp = stopped ? Math.max(0, 1 - Math.max(0, i - 39) / 13) : 1;
    return {
      time: i,
      targetSpeed: stopped && i >= 40 ? 0 : 36,
      speed:
        lost && i > 48
          ? null
          : Math.max(0, (35.8 * ramp + Math.sin(i * 0.6) * 0.35) * stopRamp),
      targetWheel: stopped && i >= 40 ? 0 : 5,
      wheel: stopped ? 5 * stopRamp : 4.8 + Math.sin(i * 0.4) * 0.15,
      rpm: null,
      targetRpm: null,
      phase: null,
      bus: null,
      steer: null,
      targetSteer: null,
    };
  });
  const selectTheme = (next: Theme) => {
    setTheme(next);
    const url = new URL(location.href);
    url.searchParams.set("preview", next);
    history.replaceState(null, "", url);
  };
  return (
    <div className={`design-preview ${theme}`}>
      <div className="concept-toolbar">
        <div>
          <span className="concept-preview-tag">
            <FlaskConical size={15} /> DESIGN STUDY
          </span>
          <strong>디자인 시안</strong>
          <span>예시 데이터 · 실제 보드 명령 없음</span>
        </div>
        <div
          className="concept-theme-tabs"
          role="group"
          aria-label="디자인 선택"
        >
          <button
            aria-pressed={theme === "pitwall"}
            onClick={() => selectTheme("pitwall")}
          >
            <span>01</span> Pit Wall
          </button>
          <button
            aria-pressed={theme === "rack"}
            onClick={() => selectTheme("rack")}
          >
            <span>02</span> Control Rack
          </button>
        </div>
        <a href="/" target="_blank" rel="noreferrer">
          <ArrowLeft size={15} /> 운용 화면
        </a>
      </div>
      <header className="concept-header">
        <a className="concept-brand" href="#concept-overview">
          <i />
          <b>KAI</b>
          <span>
            VEHICLE SYSTEMS
            <br />
            <strong>EV26 / HIL LAB</strong>
          </span>
        </a>
        <div className="concept-session">
          <span>WORKSPACE / 01</span>
          <strong>VEHICLE DYNAMICS</strong>
        </div>
        <div className="concept-output">
          <i /> DESIGN PREVIEW <span>명령 출력 없음</span>
        </div>
      </header>
      <div className="concept-workspace">
        <nav className="concept-rail" aria-label="시안 섹션">
          {[
            { id: "overview", label: "벤치", Icon: Gauge },
            { id: "02", label: "구동", Icon: Activity },
            { id: "03", label: "조향", Icon: CircleGauge },
            { id: "04", label: "안전", Icon: ShieldCheck },
          ].map(({ id, label, Icon }) => (
            <a href={`#concept-${id}`} key={id}>
              <Icon size={22} />
              <span>{label}</span>
            </a>
          ))}
          <span className="concept-rail-end">
            HIL
            <br />
            EV26
          </span>
        </nav>
        <main className="concept-main" id="concept-overview">
          <div className="concept-heading">
            <div>
              <span className="concept-kicker">
                {theme === "pitwall" ? "01 / PIT WALL" : "02 / CONTROL RACK"}
              </span>
              <h1>
                벤치 모니터<span>Vehicle HIL</span>
              </h1>
              <p>
                {theme === "pitwall"
                  ? "핵심 응답을 한눈에. 레이스 엔지니어의 텔레메트리 데스크."
                  : "입력부터 보드 응답까지. 계측 장비처럼 정돈된 테스트 랙."}
              </p>
            </div>
            <div className="concept-scenes">
              <span>예시 상태 전환</span>
              <div role="group" aria-label="예시 상태">
                {scenes.map((s) => (
                  <button
                    key={s.value}
                    aria-pressed={scene === s.value}
                    onClick={() => setScene(s.value)}
                  >
                    {s.label}
                  </button>
                ))}
              </div>
            </div>
          </div>
          <div className={`concept-condition ${scene}`} role="status">
            {scene === "tracking" ? (
              <Check size={18} />
            ) : lost ? (
              <WifiOff size={18} />
            ) : (
              <ShieldCheck size={18} />
            )}
            <strong>
              {lost
                ? "EZ 피드백 지연"
                : stopped
                  ? "E-stop 입력 감지"
                  : "속도 추종 · 정상 응답"}
            </strong>
            <span>
              {lost
                ? "Part I 수신 지연 · 현재 속도 미확인"
                : stopped
                  ? "제동 요청 ON · 구동 출력 정지"
                  : "목표 36.0 km/h → 예시 관측 35.8 km/h"}
            </span>
            <b>EXAMPLE</b>
          </div>
          <div className="concept-grid">
            <Module
              id="01"
              title="차량 응답"
              aside={<span className="concept-small">SIMULATED SNAPSHOT</span>}
              className="concept-overview"
            >
              <div className="concept-response">
                <div className="concept-speed">
                  <div className="concept-kicker">
                    VEHICLE SPEED <span>예시 관측</span>
                  </div>
                  <div className="concept-speed-value">
                    <strong>{speed}</strong>
                    <span>km/h</span>
                  </div>
                  <div className="concept-speed-track">
                    {Array.from({ length: 24 }, (_, i) => (
                      <i
                        key={i}
                        className={!lost && !stopped && i < 18 ? "on" : ""}
                      />
                    ))}
                  </div>
                  <div className="concept-target">
                    <span>
                      목표 <b>{stopped ? "0.0" : "36.0"}</b> km/h
                    </span>
                    <span>
                      오차 <b>{lost ? "—" : stopped ? "0.0" : "+0.2"}</b>
                    </span>
                  </div>
                  <div className="concept-secondary">
                    <div>
                      <span>로드휠 각도</span>
                      <strong>
                        {stopped ? "0.0" : "+4.8"}
                        <small>deg</small>
                      </strong>
                      <p>목표 {stopped ? "0.0" : "+5.0"}°</p>
                    </div>
                    <div className={stopped ? "alarm" : ""}>
                      <span>브레이크 요청</span>
                      <strong>{stopped ? "제동" : "해제"}</strong>
                      <p>{stopped ? "E-STOP / DASH" : "PWM RELEASED"}</p>
                    </div>
                  </div>
                </div>
                <div className="concept-car">
                  <VehicleDiagram scene={scene} />
                  <div>
                    <span>STM_A · 구동</span>
                    <span>STM_B · 제동</span>
                  </div>
                </div>
              </div>
              <div className="concept-power">
                <div>
                  <span>모터 회전수</span>
                  <b>
                    {lost ? "—" : stopped ? "0" : "1,860"}
                    <small>RPM</small>
                  </b>
                </div>
                <div>
                  <span>상전류</span>
                  <b>
                    {lost ? "—" : stopped ? "0.0" : "24.6"}
                    <small>A</small>
                  </b>
                </div>
                <div>
                  <span>DC 전압</span>
                  <b>
                    54.8<small>V</small>
                  </b>
                </div>
                <div>
                  <span>모터 온도</span>
                  <b>
                    38<small>°C</small>
                  </b>
                </div>
              </div>
            </Module>
            <Module
              id="02"
              title="속도 응답"
              aside={<ArrowUpRight size={18} />}
              className="concept-speed-chart"
            >
              <div className="concept-chart-caption">
                <strong>
                  {lost
                    ? "피드백 수신 지연"
                    : stopped
                      ? "정지 응답"
                      : "목표 ↔ 관측"}
                </strong>
                <span>km/h · 예시 60초</span>
              </div>
              <Chart
                points={points}
                traces={[
                  {
                    key: "targetSpeed",
                    label: "목표",
                    color: "var(--p-target)",
                  },
                  {
                    key: "speed",
                    label: "예시 관측",
                    color: "var(--p-accent)",
                  },
                ]}
                unit="km/h"
                height={190}
              />
            </Module>
            <Module
              id="03"
              title="조향 응답"
              aside={<ArrowDownRight size={18} />}
              className="concept-steering-chart"
            >
              <div className="concept-chart-caption">
                <strong>{stopped ? "중립 복귀" : "로드휠 위치 추종"}</strong>
                <span>deg · 예시 60초</span>
              </div>
              <Chart
                points={points}
                traces={[
                  {
                    key: "targetWheel",
                    label: "목표",
                    color: "var(--p-target)",
                  },
                  {
                    key: "wheel",
                    label: "예시 관측",
                    color: "var(--p-accent)",
                  },
                ]}
                unit="deg"
                height={150}
              />
            </Module>
            <Module
              id="04"
              title="연결 · 수신 상태"
              aside={<Radio size={18} />}
              className="concept-reception"
            >
              <div className="concept-board-row">
                {["STM_TEST", "STM_A", "STM_B", "UNO R4"].map((b) => (
                  <span key={b}>
                    <i />
                    {b}
                    <small>예시 연결</small>
                  </span>
                ))}
              </div>
              <div className="concept-can-row">
                <div>
                  <span>
                    EZkontrol Part I <small>0x1801D0EF</small>
                  </span>
                  <strong className={lost ? "alarm" : ""}>
                    {lost ? "842" : "12"}
                    <small>ms 전</small>
                  </strong>
                  <b className={lost ? "alarm" : ""}>
                    {lost ? "STALE" : "VALID"}
                  </b>
                </div>
                <div>
                  <span>
                    EZkontrol Part II <small>0x1802D0EF</small>
                  </span>
                  <strong>
                    18<small>ms 전</small>
                  </strong>
                  <b>VALID</b>
                </div>
              </div>
              <div className="concept-reception-foot">
                <span>
                  ROS /hil/status <b>10.0 Hz</b>
                </span>
                <span>
                  평균 주기 <b>100 ms</b>
                </span>
                <span>
                  마지막 수신 <b>8 ms 전</b>
                </span>
              </div>
            </Module>
            <Module
              id="05"
              title="운전자 입력"
              aside={<SlidersHorizontal size={18} />}
              className="concept-remote"
            >
              <p className="concept-demo-note">
                시안 조작 체험 · 보드에는 전송되지 않습니다.
              </p>
              <div className="concept-input-choice">
                <span>운전 모드</span>
                <div role="group" aria-label="시안 운전 모드">
                  {["MANUAL", "AUTO"].map((v) => (
                    <button
                      key={v}
                      aria-pressed={mode === v}
                      onClick={() => setMode(v)}
                    >
                      {v}
                    </button>
                  ))}
                </div>
              </div>
              <div className="concept-input-choice">
                <span>주행 방향</span>
                <div role="group" aria-label="시안 주행 방향">
                  {["FWD", "REV"].map((v) => (
                    <button
                      key={v}
                      aria-pressed={gear === v}
                      onClick={() => setGear(v)}
                    >
                      {v}
                    </button>
                  ))}
                </div>
              </div>
              <div className="concept-pedal">
                <label htmlFor="concept-pedal">
                  가속 페달 <span>입력 예시</span>
                </label>
                <strong>
                  {pedal}
                  <small>%</small>
                </strong>
                <input
                  id="concept-pedal"
                  type="range"
                  min="0"
                  max="100"
                  value={pedal}
                  onChange={(e) => setPedal(Number(e.target.value))}
                />
                <div>
                  <span>0</span>
                  <span>100</span>
                </div>
              </div>
              <div className="concept-interlock">
                <span>
                  <ShieldCheck size={17} /> E-stop 입력 예시
                </span>
                <button
                  role="switch"
                  aria-label="시안 E-stop"
                  aria-checked={stopped}
                  onClick={() => setScene(stopped ? "tracking" : "estop")}
                >
                  {stopped ? "ON" : "OFF"}
                  <i />
                </button>
              </div>
              <div className="concept-remote-summary">
                <Cpu size={17} />
                <p>
                  입력 선택과 보드 관측을 구분합니다.
                  <br />
                  <strong>이 화면의 수신값은 고정된 예시입니다.</strong>
                </p>
              </div>
            </Module>
            <div className="concept-event">
              <span>예시 이벤트</span>
              <code>00:42.180</code>
              <b>
                {lost
                  ? "EZ PART I TIMEOUT"
                  : stopped
                    ? "ESTOP DASH ASSERTED"
                    : "SPEED TRACKING"}
              </b>
              <span>
                {lost
                  ? "지연값 대신 미확인 표시"
                  : stopped
                    ? "정지 사유와 제동 요청 강조"
                    : "목표 속도에 수렴"}
              </span>
              <ChevronRight size={16} />
            </div>
          </div>
          <footer className="concept-footer">
            <span>KAI · EV26 ENGINEERING</span>
            <span>DESIGN STUDY / {theme === "pitwall" ? "01" : "02"}</span>
            <span>실제 ROS · USB 연결 없음</span>
          </footer>
        </main>
      </div>
    </div>
  );
}
