import {
  Check,
  ChevronRight,
  ClipboardList,
  Download,
  FlaskConical,
  Play,
  Square,
  X,
} from "lucide-react";
import { Badge, Details, Fact, fmt, Panel } from "../components/instruments";
import { download } from "../domain/download";
import { scenarioDescriptions } from "../domain/format";
import type { Data } from "../domain/types";
import { scenarioValue } from "../domain/units";
import { useWorkbench } from "../hooks/useWorkbench";
export function ScenariosPage() {
  const {
    state,
    scenarioId,
    setScenarioId,
    scenarioArm,
    setScenarioArm,
    disabled,
    mine,
    running,
    busy,
    setModal,
    act,
    past,
  } = useWorkbench();
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
                <Square size={14} /> 중단 · Bench Reset
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
              실행하면 실제 관측값을 기준으로 각 조건의 통과 여부를 표시합니다.
            </p>
          </div>
        ) : (
          <>
            <div className="run-heading">
              <strong>{run.scenario}</strong>
              <span>
                {fmt(run.duration_sec, 1)}s{" "}
                {running ? `· ${run.step_index} / ${run.step_count} 단계` : ""}
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
                      {record.detail ?? record.hil_commands?.join(" / ") ?? ""}
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
