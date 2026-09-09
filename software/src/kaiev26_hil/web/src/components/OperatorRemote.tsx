import { LockKeyhole, SlidersHorizontal } from "lucide-react";
import { gears, modes } from "../domain/format";
import { useWorkbench } from "../hooks/useWorkbench";
import { Choice, fmt, Switch } from "./instruments";

export function OperatorRemote() {
  const {
    hil,
    fresh,
    disabled,
    lockReason,
    pedal,
    setPedal,
    setPedalEditing,
    command,
    state,
  } = useWorkbench();
  return (
    <section className="operator-remote" aria-label="Operator Remote">
      <header className="remote-header">
        <SlidersHorizontal size={17} />
        <div>
          <h2>Operator Remote</h2>
          <span>STM_TEST · 운전자 입력</span>
        </div>
      </header>
      <div className={`remote-lock ${disabled ? "locked" : ""}`} role="status">
        {disabled ? (
          <>
            <LockKeyhole size={13} />
            {lockReason}
          </>
        ) : (
          <>
            <i className="status-dot live" />
            조작 가능 · 보드 관측값 기준
          </>
        )}
      </div>
      <div className="remote-section">
        <Choice
          label="운전 모드"
          options={modes}
          value={hil.mode}
          disabled={disabled}
          onChange={(value) => command(`SET MODE ${value}`)}
        />
        <Choice
          label="주행 방향"
          options={gears}
          value={hil.gear}
          disabled={disabled}
          onChange={(value) =>
            command(`SET GEAR ${value === "FWD" ? "FORWARD" : "REVERSE"}`)
          }
        />
      </div>
      <div className="remote-section">
        <div className="pedal-heading">
          <label htmlFor="remote-pedal">가속 페달</label>
          <strong>
            {fresh ? fmt(pedal, 0) : "—"}
            <small>%</small>
          </strong>
        </div>
        <input
          id="remote-pedal"
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
            if (disabled) return;
            command(`SET PEDAL ${e.currentTarget.value}`);
            setPedalEditing(false);
          }}
          onKeyUp={(e) => {
            if (
              !disabled &&
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
          <span>0</span>
          <span>놓으면 한 번 전송</span>
          <span>100</span>
        </div>
        <div className="remote-observed">
          <span>보드 관측</span>
          <b>{fmt(hil.pedal, 0)} %</b>
        </div>
      </div>
      <div className="remote-section">
        <h3>E-stop 입력 주입</h3>
        <p>STM_TEST 입력 주입입니다. 물리 E-stop이 아닙니다.</p>
        {[
          ["DASH", "대시보드 E-stop", "estop_dash"],
          ["SIDE", "차체 측면 E-stop", "estop_side"],
          ["RF", "무선 리모컨 E-stop", "estop_rf"],
        ].map(([key, label, field]) => (
          <div
            className={`remote-estop ${hil[field] ? "asserted" : ""}`}
            key={key}
          >
            <Switch
              label={label}
              description={`${key} · ${typeof hil[field] === "boolean" ? (hil[field] ? "관측 ON" : "관측 OFF") : "관측값 없음"}`}
              value={hil[field]}
              disabled={disabled}
              danger
              onChange={(value) =>
                command(`SET ESTOP ${key} ${value ? "ON" : "OFF"}`)
              }
            />
          </div>
        ))}
      </div>
      <footer className="remote-footer">
        <span>/hil/status</span>
        <span>
          {fresh
            ? `${fmt(state.topics["/hil/status"]?.age_ms, 0)} ms ago`
            : "NO FRESH DATA"}
        </span>
        <p>
          선택 표시는 실제 관측값입니다. ACK는 입력 적용 관측을 대신하지
          않습니다.
        </p>
      </footer>
    </section>
  );
}
