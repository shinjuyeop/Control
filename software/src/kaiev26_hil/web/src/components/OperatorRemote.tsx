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
  } = useWorkbench();
  return (
    <section className="operator-remote" aria-label="Operator Input">
      <header className="remote-header">
        <SlidersHorizontal size={17} />
        <div>
          <h2>OPERATOR INPUT</h2>
          <span>STM_TEST · 운전자 입력</span>
        </div>
      </header>
      {disabled && (
        <div className="remote-lock locked" role="status">
          <LockKeyhole size={13} />
          {lockReason}
        </div>
      )}
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
            setPedalEditing(false);
            if (!disabled) command(`SET PEDAL ${e.currentTarget.value}`);
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
              setPedalEditing(false);
              if (!disabled) command(`SET PEDAL ${e.currentTarget.value}`);
            }
          }}
          onPointerCancel={() => setPedalEditing(false)}
        />
        <div className="range-labels">
          <span>0</span>
          <span>100</span>
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
    </section>
  );
}
