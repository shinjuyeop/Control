import { Play, Square } from "lucide-react";
import { radToDeg } from "../domain/units";
import { useWorkbench } from "../hooks/useWorkbench";
import { Badge, Panel, Switch } from "./instruments";
export function ProductionCommand() {
  const {
    generator,
    commandState,
    production,
    setProduction,
    setProductionEdited,
    disabled,
    mine,
    busy,
    running,
    act,
    setModal,
    productionValues,
  } = useWorkbench();
  return (
    <Panel
      className="command-source"
      title="주행 명령"
      eyebrow="HOST COMMAND"
      aside={<Badge tone={commandState.tone}>{commandState.text}</Badge>}
    >
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
                    speed_kmh: e.target.value,
                  });
                }}
                onBlur={(e) => {
                  if (e.target.value === "" && !e.target.validity.badInput)
                    setProduction((draft) => ({ ...draft, speed_kmh: "0" }));
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
                    steering_deg: e.target.value,
                  });
                }}
                onBlur={(e) => {
                  if (e.target.value === "" && !e.target.validity.badInput)
                    setProduction((draft) => ({ ...draft, steering_deg: "0" }));
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
    </Panel>
  );
}
