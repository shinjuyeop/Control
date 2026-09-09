import { Play } from "lucide-react";
import { useEffect, useRef } from "react";
import { useWorkbench } from "../hooks/useWorkbench";
import { Notice } from "./instruments";
export function CommandDialog() {
  const {
    modal,
    setModal,
    production,
    scenarioId,
    setScenarioArm,
    act,
    productionValues,
  } = useWorkbench();
  const dialog = useRef<HTMLDialogElement>(null);
  useEffect(() => {
    if (modal) dialog.current?.showModal();
    else dialog.current?.close();
  }, [modal]);
  return (
    <dialog
      ref={dialog}
      className="command-dialog"
      aria-labelledby="arm-title"
      onCancel={() => setModal(null)}
      onClick={(e) => {
        if (e.target === dialog.current) setModal(null);
      }}
    >
      <span className="eyebrow">COMMAND SOURCE / ARM</span>
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
        연결된 HIL 벤치에 명령을 보냅니다. 조작 연결이 끊기면 발행이 해제됩니다.
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
            } else act("production", { values: productionValues(true) });
          }}
        >
          <Play size={14} />
          {modal === "scenario" ? "ARM · 시험 시작" : "ARM · 발행"}
        </button>
      </div>
    </dialog>
  );
}
