import { ArrowLeftRight, Play } from "lucide-react";
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
    takeoverRevision,
    state,
    online,
    busy,
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
      <span className="eyebrow">
        {modal === "takeover"
          ? "CONTROL OWNERSHIP / TRANSFER"
          : "COMMAND SOURCE / ARM"}
      </span>
      <h2 id="arm-title">
        {modal === "takeover"
          ? "조작권을 강제로 인계합니다"
          : modal === "scenario"
            ? "시나리오를 실행합니다"
            : "주행 명령을 발행합니다"}
      </h2>
      <p>
        {modal === "takeover"
          ? "기존 시나리오와 Host 명령 발행을 중단한 뒤 이 브라우저로 조작권을 가져옵니다. 기존 브라우저는 조회 모드로 전환됩니다."
          : modal === "scenario"
            ? scenarioId
            : `속도 ${production.speed_kmh} km/h · 조향 ${production.steering_deg}° · 브레이크 ${production.brake_engage ? "ON" : "OFF"}`}
      </p>
      <Notice>
        {modal === "takeover"
          ? "페달·기어·E-stop 입력은 유지됩니다. MANUAL 페달 입력도 유지되므로 차량 정지를 뜻하지 않습니다. Host 명령은 인계 후 다시 ARM해야 합니다."
          : "연결된 HIL 벤치에 명령을 보냅니다. 조작 연결이 끊기면 발행이 해제됩니다."}
      </Notice>
      {modal === "takeover" && takeoverRevision !== state.control.revision && (
        <p className="inline-error">
          조작권 상태가 변경됐습니다. 닫고 현재 상태에서 다시 시도하세요.
        </p>
      )}
      <div className="actions">
        <button autoFocus onClick={() => setModal(null)}>
          취소
        </button>
        <button
          className={modal === "takeover" ? "reset-button" : "primary"}
          disabled={
            modal === "takeover" &&
            (!online ||
              busy ||
              state.control.transitioning ||
              takeoverRevision === undefined ||
              takeoverRevision !== state.control.revision)
          }
          onClick={() => {
            const kind = modal;
            setModal(null);
            if (kind === "takeover") {
              act("takeover", {
                confirmed: true,
                expected_revision: takeoverRevision,
              });
            } else if (kind === "scenario") {
              setScenarioArm(false);
              act("scenario", { name: scenarioId, arm: true });
            } else act("production", { values: productionValues(true) });
          }}
        >
          {modal === "takeover" ? (
            <ArrowLeftRight size={14} />
          ) : (
            <Play size={14} />
          )}
          {modal === "takeover"
            ? "Host 명령 중단 후 인계"
            : modal === "scenario"
              ? "ARM · 시험 시작"
              : "ARM · 발행"}
        </button>
      </div>
    </dialog>
  );
}
