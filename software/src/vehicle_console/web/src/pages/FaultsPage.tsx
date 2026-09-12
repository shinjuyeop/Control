import { FaultSummary } from "../components/SubsystemSignals";
import { RotateCcw } from "lucide-react";
import {
  Chart,
  Fact,
  fmt,
  Notice,
  Panel,
  Switch,
} from "../components/instruments";
import { age, targetColor, boolean, feedbackColor } from "../domain/format";
import { useWorkbench } from "../hooks/useWorkbench";
export function FaultsPage() {
  const { state, hil, points, disabled, commands, command, linkBadge } =
    useWorkbench();
  return (
    <>
      <FaultSummary />
      <Notice>
        CAN OFF는 인터페이스 동작을 끄고, TX DROP은 에뮬레이터의 송신을
        차단합니다. 초기화 성공과 실제 프레임 수신은 별개입니다.
      </Notice>
      <div className="grid-two">
        {(["ez", "keya"] as const).map((bus) => (
          <Panel
            key={bus}
            title={bus === "ez" ? "EZkontrol · 구동" : "KEYA · 조향"}
            eyebrow={
              bus === "ez"
                ? "CAN 1 / INJECTION CONTROLS"
                : "CAN 2 / INJECTION CONTROLS"
            }
            className="fault-control-panel"
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
            { key: "targetSpeed", label: "Governor 출력", color: targetColor },
            { key: "speed", label: "STM_A 피드백", color: feedbackColor },
          ]}
          unit="km/h"
        />
        <Notice>
          피드백이 무효해지면 그래프가 끊깁니다. EZ 피드백 timeout 자체가 구동을
          차단하는지는 production 정책을 따릅니다.
        </Notice>
      </Panel>
    </>
  );
}
