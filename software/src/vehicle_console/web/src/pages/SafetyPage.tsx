import { Badge, Fact, fmt, Notice, Panel } from "../components/instruments";
import { brakeReasons, CommandTelemetry } from "../components/VehicleTelemetry";
import { boolean } from "../domain/format";
import { SafetyChain } from "../components/SubsystemSignals";
import { useWorkbench } from "../hooks/useWorkbench";
export function SafetyPage() {
  const { state, online, hil, drive, brake } = useWorkbench();
  return (
    <>
      <Panel
        title="E-stop · 제동 · 구동 정지 경로"
        eyebrow="SAFETY SIGNAL CHAIN"
        className="pipeline-panel"
      >
        <SafetyChain />
      </Panel>
      <div className="grid-two">
        <Panel title="브레이크 전기 신호" eyebrow="STM_B PWM → STM_TEST">
          <Fact label="PWM 펄스 폭" value={fmt(hil.brake_us, 0)} unit="μs" />
          <Fact
            label="PWM 주기"
            value={fmt(hil.brake_period_us, 0)}
            unit="μs"
          />
          <Fact
            label="PWM 신호"
            value={
              <Badge
                tone={
                  typeof hil.pwm_lost !== "boolean"
                    ? "muted"
                    : hil.pwm_lost
                      ? "danger"
                      : "muted"
                }
              >
                {boolean(hil.pwm_lost, "LOST", "RECEIVING")}
              </Badge>
            }
          />
          <Fact
            label="정지 사유"
            value={brakeReasons(brake.stop_reason_bits)}
          />
          <Notice>
            브레이크 실제 위치 피드백은 미배선입니다. PWM 판정은 서보의 물리적
            작동을 증명하지 않습니다.
          </Notice>
        </Panel>
        <Panel
          title="구동 차단 관련 관측"
          eyebrow="REPORTED BY STM_A / STM_TEST"
        >
          <Fact
            label="STM_A stop_request"
            value={boolean(drive.stop_request)}
          />
          <Fact
            label="STM_A RUN 명령"
            value={boolean(drive.tractive_commanded)}
          />
          <Fact label="EZ 수신 RUN 명령" value={boolean(hil.ez_running)} />
          <Fact
            label="STM_A 상전류 명령"
            value={fmt(drive.commanded_phase_current_a)}
            unit="A"
          />
          <Fact
            label="STM_A Host timeout"
            value={boolean(drive.command_timeout)}
          />
          <p className="muted">
            각 보드가 보고한 신호입니다. 전류 0만으로 물리적인 구동 차단을
            판정하지 않습니다.
          </p>
        </Panel>
      </div>
      <CommandTelemetry state={state} online={online} />
      <Notice>
        Bench Reset은 주행 명령을 끄고 기본 입력을 재적용합니다. E-stop 해제와
        Plant ON을 포함하며 물리 비상정지가 아닙니다.
      </Notice>
    </>
  );
}
