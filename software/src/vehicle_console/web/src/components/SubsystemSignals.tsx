import { Badge, fmt } from "./instruments";
import { SignalFlow, StateRail, type StateItem } from "./SignalInstruments";
import { useWorkbench } from "../hooks/useWorkbench";
import { reception, reported, steeringState } from "../domain/presentation";
import { boolean, canLink } from "../domain/format";
import { brakeReasons, degrees, speedText } from "../domain/vehicle";

export function SystemState() {
  const { state, online, drive, brake, steering, commandState, navigate } =
    useWorkbench();
  return (
    <StateRail
      label="시스템 주요 상태"
      items={[
        {
          label: "HOST COMMAND",
          value: commandState.text,
          detail: "명령 출처 · 발행 상태",
          onSelect: () => navigate("drive"),
        },
        {
          label: "DRIVE / STM_A",
          value: boolean(drive.tractive_commanded, "RUN", "STOP"),
          status: reception(state, online, "/drive/status"),
          detail: drive.stop_request
            ? "구동 정지 요청 있음"
            : "명령 · 모터 피드백",
          onSelect: () => navigate("drive"),
        },
        {
          label: "STEERING / KEYA",
          value: steeringState(steering).label,
          status: reception(state, online, "/steering/status"),
          detail:
            steeringState(steering).tone === "danger"
              ? "고장·E-stop 상태 확인"
              : "목표 · 엔코더 환산각",
          onSelect: () => navigate("steering"),
        },
        {
          label: "BRAKE / STM_B",
          value: boolean(
            brake.brake_commanded,
            "BRAKE REQUEST",
            "RELEASE REQUEST",
          ),
          status: reception(state, online, "/brake/status"),
          detail: brakeReasons(brake.stop_reason_bits),
          onSelect: () => navigate("safety"),
        },
      ]}
    />
  );
}

export function DriveFlow() {
  const { state, online, fresh, hil, drive, observed } = useWorkbench();
  const planning = observed("/planning/command"),
    command = observed("/vehicle/command");
  const stages: StateItem[] =
    drive.mode === 0
      ? [
          {
            label: "STM_TEST 입력",
            value: `${fmt(hil.pedal, 0)} % 페달`,
            detail: "MANUAL · ADC / GPIO",
            status: reception(state, online, "/hil/status"),
          },
          {
            label: "STM_A",
            value: `${fmt(drive.commanded_phase_current_a, 1)} A`,
            detail: "MANUAL 상전류 지령",
            status: reception(state, online, "/drive/status"),
          },
        ]
      : [
          {
            label: "Host 명령",
            value: speedText(planning.speed_target_mps),
            detail: "/planning/command",
            status: reception(state, online, "/planning/command"),
          },
          {
            label: "Governor",
            value: speedText(command.speed_target_mps),
            detail: "/vehicle/command",
            status: reception(state, online, "/vehicle/command"),
          },
          {
            label: "STM_A",
            value: drive.controller_control_mode === 0 ? `${fmt(drive.commanded_phase_current_a, 1)} A` : `${fmt(drive.commanded_motor_rpm, 0)} RPM`,
            detail: drive.controller_control_mode === 0 ? "Torque 전류 명령 · RPM 상한" : "Speed RPM 명령 · 전류 한도",
            status: reception(state, online, "/drive/status"),
          },
        ];
  const link = canLink(hil, fresh, "ez");
  stages.push(
    {
      label: "EZkontrol · HIL",
      value: boolean(hil.ez_running, "RUN", "STOP"),
      detail: "STM_TEST CAN 에뮬레이터",
      status: {
        label: link.text,
        tone: link.tone as "good" | "warning" | "danger" | "muted",
      },
    },
    {
      label: "모터 피드백",
      value: drive.part1_valid ? speedText(drive.speed_mps) : "—",
      detail: drive.part1_valid
        ? `${fmt(drive.motor_rpm, 0)} RPM · Plant ${fmt(hil.rpm, 0)} RPM`
        : `Part I 대기 · Plant ${fmt(hil.rpm, 0)} RPM`,
      status: reception(state, online, "/drive/status"),
    },
  );
  return (
    <SignalFlow
      label={
        drive.mode === 0
          ? "MANUAL · PEDAL → TORQUE"
          : drive.mode === 1
            ? "AUTO · HOST → MOTOR"
            : "HOST COMMAND PATH · STM_A MODE UNKNOWN"
      }
      stages={stages}
    />
  );
}

export function SteeringFlow() {
  const { state, online, hil, steering, observed } = useWorkbench();
  return (
    <SignalFlow
      label="STEERING · HOST CONTROLLER → KEYA"
      stages={[
        {
          label: "Host 명령",
          value: degrees(observed("/planning/command").steering_target_rad),
          detail: "/planning/command",
          status: reception(state, online, "/planning/command"),
        },
        {
          label: "Governor",
          value: degrees(observed("/vehicle/command").steering_target_rad),
          detail: "/vehicle/command · 로드휠 목표",
          status: reception(state, online, "/vehicle/command"),
        },
        {
          label: "Lateral controller",
          value: steeringState(steering).label,
          detail: "위치 명령 · 속도 제한 · 보정",
          status: steeringState(steering),
        },
        {
          label: "KEYA · HIL",
          value: boolean(hil.keya_enabled, "ENABLED", "DISABLED"),
          detail: `${fmt(hil.steer, 1)}° 모터축 · CAN → STM_TEST`,
          status: reception(state, online, "/hil/status"),
        },
        {
          label: "엔코더 환산",
          value: steering.feedback_valid
            ? degrees(steering.actual_wheel_angle_rad)
            : "—",
          detail: "중앙 등가 로드휠 각도",
          status: reception(state, online, "/steering/status"),
        },
      ]}
    />
  );
}

export function BenchSafetyFlow() {
  const { state, online, hil, brake, drive } = useWorkbench();
  const stops = [hil.estop_dash, hil.estop_side, hil.estop_rf];
  const active = stops.some((v) => v === true)
    ? true
    : stops.every((v) => v === false)
      ? false
      : undefined;
  return (
    <SignalFlow
      label="SAFETY · STM_B OUTPUTS"
      note="STM_B: PWM → 브레이크 · UART → STM_A · CAN → UNO"
      stages={[
        {
          label: "E-stop 입력",
          value: boolean(active, "ACTIVE", "CLEAR"),
          detail: "STM_TEST DASH / SIDE / RF",
          status: reported(active, "ACTIVE", "CLEAR", "danger"),
        },
        {
          label: "STM_B safety",
          value: boolean(
            brake.brake_commanded,
            "BRAKE REQUEST",
            "RELEASE REQUEST",
          ),
          detail: brakeReasons(brake.stop_reason_bits),
          status: reception(state, online, "/brake/status"),
        },
        {
          label: "PWM · UART · CAN",
          value: `${hil.brake ?? "—"} / ${boolean(drive.stop_request, "STOP", "CLEAR")}`,
          detail: `병렬 출력 · PWM / STM_A 정지 요청 / UNO ${online && state.display?.fresh ? (state.display.report?.can_lost ? "CAN LOST" : "CAN LIVE") : "UNKNOWN"}`,
        },
      ]}
    />
  );
}

export function SafetyChain() {
  const { state, online, hil, drive, brake, observed } = useWorkbench();
  const reason =
    typeof brake.stop_reason_bits === "number"
      ? !!(brake.stop_reason_bits & 4)
      : undefined;
  return (
    <div className="safety-chain">
      <div className="safety-inputs" role="group" aria-label="E-stop 입력 관측">
        <span className="eyebrow">STM_TEST · E-STOP INPUTS</span>
        {[
          ["DASH", hil.estop_dash],
          ["SIDE", hil.estop_side],
          ["RF", hil.estop_rf],
        ].map(([name, value]) => {
          const status = reported(value, "ACTIVE", "CLEAR", "danger");
          return (
            <div key={String(name)} className={`safety-input ${status.tone}`}>
              <strong>{name}</strong>
              <Badge tone={status.tone}>{status.label}</Badge>
            </div>
          );
        })}
        <small>GPIO → STM_B</small>
      </div>
      <div className="safety-outputs">
        <StateRail
          label="STM_B safety 판단"
          items={[
            {
              label: "STM_B E-STOP",
              value: boolean(reason, "ACTIVE", "CLEAR"),
              status: reported(reason, "ACTIVE", "CLEAR", "danger"),
              detail: "STM_B가 보고한 E-stop 사유",
            },
            {
              label: "HOST BRAKE",
              value: boolean(
                observed("/vehicle/command").brake_engage,
                "REQUEST",
                "OFF",
              ),
              detail: "AUTO에서 제동 판단에 반영",
              status: reception(state, online, "/vehicle/command"),
            },
          ]}
        />
        <SignalFlow
          label="BRAKE · PWM OUTPUT"
          stages={[
            {
              label: "STM_B 제동 요청",
              value: boolean(brake.brake_commanded, "ENGAGED", "RELEASED"),
              detail: brakeReasons(brake.stop_reason_bits),
              status: reception(state, online, "/brake/status"),
            },
            {
              label: "STM_TEST PWM 관측",
              value: hil.brake ?? "UNKNOWN",
              detail: `${fmt(hil.brake_us, 0)} µs · 물리 서보 위치 미계측`,
              status: reported(
                hil.pwm_lost,
                "SIGNAL LOST",
                "RECEIVING",
                "danger",
              ),
            },
          ]}
        />
        <SignalFlow
          label="TORQUE CUT · STM_B UART → STM_A"
          stages={[
            {
              label: "STM_A stop_request",
              value: boolean(drive.stop_request, "STOP", "CLEAR"),
              detail: "STM_B 요청과 STM_A 자체 조건 합산",
              status: reception(state, online, "/drive/status"),
            },
            {
              label: "EZ 수신 RUN 명령",
              value: boolean(hil.ez_running, "RUN", "STOP"),
              detail: `${fmt(drive.commanded_phase_current_a, 1)} A · STM_A 상전류 명령`,
              status: reception(state, online, "/hil/status"),
            },
          ]}
        />
      </div>
    </div>
  );
}

export function FaultSummary() {
  const { state, online, fresh, hil, mine, running } = useWorkbench();
  const known =
    fresh &&
    typeof state.desired.ez_drop_tx === "boolean" &&
    typeof state.desired.keya_drop_tx === "boolean";
  const items: StateItem[] = [];
  for (const bus of ["ez", "keya"]) {
    const prefix = bus.toUpperCase();
    if (hil[`${bus}_can`] === false)
      items.push({
        label: `${prefix} CAN`,
        value: "OFF",
        detail: "에뮬레이터 관측",
        status: { label: "ACTIVE", tone: "warning" },
      });
    if (hil[`${bus}_fault`] === true)
      items.push({
        label: `${prefix} FAULT`,
        value: "INJECTED",
        detail: "에뮬레이터 관측",
        status: { label: "ACTIVE", tone: "danger" },
      });
    if (online && state.desired[`${bus}_drop_tx`] === true)
      items.push({
        label: `${prefix} TX DROP`,
        value: "REQUESTED",
        detail: "Bridge 요청 상태 · 송신 차단 의도",
        status: { label: "REQUEST", tone: "warning" },
      });
  }
  return (
    <div className="fault-overview">
      <div className="instrument-title">
        <div>
          <span className="eyebrow">FAULT INJECTION / ACTIVE CONDITIONS</span>
          <h2>
            {items.length
              ? `${items.length}개 고장·차단 상태`
              : known
                ? "주입·차단 상태 없음"
                : "주입 상태 미확인"}
          </h2>
        </div>
        <Badge tone={mine && fresh && !running ? "warning" : "muted"}>
          {mine && fresh && !running ? "CONTROL OWNED" : "CONTROLS LOCKED"}
        </Badge>
      </div>
      {items.length ? (
        <StateRail label="현재 고장 주입 상태" items={items} />
      ) : (
        <p>
          {known
            ? "CAN·Fault 관측과 TX DROP 요청 상태를 아래에서 확인합니다."
            : "최신 상태를 기다립니다. 이전 요청값은 실제 상태의 증명이 아닙니다."}
        </p>
      )}
    </div>
  );
}
