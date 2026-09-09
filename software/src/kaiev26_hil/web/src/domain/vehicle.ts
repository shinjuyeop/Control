import { fmt } from "./format";
import { mpsToKmh, radToDeg } from "./units";
export const states = [
  "INIT",
  "DISABLED",
  "WAIT_CMD",
  "ACTIVE",
  "ESTOP",
  "BRAKE",
  "CAN_LOST",
  "FEEDBACK_LOST",
  "FAULT",
];
export const reasons = [
  "NONE",
  "MODE_NOT_RECEIVED",
  "MODE_TIMEOUT",
  "MANUAL_MODE",
  "INVALID_MODE",
  "ESTOP",
  "COMMAND_BRAKE",
  "STEER_COMMAND_TIMEOUT",
  "CAN_TIMEOUT",
  "FEEDBACK_INVALID",
  "MOTOR_FAULT",
];
export const enumText = (names: string[], value: unknown) =>
  typeof value === "number" ? (names[value] ?? `UNKNOWN (${value})`) : "—";
export const bitHex = (value: unknown, digits = 4) =>
  typeof value === "number"
    ? `0x${value.toString(16).toUpperCase().padStart(digits, "0")}`
    : "—";
export const boolText = (value: unknown) =>
  typeof value === "boolean" ? (value ? "ON" : "OFF") : "—";
export const degrees = (value: unknown) =>
  typeof value === "number" ? `${fmt(radToDeg(value), 2)}°` : "—";
export const speedText = (value: unknown) =>
  typeof value === "number" ? `${fmt(mpsToKmh(value), 2)} km/h` : "—";
export function brakeReasons(value: unknown) {
  if (typeof value !== "number") return "—";
  const names = [
    [1, "COMMAND_BRAKE"],
    [4, "ESTOP"],
    [8, "COMMAND_INVALID"],
  ] as const;
  return (
    [
      ...names.filter(([bit]) => value & bit).map(([, label]) => label),
      ...(value & ~13 ? [bitHex(value & ~13)] : []),
    ].join(" · ") || "NONE"
  );
}
type Polarity = "bad_true" | "bad_false" | "warn_true" | "info";
export type Flag = [string, string, Polarity];
export const driveFlags: Flag[] = [
  ["command_received", "Host 명령 수신", "bad_false"],
  ["command_timeout", "Host timeout", "bad_true"],
  ["stop_request", "구동 정지 요청", "bad_true"],
  ["tractive_commanded", "RUN 명령", "info"],
  ["tractive_feedback", "RUN 피드백", "info"],
  ["tractive_feedback_valid", "RUN 피드백 유효", "bad_false"],
  ["part1_valid", "Part I 유효", "bad_false"],
  ["part2_valid", "Part II 유효", "bad_false"],
  ["fault_bits_valid", "Fault bits 유효", "info"],
];
export const steeringFlags: Flag[] = [
  ["communication_ok", "통신 정상", "bad_false"],
  ["can_timeout", "CAN timeout", "bad_true"],
  ["feedback_valid", "피드백 유효", "bad_false"],
  ["feedback_timeout", "피드백 timeout", "bad_true"],
  ["mode_received", "모드 수신", "bad_false"],
  ["mode_timeout", "모드 timeout", "bad_true"],
  ["steer_command_received", "조향 명령 수신", "bad_false"],
  ["steer_command_timeout", "조향 명령 timeout", "bad_true"],
  ["pending_command_valid", "대기 명령 유효", "info"],
  ["movement_target_valid", "이동 목표 유효", "info"],
  ["estop_active", "E-stop", "bad_true"],
  ["command_brake_active", "명령 제동", "bad_true"],
  ["enabled_commanded", "Enable 명령", "info"],
  ["enabled_feedback", "Enable 피드백", "info"],
  ["enabled_feedback_valid", "Enable 피드백 지원", "info"],
  ["enable_mismatch", "Enable 불일치", "bad_true"],
  ["tracking_control_enabled", "추종 제어", "info"],
  ["tracking_error_warning", "추종 경고", "warn_true"],
  ["tracking_error_fault", "추종 고장", "bad_true"],
];
