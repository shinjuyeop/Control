/** Operator units. ROS payloads remain m/s and radians. */
export const MPS_TO_KMH = 3.6;
export const RAD_TO_DEG = 180 / Math.PI;
export const mpsToKmh = (value: number) => value * MPS_TO_KMH;
export const kmhToMps = (value: number) => value / MPS_TO_KMH;
export const radToDeg = (value: number) => value * RAD_TO_DEG;
export const degToRad = (value: number) => value / RAD_TO_DEG;

export function scenarioValue(field: string, value: unknown): string {
  if (typeof value !== "number") return String(value);
  if (field.endsWith("_mps")) return `${mpsToKmh(value).toFixed(2)} km/h`;
  if (field.endsWith("_rad")) return `${radToDeg(value).toFixed(2)}°`;
  if (field === "speed") return `${value} km/h`; // STM_TEST protocol already uses km/h.
  if (field === "steering_position") return `${value}°`;
  return String(value);
}
