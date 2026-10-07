import type { Snapshot } from "./types";

const required: Record<string, string> = {
  overview: "observe_vehicle", drive: "observe_vehicle", steering: "observe_vehicle",
  safety: "observe_vehicle", runtime: "observe_runtime", diagnostics: "observe_diagnostics",
  bench: "control_bench", faults: "fault_injection", scenarios: "scenarios", display: "virtual_display",
};

export function pageAllowed(state: Snapshot, page: string) {
  return !!required[page] && state.capabilities.includes(required[page]);
}

export function capabilities(state: Snapshot) {
  return {
    control: state.profile === "hil" && state.capabilities.includes("control_bench"),
    touch: state.profile === "hil" && state.capabilities.includes("virtual_touch"),
  };
}

export const vehicleTopics = [
  "/vehicle/command", "/drive/status", "/drive/longitudinal_status", "/steering/status", "/brake/status",
  "/stm_a/runtime_stats", "/stm_b/runtime_stats", "/diagnostics",
];

export const sourceTopics = (state: Snapshot) => state.profile === "hil"
  ? ["/hil/status", "/drive/status", "/drive/longitudinal_status", "/brake/status", "/steering/status", "/vehicle/state",
     "/planning/command", "/vehicle/command", "/diagnostics", "/stm_a/runtime_stats", "/stm_b/runtime_stats"]
  : vehicleTopics;
