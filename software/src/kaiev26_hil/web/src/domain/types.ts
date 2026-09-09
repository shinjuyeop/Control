export type Data = Record<string, any>;
export type Sample = {
  data: Data;
  fresh: boolean;
  age_ms: number;
  rx_hz?: number | null;
  period_ms?: number | null;
  rx_count?: number;
};
export type Event = { id: number; time: number; kind: string; text: string };
export type Snapshot = {
  type: "state";
  time: number;
  topics: Record<string, Sample>;
  connected: boolean;
  desired: Data;
  generator: Data;
  generator_fresh: boolean;
  control: { mine: boolean; occupied: boolean; armed: boolean };
  events: Event[];
  scenario: Data | null;
  catalog: Data[];
  display?: DisplayState;
};
/** Plot history: vehicle speed in km/h, wheel/shaft angles in degrees. */
export type Point = {
  time: number;
  speed: number | null;
  targetSpeed: number | null;
  rpm: number | null;
  targetRpm: number | null;
  phase: number | null;
  bus: number | null;
  steer: number | null;
  targetSteer: number | null;
  wheel: number | null;
  targetWheel: number | null;
};
export type DisplayState = {
  enabled: boolean;
  port: string;
  serial_open: boolean;
  fresh: boolean;
  report_age_ms: number | null;
  waiting_for_report: boolean;
  error: string;
  image: string | null;
  report: {
    page: string;
    can_lost: boolean;
    can_age_ms: number;
    uptime_ms: number;
    seq: number;
    values: Data;
  } | null;
};
