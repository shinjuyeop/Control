import type { DisplayState } from "./VirtualDisplay";
import { MPS_TO_KMH, RAD_TO_DEG } from "./units";
import { useEffect, useRef, useState } from "react";
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
const empty: Snapshot = {
  type: "state",
  time: 0,
  topics: {},
  connected: false,
  desired: {},
  generator: {},
  generator_fresh: false,
  control: { mine: false, occupied: false, armed: false },
  events: [],
  scenario: null,
  catalog: [],
};
export function useConsole() {
  const [state, setState] = useState<Snapshot>(empty);
  const [online, setOnline] = useState(false);
  const [history, setHistory] = useState<Point[]>([]);
  const socket = useRef<WebSocket | null>(null);
  const serial = useRef(0);
  const pending = useRef(
    new Map<
      number,
      {
        resolve: (s: string) => void;
        reject: (e: Error) => void;
        timer: ReturnType<typeof setTimeout>;
      }
    >(),
  );
  useEffect(() => {
    let disposed = false;
    let retry: ReturnType<typeof setTimeout>;
    let heartbeat: ReturnType<typeof setInterval>;
    let lastFrame = 0;
    const expire = () => {
      pending.current.forEach((p) => {
        clearTimeout(p.timer);
        p.reject(new Error("서버 연결이 끊겼습니다."));
      });
      pending.current.clear();
    };
    function connect() {
      if (disposed) return;
      const ws = new WebSocket(
        `${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/ws`,
      );
      socket.current = ws;
      ws.onopen = () => {
        lastFrame = Date.now();
        heartbeat = setInterval(() => {
          if (Date.now() - lastFrame > 2500) {
            setOnline(false);
            ws.close();
            return;
          }
          if (ws.readyState === WebSocket.OPEN)
            ws.send(JSON.stringify({ action: "heartbeat" }));
        }, 700);
      };
      ws.onmessage = (event) => {
        const payload = JSON.parse(event.data);
        if (payload.type === "result") {
          const item = pending.current.get(payload.id);
          if (item) {
            clearTimeout(item.timer);
            pending.current.delete(payload.id);
            payload.ok
              ? item.resolve(payload.message)
              : item.reject(new Error(payload.message));
          }
          return;
        }
        if (payload.type !== "state") return;
        lastFrame = Date.now();
        setOnline(true);
        const next = payload as Snapshot;
        setState(next);
        const value = (
          topic: string,
          field: string,
          valid?: string,
          scale = 1,
        ) => {
          const sample = next.topics[topic];
          const n = sample?.data[field];
          return sample?.fresh &&
            (!valid || sample.data[valid]) &&
            typeof n === "number" &&
            Number.isFinite(n)
            ? n * scale
            : null;
        };
        const point: Point = {
          time: next.time,
          speed: value("/drive/status", "speed_mps", "part1_valid", MPS_TO_KMH),
          targetSpeed: value(
            "/vehicle/command",
            "speed_target_mps",
            undefined,
            MPS_TO_KMH,
          ),
          rpm: value("/hil/status", "rpm"),
          targetRpm: value("/hil/status", "ez_target_rpm"),
          bus: value("/drive/status", "bus_current_a", "part1_valid"),
          phase: value("/drive/status", "phase_current_a", "part1_valid"),
          steer: value("/hil/status", "steer"),
          targetSteer: value("/hil/status", "steer_target"),
          wheel: value(
            "/steering/status",
            "actual_wheel_angle_rad",
            "feedback_valid",
            RAD_TO_DEG,
          ),
          targetWheel: value(
            "/steering/status",
            "commanded_wheel_angle_rad",
            undefined,
            RAD_TO_DEG,
          ),
        };
        setHistory((old) => [
          ...old.filter((p) => p.time > next.time - 60),
          point,
        ]);
      };
      ws.onclose = () => {
        clearInterval(heartbeat);
        setOnline(false);
        expire();
        if (!disposed) retry = setTimeout(connect, 1500);
      };
      ws.onerror = () => ws.close();
    }
    connect();
    return () => {
      disposed = true;
      clearTimeout(retry);
      clearInterval(heartbeat);
      socket.current?.close();
      expire();
    };
  }, []);
  const send = (action: string, args: Data = {}) =>
    new Promise<string>((resolve, reject) => {
      const ws = socket.current;
      if (!ws || ws.readyState !== WebSocket.OPEN) {
        reject(new Error("서버에 연결되지 않았습니다."));
        return;
      }
      const id = ++serial.current;
      const timer = setTimeout(() => {
        pending.current.delete(id);
        reject(
          new Error("응답 대기 시간이 초과됐습니다. 실제 상태를 확인하세요."),
        );
      }, 10000);
      pending.current.set(id, { resolve, reject, timer });
      ws.send(JSON.stringify({ id, action, ...args }));
    });
  return { state, online, history, send };
}
