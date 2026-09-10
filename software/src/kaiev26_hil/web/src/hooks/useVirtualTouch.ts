import { useEffect, useRef, useState, type PointerEvent } from "react";
import type { Data } from "../domain/types";

type Drag = {
  pointer: number;
  element: HTMLImageElement;
  x: number;
  y: number;
  revision: number;
  generation: number;
  dirty: boolean;
  sentAt: number;
};

// The image's content rect (without the surrounding bezel) is the LCD space.
function coordinate(event: PointerEvent<HTMLImageElement>) {
  const rect = event.currentTarget.getBoundingClientRect();
  return {
    x: Math.max(0, Math.min(479, Math.floor((event.clientX - rect.left) / rect.width * 480))),
    y: Math.max(0, Math.min(319, Math.floor((event.clientY - rect.top) / rect.height * 320))),
  };
}

export function useVirtualTouch(
  ready: boolean,
  revision: number | undefined,
  generation: number | undefined,
  send: (action: string, args: Data) => Promise<string>,
) {
  const drag = useRef<Drag | null>(null);
  const sender = useRef(send);
  sender.current = send;
  const moving = useRef(false);
  const ending = useRef(false);
  const [error, setError] = useState("");

  function transmit(phase: string, value: Drag) {
    return sender.current("virtual_touch", {
      phase, x: value.x, y: value.y,
      revision: value.revision, generation: value.generation,
    });
  }

  function detach() {
    const value = drag.current;
    drag.current = null;
    if (value?.element.hasPointerCapture(value.pointer))
      value.element.releasePointerCapture(value.pointer);
    return value;
  }

  function cancel() {
    const value = detach();
    if (value) void transmit("C", value).catch(() => {});
  }

  function failed(value: Drag, reason: unknown) {
    if (drag.current === value) cancel();
    setError((reason as Error).message);
  }

  useEffect(() => {
    // Keep a stationary held pointer alive, without turning browser pixel rate
    // into USB traffic. At most one MOVE request is awaiting the backend.
    const timer = setInterval(() => {
      const value = drag.current;
      if (!value || moving.current) return;
      const now = performance.now();
      if (!value.dirty && now - value.sentAt < 200) return;
      value.dirty = false;
      value.sentAt = now;
      moving.current = true;
      void transmit("M", value)
        .catch((reason) => failed(value, reason))
        .finally(() => { moving.current = false; });
    }, 20);
    const blur = () => cancel();
    const hidden = () => { if (document.hidden) cancel(); };
    window.addEventListener("blur", blur);
    document.addEventListener("visibilitychange", hidden);
    return () => {
      clearInterval(timer);
      window.removeEventListener("blur", blur);
      document.removeEventListener("visibilitychange", hidden);
      cancel();
    };
  }, [ready, revision, generation]);

  return {
    error,
    handlers: {
      onPointerDown(event: PointerEvent<HTMLImageElement>) {
        if (drag.current) { cancel(); return; } // Multi-touch cancels the whole drag.
        if (!ready || ending.current || !event.isPrimary || event.button !== 0 ||
            revision === undefined || generation === undefined) return;
        event.preventDefault();
        const value: Drag = {
          pointer: event.pointerId, element: event.currentTarget,
          ...coordinate(event), revision, generation, dirty: false,
          sentAt: performance.now(),
        };
        drag.current = value;
        event.currentTarget.setPointerCapture(event.pointerId);
        setError("");
        void transmit("D", value).catch((reason) => failed(value, reason));
      },
      onPointerMove(event: PointerEvent<HTMLImageElement>) {
        const value = drag.current;
        if (!value || value.pointer !== event.pointerId) return;
        Object.assign(value, coordinate(event), { dirty: true });
      },
      onPointerUp(event: PointerEvent<HTMLImageElement>) {
        if (drag.current?.pointer !== event.pointerId) return;
        const value = detach()!;
        Object.assign(value, coordinate(event));
        ending.current = true;
        void transmit("U", value)
          .catch((reason) => failed(value, reason))
          .finally(() => { ending.current = false; });
      },
      onPointerCancel() { cancel(); },
      onLostPointerCapture(event: PointerEvent<HTMLImageElement>) {
        if (drag.current?.pointer === event.pointerId) cancel();
      },
    },
  };
}
