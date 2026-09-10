import { Expand, Monitor, Shrink, Usb } from "lucide-react";
import { useState } from "react";
import { Badge, Details, Fact, fmt, Panel } from "../components/instruments";
import type { DisplayState } from "../domain/types";
import { StateRail } from "../components/SignalInstruments";
import { useWorkbench } from "../hooks/useWorkbench";
import { useVirtualTouch } from "../hooks/useVirtualTouch";

export function VirtualDisplay({
  display,
  online,
}: {
  display?: DisplayState;
  online: boolean;
}) {
  const [scale, setScale] = useState(1);
  const fresh = online && !!display?.fresh;
  const report = display?.report;
  const { state, mine, sendVirtualTouch } = useWorkbench();
  const ready = fresh && !!display?.serial_open && !!display.touch_ready && mine &&
    !state.control.transitioning;
  const touch = useVirtualTouch(ready, state.control.revision, display?.generation, sendVirtualTouch);
  const touchStatus = !online ? "WEB OFFLINE" : !fresh || !display?.serial_open ? "UNO STALE" :
    !report?.vtouch ? "VTOUCH UNAVAILABLE" : !mine || state.control.transitioning ? "VIEW ONLY" : "VTOUCH READY";
  return (
    <div className="virtual-display-container">
      <div className="virtual-display-layout">
        <Panel
          title="UNO Dashboard"
          eyebrow="480 × 320 · FIRMWARE RENDERER"
          aside={
            <div className="actions">
              <button
                className="icon-button lcd-scale"
                aria-label="LCD 원본 크기"
                aria-pressed={scale === 1}
                onClick={() => setScale(1)}
              >
                <Shrink size={16} />
              </button>
              <button
                className="icon-button lcd-scale"
                aria-label="LCD 두 배 확대"
                aria-pressed={scale === 2}
                onClick={() => setScale(2)}
              >
                <Expand size={16} />
              </button>
              <Badge tone={fresh ? "good" : "muted"}>
                {fresh ? "UNO LIVE" : "수신 대기 / 지연"}
              </Badge>
              <Badge tone={ready ? "good" : "muted"}>{touchStatus}</Badge>
            </div>
          }
        >
          <div className="lcd-stage">
            <div
              className={`lcd-panel ${fresh ? "" : "stale"}`}
              style={{ width: 480 * scale }}
            >
              {display?.image ? (
                <img
                  {...touch.handlers}
                  className={ready ? "lcd-touch-ready" : ""}
                  draggable={false}
                  src={display.image}
                  alt="UNO가 적용한 상태로 재구성한 가상 LCD"
                  width={480}
                  height={320}
                />
              ) : (
                <div className="lcd-empty">
                  <Monitor size={42} />
                  <strong>UNO 표시 상태 수신 대기</strong>
                  <p>UNO가 보고한 값으로 화면을 그립니다.</p>
                </div>
              )}
              {display?.image && !fresh && (
                <div className="lcd-stale-overlay">
                  마지막 수신 화면 · 현재 상태 미확인
                </div>
              )}
            </div>
          </div>
          <div className="lcd-caption">
            <span>논리 해상도 480 × 320 · {scale}×</span>
            <span>{fresh ? report?.page : "현재 페이지 미확인"}</span>
          </div>
          <p className="muted">조작권을 가진 브라우저에서 좌우로 드래그하세요. 페이지는 UNO 적용 보고 후 갱신됩니다.</p>
          {touch.error && <p className="inline-error" role="alert">{touch.error}</p>}
          <Details title="LCD mirror의 확인 범위">
            실제 펌웨어의 화면 그리기 코드와 아셋으로 재구성합니다. SPI
            전송·패널·백라이트의 물리 상태는 이 화면으로 확인할 수 없습니다.
            가상 터치는 Dashboard 페이지 이동만 수행합니다.
          </Details>
        </Panel>
        <Panel
          title="디스플레이 연결"
          eyebrow="UNO → USB → WEB"
          aside={<Usb size={20} />}
        >
          <StateRail
            label="LCD 연결 요약"
            items={[
              {
                label: "UNO REPORT",
                value: fresh ? "RECEIVING" : "WAITING",
                detail: online
                  ? `${fmt(display?.report_age_ms, 0)} ms since report`
                  : "Web 연결 대기",
              },
              {
                label: "DASHBOARD CAN",
                value: !fresh
                  ? "UNKNOWN"
                  : report?.can_lost
                    ? "CAN LOST"
                    : "RECEIVING",
                status: !fresh
                  ? undefined
                  : {
                      label: report?.can_lost ? "LOST" : "LIVE",
                      tone: report?.can_lost ? "danger" : "good",
                    },
              },
            ]}
          />
          <Details title="USB · CAN 상세">
            <Fact
              label="USB serial"
              value={
                <Badge tone={online && display?.serial_open ? "good" : "muted"}>
                  {online && display?.serial_open ? "OPEN" : "연결 대기"}
                </Badge>
              }
            />
            <Fact
              label="UNO 상태 보고"
              value={
                <Badge tone={fresh ? "good" : "warning"}>
                  {fresh ? "수신 중" : "미수신 / 지연"}
                </Badge>
              }
            />
            <Fact
              label="보고 수신 이후"
              value={online ? fmt(display?.report_age_ms, 0) : "—"}
              unit="ms"
            />
            <Fact label="UNO 페이지" value={fresh ? report?.page : "—"} />
            <Fact
              label="Dashboard CAN"
              value={
                <Badge
                  tone={!fresh ? "muted" : report?.can_lost ? "danger" : "good"}
                >
                  {!fresh
                    ? "UNKNOWN"
                    : report?.can_lost
                      ? "CAN LOST"
                      : "수신 중"}
                </Badge>
              }
            />
            <Fact
              label="완성 CAN snapshot 이후"
              value={
                fresh && report && report.can_age_ms >= 0
                  ? fmt(report.can_age_ms, 0)
                  : "—"
              }
              unit="ms"
            />
            <Fact label="UNO snapshot 누적" value={fresh ? report?.seq : "—"} />
            <Fact label="Touch 오류 누적" value={fresh ? report?.touch_errors ?? "—" : "—"} />
            <Fact
              label="UNO uptime"
              value={fresh && report ? fmt(report.uptime_ms / 1000, 1) : "—"}
              unit="s"
            />
          </Details>
          <div className="display-connection-note">
            {!display?.enabled ? (
              <p>
                가상 LCD를 사용하려면 Web launch에{" "}
                <code>dashboard_port:=/dev/arduino_hil</code>을 추가해 UNO USB
                상태 보고를 연결하세요.
              </p>
            ) : display.waiting_for_report ? (
              <p>
                USB 포트는 열렸지만 지원되는 상태 보고를 아직 받지 못했습니다.
                UNO에 LCD 상태 보고 기능이 포함된 펌웨어가 필요합니다.
              </p>
            ) : null}
            {display?.port && (
              <p>
                <code>{display.port}</code>
              </p>
            )}
            {display?.error && (
              <p className="inline-error" role="alert">
                {display.error}
              </p>
            )}
          </div>
          <Details title="UNO가 보고한 표시값">
            {report ? (
              Object.entries(report.values).map(([key, value]) => (
                <Fact key={key} label={key} value={String(value)} />
              ))
            ) : (
              <p className="muted">수신한 표시값이 없습니다.</p>
            )}
          </Details>
        </Panel>
      </div>
    </div>
  );
}
