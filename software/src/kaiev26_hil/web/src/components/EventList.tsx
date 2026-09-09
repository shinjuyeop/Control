import { Activity } from "lucide-react";
export function EventList({
  events,
}: {
  events: { id: number; time: number; kind: string; text: string }[];
}) {
  return events.length ? (
    <div className="event-list">
      {events.map((event) => (
        <div
          className={`event-row ${event.kind === "error" ? "error" : ""}`}
          key={event.id}
        >
          <span className="event-time">
            {new Date(event.time * 1000).toLocaleTimeString("ko-KR", {
              hour12: false,
            })}
          </span>
          <span className="event-kind">{event.kind}</span>
          <span>{event.text}</span>
        </div>
      ))}
    </div>
  ) : (
    <div className="empty compact">
      <Activity size={21} />
      <p>연결 및 명령 이벤트가 여기에 표시됩니다.</p>
    </div>
  );
}
