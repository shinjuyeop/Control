"""Display-only UNO bench capture and matching native replay."""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import select
import shlex
import struct
import subprocess
import time

from simulator.model import (
    SCENARIOS, SCENARIO_BUILDERS, STATUS_GEAR_REVERSE, STATUS_MODE_AUTO,
    STATUS_VEHICLE_VALID,
    _speed_centi_mps,
)
from simulator.backend import FRAME_IDS, NativeDashboard, _can_payloads


PAGE_CODES = {"startup": 0, "manual": 1, "auto": 2, "transport": 3}
DEFAULT_REGION_MASK = {
    "startup": 0xFFFF, "manual": 0x0002, "auto": 0x0200,
    "transport": 0, "startup_fixed": 0,
}
BOARD_SERIAL_SETTLE_S = 2.0
BOARD_SERIAL_QUIET_S = 0.5
BOARD_SERIAL_SETTLE_LIMIT_S = 20.0
BENCH_EPOCH_LEAD_US = 1_000_000
REMOTE_BRIDGE = r'''import fcntl,os,select,struct,sys,termios
path=sys.argv[1]
fd=os.open(path,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
try:
 a=termios.tcgetattr(fd)
 a[0]=0; a[1]=0; a[2]=termios.CS8|termios.CREAD|termios.CLOCAL; a[3]=0
 a[4]=termios.B115200; a[5]=termios.B115200
 termios.tcsetattr(fd,termios.TCSANOW,a)
 fcntl.ioctl(fd,getattr(termios,"TIOCMBIS",0x5416),struct.pack("I",0x002))
 pending=bytearray()
 stdin_open=True
 while stdin_open:
  readable,writable,_=select.select(
   [sys.stdin.buffer,fd], [fd] if pending else [], [], 0.005)
  if sys.stdin.buffer in readable:
   data=os.read(sys.stdin.fileno(),4096)
   if not data: stdin_open=False
   else: pending.extend(data)
  if fd in writable and pending:
   try:
    sent=os.write(fd,pending)
    del pending[:sent]
   except BlockingIOError: pass
  if fd in readable:
   try: data=os.read(fd,4096)
   except BlockingIOError: data=b''
   if data: os.write(sys.stdout.fileno(),data)
finally:
 os.close(fd)
'''


def _held_sample(page: str, elapsed_s: float):
    base_name = "auto_healthy" if page == "auto" else "road"
    sample = deepcopy(SCENARIO_BUILDERS[base_name](0.0))
    if elapsed_s < 1.2:
        step = 0
        speed_cms = 125
    else:
        speeds = (425, 175, 560, 290)
        step = int((elapsed_s - 1.2) / 1.7)
        speed_cms = speeds[min(step, len(speeds) - 1)]
    sample.values.speed_centi_mps = speed_cms
    sample.values.pedal_percent = (18, 63, 31, 76, 42)[min(step, 4)]
    sample.values.bus_current_deciamp = (
        360, 1670, 730, 2380, 1040
    )[min(step, 4)]
    sample.values.phase_current_deciamp = (
        810, 3440, 1510, 4380, 2190
    )[min(step, 4)]
    if step == 2:
        sample.values.status_flags |= STATUS_GEAR_REVERSE
    if page == "auto":
        sample.values.status_flags |= STATUS_MODE_AUTO
    return sample


def _mode_holdout_sample(elapsed_s: float):
    sample = deepcopy(SCENARIO_BUILDERS["auto_healthy"](0.0))
    if elapsed_s < 1.4 or elapsed_s >= 4.9:
        sample.values.status_flags &= ~STATUS_MODE_AUTO
    else:
        sample.values.status_flags |= STATUS_MODE_AUTO
    return sample


def build_timetable(
    page: str,
    duration_s: float,
    scenario: str,
    *,
    held_steps: bool = False,
    mode_holdout: bool = False,
) -> list[dict[str, object]]:
    if page in ("startup", "transport", "startup_fixed"):
        return []
    records: list[dict[str, object]] = []
    alive = 0
    sample_us = 200_000
    end_us = round(duration_s * 1_000_000)
    while sample_us + 31_000 <= end_us:
        elapsed_s = (sample_us - 200_000) / 1_000_000
        if mode_holdout:
            sample = _mode_holdout_sample(elapsed_s)
        elif held_steps:
            sample = _held_sample(page, elapsed_s)
        else:
            sample = SCENARIO_BUILDERS[scenario](elapsed_s)
        payloads = _can_payloads(sample, alive)
        for slot, offset_us in enumerate((0, 10_000, 20_000, 30_000)):
            for frame_index, separation_us in ((slot, 0), (slot + 4, 1_000)):
                records.append({
                    "arrival_us": sample_us + offset_us + separation_us,
                    "id": FRAME_IDS[frame_index],
                    "payload": payloads[frame_index].hex().upper(),
                })
        sample_us += 50_000
        alive = (alive + 1) & 0xFF
    return records


def replay_host(
    page: str,
    duration_s: float,
    records: list[dict[str, object]],
    region_mask: int,
    timing_profile: Path | None,
) -> dict[str, object]:
    native = NativeDashboard(timing_profile)
    try:
        epoch_us = native.run_until_ready()
        native.clear_events(region_mask)
        history_before = native.auto_history_count
        if page == "startup":
            native.replay_startup()
            while not native.replay_active:
                native.run_until(native.now_us + 1_000)
            while native.replay_active:
                native.run_until(native.now_us + 1_000)
        else:
            for record in records:
                native.schedule_frame(
                    epoch_us + int(record["arrival_us"]), int(record["id"]),
                    bytes.fromhex(str(record["payload"])),
                )
            native.run_until(epoch_us + round(duration_s * 1_000_000))
        events = native.events()
        for event in events:
            event["relative_us"] = int(event["time_ms"]) * 1000 - epoch_us
        startup_frames = native.startup_frames()
        for event in startup_frames:
            event["relative_us"] = int(event["time_us"]) - epoch_us
        startup_start_us, startup_completed_us = native.startup_times_us
        startup = None
        if startup_start_us and startup_completed_us:
            startup = {
                "start_us": startup_start_us,
                "completed_us": startup_completed_us,
                "relative_start_us": startup_start_us - epoch_us,
                "relative_completed_us": startup_completed_us - epoch_us,
                "duration_us": startup_completed_us - startup_start_us,
            }
        return {
            "epoch_us": epoch_us,
            "events": events,
            "startup": startup,
            "startup_frames": startup_frames,
            "auto_history_observations": native.auto_history_count - history_before,
            "profile_calibrated": native.profile_calibrated,
        }
    finally:
        native.close()


def _parse_fields(line: str) -> dict[str, object]:
    fields: dict[str, object] = {}
    for token in line.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        try:
            fields[key] = int(value, 0)
        except ValueError:
            fields[key] = value
    return fields


def _capture_lines(lines: list[str]) -> dict[str, object]:
    result: dict[str, object] = {
        "state": None, "rows": [], "spi": [], "regions": [],
        "modes": [], "startup": None, "startup_frames": [],
        "transport": [], "meta": None,
    }
    for line in lines:
        fields = _parse_fields(line)
        if line.startswith("TIMING page="):
            result["meta"] = fields
        elif line.startswith("TIMING state "):
            result["state"] = fields
        elif line.startswith("TIMING row "):
            result["rows"].append(fields)
        elif line.startswith("TIMING spi "):
            result["spi"].append(fields)
        elif line.startswith("TIMING transport "):
            result["transport"].append(fields)
        elif line.startswith("TIMING region "):
            result["regions"].append(fields)
        elif line.startswith("MODE"):
            fields["kind"] = "mode_done" if line.startswith("MODE DONE") else "mode_start"
            result["modes"].append(fields)
        elif line.startswith("TIMING startup "):
            if fields.get("start_us", 0) and fields.get("completed_us", 0):
                result["startup"] = fields
        elif line.startswith("TIMING startup_frame "):
            result["startup_frames"].append(fields)
    return result


def _normalize_board_events(board: dict[str, object], epoch_us: int) -> None:
    for event in board["regions"]:
        event["relative_us"] = int(event["completed_ms"]) * 1000 - epoch_us
        event["kind"] = "region"
        event["region"] = int(event["bit"])
    for event in board["modes"]:
        event["relative_us"] = int(event["time"]) * 1000 - epoch_us
        event["direction_auto"] = event.get("direction") == "UP_AUTO"
    startup = board.get("startup")
    if startup:
        startup["relative_start_us"] = int(startup["start_us"]) - epoch_us
        startup["relative_completed_us"] = (
            int(startup["completed_us"]) - epoch_us
        )
        startup["duration_us"] = (
            int(startup["completed_us"]) - int(startup["start_us"])
        )
    for event in board.get("startup_frames", []):
        event["relative_us"] = int(event["time_us"]) - epoch_us


def _native_snapshots(records: list[dict[str, object]]) -> list[dict[str, int | bool]]:
    statuses: dict[int, tuple[int, int]] = {}
    snapshots: list[dict[str, int | bool]] = []
    for record in records:
        identifier = int(record["id"])
        payload = bytes.fromhex(str(record["payload"]))
        alive = payload[1]
        if identifier == FRAME_IDS[0]:
            statuses[alive] = (
                payload[2], struct.unpack_from("<h", payload, 6)[0]
            )
        elif identifier == FRAME_IDS[3] and alive in statuses:
            flags, speed = statuses.pop(alive)
            snapshots.append({
                "alive": alive,
                "completed_us": int(record["arrival_us"]),
                "flags": flags,
                "speed_centi_mps": speed,
                "auto": bool(
                    flags & STATUS_VEHICLE_VALID and flags & STATUS_MODE_AUTO
                ),
            })
    return snapshots


def _expected_transitions(
    snapshots: list[dict[str, int | bool]],
) -> list[dict[str, int | bool | str]]:
    expected: list[dict[str, int | bool | str]] = []
    displayed_auto = False
    for snapshot in snapshots:
        desired_auto = bool(snapshot["auto"])
        if desired_auto != displayed_auto:
            expected.append({
                "key": f"mode_{len(expected) + 1}",
                "trigger_us": int(snapshot["completed_us"]),
                "alive": int(snapshot["alive"]),
                "direction_auto": desired_auto,
            })
            displayed_auto = desired_auto
    return expected


def _held_speed_steps(
    snapshots: list[dict[str, int | bool]],
) -> list[dict[str, int | str]]:
    steps: list[dict[str, int | str]] = []
    previous_speed: int | None = None
    for snapshot in snapshots:
        speed = int(snapshot["speed_centi_mps"])
        if previous_speed is not None and speed != previous_speed:
            steps.append({
                "key": f"held_speed_{len(steps) + 1}",
                "trigger_us": int(snapshot["completed_us"]),
                "alive": int(snapshot["alive"]),
                "speed_centi_mps": speed,
            })
        previous_speed = speed
    return steps


def _event_keyed_regions(
    events: list[dict[str, object]],
    snapshots: list[dict[str, int | bool]],
) -> dict[tuple[int, int, int], dict[str, object]]:
    keyed: dict[tuple[int, int, int], dict[str, object]] = {}
    occurrences: dict[tuple[int, int], int] = {}
    for event in events:
        if event.get("kind") != "region":
            continue
        event_us = int(event["relative_us"])
        input_alive = -1
        for snapshot in snapshots:
            if int(snapshot["completed_us"]) <= event_us + 2_000:
                input_alive = int(snapshot["alive"])
            else:
                break
        base = (int(event["region"]), input_alive)
        occurrence = occurrences.get(base, 0)
        occurrences[base] = occurrence + 1
        keyed[(base[0], base[1], occurrence)] = event
    return keyed


def _append_comparison(
    comparisons: list[dict[str, object]],
    key: str,
    board_event: dict[str, object],
    host_event: dict[str, object],
    **fields: object,
) -> None:
    difference_us = (
        int(host_event["relative_us"]) - int(board_event["relative_us"])
    )
    comparisons.append({
        "key": key,
        **fields,
        "board_relative_us": int(board_event["relative_us"]),
        "host_relative_us": int(host_event["relative_us"]),
        "difference_us": difference_us,
        "within_20ms": abs(difference_us) <= 20_000,
    })


def _compare_capture_events(
    data: dict[str, object], host: dict[str, object]
) -> dict[str, object]:
    board = data["board"]
    board_events = sorted(
        [*board["regions"], *board["modes"]],
        key=lambda event: int(event["relative_us"]),
    )
    host_events = sorted(
        [event for event in host["events"]
         if event["kind"] in ("region", "mode_start", "mode_done")],
        key=lambda event: int(event["relative_us"]),
    )
    snapshots = _native_snapshots(data["input_timetable"])
    comparisons: list[dict[str, object]] = []
    missing: list[dict[str, object]] = []
    matched_board: set[int] = set()
    matched_host: set[int] = set()
    startup_frame_trajectory: list[dict[str, object]] = []

    if data["page"] == "startup":
        board_startup = board.get("startup")
        host_startup = host.get("startup")
        if board_startup and host_startup:
            board_event = {
                "relative_us": int(board_startup["duration_us"])
            }
            host_event = {"relative_us": int(host_startup["duration_us"])}
            _append_comparison(
                comparisons, "startup_play", board_event, host_event,
                kind="startup", boundary="playStartup_internal",
                board_start_us=int(board_startup["relative_start_us"]),
                board_completed_us=int(board_startup["relative_completed_us"]),
                host_start_us=int(host_startup["relative_start_us"]),
                host_completed_us=int(host_startup["relative_completed_us"]),
            )
        else:
            if not board_startup:
                missing.append({"key": "startup_play", "side": "board"})
            if not host_startup:
                missing.append({"key": "startup_play", "side": "host"})
        board_frames = board.get("startup_frames", [])
        host_frames = host.get("startup_frames", [])
        frame_count = max(len(board_frames), len(host_frames))
        for index in range(frame_count):
            board_frame = board_frames[index] if index < len(board_frames) else None
            host_frame = host_frames[index] if index < len(host_frames) else None
            startup_frame_trajectory.append({
                "index": index,
                "board": board_frame,
                "host": host_frame,
                "time_difference_us": (
                    int(host_frame["relative_us"]) -
                    int(board_frame["relative_us"])
                    if board_frame is not None and host_frame is not None
                    else None
                ),
                "elapsed_difference_ms": (
                    int(host_frame["elapsed_ms"]) -
                    int(board_frame["elapsed_ms"])
                    if board_frame is not None and host_frame is not None
                    else None
                ),
            })
    else:
        if data.get("held_steps"):
            steps = _held_speed_steps(snapshots)
            region = int(data["region_mask"])
            for index, step in enumerate(steps):
                start_us = int(step["trigger_us"])
                end_us = (
                    int(steps[index + 1]["trigger_us"])
                    if index + 1 < len(steps)
                    else round(float(data["duration_s"]) * 1_000_000) + 1
                )
                selected: dict[str, dict[str, object]] = {}
                for side, events, matched in (
                    ("board", board_events, matched_board),
                    ("host", host_events, matched_host),
                ):
                    candidates = [
                        (event_index, event) for event_index, event in enumerate(events)
                        if event.get("kind") == "region"
                        and int(event.get("region", 0)) == region
                        and start_us - 2_000 <= int(event["relative_us"]) < end_us
                    ]
                    if candidates:
                        event_index, event = candidates[0]
                        selected[side] = event
                        matched.add(event_index)
                    else:
                        missing.append({
                            "key": step["key"], "side": side,
                            "kind": "region", "region": region,
                            "input_alive": step["alive"],
                            "trigger_us": start_us,
                        })
                if len(selected) == 2:
                    _append_comparison(
                        comparisons, str(step["key"]),
                        selected["board"], selected["host"],
                        kind="region", region=region,
                        input_alive=step["alive"], trigger_us=start_us,
                        speed_centi_mps=step["speed_centi_mps"],
                    )
        else:
            board_regions = _event_keyed_regions(board_events, snapshots)
            host_regions = _event_keyed_regions(host_events, snapshots)
            for key in sorted(set(board_regions) | set(host_regions)):
                board_event = board_regions.get(key)
                host_event = host_regions.get(key)
                label = f"region_{key[0]:04x}_alive_{key[1]}_{key[2]}"
                if board_event is None:
                    missing.append({"key": label, "side": "board"})
                elif host_event is None:
                    missing.append({"key": label, "side": "host"})
                else:
                    _append_comparison(
                        comparisons, label, board_event, host_event,
                        kind="region", region=key[0], input_alive=key[1],
                    )
                    matched_board.add(board_events.index(board_event))
                    matched_host.add(host_events.index(host_event))

        transitions = _expected_transitions(snapshots)
        for index, transition in enumerate(transitions):
            start_us = int(transition["trigger_us"])
            end_us = (
                int(transitions[index + 1]["trigger_us"])
                if index + 1 < len(transitions)
                else round(float(data["duration_s"]) * 1_000_000) + 1
            )
            for kind in ("mode_start", "mode_done"):
                selected = {}
                for side, events, matched in (
                    ("board", board_events, matched_board),
                    ("host", host_events, matched_host),
                ):
                    candidates = [
                        (event_index, event) for event_index, event in enumerate(events)
                        if event.get("kind") == kind
                        and bool(event.get("direction_auto")) ==
                            bool(transition["direction_auto"])
                        and start_us - 2_000 <= int(event["relative_us"]) < end_us
                    ]
                    if candidates:
                        event_index, event = candidates[0]
                        selected[side] = event
                        matched.add(event_index)
                    else:
                        missing.append({
                            "key": f"{transition['key']}_{kind}", "side": side,
                            "kind": kind,
                            "direction_auto": transition["direction_auto"],
                            "input_alive": transition["alive"],
                            "trigger_us": start_us,
                        })
                if len(selected) == 2:
                    _append_comparison(
                        comparisons, f"{transition['key']}_{kind}",
                        selected["board"], selected["host"], kind=kind,
                        direction_auto=transition["direction_auto"],
                        input_alive=transition["alive"], trigger_us=start_us,
                    )

    board_meta = board.get("meta") or {}
    board_region_overflow = bool(board_meta.get("region_overflow", 0))
    board_mode_overflow = bool(board_meta.get("mode_overflow", 0))
    board_startup_frame_overflow = bool(
        board_meta.get("startup_frame_overflow", 0)
    )
    board_overflow = (
        board_region_overflow or board_mode_overflow or
        board_startup_frame_overflow
    )
    maximum = max((abs(int(item["difference_us"])) for item in comparisons),
                  default=None)
    return {
        "comparisons": comparisons,
        "startup_frame_trajectory": startup_frame_trajectory,
        "missing": missing,
        "unmatched_board_events": [
            event for index, event in enumerate(board_events)
            if index not in matched_board
        ],
        "unmatched_host_events": [
            event for index, event in enumerate(host_events)
            if index not in matched_host
        ],
        "board_region_event_overflow": board_region_overflow,
        "board_mode_event_overflow": board_mode_overflow,
        "board_startup_frame_event_overflow": board_startup_frame_overflow,
        "max_abs_difference_us": maximum,
        "passed_20ms": bool(comparisons) and not missing and not board_overflow
            and maximum is not None and maximum <= 20_000,
    }


def replay_capture(
    capture_path: Path,
    timing_profile: Path | None,
    output: Path,
) -> dict[str, object]:
    data = json.loads(capture_path.read_text(encoding="utf-8"))
    host = replay_host(
        data["page"], float(data["duration_s"]), data["input_timetable"],
        int(data["region_mask"]), timing_profile,
    )
    comparison = _compare_capture_events(data, host)
    result = {"capture": str(capture_path), "host": host, **comparison}
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def aggregate_profile(captures: list[Path], output: Path) -> dict[str, object]:
    pages = ("startup", "manual", "auto")
    state_sum = {page: 0 for page in pages}
    state_count = {page: 0 for page in pages}
    row_sum = {page: [0] * 320 for page in pages}
    row_count = {page: [0] * 320 for page in pages}
    spi_overhead = 0
    spi_count = 0
    for path in captures:
        data = json.loads(path.read_text(encoding="utf-8"))
        page = data["page"]
        state = data["board"].get("state")
        if state:
            state_sum[page] += int(state.get("sum_us", 0))
            state_count[page] += int(state.get("count", 0))
        for row in data["board"].get("rows", []):
            y = int(row["y"])
            row_sum[page][y] += int(row.get("sum_us", 0))
            row_count[page][y] += int(row.get("count", 0))
        for spi in data["board"].get("spi", []):
            spi_overhead += int(spi.get("overhead_us", 0))
            spi_count += int(spi.get("count", 0))
    profile = {
        "calibrated": False,
        "state_prepare_us": {
            page: round(state_sum[page] / state_count[page])
            if state_count[page] else 0 for page in pages
        },
        "row_compose_us": {
            page: [
                round(total / count) if count else 0
                for total, count in zip(row_sum[page], row_count[page])
            ] for page in pages
        },
        "spi_call_us": round(spi_overhead / spi_count) if spi_count else 0,
        "loop_us": 10,
        "source_captures": [str(path) for path in captures],
        "coverage": {
            "state_unmeasured_pages": [
                page for page in pages if state_count[page] == 0
            ],
            "row_unmeasured": {
                page: [index for index, count in enumerate(row_count[page])
                       if count == 0]
                for page in pages
            },
        },
    }
    output.write_text(json.dumps(profile, indent=2), encoding="utf-8")
    return profile


def merge_profile_components(
    base_profile: Path,
    transport_capture: Path,
    startup_captures: list[Path],
    output: Path,
) -> dict[str, object]:
    profile = json.loads(base_profile.read_text(encoding="utf-8"))
    transport_data = json.loads(
        transport_capture.read_text(encoding="utf-8")
    )
    metrics = transport_data["board"].get("transport", [])
    by_key = {
        (str(metric["kind"]), int(metric.get("length", 0))): metric
        for metric in metrics
    }

    def mean_ns(kind: str, length: int = 0, wire_bytes: int = 0) -> int:
        metric = by_key.get((kind, length))
        if metric is None or int(metric.get("count", 0)) == 0:
            raise ValueError(f"missing transport measurement: {kind}[{length}]")
        count = int(metric["count"])
        measured_ns = int(metric["sum_us"]) * 1000
        wire_ns = round(count * wire_bytes * 8_000_000_000 / 12_000_000)
        return max(0, round((measured_ns - wire_ns) / count))

    spi_buffer = [0] + [0] * 256
    for length in range(2, 257):
        spi_buffer[length] = mean_ns(
            "spi_buffer", length, wire_bytes=length
        )
    spi_buffer[1] = mean_ns("spi_byte", wire_bytes=1)
    rgb565_pack = [0] + [
        mean_ns("rgb565_pack", pixels) for pixels in range(1, 129)
    ]
    solid_pack = [0] + [
        mean_ns("solid_pack", pixels) for pixels in range(1, 129)
    ]

    startup_by_elapsed: dict[int, dict[str, object]] = {}
    for path in startup_captures:
        data = json.loads(path.read_text(encoding="utf-8"))
        elapsed_ms = int(data["startup_elapsed_ms"])
        if elapsed_ms in startup_by_elapsed:
            raise ValueError(f"duplicate startup elapsed_ms: {elapsed_ms}")
        startup_by_elapsed[elapsed_ms] = data["board"]
    elapsed_values = list(range(0, 1401, 10))
    if sorted(startup_by_elapsed) != elapsed_values:
        missing = sorted(set(elapsed_values) - set(startup_by_elapsed))
        raise ValueError(f"startup calibration is incomplete: {missing}")

    startup_state: list[int] = []
    startup_rows: list[list[int]] = []
    for elapsed_ms in elapsed_values:
        board = startup_by_elapsed[elapsed_ms]
        state = board.get("state")
        if state is None or int(state.get("count", 0)) == 0:
            raise ValueError(f"missing startup state at {elapsed_ms} ms")
        startup_state.append(
            round(int(state["sum_us"]) / int(state["count"]))
        )
        rows = [0] * 320
        for row in board.get("rows", []):
            y = int(row["y"])
            count = int(row.get("count", 0))
            if count:
                rows[y] = round(int(row["sum_us"]) / count)
        startup_rows.append(rows)

    profile["calibrated"] = False
    profile["spi_call_us"] = 0
    profile["transport"] = {
        "transaction_begin_ns": mean_ns("transaction_begin"),
        "transaction_end_ns": mean_ns("transaction_end"),
        "spi_byte_extra_ns": mean_ns("spi_byte", wire_bytes=1),
        "spi_buffer_extra_ns": spi_buffer,
        "rgb565_pack_ns": rgb565_pack,
        "solid_pack_ns": solid_pack,
    }
    profile["startup_cpu"] = {
        "elapsed_ms": elapsed_values,
        "state_prepare_us": startup_state,
        "row_compose_us": startup_rows,
    }
    profile["component_captures"] = {
        "base_profile": str(base_profile),
        "transport_capture": str(transport_capture),
        "startup_captures": [str(path) for path in startup_captures],
    }
    output.write_text(json.dumps(profile, indent=2), encoding="utf-8")
    return profile


def capture_board(
    page: str,
    board_host: str,
    board_port: str,
    duration_s: float,
    scenario: str,
    output: Path,
    *,
    timing_profile: Path | None = None,
    held_steps: bool = False,
    startup_elapsed_ms: int | None = None,
    mode_holdout: bool = False,
) -> dict[str, object]:
    region_mask = DEFAULT_REGION_MASK[page]
    records = build_timetable(
        page, duration_s, scenario, held_steps=held_steps,
        mode_holdout=mode_holdout,
    )
    remote_command = (
        f"python3 -u -c {shlex.quote(REMOTE_BRIDGE)} {shlex.quote(board_port)}"
    )
    process = subprocess.Popen(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
         board_host, remote_command], stdin=subprocess.PIPE,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0,
    )
    lines: list[str] = []
    buffer = bytearray()
    ready_epoch: int | None = None
    bench_ended = False
    sent = 0
    started = time.monotonic()
    try:
        assert process.stdin is not None and process.stdout is not None
        settle_started = time.monotonic()
        quiet_since = settle_started
        while (
            time.monotonic() - settle_started < BOARD_SERIAL_SETTLE_S
            or time.monotonic() - quiet_since < BOARD_SERIAL_QUIET_S
        ):
            if time.monotonic() - settle_started >= BOARD_SERIAL_SETTLE_LIMIT_S:
                raise TimeoutError("serial output did not become idle before bench run")
            readable, _, _ = select.select([process.stdout], [], [], 0.05)
            if process.stdout in readable:
                chunk = os.read(process.stdout.fileno(), 4096)
                if not chunk and process.poll() is not None:
                    raise RuntimeError("remote serial bridge exited during settle")
                if chunk:
                    quiet_since = time.monotonic()
                buffer.extend(chunk)
                while b"\n" in buffer:
                    raw, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    lines.append(raw.rstrip(b"\r").decode("utf-8", "replace"))
        command = (
            f"F {startup_elapsed_ms}\n"
            if page == "startup_fixed"
            else f"B {PAGE_CODES[page]} {round(duration_s * 1000)} {region_mask:04X}\n"
        )
        process.stdin.write(command.encode() + b"\n" * 64)
        process.stdin.flush()
        command_sent = time.monotonic()
        expected_lead_us = (
            BENCH_EPOCH_LEAD_US if page in ("manual", "auto") else 0
        )
        started = command_sent + expected_lead_us / 1_000_000
        deadline = started + duration_s + 40.0
        while time.monotonic() < deadline:
            elapsed_us = round((time.monotonic() - started) * 1_000_000)
            while sent < len(records) and int(records[sent]["arrival_us"]) <= elapsed_us + 100_000:
                record = records[sent]
                process.stdin.write(
                    f"C {record['arrival_us']} {int(record['id']):03X} {record['payload']}\n".encode()
                )
                sent += 1
            process.stdin.flush()
            readable, _, _ = select.select([process.stdout], [], [], 0.01)
            if process.stdout in readable:
                chunk = os.read(process.stdout.fileno(), 4096)
                if not chunk and process.poll() is not None:
                    break
                buffer.extend(chunk)
                while b"\n" in buffer:
                    raw, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    line = raw.rstrip(b"\r").decode("utf-8", "replace")
                    lines.append(line)
                    if line.startswith("BENCH READY "):
                        ready_fields = _parse_fields(line)
                        ready_epoch = int(ready_fields["epoch_us"])
                        if int(ready_fields["lead_us"]) != expected_lead_us:
                            raise RuntimeError("bench epoch lead mismatch")
                        lines = [line]
                    if line.startswith("BENCH ERROR "):
                        raise RuntimeError(line)
                    if ready_epoch is not None and line.startswith("BENCH END "):
                        bench_ended = True
                        board_data = _capture_lines(lines)
                        _normalize_board_events(board_data, ready_epoch)
                        host_data = (
                            replay_host(
                                page, duration_s, records, region_mask,
                                timing_profile,
                            )
                            if page in ("startup", "manual", "auto") else None
                        )
                        result = {
                            "page": page, "duration_s": duration_s,
                            "scenario": scenario,
                            "held_steps": held_steps,
                            "mode_holdout": mode_holdout,
                            "startup_elapsed_ms": startup_elapsed_ms,
                            "region_mask": region_mask, "board_epoch_us": ready_epoch,
                            "input_timetable": records, "board": board_data,
                            "host": host_data, "raw_lines": lines,
                        }
                        output.write_text(json.dumps(result, indent=2), encoding="utf-8")
                        return result
        raise TimeoutError("BENCH END was not received")
    finally:
        if process.stdin is not None and not process.stdin.closed:
            if ready_epoch is not None and not bench_ended and process.poll() is None:
                try:
                    process.stdin.write(b"E\n")
                    process.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass
            process.stdin.close()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        check = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
             board_host, f"fuser {shlex.quote(board_port)} 2>/dev/null || true"],
            check=True, capture_output=True, text=True,
        )
        if check.stdout.strip():
            raise RuntimeError(f"serial port still owned: {check.stdout.strip()}")


def capture_startup_series(
    board_host: str,
    board_port: str,
    output_dir: Path,
) -> list[Path]:
    remote_command = (
        f"python3 -u -c {shlex.quote(REMOTE_BRIDGE)} {shlex.quote(board_port)}"
    )
    process = subprocess.Popen(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
         board_host, remote_command],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        bufsize=0,
    )
    buffer = bytearray()
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs: list[Path] = []
    try:
        assert process.stdin is not None and process.stdout is not None
        settle_started = time.monotonic()
        quiet_since = settle_started
        while (
            time.monotonic() - settle_started < BOARD_SERIAL_SETTLE_S
            or time.monotonic() - quiet_since < BOARD_SERIAL_QUIET_S
        ):
            if time.monotonic() - settle_started >= BOARD_SERIAL_SETTLE_LIMIT_S:
                raise TimeoutError("serial output did not become idle")
            readable, _, _ = select.select([process.stdout], [], [], 0.05)
            if process.stdout in readable:
                chunk = os.read(process.stdout.fileno(), 4096)
                if not chunk and process.poll() is not None:
                    raise RuntimeError("remote serial bridge exited during settle")
                if chunk:
                    quiet_since = time.monotonic()
                buffer.extend(chunk)
                while b"\n" in buffer:
                    _, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)

        for elapsed_ms in range(0, 1401, 10):
            process.stdin.write(f"F {elapsed_ms}\n".encode() + b"\n" * 64)
            process.stdin.flush()
            lines: list[str] = []
            ready_epoch: int | None = None
            deadline = time.monotonic() + 20.0
            complete = False
            while time.monotonic() < deadline:
                readable, _, _ = select.select([process.stdout], [], [], 0.05)
                if process.stdout not in readable:
                    continue
                chunk = os.read(process.stdout.fileno(), 4096)
                if not chunk and process.poll() is not None:
                    raise RuntimeError("remote serial bridge exited")
                buffer.extend(chunk)
                while b"\n" in buffer:
                    raw, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    line = raw.rstrip(b"\r").decode("utf-8", "replace")
                    if line.startswith("BENCH READY "):
                        fields = _parse_fields(line)
                        ready_epoch = int(fields["epoch_us"])
                        if int(fields["lead_us"]) != 0:
                            raise RuntimeError("fixed startup epoch lead mismatch")
                        lines = [line]
                    elif ready_epoch is not None:
                        lines.append(line)
                    if ready_epoch is not None and line.startswith("BENCH ERROR "):
                        raise RuntimeError(line)
                    if ready_epoch is not None and line.startswith("BENCH END "):
                        board_data = _capture_lines(lines)
                        _normalize_board_events(board_data, ready_epoch)
                        result = {
                            "page": "startup_fixed",
                            "duration_s": 5.0,
                            "scenario": "fixed_startup_frame",
                            "held_steps": False,
                            "mode_holdout": False,
                            "startup_elapsed_ms": elapsed_ms,
                            "region_mask": 0,
                            "board_epoch_us": ready_epoch,
                            "input_timetable": [],
                            "board": board_data,
                            "host": None,
                            "raw_lines": lines,
                        }
                        output = output_dir / f"cal-startup-{elapsed_ms:04d}.json"
                        output.write_text(
                            json.dumps(result, indent=2), encoding="utf-8"
                        )
                        outputs.append(output)
                        complete = True
                        break
                if complete:
                    break
            if not complete:
                raise TimeoutError(
                    f"BENCH END was not received for {elapsed_ms} ms"
                )
            time.sleep(0.02)
        return outputs
    finally:
        if process.stdin is not None and not process.stdin.closed:
            process.stdin.close()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        check = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
             board_host,
             f"fuser {shlex.quote(board_port)} 2>/dev/null || true"],
            check=True, capture_output=True, text=True,
        )
        if check.stdout.strip():
            raise RuntimeError(f"serial port still owned: {check.stdout.strip()}")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=SCENARIOS, default="road")
    parser.add_argument("--timing-profile", type=Path)
    parser.add_argument(
        "--board-capture",
        choices=("startup", "manual", "auto", "transport", "startup_fixed"),
    )
    parser.add_argument("--board-host", default="kai")
    parser.add_argument("--board-port")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--capture-output", type=Path)
    parser.add_argument("--held-steps", action="store_true")
    parser.add_argument("--mode-holdout", action="store_true")
    parser.add_argument("--startup-elapsed-ms", type=int)
    parser.add_argument("--replay-capture", type=Path)
    parser.add_argument("--profile-from-captures", type=Path, nargs="+")
    parser.add_argument("--profile-output", type=Path)
    parser.add_argument("--base-profile", type=Path)
    parser.add_argument("--transport-capture", type=Path)
    parser.add_argument("--startup-captures", type=Path, nargs="+")
    parser.add_argument("--startup-series-output", type=Path)
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    if args.startup_series_output is not None:
        if args.board_port is None:
            raise SystemExit("--board-port is required")
        outputs = capture_startup_series(
            args.board_host, args.board_port, args.startup_series_output
        )
        print(f"saved {len(outputs)} fixed startup captures")
        return 0
    if args.base_profile is not None:
        if (args.transport_capture is None or args.startup_captures is None or
                args.profile_output is None):
            raise SystemExit(
                "--transport-capture, --startup-captures, and "
                "--profile-output are required for profile components"
            )
        merge_profile_components(
            args.base_profile, args.transport_capture,
            args.startup_captures, args.profile_output,
        )
        print(f"saved {args.profile_output} (profile components calibrated=false)")
        return 0
    if args.profile_from_captures is not None:
        if args.profile_output is None:
            raise SystemExit("--profile-output is required")
        aggregate_profile(args.profile_from_captures, args.profile_output)
        print(f"saved {args.profile_output} (calibrated=false candidate profile)")
        return 0
    if args.replay_capture is not None:
        if args.capture_output is None:
            raise SystemExit("--capture-output is required")
        result = replay_capture(
            args.replay_capture, args.timing_profile, args.capture_output
        )
        print(
            f"saved {args.capture_output} "
            f"({len(result['comparisons'])} event comparisons, "
            f"passed_20ms={result['passed_20ms']})"
        )
        return 0 if result["passed_20ms"] else 2
    if args.board_capture is not None:
        if args.board_port is None or args.capture_output is None:
            raise SystemExit("--board-port and --capture-output are required")
        if args.board_capture == "startup_fixed" and (
            args.startup_elapsed_ms is None or
            not 0 <= args.startup_elapsed_ms <= 1400
        ):
            raise SystemExit(
                "--startup-elapsed-ms 0..1400 is required for startup_fixed"
            )
        result = capture_board(
            args.board_capture, args.board_host, args.board_port,
            args.duration, args.scenario, args.capture_output,
            timing_profile=args.timing_profile, held_steps=args.held_steps,
            startup_elapsed_ms=args.startup_elapsed_ms,
            mode_holdout=args.mode_holdout,
        )
        print(
            f"saved {args.capture_output} ({result['page']}, "
            f"{len(result['input_timetable'])} CAN records)"
        )
        return 0
    parser.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
