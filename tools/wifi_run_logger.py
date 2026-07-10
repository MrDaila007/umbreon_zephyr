#!/usr/bin/env python3
"""Collect RUN telemetry over the ESP8266 WiFi bridge."""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import select
import socket
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path


CSV_RE = re.compile(r"^\d+,")
BOOT_MARKERS = ("$BOOT", "BOOT", "Zephyr")
FAULT_MARKERS = ("USAGE FAULT", "HARD FAULT", "FATAL", "ASSERT", "panic", "Kernel panic", "<err>")


class StopLogging(Exception):
    """Raised when robot telemetry says the run has stopped."""


@dataclass
class Sample:
    line_no: int
    ms: int
    sensors: list[int]
    steer: int
    speed: float
    target: float
    yaw: float
    heading: float


@dataclass
class Summary:
    lines: int = 0
    csv_valid: int = 0
    csv_malformed: int = 0
    first_ms: int | None = None
    last_ms: int | None = None
    timestamp_gaps: list[dict] = field(default_factory=list)
    timestamp_backwards: list[dict] = field(default_factory=list)
    boot_markers: list[dict] = field(default_factory=list)
    fault_markers: list[dict] = field(default_factory=list)
    statuses: list[dict] = field(default_factory=list)
    run_lines: int = 0
    battery_values: list[float] = field(default_factory=list)
    power: list[dict] = field(default_factory=list)
    cfg: dict[str, str] = field(default_factory=dict)
    pid: list[dict] = field(default_factory=list)
    max_speed: float = 0.0
    max_target: float = 0.0
    max_abs_steer: int = 0
    speed_error_abs_sum: float = 0.0
    speed_error_count: int = 0
    overshoot_max: float = 0.0
    undershoot_max: float = 0.0
    sensor_min: list[int | None] = field(default_factory=lambda: [None] * 6)
    sensor_max: list[int | None] = field(default_factory=lambda: [None] * 6)


class Link:
    def send_line(self, command: str) -> None:
        raise NotImplementedError

    def read_available(self, timeout_s: float) -> list[str]:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


class TcpLink(Link):
    def __init__(self, host: str, port: int, timeout_s: float) -> None:
        self.sock = socket.create_connection((host, port), timeout=timeout_s)
        self.sock.setblocking(False)
        self.buf = b""

    def send_line(self, command: str) -> None:
        line = command if command.endswith("\n") else command + "\n"
        self.sock.sendall(line.encode("ascii", errors="replace"))

    def read_available(self, timeout_s: float) -> list[str]:
        lines: list[str] = []
        readable, _, _ = select.select([self.sock], [], [], timeout_s)
        if not readable:
            return lines
        while True:
            try:
                chunk = self.sock.recv(4096)
            except BlockingIOError:
                break
            if not chunk:
                raise ConnectionError("TCP connection closed")
            self.buf += chunk
            if len(chunk) < 4096:
                break
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            lines.append(raw.decode("utf-8", errors="replace").rstrip("\r"))
        return lines

    def close(self) -> None:
        self.sock.close()


class WebSocketLink(Link):
    def __init__(self, host: str, port: int, timeout_s: float) -> None:
        self.sock = socket.create_connection((host, port), timeout=timeout_s)
        self.sock.setblocking(True)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        req = (
            "GET / HTTP/1.1\r\n"
            f"Host: {host}:{port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(req.encode("ascii"))
        resp = self.sock.recv(1024)
        if b" 101 " not in resp and b" 101\r\n" not in resp:
            raise ConnectionError("WebSocket upgrade failed")
        self.sock.setblocking(False)
        self.buf = b""

    def send_line(self, command: str) -> None:
        payload = (command if command.endswith("\n") else command + "\n").encode("ascii", errors="replace")
        mask = os.urandom(4)
        if len(payload) < 126:
            header = struct.pack("!BB", 0x81, 0x80 | len(payload))
        else:
            header = struct.pack("!BBH", 0x81, 0x80 | 126, len(payload))
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        self.sock.sendall(header + mask + masked)

    def read_available(self, timeout_s: float) -> list[str]:
        lines: list[str] = []
        readable, _, _ = select.select([self.sock], [], [], timeout_s)
        if not readable:
            return lines
        while True:
            try:
                chunk = self.sock.recv(4096)
            except BlockingIOError:
                break
            if not chunk:
                raise ConnectionError("WebSocket connection closed")
            self.buf += chunk
            if len(chunk) < 4096:
                break

        while len(self.buf) >= 2:
            opcode = self.buf[0] & 0x0F
            masked = (self.buf[1] & 0x80) != 0
            length = self.buf[1] & 0x7F
            header_len = 2
            if length == 126:
                if len(self.buf) < 4:
                    break
                length = struct.unpack("!H", self.buf[2:4])[0]
                header_len = 4
            elif length == 127:
                if len(self.buf) < 10:
                    break
                length = struct.unpack("!Q", self.buf[2:10])[0]
                header_len = 10
            mask_len = 4 if masked else 0
            frame_len = header_len + mask_len + length
            if len(self.buf) < frame_len:
                break
            payload = self.buf[header_len + mask_len:frame_len]
            if masked:
                key = self.buf[header_len:header_len + 4]
                payload = bytes(b ^ key[i % 4] for i, b in enumerate(payload))
            self.buf = self.buf[frame_len:]
            if opcode == 0x1:
                text = payload.decode("utf-8", errors="replace")
                lines.extend(text.rstrip("\r\n").splitlines())
            elif opcode == 0x8:
                raise ConnectionError("WebSocket close frame received")
        return lines

    def close(self) -> None:
        self.sock.close()


def parse_key_values(payload: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for part in payload.split(","):
        if "=" in part:
            key, value = part.split("=", 1)
            out[key.strip()] = value.strip()
    return out


def parse_csv(line_no: int, line: str) -> Sample | None:
    if not CSV_RE.match(line):
        return None
    parts = line.split(",")
    if len(parts) != 12:
        raise ValueError("wrong field count")
    return Sample(
        line_no=line_no,
        ms=int(parts[0]),
        sensors=[int(v) for v in parts[1:7]],
        steer=int(parts[7]),
        speed=float(parts[8]),
        target=float(parts[9]),
        yaw=float(parts[10]),
        heading=float(parts[11]),
    )


def analyze(lines: list[str], gap_ms: int) -> Summary:
    summary = Summary()
    prev: Sample | None = None
    for line_no, raw in enumerate(lines, 1):
        line = raw.strip()
        summary.lines = line_no
        if not line:
            continue
        if any(marker in line for marker in BOOT_MARKERS):
            summary.boot_markers.append({"line": line_no, "text": line[:220]})
        if any(marker in line for marker in FAULT_MARKERS):
            summary.fault_markers.append({"line": line_no, "text": line[:220]})
        if line.startswith("$STS:"):
            summary.statuses.append({"line": line_no, "text": line})
        elif line.startswith("$RUN:"):
            summary.run_lines += 1
        elif line.startswith("$BAT:"):
            try:
                v = float(line[5:])
                if v > 0.1:
                    summary.battery_values.append(v)
            except ValueError:
                pass
        elif line.startswith("$PWR:"):
            fields = parse_key_values(line[5:])
            if fields:
                fields["line"] = str(line_no)
                summary.power.append(fields)
        elif line.startswith("$CFG:"):
            summary.cfg.update(parse_key_values(line[5:]))
        elif line.startswith("$PID:"):
            fields = parse_key_values(line[5:])
            if fields:
                fields["line"] = str(line_no)
                summary.pid.append(fields)

        if line and line[0].isdigit():
            try:
                sample = parse_csv(line_no, line)
            except ValueError:
                summary.csv_malformed += 1
                continue
            if sample is None:
                continue
            summary.csv_valid += 1
            summary.first_ms = sample.ms if summary.first_ms is None else summary.first_ms
            summary.last_ms = sample.ms
            summary.max_speed = max(summary.max_speed, sample.speed)
            summary.max_target = max(summary.max_target, sample.target)
            summary.max_abs_steer = max(summary.max_abs_steer, abs(sample.steer))
            if sample.target > 0.01:
                err = sample.speed - sample.target
                summary.speed_error_abs_sum += abs(err)
                summary.speed_error_count += 1
                summary.overshoot_max = max(summary.overshoot_max, err)
                summary.undershoot_max = max(summary.undershoot_max, -err)
            for idx, value in enumerate(sample.sensors):
                summary.sensor_min[idx] = value if summary.sensor_min[idx] is None else min(summary.sensor_min[idx], value)
                summary.sensor_max[idx] = value if summary.sensor_max[idx] is None else max(summary.sensor_max[idx], value)
            if prev is not None:
                dt = sample.ms - prev.ms
                if dt < 0:
                    summary.timestamp_backwards.append({"line": line_no, "prev_ms": prev.ms, "ms": sample.ms, "delta_ms": dt})
                elif dt > gap_ms:
                    summary.timestamp_gaps.append({"line": line_no, "prev_ms": prev.ms, "ms": sample.ms, "delta_ms": dt})
            prev = sample
    return summary


def send(link: Link, raw_log, command: str, mirror: bool) -> None:
    raw_log.write(f"> {command}\n")
    raw_log.flush()
    if mirror:
        print(f"> {command}")
    link.send_line(command)


def should_stop_on_line(line: str, state: dict[str, bool]) -> str | None:
    if line.startswith("$STS:RUN") or line.startswith("$STS:STARTING"):
        state["active_seen"] = True
    elif line.startswith("$RUN:") or CSV_RE.match(line):
        state["active_seen"] = True
    elif state.get("active_seen", False) and any(marker in line for marker in BOOT_MARKERS):
        return f"robot rebooted: {line[:120]}"
    elif line.startswith("$STS:STOP") and state.get("active_seen", False):
        state["stop_seen"] = True
        return "robot reported $STS:STOP"
    return None


def drain(
    link: Link,
    raw_log,
    lines: list[str],
    until: float,
    mirror: bool,
    state: dict[str, bool],
    stop_on_status: bool,
) -> None:
    while time.monotonic() < until:
        for line in link.read_available(0.1):
            raw_log.write(line + "\n")
            lines.append(line)
            if mirror:
                print(line)
            stop_reason = should_stop_on_line(line, state)
            if stop_on_status and stop_reason:
                raw_log.flush()
                raise StopLogging(stop_reason)
    raw_log.flush()


def write_summary(path: Path, args, summary: Summary, raw_log: Path, stop_reason: str) -> None:
    mean_abs_error = (
        summary.speed_error_abs_sum / summary.speed_error_count
        if summary.speed_error_count else None
    )
    payload = {
        "raw_log": str(raw_log),
        "host": args.host,
        "transport": args.transport,
        "duration_s": args.duration,
        "started_by_logger": args.start,
        "stop_reason": stop_reason,
        "analysis": {
            "lines": summary.lines,
            "csv_valid": summary.csv_valid,
            "csv_malformed": summary.csv_malformed,
            "first_ms": summary.first_ms,
            "last_ms": summary.last_ms,
            "span_s": ((summary.last_ms - summary.first_ms) / 1000.0
                       if summary.first_ms is not None and summary.last_ms is not None else 0.0),
            "timestamp_gaps": summary.timestamp_gaps[:30],
            "timestamp_backwards": summary.timestamp_backwards[:30],
            "boot_markers": summary.boot_markers[:20],
            "fault_markers": summary.fault_markers[:20],
            "statuses": summary.statuses[-30:],
            "run_lines": summary.run_lines,
            "battery_min": min(summary.battery_values) if summary.battery_values else None,
            "battery_max": max(summary.battery_values) if summary.battery_values else None,
            "power_snapshots": summary.power[-60:],
            "cfg": summary.cfg,
            "pid_snapshots": summary.pid[-60:],
            "max_speed": summary.max_speed,
            "max_target": summary.max_target,
            "mean_abs_speed_error": mean_abs_error,
            "overshoot_max": summary.overshoot_max,
            "undershoot_max": summary.undershoot_max,
            "max_abs_steer": summary.max_abs_steer,
            "sensor_min": summary.sensor_min,
            "sensor_max": summary.sensor_max,
        },
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--transport", choices=("tcp", "ws"), default="tcp")
    parser.add_argument("--port", type=int, help="default: 23 for tcp, 81 for ws")
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--prelude-s", type=float, default=2.0)
    parser.add_argument("--pid-interval", type=float, default=1.0)
    parser.add_argument("--status-interval", type=float, default=5.0)
    parser.add_argument("--gap-ms", type=int, default=250)
    parser.add_argument("--start", action="store_true", help="send $START after reading settings")
    parser.add_argument("--no-stop", action="store_true", help="do not send $STOP when logging ends")
    parser.add_argument("--no-stop-on-status", action="store_true", help="keep logging after external $STS:STOP")
    parser.add_argument("--command", action="append", default=[], help="extra command before RUN, for example '$SET:TGF=500'")
    parser.add_argument("--raw-log", type=Path, default=Path("/tmp/umbreon_wifi_run.log"))
    parser.add_argument("--summary", type=Path, default=Path("/tmp/umbreon_wifi_run.json"))
    parser.add_argument("--mirror", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port if args.port is not None else (23 if args.transport == "tcp" else 81)
    args.raw_log.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    link: Link | None = None
    lines: list[str] = []
    state = {"active_seen": False, "stop_seen": False}
    stop_reason = "duration"
    try:
        link = TcpLink(args.host, port, args.connect_timeout) if args.transport == "tcp" else WebSocketLink(args.host, port, args.connect_timeout)
        with args.raw_log.open("w", encoding="utf-8", errors="replace") as raw_log:
            try:
                for command in ("$GET", "$STATUS", "$BAT", "$PWR", "$PID", "$SYS", "$DIAG"):
                    send(link, raw_log, command, args.mirror)
                    drain(link, raw_log, lines, time.monotonic() + 0.35, args.mirror,
                          state, False)
                drain(link, raw_log, lines, time.monotonic() + args.prelude_s, args.mirror,
                      state, False)
                for command in args.command:
                    send(link, raw_log, command, args.mirror)
                    drain(link, raw_log, lines, time.monotonic() + 0.5, args.mirror,
                          state, not args.no_stop_on_status)
                if args.start:
                    send(link, raw_log, "$START", args.mirror)
                end = time.monotonic() + args.duration
                next_pid = time.monotonic() + args.pid_interval
                next_status = time.monotonic() + args.status_interval
                while time.monotonic() < end:
                    now = time.monotonic()
                    if args.pid_interval > 0 and now >= next_pid:
                        send(link, raw_log, "$PID", args.mirror)
                        next_pid = now + args.pid_interval
                    if args.status_interval > 0 and now >= next_status:
                        send(link, raw_log, "$STATUS", args.mirror)
                        send(link, raw_log, "$BAT", args.mirror)
                        send(link, raw_log, "$PWR", args.mirror)
                        next_status = now + args.status_interval
                    drain(link, raw_log, lines, min(end, now + 0.2), args.mirror,
                          state, not args.no_stop_on_status)
            except StopLogging as exc:
                stop_reason = str(exc)
            except (ConnectionError, ConnectionResetError, BrokenPipeError, OSError) as exc:
                stop_reason = f"connection closed: {exc}"
            except KeyboardInterrupt:
                stop_reason = "interrupted"
            finally:
                if not args.no_stop and not state.get("stop_seen", False):
                    try:
                        send(link, raw_log, "$STOP", args.mirror)
                        drain(link, raw_log, lines, time.monotonic() + 1.5, args.mirror,
                              state, False)
                    except (ConnectionError, ConnectionResetError, BrokenPipeError, OSError):
                        pass
    finally:
        if link is not None:
            link.close()

    if args.raw_log.exists():
        lines = [line for line in args.raw_log.read_text(errors="ignore").splitlines()
                 if not line.startswith("> ")]
    summary = analyze(lines, args.gap_ms)
    write_summary(args.summary, args, summary, args.raw_log, stop_reason)
    print(f"raw_log={args.raw_log}")
    print(f"summary={args.summary}")
    print(f"stop_reason={stop_reason}")
    print(
        "csv_valid={csv} run_lines={run} pid={pid} max_speed={speed:.2f} max_target={target:.2f} bat_min={bat}".format(
            csv=summary.csv_valid,
            run=summary.run_lines,
            pid=len(summary.pid),
            speed=summary.max_speed,
            target=summary.max_target,
            bat=(f"{min(summary.battery_values):.2f}" if summary.battery_values else "n/a"),
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
