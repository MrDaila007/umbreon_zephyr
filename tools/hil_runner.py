#!/usr/bin/env python3
"""Hardware-in-the-loop UART test runner for Umbreon firmware."""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover
    serial = None


BOOT_MARKERS = (
    "BOOT",
    "Zephyr",
    "Firmware boot",
    "$BOOT",
)

FAULT_MARKERS = (
    "USAGE FAULT",
    "HARD FAULT",
    "FATAL",
    "ASSERT",
    "panic",
    "Kernel panic",
    "<err>",
)

CSV_RE = re.compile(r"^\d+,")


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
class Analysis:
    lines: int = 0
    csv_valid: int = 0
    csv_malformed: int = 0
    first_ms: int | None = None
    last_ms: int | None = None
    timestamp_backwards: list[dict] = field(default_factory=list)
    timestamp_gaps: list[dict] = field(default_factory=list)
    boot_markers: list[dict] = field(default_factory=list)
    fault_markers: list[dict] = field(default_factory=list)
    statuses: list[dict] = field(default_factory=list)
    run_lines: int = 0
    battery_values: list[float] = field(default_factory=list)
    max_speed: float = 0.0
    max_target: float = 0.0
    max_abs_steer: int = 0
    sensor_min: list[int | None] = field(default_factory=lambda: [None] * 6)
    sensor_max: list[int | None] = field(default_factory=lambda: [None] * 6)


class HilFailure(RuntimeError):
    pass


def decode(raw: bytes) -> str:
    return raw.decode("utf-8", errors="replace").rstrip("\r\n")


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


def analyze_lines(lines: Iterable[str], gap_ms: int) -> Analysis:
    out = Analysis()
    prev: Sample | None = None

    for line_no, raw in enumerate(lines, 1):
        line = raw.strip()
        out.lines = line_no
        if not line:
            continue

        if any(marker in line for marker in BOOT_MARKERS):
            out.boot_markers.append({"line": line_no, "text": line[:220]})
        if any(marker in line for marker in FAULT_MARKERS):
            out.fault_markers.append({"line": line_no, "text": line[:220]})
        if line.startswith("$STS:"):
            out.statuses.append({"line": line_no, "text": line})
        if line.startswith("$RUN:"):
            out.run_lines += 1
        if line.startswith("$BAT:"):
            try:
                v = float(line[5:])
                if v > 0.1:  # 0.00 means battery disabled; skip
                    out.battery_values.append(v)
            except ValueError:
                pass

        if line and line[0].isdigit():
            try:
                sample = parse_csv(line_no, line)
            except ValueError:
                out.csv_malformed += 1
                continue
            if sample is None:
                continue

            out.csv_valid += 1
            if out.first_ms is None:
                out.first_ms = sample.ms
            out.last_ms = sample.ms
            out.max_speed = max(out.max_speed, sample.speed)
            out.max_target = max(out.max_target, sample.target)
            out.max_abs_steer = max(out.max_abs_steer, abs(sample.steer))
            for idx, value in enumerate(sample.sensors):
                out.sensor_min[idx] = value if out.sensor_min[idx] is None else min(out.sensor_min[idx], value)
                out.sensor_max[idx] = value if out.sensor_max[idx] is None else max(out.sensor_max[idx], value)

            if prev is not None:
                dt = sample.ms - prev.ms
                if dt < 0:
                    out.timestamp_backwards.append(
                        {"line": line_no, "prev_ms": prev.ms, "ms": sample.ms, "delta_ms": dt}
                    )
                elif dt > gap_ms:
                    out.timestamp_gaps.append(
                        {"line": line_no, "prev_ms": prev.ms, "ms": sample.ms, "delta_ms": dt}
                    )
            prev = sample

    return out


def send_line(ser: "serial.Serial", command: str, raw_log) -> None:
    line = command if command.endswith("\n") else command + "\n"
    raw_log.write(f"> {line}")
    raw_log.flush()
    ser.write(line.encode("ascii", errors="replace"))
    ser.flush()


def read_for(ser: "serial.Serial", duration_s: float, raw_log, mirror: bool) -> list[str]:
    end = time.monotonic() + duration_s
    lines: list[str] = []
    while time.monotonic() < end:
        raw = ser.readline()
        if not raw:
            continue
        line = decode(raw)
        lines.append(line)
        raw_log.write(line + "\n")
        if mirror:
            print(line)
    raw_log.flush()
    return lines


def wait_for_pattern(
    ser: "serial.Serial",
    pattern: str,
    timeout_s: float,
    raw_log,
    mirror: bool,
) -> bool:
    rx = re.compile(pattern)
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        raw = ser.readline()
        if not raw:
            continue
        line = decode(raw)
        raw_log.write(line + "\n")
        if mirror:
            print(line)
        if rx.search(line):
            raw_log.flush()
            return True
    raw_log.flush()
    return False


def _settle_drain(ser: "serial.Serial", timeout_s: float) -> None:
    """Read and discard serial output until $BOOT:READY or timeout.

    Called before opening the raw log so initial boot messages are not
    written to the analysis window and cannot trigger boot-marker failures.
    """
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        raw = ser.readline()
        if raw and b"$BOOT:READY" in raw:
            return


def safe_stop(ser: "serial.Serial", raw_log, mirror: bool) -> list[str]:
    for command in ("$STOP", "$DRVOFF", "$STATUS", "$BAT"):
        send_line(ser, command, raw_log)
        time.sleep(0.05)
    return read_for(ser, 2.0, raw_log, mirror)


def run_smoke(ser: "serial.Serial", raw_log, args) -> list[str]:
    lines: list[str] = []
    lines += safe_stop(ser, raw_log, args.mirror)
    for command in ("$PING", "$DIAG", "$TEST:lidar", "$TEST:taho", "$MONITOR"):
        send_line(ser, command, raw_log)
        if command == "$TEST:lidar":
            lines += read_for(ser, 6.0, raw_log, args.mirror)
        elif command == "$TEST:taho":
            lines += read_for(ser, 6.0, raw_log, args.mirror)
        elif command == "$MONITOR":
            lines += read_for(ser, args.monitor_s, raw_log, args.mirror)
            send_line(ser, "$STOP", raw_log)
            lines += read_for(ser, 1.5, raw_log, args.mirror)
        else:
            lines += read_for(ser, 1.0, raw_log, args.mirror)
    lines += safe_stop(ser, raw_log, args.mirror)
    return lines


def run_endurance(ser: "serial.Serial", raw_log, args) -> list[str]:
    lines: list[str] = []
    lines += safe_stop(ser, raw_log, args.mirror)
    if args.set_battery:
        send_line(ser, "$SET:BEN=1", raw_log)
        lines += read_for(ser, 0.5, raw_log, args.mirror)
    if args.tgf is not None:
        send_line(ser, f"$SET:TGF={args.tgf}", raw_log)
        lines += read_for(ser, 0.5, raw_log, args.mirror)
    send_line(ser, "$DIAG", raw_log)
    lines += read_for(ser, 1.0, raw_log, args.mirror)
    send_line(ser, "$START", raw_log)
    if not wait_for_pattern(ser, r"\$STS:RUN|^[0-9]+,", 5.0, raw_log, args.mirror):
        raise HilFailure("RUN did not produce telemetry within 5 seconds")
    lines += read_for(ser, args.duration, raw_log, args.mirror)
    lines += safe_stop(ser, raw_log, args.mirror)
    return lines


def run_motor(ser: "serial.Serial", raw_log, args) -> list[str]:
    if not args.allow_motor:
        raise HilFailure("motor profile requires --allow-motor")
    lines: list[str] = []
    lines += safe_stop(ser, raw_log, args.mirror)
    if args.tgf is not None:
        send_line(ser, f"$SET:TGF={args.tgf}", raw_log)
        lines += read_for(ser, 0.5, raw_log, args.mirror)
    for test_name in args.motor_test:
        send_line(ser, f"$TEST:{test_name}", raw_log)
        timeout = 14.0 if test_name == "speed" else 8.0
        lines += read_for(ser, timeout, raw_log, args.mirror)
        lines += safe_stop(ser, raw_log, args.mirror)
    return lines


def assert_analysis(profile: str, analysis: Analysis, args) -> list[str]:
    failures: list[str] = []
    if analysis.boot_markers:
        failures.append(f"boot markers seen: {len(analysis.boot_markers)}")
    if analysis.fault_markers:
        failures.append(f"fault markers seen: {len(analysis.fault_markers)}")
    if analysis.timestamp_backwards:
        failures.append(f"timestamp went backwards: {len(analysis.timestamp_backwards)}")
    if len(analysis.timestamp_gaps) > args.max_gaps:
        failures.append(f"too many timestamp gaps > {args.gap_ms} ms: {len(analysis.timestamp_gaps)}")
    if analysis.csv_valid < args.min_csv:
        failures.append(f"too few valid telemetry frames: {analysis.csv_valid} < {args.min_csv}")
    if analysis.max_speed > args.max_speed:
        failures.append(f"speed spike {analysis.max_speed:.2f} m/s > {args.max_speed:.2f}")
    if args.set_battery and analysis.battery_values and min(analysis.battery_values) < args.min_battery:
        failures.append(f"battery {min(analysis.battery_values):.2f} V < {args.min_battery:.2f} V")
    if profile == "endurance" and analysis.run_lines == 0:
        failures.append("no $RUN lines captured during endurance profile")
    return failures


def write_summary(path: Path, profile: str, analysis: Analysis, failures: list[str], raw_log: Path) -> None:
    payload = {
        "profile": profile,
        "raw_log": str(raw_log),
        "pass": not failures,
        "failures": failures,
        "analysis": {
            "lines": analysis.lines,
            "csv_valid": analysis.csv_valid,
            "csv_malformed": analysis.csv_malformed,
            "first_ms": analysis.first_ms,
            "last_ms": analysis.last_ms,
            "span_s": (
                (analysis.last_ms - analysis.first_ms) / 1000.0
                if analysis.first_ms is not None and analysis.last_ms is not None
                else 0.0
            ),
            "timestamp_backwards": analysis.timestamp_backwards[:20],
            "timestamp_gaps": analysis.timestamp_gaps[:20],
            "boot_markers": analysis.boot_markers[:20],
            "fault_markers": analysis.fault_markers[:20],
            "statuses": analysis.statuses[-20:],
            "run_lines": analysis.run_lines,
            "battery_min": min(analysis.battery_values) if analysis.battery_values else None,
            "battery_max": max(analysis.battery_values) if analysis.battery_values else None,
            "max_speed": analysis.max_speed,
            "max_target": analysis.max_target,
            "max_abs_steer": analysis.max_abs_steer,
            "sensor_min": analysis.sensor_min,
            "sensor_max": analysis.sensor_max,
        },
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("smoke", "endurance", "motor"), default="smoke")
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=300.0, help="endurance RUN duration in seconds")
    parser.add_argument("--monitor-s", type=float, default=10.0, help="smoke MONITOR duration in seconds")
    parser.add_argument("--raw-log", type=Path, default=Path("/tmp/umbreon_hil_raw.log"))
    parser.add_argument("--summary", type=Path, default=Path("/tmp/umbreon_hil_summary.json"))
    parser.add_argument("--mirror", action="store_true", help="print UART lines while running")
    parser.add_argument("--allow-motor", action="store_true", help="required for motor profile")
    parser.add_argument("--motor-test", action="append", choices=("esc", "speed"), default=["esc"])
    parser.add_argument("--set-battery", action="store_true", help="enable battery messages at runtime")
    parser.add_argument("--tgf", type=int, help="temporary tach glitch filter in microseconds")
    parser.add_argument("--gap-ms", type=int, default=250)
    parser.add_argument("--max-gaps", type=int, default=0)
    parser.add_argument("--min-csv", type=int, default=1)
    parser.add_argument("--max-speed", type=float, default=6.0)
    parser.add_argument("--min-battery", type=float, default=6.5)
    parser.add_argument("--settle-s", type=float, default=0.0,
                        help="drain initial boot output before logging (seconds); smoke profile only")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    if serial is None:
        print("pyserial is not installed. Run: make hil-deps", file=sys.stderr)
        return 2

    args = parse_args(argv)
    args.raw_log.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)

    all_lines: list[str] = []
    failures: list[str] = []

    try:
        with serial.Serial(args.serial_port, args.baud, timeout=0.2) as ser:
            if args.profile == "smoke" and args.settle_s > 0:
                _settle_drain(ser, args.settle_s)
            with args.raw_log.open("w", encoding="utf-8", errors="replace") as raw_log:
                if args.profile == "smoke":
                    all_lines = run_smoke(ser, raw_log, args)
                elif args.profile == "endurance":
                    all_lines = run_endurance(ser, raw_log, args)
                elif args.profile == "motor":
                    all_lines = run_motor(ser, raw_log, args)
    except HilFailure as exc:
        failures.append(str(exc))
    except KeyboardInterrupt:
        failures.append("interrupted")
    except Exception as exc:
        failures.append(f"runner error: {exc}")

    if args.raw_log.exists():
        all_lines = args.raw_log.read_text(errors="ignore").splitlines()
    analysis = analyze_lines(all_lines, args.gap_ms)
    failures.extend(assert_analysis(args.profile, analysis, args))
    write_summary(args.summary, args.profile, analysis, failures, args.raw_log)

    print(f"raw_log={args.raw_log}")
    print(f"summary={args.summary}")
    print(
        "csv_valid={csv} boot={boot} faults={faults} gaps={gaps} max_speed={speed:.2f} bat_min={bat}".format(
            csv=analysis.csv_valid,
            boot=len(analysis.boot_markers),
            faults=len(analysis.fault_markers),
            gaps=len(analysis.timestamp_gaps),
            speed=analysis.max_speed,
            bat=(f"{min(analysis.battery_values):.2f}" if analysis.battery_values else "n/a"),
        )
    )
    if failures:
        print("FAIL:")
        for failure in failures:
            print(f"- {failure}")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
