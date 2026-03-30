#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

LISTEN_HOST="${LISTEN_HOST:-127.0.0.1}"
LISTEN_PORT="${LISTEN_PORT:-8023}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
SERIAL_BAUD="${SERIAL_BAUD:-115200}"

exec "$PYTHON_BIN" "$ROOT_DIR/tools/hil_bridge.py" \
  --mode real \
  --listen-host "$LISTEN_HOST" \
  --listen-port "$LISTEN_PORT" \
  --serial-port "$SERIAL_PORT" \
  --serial-baud "$SERIAL_BAUD"
