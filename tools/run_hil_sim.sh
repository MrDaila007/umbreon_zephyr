#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
SIM_PY="${SIM_PY:-/home/danila/Documents/roborace/simulation/sim.py}"

LISTEN_HOST="${LISTEN_HOST:-127.0.0.1}"
LISTEN_PORT="${LISTEN_PORT:-8123}"
SIM_HOST="${SIM_HOST:-127.0.0.1}"
SIM_PORT="${SIM_PORT:-8023}"
SIM_SENSORS="${SIM_SENSORS:-6}"

cleanup() {
  if [[ -n "${SIM_PID:-}" ]]; then
    kill "$SIM_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

"$PYTHON_BIN" "$SIM_PY" --bridge --headless --sensors "$SIM_SENSORS" --ticks 999999 >/tmp/hil_sim.log 2>&1 &
SIM_PID=$!
sleep 1

exec "$PYTHON_BIN" "$ROOT_DIR/tools/hil_bridge.py" \
  --mode sim \
  --listen-host "$LISTEN_HOST" \
  --listen-port "$LISTEN_PORT" \
  --sim-host "$SIM_HOST" \
  --sim-port "$SIM_PORT"
