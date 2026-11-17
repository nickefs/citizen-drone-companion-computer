#!/bin/bash
# Fixed copy - see docs/CHANGES.md.
set -euo pipefail

HOTSPOT_CONN="Hotspot"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

VENV_PY="${VENV_PY:-/home/pi/venvs/drone/bin/python}"
PORTAL="$PROJECT_ROOT/src/mission_portal.py"
WORKER="$PROJECT_ROOT/src/fc_worker.py"
DONE_FLAG="/home/pi/done.flag"
WORKER_COMMAND_DIR="/home/pi/worker_commands"
WORKER_STATUS_FILE="/home/pi/worker_status.json"

# Prints the worker state from worker_status.json ("unknown" if unreadable)
worker_state() {
  "$VENV_PY" -c 'import json,sys
try: print(json.load(open(sys.argv[1])).get("state","unknown"))
except Exception: print("unknown")' "$WORKER_STATUS_FILE"
}

is_flight_state() {
  case "$1" in
    starting|running|flight_mode|link_lost) return 0 ;;
    *) return 1 ;;
  esac
}

echo "[1/4] Starting hotspot..."
sudo nmcli connection up "$HOTSPOT_CONN"

echo "[2/4] Starting FC worker..."
rm -f "$DONE_FLAG"
mkdir -p "$WORKER_COMMAND_DIR"
rm -f "$WORKER_COMMAND_DIR"/*.json "$WORKER_COMMAND_DIR"/*.tmp   # no stale commands from a previous run
"$VENV_PY" "$WORKER" &
WORKER_PID=$!

echo "[3/4] Starting portal at http://192.168.4.1:8000 ..."
"$VENV_PY" "$PORTAL" &
PORTAL_PID=$!

echo "Waiting for Finish..."
while [[ ! -f "$DONE_FLAG" ]]; do
  sleep 1
done

# Never stop the worker while the drone may be in the air (photos would stop).
while is_flight_state "$(worker_state)"; do
  echo "Worker reports a flight in progress - waiting before shutting down..."
  sleep 5
done

echo "Finish requested. Stopping portal..."
kill "$PORTAL_PID" 2>/dev/null || true
wait "$PORTAL_PID" 2>/dev/null || true

echo "Stopping worker..."
# The worker also sees done.flag and exits by itself after flushing photo metadata.
for _ in $(seq 1 30); do
  kill -0 "$WORKER_PID" 2>/dev/null || break
  sleep 1
done
kill "$WORKER_PID" 2>/dev/null || true
wait "$WORKER_PID" 2>/dev/null || true

echo "[4/4] Stopping hotspot..."
sudo nmcli connection down "$HOTSPOT_CONN" || true

echo "Done."
