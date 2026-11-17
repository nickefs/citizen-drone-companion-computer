#!/bin/bash
# Fixed copy - see docs/CHANGES.md. Same as mission_script2.sh but without the hotspot.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

VENV_PY="${VENV_PY:-/home/pi/venvs/drone/bin/python}"
PORTAL="$PROJECT_ROOT/src/mission_portal.py"
WORKER="$PROJECT_ROOT/src/fc_worker.py"
DONE_FLAG="/home/pi/done.flag"
WORKER_COMMAND_DIR="/home/pi/worker_commands"
WORKER_STATUS_FILE="/home/pi/worker_status.json"

echo "[0/4] Stopping old portal/worker processes..."
# Only our two scripts - not every Python process on the Pi.
pkill -f "$PORTAL" 2>/dev/null || true
pkill -f "$WORKER" 2>/dev/null || true
sleep 1

echo "[1/4] Initializing defaults..."
rm -f "$DONE_FLAG"
mkdir -p "$WORKER_COMMAND_DIR"
rm -f "$WORKER_COMMAND_DIR"/*.json "$WORKER_COMMAND_DIR"/*.tmp
rm -f "$WORKER_STATUS_FILE"

echo "[2/4] Starting portal locally..."
"$VENV_PY" "$PORTAL" &
PORTAL_PID=$!

sleep 2

echo "[3/4] Starting FC worker..."
"$VENV_PY" "$WORKER" &
WORKER_PID=$!

echo "Waiting for Finish..."
while [[ ! -f "$DONE_FLAG" ]]; do
  sleep 1
done

echo "Finish requested. Stopping portal..."
kill "$PORTAL_PID" 2>/dev/null || true
wait "$PORTAL_PID" 2>/dev/null || true

echo "Stopping worker..."
for _ in $(seq 1 30); do
  kill -0 "$WORKER_PID" 2>/dev/null || break
  sleep 1
done
kill "$WORKER_PID" 2>/dev/null || true
wait "$WORKER_PID" 2>/dev/null || true

echo "[4/4] Done."
