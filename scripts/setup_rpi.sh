#!/bin/bash
# Raspberry Pi provisioning. Run as the pi user (not with sudo):
#   HOTSPOT_PASSWORD='your-password' ./scripts/setup_rpi.sh
set -euo pipefail

HOTSPOT_SSID="${HOTSPOT_SSID:-Pi-Upload}"
: "${HOTSPOT_PASSWORD:?Set HOTSPOT_PASSWORD before running this script (minimum 8 characters).}"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MISSION_SCRIPT="$PROJECT_ROOT/scripts/mission_script2.sh"

sudo apt update

# --no-install-recommends: skip optional extras (mesa/llvm graphics drivers) the drone doesn't use
sudo apt install -y --no-install-recommends \
  python3 python3-venv python3-pip \
  network-manager \
  python3-libcamera python3-kms++ python3-picamera2 \
  libimage-exiftool-perl python3-pil

python3 -m venv --system-site-packages /home/pi/venvs/drone
source /home/pi/venvs/drone/bin/activate

python -m pip install --upgrade pip setuptools wheel
pip install -r "$PROJECT_ROOT/requirements.txt"

sudo nmcli connection delete Hotspot 2>/dev/null || true
sudo nmcli connection add type wifi ifname wlan0 con-name Hotspot ssid "$HOTSPOT_SSID"
sudo nmcli connection modify Hotspot \
  802-11-wireless.mode ap \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "$HOTSPOT_PASSWORD" \
  ipv4.addresses 192.168.4.1/24 \
  ipv4.method shared

# Start mission_script2.sh (hotspot + worker + portal) on every boot.
# Enabled only, not started now: starting the hotspot here would drop this SSH session.
sudo tee /etc/systemd/system/drone-mission.service > /dev/null <<EOF
[Unit]
Description=Drone hotspot + FC worker + mission portal
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi
ExecStart=$MISSION_SCRIPT
Restart=on-failure
RestartSec=5
KillMode=mixed
TimeoutStopSec=40

[Install]
WantedBy=multi-user.target
EOF
chmod +x "$MISSION_SCRIPT" "$PROJECT_ROOT/scripts/run_local.sh"
sudo systemctl daemon-reload
sudo systemctl enable drone-mission.service

echo "Done."
echo "Activate venv with: source /home/pi/venvs/drone/bin/activate"
echo "Mission starts on next boot (sudo reboot) from $MISSION_SCRIPT."
