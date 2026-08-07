#!/bin/bash
set -euo pipefail

# Optional Raspberry Pi provisioning script.
# This is NOT required for normal runtime once the Pi is configured.

HOTSPOT_SSID="${HOTSPOT_SSID:-Pi-Upload}"
: "${HOTSPOT_PASSWORD:?Set HOTSPOT_PASSWORD before running this script (minimum 8 characters).}"

sudo apt update
sudo apt full-upgrade -y

sudo apt install -y \
  python3 python3-venv python3-pip \
  network-manager \
  python3-libcamera python3-kms++ python3-picamera2 \
  libimage-exiftool-perl

python3 -m venv --system-site-packages /home/pi/venvs/drone
source /home/pi/venvs/drone/bin/activate

python -m pip install --upgrade pip setuptools wheel
pip install -r "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/requirements.txt"

sudo nmcli connection delete Hotspot 2>/dev/null || true
sudo nmcli connection add type wifi ifname wlan0 con-name Hotspot ssid "$HOTSPOT_SSID"
sudo nmcli connection modify Hotspot \
  802-11-wireless.mode ap \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "$HOTSPOT_PASSWORD" \
  ipv4.addresses 192.168.4.1/24 \
  ipv4.method shared

echo "Done."
echo "Activate venv with: source /home/pi/venvs/drone/bin/activate"
