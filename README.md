# UAV Companion Computer

Raspberry Pi companion-computer software for autonomous UAV survey missions.

The system provides a local web portal for uploading survey waypoints, converts them into an ArduPilot/Mission Planner mission, uploads the mission to the flight controller over MAVLink, controls the Raspberry Pi camera, and saves geotagged survey images using flight-controller feedback.

## Main Features

- Local Flask web portal for mission upload and control
- Conversion of survey CSV waypoints to QGC WPL 110 / Mission Planner missions
- MAVLink mission upload to an ArduPilot-compatible flight controller
- Vehicle arming, AUTO-mode selection and mission start
- Distance-based or waypoint-based camera triggering
- Raspberry Pi camera control with auto or manual exposure
- Photo capture on MAVLink `CAMERA_FEEDBACK`
- EXIF GPS metadata and XMP drone attitude/altitude metadata
- Local Raspberry Pi workflow suitable for offline field operation

## Architecture

```text
Survey waypoint CSV
        |
        v
+----------------------+
|    Mission Portal    |
|       Flask UI       |
+----------+-----------+
           |
           | convert
           v
+----------------------+
| Mission Converter    |
| CSV -> QGC WPL 110   |
+----------+-----------+
           |
           | upload command
           v
+----------------------+
|      FC Worker       |
| MAVLink + Picamera2  |
+-----+-----------+----+
      |           |
      | MAVLink   | Camera
      v           v
Flight Controller   Survey Photos
      |
      | CAMERA_FEEDBACK
      v
EXIF GPS + XMP attitude metadata
```

## Project Structure

```text
uav-companion-computer/
├── src/
│   ├── converter_to_mission.py
│   ├── fc_worker.py
│   └── mission_portal.py
├── config/
│   └── drone_xmp.config
├── scripts/
│   ├── run_local.sh
│   └── setup_rpi.sh
├── docs/
│   └── screenshots/
├── requirements.txt
├── .gitignore
└── README.md
```

## Runtime Components

### `mission_portal.py`

Runs a local Flask interface on port `8000`. It allows the operator to:

- upload a waypoint CSV
- select camera-trigger mode
- configure camera exposure
- generate/download the mission file
- upload the mission to the FC worker
- start the autonomous mission
- finish the local session

### `converter_to_mission.py`

Converts CSV survey waypoints into a Mission Planner-compatible `QGC WPL 110` mission.

It supports:

- TAKEOFF
- survey navigation waypoints
- flight-speed command
- distance-based camera triggering
- waypoint-based camera triggering
- RTL by default or LAND at the final waypoint
- automatic trigger-distance calculation from camera geometry and overlap

### `fc_worker.py`

Maintains the connection to the flight controller and camera.

It is responsible for:

- MAVLink connection over `/dev/serial0`
- mission upload
- arming
- switching to AUTO mode
- starting the mission
- listening for `CAMERA_FEEDBACK`
- capturing Raspberry Pi camera images
- writing GPS EXIF metadata
- writing drone XMP altitude and attitude metadata

## Raspberry Pi Requirements

The project is intended for Raspberry Pi OS with:

- Raspberry Pi camera and Picamera2
- ArduPilot-compatible flight controller
- UART connection to the flight controller
- Python 3
- ExifTool

Python dependencies are listed in `requirements.txt`.

Picamera2 is normally installed through Raspberry Pi OS packages rather than PyPI.

## Installation

Create a virtual environment with access to system packages:

```bash
python3 -m venv --system-site-packages /home/pi/venvs/drone
source /home/pi/venvs/drone/bin/activate
pip install -r requirements.txt
```

Install Raspberry Pi camera packages and ExifTool through the OS package manager as required.

An optional provisioning helper is included at:

```text
scripts/setup_rpi.sh
```

It is not part of the normal runtime.

## Run

From the repository root:

```bash
chmod +x scripts/run_local.sh
./scripts/run_local.sh
```

The web portal is then available on port `8000`.

## Data Flow

1. The operator uploads a waypoint CSV through the portal.
2. The converter generates a `.waypoints` mission file.
3. The portal sends an upload command to the FC worker.
4. The worker uploads the mission to the flight controller.
5. The operator starts the flight from the portal.
6. The worker arms the vehicle, selects AUTO mode and starts the mission.
7. During flight, `CAMERA_FEEDBACK` messages trigger image capture.
8. GPS position, altitude and aircraft attitude are written into each image.
9. When the mission finishes and the vehicle disarms, capture stops and control returns to portal mode.

## Notes

This repository contains the Raspberry Pi / companion-computer side of the UAV system. Survey-area geometry and waypoint generation can be handled by a separate survey-planner component.

Hardware configuration, ArduPilot parameters and serial-port mapping depend on the specific UAV setup.
