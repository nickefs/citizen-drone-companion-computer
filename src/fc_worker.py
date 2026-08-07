#!/usr/bin/env python3
import json
import os
import time
from fractions import Fraction

import piexif
import subprocess
from pathlib import Path
from picamera2 import Picamera2
from pymavlink import mavutil


# ============================================================
# Paths / config
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]
XMP_CONFIG = PROJECT_ROOT / "config" / "drone_xmp.config"


FC_DEVICE = "/dev/serial0"
FC_BAUD = 57600
ARDUCOPTER_MODE_AUTO = 3

COMMAND_FILE = "/home/pi/worker_command.json"
STATUS_FILE = "/home/pi/worker_status.json"
DONE_FLAG = "/home/pi/done.flag"

PHOTO_DIR = "/home/pi/photos"
os.makedirs(PHOTO_DIR, exist_ok=True)


# ============================================================
# EXIF helpers -XMP writing
# ============================================================

def _dms(deg: float):
    deg = abs(deg)
    d = int(deg)
    m = int((deg - d) * 60)
    s = (deg - d - m / 60) * 3600
    f = Fraction(s).limit_denominator(1000)
    return ((d, 1), (m, 1), (f.numerator, f.denominator))


def _rat(x: float, den: int = 1000):
    f = Fraction(x).limit_denominator(den)
    return (f.numerator, f.denominator)


def write_xmp_drone(
    jpeg_path: str,
    config_path: str,
    absolute_alt: float,
    relative_alt: float,
    flight_yaw: float,
    flight_pitch: float,
    flight_roll: float,
):
    cmd = [
        "exiftool",
        "-config", config_path,
        "-overwrite_original",
        f"-XMP-drone:AbsoluteAltitude={absolute_alt:.3f}",
        f"-XMP-drone:RelativeAltitude={relative_alt:.3f}",
        f"-XMP-drone:FlightYawDegree={flight_yaw:.3f}",
        f"-XMP-drone:FlightPitchDegree={flight_pitch:.3f}",
        f"-XMP-drone:FlightRollDegree={flight_roll:.3f}",
        f"-XMP-drone:GimbalYawDegree={0:.3f}",
        f"-XMP-drone:GimbalPitchDegree={-90.0:.3f}",
        f"-XMP-drone:GimbalRollDegree={0:.3f}",
        jpeg_path,
    ]
    subprocess.run(cmd, check=True)


def write_exif_and_xmp_from_feedback(jpeg_path: str, msg):
    sec = msg.time_usec // 1_000_000
    usec = msg.time_usec % 1_000_000
    tm = time.gmtime(sec)

    lat = msg.lat / 1e7
    lon = msg.lng / 1e7
    alt = float(msg.alt_msl)
    yaw = float(msg.yaw)
    pitch = float(msg.pitch)
    roll = float(msg.roll)

    relative_alt = msg.alt_rel
    focal_length_mm = 3.04

    ex = {"0th": {}, "Exif": {}, "GPS": {}, "1st": {}, "thumbnail": None}
    ex["0th"][piexif.ImageIFD.Software] = "Picamera2+MAVLink Worker"
    ex["0th"][piexif.ImageIFD.Make] = "Citizen Drone"
    ex["0th"][piexif.ImageIFD.Model] = "V1"
    ex["0th"][piexif.ImageIFD.Orientation] = 1
    ex["0th"][piexif.ImageIFD.XResolution] = (72, 1)
    ex["0th"][piexif.ImageIFD.YResolution] = (72, 1)
    ex["0th"][piexif.ImageIFD.ResolutionUnit] = 2

    date_str = f"{tm.tm_year:04d}:{tm.tm_mon:02d}:{tm.tm_mday:02d}"
    time_str = f"{tm.tm_hour:02d}:{tm.tm_min:02d}:{tm.tm_sec:02d}"

    ex["Exif"][piexif.ExifIFD.DateTimeDigitized] = f"{date_str} {time_str}"
    ex["Exif"][piexif.ExifIFD.SubSecTimeOriginal] = f"{usec:06d}"
    ex["Exif"][piexif.ExifIFD.FocalLength] = _rat(focal_length_mm)

    ex["GPS"][piexif.GPSIFD.GPSLatitudeRef] = b"N" if lat >= 0 else b"S"
    ex["GPS"][piexif.GPSIFD.GPSLatitude] = _dms(lat)
    ex["GPS"][piexif.GPSIFD.GPSLongitudeRef] = b"E" if lon >= 0 else b"W"
    ex["GPS"][piexif.GPSIFD.GPSLongitude] = _dms(lon)
    ex["GPS"][piexif.GPSIFD.GPSAltitudeRef] = 0 if alt >= 0 else 1
    ex["GPS"][piexif.GPSIFD.GPSAltitude] = _rat(abs(alt))
    ex["GPS"][piexif.GPSIFD.GPSImgDirectionRef] = b"T"
    ex["GPS"][piexif.GPSIFD.GPSImgDirection] = _rat(yaw % 360.0)
    ex["GPS"][piexif.GPSIFD.GPSDateStamp] = date_str

    piexif.insert(piexif.dump(ex), jpeg_path)

    write_xmp_drone(
        jpeg_path=jpeg_path,
        config_path=str(XMP_CONFIG),
        absolute_alt=alt,
        relative_alt=relative_alt,
        flight_yaw=yaw,
        flight_pitch=pitch,
        flight_roll=roll,
    )


# ============================================================
# Mission file parser
# ============================================================

def read_qgc_waypoints(filename: str):
    """
    Read QGC WPL 110 / Mission Planner .waypoints file
    and return a list of mission items.
    """
    mission = []

    with open(filename, "r", encoding="utf-8") as f:
        lines = f.readlines()

    if not lines or not lines[0].strip().startswith("QGC WPL"):
        raise ValueError("Το αρχείο δεν είναι QGC waypoint file")

    for line in lines[1:]:
        line = line.strip()
        if not line:
            continue

        parts = line.split()
        if len(parts) < 12:
            raise ValueError(f"Λάθος mission line: {line}")

        item = {
            "seq": int(parts[0]),
            "current": int(parts[1]),
            "frame": int(parts[2]),
            "command": int(parts[3]),
            "param1": float(parts[4]),
            "param2": float(parts[5]),
            "param3": float(parts[6]),
            "param4": float(parts[7]),
            "lat": float(parts[8]),
            "lon": float(parts[9]),
            "alt": float(parts[10]),
            "autocontinue": int(parts[11]),
        }
        mission.append(item)

    return mission


# ============================================================
# Worker
# ============================================================

class FCWorker:
    def __init__(self):
        # MAVLink connection
        self.master = None

        # Camera
        self.cam = None
        self.camera_settings = {
            "mode": "auto",
            "exposure_time": 1200,
            "analogue_gain": 1.0,
        }

        # State
        self.capture_enabled = False
        self.mission_uploaded = False
        self.running = True
        self.last_command_id = None

        self.state = "booting"
        self.message = "Worker starting"

    def write_status(self):
        """
        Write worker state so the portal can display it.
        """
        data = {
            "connected": self.master is not None,
            "mission_uploaded": self.mission_uploaded,
            "capture_enabled": self.capture_enabled,
            "state": self.state,
            "message": self.message,
            "last_command_id": self.last_command_id,
            "camera_settings": self.camera_settings,
            "timestamp": time.time(),
        }

        tmp = STATUS_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f)
        os.replace(tmp, STATUS_FILE)

    def set_status(self, state: str, message: str):
        """
        Helper to update state and write status immediately.
        """
        self.state = state
        self.message = message
        print(f"[STATUS] {state}: {message}")
        self.write_status()

    def connect_fc(self):
        """
        Open MAVLink connection once and keep it alive.
        """
        self.set_status("connecting_fc", f"Connecting to FC on {FC_DEVICE}...")
        self.master = mavutil.mavlink_connection(FC_DEVICE, baud=FC_BAUD)
        self.master.wait_heartbeat()
        self.master.mav.request_data_stream_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1
        )
        self.set_status("connected", "Connected to flight controller")
        time.sleep(10)

    def setup_camera(self):
        """
        Initialize camera once.
        """
        self.set_status("camera_setup", "Initializing camera...")
        self.cam = Picamera2()

        config = self.cam.create_still_configuration(main={"size": (3280, 2464)})
        self.cam.configure(config)
      
        self.cam.start()
        self.apply_camera_settings(
            mode=self.camera_settings["mode"],
            exposure_time=self.camera_settings["exposure_time"],
            analogue_gain=self.camera_settings["analogue_gain"],
        )
        time.sleep(1.0)
        self.set_status("idle", "Camera ready, waiting for commands")

    def apply_camera_settings(self, mode: str, exposure_time=None, analogue_gain=None):
        """
        Apply exposure controls to the already configured Picamera2 instance.
        """
        if self.cam is None:
            raise RuntimeError("Camera is not initialized yet")

        mode = (mode or "manual").strip().lower()
        if mode not in ("auto", "manual"):
            raise ValueError("Camera mode must be auto or manual")

        if mode == "auto":
            self.cam.set_controls({"AeEnable": True})
            self.camera_settings = {
                "mode": "auto",
                "exposure_time": self.camera_settings.get("exposure_time", 1200),
                "analogue_gain": self.camera_settings.get("analogue_gain", 1.0),
            }
            self.set_status("idle", "Camera exposure set to auto")
            return

        exposure_time = int(exposure_time)
        analogue_gain = float(analogue_gain)
        if exposure_time <= 0:
            raise ValueError("Exposure time must be > 0")
        if analogue_gain <= 0:
            raise ValueError("Analogue gain must be > 0")

        self.cam.set_controls({
            "AeEnable": False,
            "ExposureTime": exposure_time,
            "AnalogueGain": analogue_gain,
        })
        self.camera_settings = {
            "mode": "manual",
            "exposure_time": exposure_time,
            "analogue_gain": analogue_gain,
        }
        self.set_status(
            "idle",
            f"Camera exposure set to manual: {exposure_time} us, gain {analogue_gain}",
        )

    def read_command(self):
        """
        Read the command file from the portal.
        Return None if there is no new command.
        """
        if not os.path.exists(COMMAND_FILE):
            return None

        try:
            with open(COMMAND_FILE, "r", encoding="utf-8") as f:
                cmd = json.load(f)
        except Exception:
            return None

        cmd_id = cmd.get("id")
        if cmd_id is None:
            return None

        if cmd_id == self.last_command_id:
            return None

        return cmd

    def is_armed_from_heartbeat(self, hb_msg):
        """
        Return True if vehicle is armed based on HEARTBEAT base_mode.
        """
        return bool(hb_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def wait_for_heartbeat_condition(self, condition, timeout_s: float):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg is not None and condition(msg):
                return msg
        return None

    def wait_until_armed(self, timeout_s: float = 10.0):
        msg = self.wait_for_heartbeat_condition(self.is_armed_from_heartbeat, timeout_s)
        if msg is None:
            raise TimeoutError("Vehicle did not report armed state")

    def set_auto_mode_confirmed(self, retries: int = 5, timeout_s: float = 2.0):
        """
        Set ArduCopter AUTO mode and confirm it from HEARTBEAT.custom_mode.
        If the RC flight-mode switch forces another mode, confirmation will fail.
        """
        for attempt in range(1, retries + 1):
            self.set_status("starting", f"Setting AUTO mode ({attempt}/{retries})...")

            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                ARDUCOPTER_MODE_AUTO
            )

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                ARDUCOPTER_MODE_AUTO,
                0, 0, 0, 0, 0
            )

            msg = self.wait_for_heartbeat_condition(
                lambda hb: getattr(hb, "custom_mode", None) == ARDUCOPTER_MODE_AUTO,
                timeout_s
            )
            if msg is not None:
                return

        raise RuntimeError(
            "AUTO mode was not confirmed. Put the RC flight-mode switch in AUTO "
            "or a neutral/GCS-controlled mode, then try Start Flight again."
        )

    def handle_command(self, cmd: dict):
        """
        Execute portal command.
        Supported:
        - upload_mission
        - set_camera
        - start_flight
        - stop_capture
        - finish
        """
        self.last_command_id = cmd["id"]
        name = cmd.get("cmd")

        try:
            if name == "upload_mission":
                path = cmd.get("path")
                if not path:
                    raise ValueError("Missing 'path' in upload_mission command")
                self.upload_mission(path)

            elif name == "start_flight":
                self.start_flight()

            elif name == "set_camera":
                self.apply_camera_settings(
                    mode=cmd.get("mode", "manual"),
                    exposure_time=cmd.get("exposure_time", self.camera_settings["exposure_time"]),
                    analogue_gain=cmd.get("analogue_gain", self.camera_settings["analogue_gain"]),
                )

            elif name == "idle":
                time.sleep(1.0)  

            elif name == "stop_capture":
                self.capture_enabled = False
                self.set_status("idle", "Capture stopped")

            elif name == "finish":
                self.capture_enabled = False
                self.running = False
                self.set_status("finished", "Finish command received")

            else:
                self.set_status("error", f"Unknown command: {name}")

        except Exception as e:
            self.set_status("error", f"{name} failed: {e}")

        self.write_status()

    def upload_mission(self, mission_path: str):
     """
     Upload mission to FC using MAVLink mission protocol.
     """
     self.set_status("uploading", f"Uploading mission from {mission_path} ...")

     mission = read_qgc_waypoints(mission_path)
     if mission is None:
        raise ValueError("Failed to read mission file")

     total = len(mission)

     # Clear mission first to avoid stale upload state on the FC
     self.master.mav.mission_clear_all_send(
        self.master.target_system,
        self.master.target_component,
     )
     clear_ack = self.master.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
     if clear_ack is None:
        raise TimeoutError("Timeout waiting for mission clear ACK")
     if clear_ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"Mission clear failed, ACK type={clear_ack.type}")

     # Για απλό flight plan mission upload
     self.master.mav.mission_count_send(
        self.master.target_system,
        self.master.target_component,
        total
        # αν η pymavlink/version σου το υποστηρίζει:
        # , mavutil.mavlink.MAV_MISSION_TYPE_MISSION
     )

     last_seq_sent = -1

     while True:
        msg = self.master.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
            blocking=True,
            timeout=10
        )

        if msg is None:
            raise TimeoutError("Timeout waiting for mission protocol message")

        mtype = msg.get_type()

        if mtype == "MISSION_ACK":
            # Ελέγχεις αποτέλεσμα
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"Mission upload failed, ACK type={msg.type}")
            break

        seq = msg.seq
        if seq < 0 or seq >= total:
            raise IndexError(f"FC requested invalid mission seq {seq} (mission has {total} items)")

        item = mission[seq]

        if mtype == "MISSION_REQUEST_INT":
            self.master.mav.mission_item_int_send(
                self.master.target_system,
                self.master.target_component,
                item["seq"],
                item["frame"],
                item["command"],
                item["current"],
                item["autocontinue"],
                item["param1"],
                item["param2"],
                item["param3"],
                item["param4"],
                int(item["lat"] * 1e7),
                int(item["lon"] * 1e7),
                item["alt"],
                # αν η version σου το υποστηρίζει:
                # mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                0
            )
        else:
            self.master.mav.mission_item_send(
                self.master.target_system,
                self.master.target_component,
                item["seq"],
                item["frame"],
                item["command"],
                item["current"],
                item["autocontinue"],
                item["param1"],
                item["param2"],
                item["param3"],
                item["param4"],
                float(item["lat"]),
                float(item["lon"]),
                float(item["alt"]),
                # αν η version σου το υποστηρίζει:
                # mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                0
            )

        last_seq_sent = seq

     self.mission_uploaded = True
     self.set_status("ready_to_start", "Mission uploaded successfully")

    def start_flight(self):
        """
        Start the mission, then enter a dedicated flight loop.
        During flight loop there is no portal communication.
        """
        if not self.mission_uploaded:
            raise RuntimeError("Mission has not been uploaded yet")

        self.set_status("starting", "Arming and starting mission...")

        # enable capture before starting mission
        self.capture_enabled = True

        # ARM
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        self.wait_until_armed(timeout_s=10.0)

        self.set_auto_mode_confirmed()

        # MISSION_START
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

        self.set_status("running", "Mission started, entering flight loop")
        self.mission_execution_loop()

    def handle_camera_feedback(self, msg):
        """
        When CAMERA_FEEDBACK arrives and capture is enabled:
        capture a photo and write EXIF.
        """
        sec = msg.time_usec // 1_000_000
        usec = msg.time_usec % 1_000_000
        ts = time.strftime("%Y%m%d_%H%M%S", time.gmtime(sec)) + f"_{usec:06d}"
        path = os.path.join(PHOTO_DIR, f"photo_{ts}.jpg")

        self.cam.capture_file(path)
        write_exif_and_xmp_from_feedback(path, msg)

        print(f"[TRIGGER] Saved photo: {path}")
        self.message = f"Last photo saved: {os.path.basename(path)}"
        self.write_status()

    def mission_execution_loop(self):
        """
        Flight loop:
        - no portal commands
        - only MAVLink messages
        - save photos on CAMERA_FEEDBACK
        - exit when mission has started and vehicle becomes disarmed
        """
        mission_started = False
        self.set_status("flight_mode", "Flight loop active")

        while True:
            msg = self.master.recv_match(blocking=True)

            if msg is None:
                continue

            mtype = msg.get_type()

            if mtype == "CAMERA_FEEDBACK" and self.capture_enabled:
                self.handle_camera_feedback(msg)

            elif mtype == "HEARTBEAT":
                armed = self.is_armed_from_heartbeat(msg)

                if armed:
                    mission_started = True

                if mission_started and not armed:
                    try:
                        self.master.mav.set_mode_send(
                            self.master.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            0
                        )
                    except Exception:
                        pass

                    self.capture_enabled = False
                    self.set_status("mission_complete", "Mission finished, returning to portal mode")
                    break

    def loop(self):
        """
        Main worker loop.

        In idle mode:
        - listens to portal commands

        When start_flight is received:
        - enters mission_execution_loop()
        - returns here only after mission is complete
        """
        self.set_status("idle", "Worker ready")

        while self.running:
            if os.path.exists(DONE_FLAG):
                self.capture_enabled = False
                self.set_status("finished", "Done flag detected, stopping worker")
                break

            cmd = self.read_command()
            if cmd is not None:
                self.handle_command(cmd)

            time.sleep(0.05)

        self.cleanup()

    def cleanup(self):
        """
        Clean shutdown.
        """
        try:
            if self.cam is not None:
                self.cam.stop()
        except Exception:
            pass

        self.set_status("stopped", "Worker stopped")


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":
    worker = FCWorker()
    worker.connect_fc()
   # worker.upload_mission("/home/pi/output/mission.waypoints")
    worker.setup_camera()
    worker.loop()
