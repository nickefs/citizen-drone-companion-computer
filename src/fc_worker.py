#!/usr/bin/env python3
"""
FC worker (fixed copy). See CHANGES.md for what changed compared to Upload/fc_worker.py.

Talks to the flight controller over MAVLink, uploads missions, arms/starts the
mission, and takes a photo for every CAMERA_FEEDBACK message during the flight.
Commands come from mission_portal.py through a queue directory (one JSON file
per command), so no command is lost when two arrive close together.
"""
import json
import os
import queue
import threading
import time
from fractions import Fraction

import piexif
import subprocess
from picamera2 import Picamera2
from pymavlink import mavutil


# ============================================================
# Paths / config
# ============================================================

FC_DEVICE = "/dev/serial0"
FC_BAUD = 57600          # must match SERIALx_BAUD of the FC port the Pi is on (higher is better, e.g. 921600)
ARDUCOPTER_MODE_AUTO = 3

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
XMP_CONFIG_PATH = os.path.join(os.path.dirname(BASE_DIR), "config", "drone_xmp.config")   # absolute, so cwd does not matter

COMMAND_DIR = "/home/pi/worker_commands"     # queue: one <id>.json per command
STATUS_FILE = "/home/pi/worker_status.json"
DONE_FLAG = "/home/pi/done.flag"

PHOTO_DIR = "/home/pi/photos"
os.makedirs(PHOTO_DIR, exist_ok=True)

# Test photos from the portal go to their own folder, never mixed with survey photos
TEST_PHOTO_DIR = "/home/pi/test_photos"
TEST_PHOTO_FULL = os.path.join(TEST_PHOTO_DIR, "test_full.jpg")
TEST_PHOTO_PREVIEW = os.path.join(TEST_PHOTO_DIR, "test_preview.jpg")
TEST_PREVIEW_MAX_PX = 1280
os.makedirs(COMMAND_DIR, exist_ok=True)

# Same as before: ask the FC for all data streams at 4 Hz on this port.
# (Setting this to 0 would lighten the 57600 link, but it is left as before
# because the system is tuned and flying this way.)
FC_STREAM_RATE_HZ = 4

FC_SETTLE_S = 10.0              # same 10 s wait after the first heartbeat as the original
FC_REOPEN_S = 5.0               # retry opening the serial port this often if it is missing
HEARTBEAT_LOST_S = 5.0         # no vehicle heartbeat for this long -> report link lost
STATUS_PERIOD_S = 1.0           # how often status is refreshed (portal checks staleness)
VALID_UTC_MIN_S = 1_577_836_800  # 2020-01-01; below this CAMERA_FEEDBACK time is not real UTC

# Camera (IMX219 / Pi Camera v2)
FOCAL_LENGTH_MM = 3.04
FOCAL_LENGTH_35MM = 29          # 3.04 mm * crop factor ~9.4

# States in which the vehicle may be flying; portal must not finish/kill the worker.
FLIGHT_STATES = ("starting", "running", "flight_mode", "link_lost")


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
    # Unchanged from the original.
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


def write_exif_and_xmp_from_feedback(jpeg_path: str, fb: dict):
    """
    fb is a plain dict copied from CAMERA_FEEDBACK (see feedback_to_dict).
    EXIF is merged into what Picamera2 already wrote (exposure, gain, ...),
    instead of replacing it.
    """
    tm = time.gmtime(fb["utc_sec"])
    usec = fb["utc_usec"]

    lat = fb["lat"]
    lon = fb["lon"]
    alt = fb["alt_msl"]
    yaw = fb["yaw"]

    try:
        ex = piexif.load(jpeg_path)
    except Exception:
        ex = {"0th": {}, "Exif": {}, "GPS": {}, "1st": {}, "thumbnail": None}
    for ifd in ("0th", "Exif", "GPS", "1st"):
        ex.setdefault(ifd, {})
    ex["thumbnail"] = None
    ex["1st"] = {}

    ex["0th"][piexif.ImageIFD.Software] = "Picamera2+MAVLink Worker"
    ex["0th"][piexif.ImageIFD.Make] = "Citizen Drone"
    ex["0th"][piexif.ImageIFD.Model] = "V1"
    ex["0th"][piexif.ImageIFD.Orientation] = 1
    ex["0th"][piexif.ImageIFD.XResolution] = (72, 1)
    ex["0th"][piexif.ImageIFD.YResolution] = (72, 1)
    ex["0th"][piexif.ImageIFD.ResolutionUnit] = 2

    date_str = f"{tm.tm_year:04d}:{tm.tm_mon:02d}:{tm.tm_mday:02d}"
    time_str = f"{tm.tm_hour:02d}:{tm.tm_min:02d}:{tm.tm_sec:02d}"

    ex["0th"][piexif.ImageIFD.DateTime] = f"{date_str} {time_str}"
    ex["Exif"][piexif.ExifIFD.DateTimeOriginal] = f"{date_str} {time_str}"
    ex["Exif"][piexif.ExifIFD.DateTimeDigitized] = f"{date_str} {time_str}"
    ex["Exif"][piexif.ExifIFD.SubSecTimeOriginal] = f"{usec:06d}"
    ex["Exif"][piexif.ExifIFD.FocalLength] = _rat(FOCAL_LENGTH_MM)
    ex["Exif"][piexif.ExifIFD.FocalLengthIn35mmFilm] = FOCAL_LENGTH_35MM

    ex["GPS"][piexif.GPSIFD.GPSVersionID] = (2, 3, 0, 0)
    ex["GPS"][piexif.GPSIFD.GPSMapDatum] = "WGS-84"
    ex["GPS"][piexif.GPSIFD.GPSLatitudeRef] = b"N" if lat >= 0 else b"S"
    ex["GPS"][piexif.GPSIFD.GPSLatitude] = _dms(lat)
    ex["GPS"][piexif.GPSIFD.GPSLongitudeRef] = b"E" if lon >= 0 else b"W"
    ex["GPS"][piexif.GPSIFD.GPSLongitude] = _dms(lon)
    ex["GPS"][piexif.GPSIFD.GPSAltitudeRef] = 0 if alt >= 0 else 1
    ex["GPS"][piexif.GPSIFD.GPSAltitude] = _rat(abs(alt))
    ex["GPS"][piexif.GPSIFD.GPSImgDirectionRef] = b"T"
    ex["GPS"][piexif.GPSIFD.GPSImgDirection] = _rat(yaw % 360.0)
    ex["GPS"][piexif.GPSIFD.GPSTimeStamp] = (
        (tm.tm_hour, 1), (tm.tm_min, 1), (tm.tm_sec * 1000 + usec // 1000, 1000)
    )
    ex["GPS"][piexif.GPSIFD.GPSDateStamp] = date_str

    try:
        exif_bytes = piexif.dump(ex)
    except Exception:
        # Some camera-written tag piexif cannot re-encode: drop the camera EXIF IFD extras.
        ex["Exif"] = {k: v for k, v in ex["Exif"].items() if k in (
            piexif.ExifIFD.DateTimeOriginal, piexif.ExifIFD.DateTimeDigitized,
            piexif.ExifIFD.SubSecTimeOriginal, piexif.ExifIFD.FocalLength,
            piexif.ExifIFD.FocalLengthIn35mmFilm, piexif.ExifIFD.ExposureTime,
            piexif.ExifIFD.ISOSpeedRatings,
        )}
        exif_bytes = piexif.dump(ex)
    piexif.insert(exif_bytes, jpeg_path)

    write_xmp_drone(
        jpeg_path=jpeg_path,
        config_path=XMP_CONFIG_PATH,
        absolute_alt=alt,
        relative_alt=fb["alt_rel"],
        flight_yaw=yaw,
        flight_pitch=fb["pitch"],
        flight_roll=fb["roll"],
    )


def feedback_to_dict(msg):
    """
    Copy CAMERA_FEEDBACK into a dict (safe to hand to another thread).
    Falls back to the Pi clock when the FC has no GPS UTC time (time_usec ~ 0).
    """
    t = int(msg.time_usec)
    if t // 1_000_000 < VALID_UTC_MIN_S:
        t = int(time.time() * 1_000_000)
        time_source = "pi_clock"
    else:
        time_source = "fc_utc"
    return {
        "utc_sec": t // 1_000_000,
        "utc_usec": t % 1_000_000,
        "time_source": time_source,
        "lat": msg.lat / 1e7,
        "lon": msg.lng / 1e7,
        "alt_msl": float(msg.alt_msl),
        "alt_rel": float(msg.alt_rel),
        "yaw": float(msg.yaw),
        "pitch": float(msg.pitch),
        "roll": float(msg.roll),
        "img_idx": int(getattr(msg, "img_idx", 0)),
    }


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

    if not mission:
        raise ValueError("Mission file has no items")
    for i, item in enumerate(mission):
        if item["seq"] != i:
            raise ValueError(f"Mission item {i} has seq {item['seq']} (must be sequential from 0)")

    return mission


# ============================================================
# Photo metadata writer (background thread)
# ============================================================

class MetadataWriter:
    """
    Writes EXIF/XMP after the photo is saved, in a background thread, so the
    flight loop goes straight back to reading MAVLink (no missed / late triggers).
    Errors are logged and counted; they never stop photo capture.
    """

    def __init__(self, on_error):
        self.q = queue.Queue()
        self.on_error = on_error
        self.errors = 0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, path: str, fb: dict):
        self.q.put((path, fb))

    def pending(self):
        return self.q.qsize()

    def _run(self):
        while True:
            path, fb = self.q.get()
            try:
                write_exif_and_xmp_from_feedback(path, fb)
            except Exception as e:
                self.errors += 1
                # Only when writing metadata failed: keep the position in a sidecar file
                try:
                    with open(path + ".json", "w", encoding="utf-8") as f:
                        json.dump(fb, f)
                except Exception:
                    pass
                self.on_error(f"Metadata failed for {os.path.basename(path)}: {e}")
            finally:
                self.q.task_done()

    def flush(self, timeout_s: float = 60.0):
        deadline = time.time() + timeout_s
        while self.q.unfinished_tasks and time.time() < deadline:
            time.sleep(0.1)


# ============================================================
# Worker
# ============================================================

class FCWorker:
    def __init__(self):
        # MAVLink connection (opened in the main loop, see poll_fc_connection)
        self.master = None
        self.fc_connected_at = None    # time the autopilot heartbeat was first seen
        self.fc_ready = False          # True after the settle time: FC commands allowed
        self._fc_last_open_try = 0.0
        self._fc_note = ""

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
        self.mission_count = 0
        self.running = True
        self.last_command_id = None

        self.state = "booting"
        self.message = "Worker starting"
        self.last_error = ""
        self._last_status_write = 0.0

        # Photos
        self.photos_this_flight = 0
        self.capture_errors = 0
        self.meta = MetadataWriter(on_error=self._metadata_error)

        # STATUSTEXT messages from the FC (to show *why* arming failed etc.)
        self.recent_statustext = []

        # Last test photo taken from the portal (None until one is taken)
        self.test_photo = None

    # ---------------- status ----------------

    def write_status(self):
        """
        Write worker state so the portal can display it.
        """
        data = {
            "connected": self.fc_ready,
            "mission_uploaded": self.mission_uploaded,
            "mission_count": self.mission_count,
            "capture_enabled": self.capture_enabled,
            "state": self.state,
            "message": self.message,
            "last_error": self.last_error,
            "last_command_id": self.last_command_id,
            "camera_settings": self.camera_settings,
            "photos_this_flight": self.photos_this_flight,
            "capture_errors": self.capture_errors,
            "metadata_errors": self.meta.errors,
            "metadata_pending": self.meta.pending(),
            "test_photo": self.test_photo,
            "timestamp": time.time(),
        }

        tmp = STATUS_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f)
        os.replace(tmp, STATUS_FILE)
        self._last_status_write = time.time()

    def write_status_periodic(self):
        if time.time() - self._last_status_write >= STATUS_PERIOD_S:
            self.write_status()

    def set_status(self, state: str, message: str):
        """
        Helper to update state and write status immediately.
        """
        self.state = state
        self.message = message
        if state == "error":
            self.last_error = message
        print(f"[STATUS] {state}: {message}", flush=True)
        self.write_status()

    def set_message(self, message: str):
        """Update the message without changing the state (e.g. keep ready_to_start)."""
        self.message = message
        print(f"[INFO] {message}", flush=True)
        self.write_status()

    def _metadata_error(self, text: str):
        print(f"[WARN] {text}", flush=True)
        self.last_error = text

    # ---------------- MAVLink helpers ----------------

    def is_vehicle_heartbeat(self, msg) -> bool:
        """
        Only accept HEARTBEATs from the autopilot itself. ArduPilot forwards
        heartbeats of other MAVLink devices (GCS on telemetry radio, etc.),
        and those must not be read as 'disarmed' or 'mode changed'.
        """
        if msg is None or msg.get_type() != "HEARTBEAT":
            return False
        if msg.get_srcSystem() != self.master.target_system:
            return False
        if msg.get_srcComponent() != self.master.target_component:
            return False
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            return False
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return False
        return True

    def _note_statustext(self, msg):
        if msg is not None and msg.get_type() == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            text = text.strip("\x00 ").strip()
            if text:
                self.recent_statustext.append((time.time(), text))
                self.recent_statustext = self.recent_statustext[-20:]
                print(f"[FC] {text}", flush=True)

    def statustext_since(self, t0: float) -> str:
        texts = [t for (ts, t) in self.recent_statustext if ts >= t0]
        return " | ".join(texts[-5:])

    def poll_fc_connection(self):
        """
        Non-blocking FC connection step, called from the main loop until the FC is ready.
        The camera and test photos work while this waits, with or without a flight controller.
        FC side unchanged: wait for an autopilot heartbeat, send the same stream request,
        then wait FC_SETTLE_S before any other command is sent to the FC.
        """
        now = time.time()

        # Heartbeat received: wait out the settle time (no commands to the FC meanwhile)
        if self.fc_connected_at is not None:
            if now - self.fc_connected_at >= FC_SETTLE_S:
                self.fc_ready = True
                self.set_status("idle", "Worker ready (flight controller connected)")
            return

        # Open the serial port (retry if it is missing, e.g. a USB FC that is unplugged)
        if self.master is None:
            if now - self._fc_last_open_try < FC_REOPEN_S:
                return
            self._fc_last_open_try = now
            try:
                self.master = mavutil.mavlink_connection(FC_DEVICE, baud=FC_BAUD)
            except Exception as e:
                self.master = None
                self._fc_waiting_note(f"Cannot open {FC_DEVICE} ({e}). Camera works, flight needs the FC.")
                return

        # Read whatever arrived; look for the autopilot's heartbeat
        try:
            msg = self.master.recv_match(blocking=False)
        except Exception as e:
            self._close_fc()
            self._fc_waiting_note(f"FC link error ({e}), retrying...")
            return
        while msg is not None:
            if (
                msg.get_type() == "HEARTBEAT"
                and msg.type != mavutil.mavlink.MAV_TYPE_GCS
                and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                self.master.target_system = msg.get_srcSystem()
                self.master.target_component = msg.get_srcComponent()

                # Unchanged from the original
                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    FC_STREAM_RATE_HZ,
                    1
                )
                self.fc_connected_at = time.time()
                self.set_status(
                    "connected",
                    f"Connected to flight controller (sys {self.master.target_system}, "
                    f"comp {self.master.target_component}), settling {FC_SETTLE_S:.0f} s...",
                )
                return
            msg = self.master.recv_match(blocking=False)

        self._fc_waiting_note(
            f"No flight controller on {FC_DEVICE} @ {FC_BAUD} yet (check wiring / baud). "
            "Camera and test photo work; upload and flight need the FC."
        )

    def _fc_waiting_note(self, text: str):
        """Show 'waiting for FC' in the portal, without spamming the log every loop."""
        if self.state == "connecting_fc" and text == self._fc_note:
            return   # already shown; keeps other messages (e.g. "Test photo ready") visible
        if self.state in ("idle", "connecting_fc", "booting"):
            self._fc_note = text
            self.set_status("connecting_fc", text)

    def _close_fc(self):
        try:
            if self.master is not None:
                self.master.close()
        except Exception:
            pass
        self.master = None

    def require_fc(self):
        if not self.fc_ready:
            raise RuntimeError("Flight controller is not connected yet")

    def send_command_wait_ack(self, command: int, params, timeout_s: float = 3.0):
        """
        Send COMMAND_LONG and wait for its COMMAND_ACK.
        Returns (result_code or None on timeout, statustext collected meanwhile).
        """
        p = list(params) + [0] * (7 - len(params))
        t0 = time.time()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            command,
            0,
            *p[:7],
        )
        deadline = t0 + timeout_s
        while time.time() < deadline:
            msg = self.master.recv_match(
                type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=0.5
            )
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                self._note_statustext(msg)
                continue
            if msg.command == command:
                return msg.result, self.statustext_since(t0)
        return None, self.statustext_since(t0)

    def wait_for_heartbeat_condition(self, condition, timeout_s: float):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self.master.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                self._note_statustext(msg)
                continue
            if self.is_vehicle_heartbeat(msg) and condition(msg):
                return msg
        return None

    def is_armed_from_heartbeat(self, hb_msg):
        """
        Return True if vehicle is armed based on HEARTBEAT base_mode.
        """
        return bool(hb_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def arm(self):
        t0 = time.time()
        result, texts = self.send_command_wait_ack(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1], timeout_s=5.0
        )
        if result is not None and result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"Arming refused (result {result}). {texts or 'No reason reported by FC'}")
        msg = self.wait_for_heartbeat_condition(self.is_armed_from_heartbeat, 10.0)
        if msg is None:
            raise TimeoutError(
                f"Vehicle did not report armed state. {self.statustext_since(t0) or ''}".strip()
            )

    def set_auto_mode_confirmed(self, retries: int = 5, timeout_s: float = 2.0):
        """
        Set ArduCopter AUTO mode and confirm it from HEARTBEAT.custom_mode.
        If the RC flight-mode switch forces another mode, confirmation will fail.
        """
        for attempt in range(1, retries + 1):
            self.set_status("starting", f"Setting AUTO mode ({attempt}/{retries})...")

            # Same two mode requests as the original
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
            "or a neutral/GCS-controlled mode, then try Start Flight again. "
            + self.statustext_since(time.time() - retries * timeout_s)
        )

    # ---------------- camera ----------------

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
        Does not change the worker state (so 'ready_to_start' stays visible).
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
            self.set_message("Camera exposure set to auto")
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
        self.set_message(f"Camera exposure set to manual: {exposure_time} us, gain {analogue_gain}")

    def take_test_photo(self):
        """
        Ground-only test shot with the current camera settings, so you can check
        exposure / focus from the portal. Commands are never processed inside
        the flight loop, so this cannot run during a mission.
        Saves a full-size JPEG and a small preview for the phone.
        """
        if self.cam is None:
            raise RuntimeError("Camera is not initialized yet")
        os.makedirs(TEST_PHOTO_DIR, exist_ok=True)
        self.set_message("Taking test photo...")

        tmp_full = TEST_PHOTO_FULL + ".tmp.jpg"
        request = self.cam.capture_request()
        try:
            request.save("main", tmp_full)
            md = request.get_metadata() or {}
        finally:
            request.release()
        os.replace(tmp_full, TEST_PHOTO_FULL)

        # Small preview so the phone loads it quickly over the hotspot
        tmp_prev = TEST_PHOTO_PREVIEW + ".tmp.jpg"
        try:
            from PIL import Image
            with Image.open(TEST_PHOTO_FULL) as im:
                im.thumbnail((TEST_PREVIEW_MAX_PX, TEST_PREVIEW_MAX_PX))
                im.save(tmp_prev, "JPEG", quality=85)
        except Exception as e:
            print(f"[WARN] Preview resize failed ({e}); using full image", flush=True)
            import shutil
            shutil.copyfile(TEST_PHOTO_FULL, tmp_prev)
        os.replace(tmp_prev, TEST_PHOTO_PREVIEW)

        def num(key, nd=2):
            v = md.get(key)
            return round(float(v), nd) if isinstance(v, (int, float)) else None

        self.test_photo = {
            "time": time.time(),
            "exposure_us": num("ExposureTime", 0),
            "analogue_gain": num("AnalogueGain"),
            "lux": num("Lux", 0),
            "camera_mode": self.camera_settings.get("mode"),
        }
        self.set_message("Test photo ready")

    # ---------------- commands (queue directory) ----------------

    def read_command(self):
        """
        Take the oldest command from the queue directory and remove it.
        Return None if the queue is empty.
        """
        try:
            names = sorted(n for n in os.listdir(COMMAND_DIR) if n.endswith(".json"))
        except FileNotFoundError:
            os.makedirs(COMMAND_DIR, exist_ok=True)
            return None

        for name in names:
            path = os.path.join(COMMAND_DIR, name)
            try:
                with open(path, "r", encoding="utf-8") as f:
                    cmd = json.load(f)
            except Exception as e:
                print(f"[WARN] Dropping unreadable command {name}: {e}", flush=True)
                cmd = None
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
            if isinstance(cmd, dict) and cmd.get("id") is not None:
                return cmd
        return None

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
                self.require_fc()
                self.upload_mission(path)

            elif name == "start_flight":
                self.require_fc()
                self.start_flight()

            elif name == "set_camera":
                self.apply_camera_settings(
                    mode=cmd.get("mode", "manual"),
                    exposure_time=cmd.get("exposure_time", self.camera_settings["exposure_time"]),
                    analogue_gain=cmd.get("analogue_gain", self.camera_settings["analogue_gain"]),
                )

            elif name == "test_photo":
                self.take_test_photo()

            elif name == "idle":
                pass

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

    # ---------------- mission upload ----------------

    def upload_mission(self, mission_path: str):
        """
        Upload mission to FC using MAVLink mission protocol.
        """
        # A failed upload must never leave an old "uploaded" flag behind.
        self.mission_uploaded = False
        self.mission_count = 0
        self.set_status("uploading", f"Uploading mission from {mission_path} ...")

        mission = read_qgc_waypoints(mission_path)
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

        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            total
        )

        while True:
            msg = self.master.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                blocking=True,
                timeout=10
            )

            if msg is None:
                raise TimeoutError("Timeout waiting for mission protocol message")

            # Ignore protocol messages addressed to someone else (e.g. a GCS on another link)
            if getattr(msg, "target_system", self.master.source_system) not in (0, self.master.source_system):
                continue

            mtype = msg.get_type()

            if mtype == "MISSION_ACK":
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
                    int(round(item["lat"] * 1e7)),
                    int(round(item["lon"] * 1e7)),
                    item["alt"],
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
                    0
                )

        self.mission_uploaded = True
        self.mission_count = total
        self.set_status("ready_to_start", f"Mission uploaded successfully ({total} items)")

    # ---------------- flight ----------------

    def start_flight(self):
        """
        Start the mission, then enter a dedicated flight loop.
        During flight loop portal commands wait in the queue.
        Like the original, no disarm command is sent if a step fails
        (ArduPilot disarms by itself on the ground after DISARM_DELAY).
        """
        if not self.mission_uploaded:
            raise RuntimeError("Mission has not been uploaded yet")

        self.set_status("starting", "Arming and starting mission...")

        self.arm()
        self.set_auto_mode_confirmed()

        result, texts = self.send_command_wait_ack(
            mavutil.mavlink.MAV_CMD_MISSION_START, [0, 0], timeout_s=3.0
        )
        if result is not None and result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"MISSION_START refused (result {result}). {texts}")

        # enable capture only once the mission is really running
        self.capture_enabled = True
        self.photos_this_flight = 0
        self.capture_errors = 0
        self.set_status("running", "Mission started, entering flight loop")
        self.mission_execution_loop()

    def handle_camera_feedback(self, msg):
        """
        When CAMERA_FEEDBACK arrives and capture is enabled:
        capture a photo now, write metadata in the background.
        Never raises: one bad photo must not stop the rest of the survey.
        """
        try:
            fb = feedback_to_dict(msg)
            ts = time.strftime("%Y%m%d_%H%M%S", time.gmtime(fb["utc_sec"])) + f"_{fb['utc_usec']:06d}"
            # Same name format as before; only if that name already exists add a suffix
            path = os.path.join(PHOTO_DIR, f"photo_{ts}.jpg")
            n = 1
            while os.path.exists(path):
                path = os.path.join(PHOTO_DIR, f"photo_{ts}_{n}.jpg")
                n += 1

            self.cam.capture_file(path)
            self.photos_this_flight += 1
            self.meta.submit(path, fb)

            print(f"[TRIGGER] Saved photo: {path}", flush=True)
            self.message = f"Photos this flight: {self.photos_this_flight}. Last: {os.path.basename(path)}"
            self.write_status()
        except Exception as e:
            self.capture_errors += 1
            self.last_error = f"Photo capture failed: {e}"
            print(f"[ERROR] {self.last_error}", flush=True)
            self.write_status()

    def mission_execution_loop(self):
        """
        Flight loop:
        - no portal commands (they stay queued)
        - only MAVLink messages
        - save photos on CAMERA_FEEDBACK
        - exit when mission has started and the *autopilot* reports disarmed
        - never blocks forever: reports link loss but keeps waiting
        """
        mission_started = False
        last_vehicle_hb = time.time()
        self.set_status("flight_mode", "Flight loop active")

        while True:
            msg = self.master.recv_match(blocking=True, timeout=1.0)
            now = time.time()

            if msg is not None:
                mtype = msg.get_type()

                if mtype == "CAMERA_FEEDBACK":
                    if self.capture_enabled:
                        self.handle_camera_feedback(msg)

                elif mtype == "STATUSTEXT":
                    self._note_statustext(msg)

                elif mtype == "HEARTBEAT" and self.is_vehicle_heartbeat(msg):
                    last_vehicle_hb = now
                    if self.state == "link_lost":
                        self.set_status("flight_mode", "Link to FC restored")

                    armed = self.is_armed_from_heartbeat(msg)
                    if armed:
                        mission_started = True

                    if mission_started and not armed:
                        # Unchanged from the original (vehicle is already landed and disarmed here)
                        try:
                            self.master.mav.set_mode_send(
                                self.master.target_system,
                                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                0
                            )
                        except Exception:
                            pass

                        self.capture_enabled = False
                        self.set_message("Landed and disarmed, finishing photo metadata...")
                        self.meta.flush()
                        self.set_status(
                            "mission_complete",
                            f"Mission finished: {self.photos_this_flight} photos, "
                            f"{self.capture_errors} capture errors, {self.meta.errors} metadata errors",
                        )
                        break

            if now - last_vehicle_hb > HEARTBEAT_LOST_S and self.state != "link_lost":
                self.set_status("link_lost", "No heartbeat from FC, still waiting (photos resume when link returns)")

            self.write_status_periodic()

    def loop(self):
        """
        Main worker loop.

        In idle mode:
        - listens to portal commands (test photo / camera work without an FC)
        - connects to the FC in the background until it is ready

        When start_flight is received:
        - enters mission_execution_loop()
        - returns here only after mission is complete
        """
        while self.running:
            if os.path.exists(DONE_FLAG):
                self.capture_enabled = False
                self.set_status("finished", "Done flag detected, stopping worker")
                break

            cmd = self.read_command()
            if cmd is not None:
                self.handle_command(cmd)

            if not self.fc_ready:
                self.poll_fc_connection()
            else:
                # keep reading the link so STATUSTEXT is seen and buffers don't fill
                msg = self.master.recv_match(blocking=False)
                while msg is not None:
                    self._note_statustext(msg)
                    msg = self.master.recv_match(blocking=False)

            self.write_status_periodic()
            time.sleep(0.05)

        self.cleanup()

    def cleanup(self):
        """
        Clean shutdown.
        """
        self.meta.flush(timeout_s=30)
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
    worker.setup_camera()   # camera first: test photos work without a flight controller
    worker.loop()           # connects to the FC in the background
