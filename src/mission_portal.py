from flask import Flask, request, redirect, url_for, send_file
import os, subprocess, time, json, sys
from pathlib import Path
from werkzeug.utils import secure_filename

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONVERTER_SCRIPT = PROJECT_ROOT / "src" / "converter_to_mission.py"

UPLOAD_DIR = "/home/pi/uploads"
OUT_DIR = "/home/pi/output"
STATUS_FILE = "/home/pi/status.txt"
DONE_FLAG = "/home/pi/done.flag"
WORKER_COMMAND_FILE = "/home/pi/worker_command.json"
WORKER_STATUS_FILE = "/home/pi/worker_status.json"

os.makedirs(UPLOAD_DIR, exist_ok=True)
os.makedirs(OUT_DIR, exist_ok=True)

app = Flask(__name__)
latest_output_path = None
has_upload = False

DEFAULT_CAMERA_MODE = "auto"
DEFAULT_EXPOSURE_TIME = 1200
DEFAULT_ANALOGUE_GAIN = 1.0

def write_status(s: str):
    with open(STATUS_FILE, "w") as f:
        f.write(s)

def send_worker_command(cmd_dict: dict):
    tmp = WORKER_COMMAND_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(cmd_dict, f)
    os.replace(tmp, WORKER_COMMAND_FILE)

def next_command_id():
    return int(time.time() * 1000)

def read_worker_data():
    if not os.path.exists(WORKER_STATUS_FILE):
        return {}
    try:
        with open(WORKER_STATUS_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}

def read_worker_status():
    data = read_worker_data()
    if not data:
        return ""
    try:
        camera = data.get("camera_settings") or {}
        camera_text = ""
        if camera:
            mode = camera.get("mode", "")
            if mode == "manual":
                camera_text = (
                    f' | Camera: manual, {camera.get("exposure_time", "")} us, '
                    f'gain {camera.get("analogue_gain", "")}'
                )
            elif mode == "auto":
                camera_text = " | Camera: auto"
        return f'{data.get("state", "")} | {data.get("message", "")}{camera_text}'
    except Exception:
        return "Worker status unavailable"

@app.get("/")
def index():
    status = ""
    if os.path.exists(STATUS_FILE):
        with open(STATUS_FILE, "r") as f:
            status = f.read()

    worker_status = read_worker_status()
    worker_data = read_worker_data()
    camera = worker_data.get("camera_settings") or {}
    camera_mode = camera.get("mode", DEFAULT_CAMERA_MODE)
    exposure_time = camera.get("exposure_time", DEFAULT_EXPOSURE_TIME)
    analogue_gain = camera.get("analogue_gain", DEFAULT_ANALOGUE_GAIN)
    download_ready = (latest_output_path is not None)

    if download_ready:
        download_html = '''
        <div id="downloadBox">
          <p><a href="/download">Download mission file</a></p>
        </div>
        '''
    else:
        download_html = '<div id="downloadBox" style="display:none"></div>'

    return f"""
    <html><body style="font-family:-apple-system,system-ui,sans-serif;padding:20px">
      <h2>Upload Waypoints -> MAVLink Mission</h2>
      <form action="/upload" method="post" enctype="multipart/form-data">
        <p>
          <input id="fileInput" type="file" name="file" />
        </p>
        <fieldset style="max-width:360px;margin:12px 0;padding:12px">
          <legend>Camera trigger</legend>
          <label>
            <input type="radio" name="trigger_mode" value="distance" checked>
            Distance
          </label>
          <label style="margin-left:16px">
            <input type="radio" name="trigger_mode" value="waypoint">
            Waypoint
          </label>
        </fieldset>
        <button type="submit">Upload & Process</button>
      </form>
      <hr>
      <p><b>Status:</b> {status}</p>
      <p><b>Worker:</b> {worker_status}</p>
      {download_html}

      <form action="/set_camera" method="post">
        <fieldset style="max-width:420px;margin:12px 0;padding:12px">
          <legend>Camera exposure</legend>
          <label>
            <input type="radio" name="camera_mode" value="auto" {"checked" if camera_mode == "auto" else ""}>
            Auto
          </label>
          <label style="margin-left:16px">
            <input type="radio" name="camera_mode" value="manual" {"checked" if camera_mode != "auto" else ""}>
            Manual
          </label>
          <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:12px">
            <label>Exposure time us
              <input type="number" min="1" step="1" name="exposure_time" value="{exposure_time}" style="width:100%">
            </label>
            <label>Analogue gain
              <input type="number" min="0.1" step="0.1" name="analogue_gain" value="{analogue_gain}" style="width:100%">
            </label>
          </div>
          <button type="submit" style="margin-top:10px">Apply Camera Settings</button>
        </fieldset>
      </form>

      <form action="/start_flight" method="post">
        <button type="submit">Start Flight</button>
      </form>

      <form action="/finish" method="post">
        <button type="submit">Finish (close hotspot)</button>
      </form>

      <script>
        const fileInput = document.getElementById("fileInput");
        const downloadBox = document.getElementById("downloadBox");
        if (fileInput && downloadBox) {{
          fileInput.addEventListener("change", () => {{
            downloadBox.style.display = "none";
          }});
        }}
      </script>
    </body></html>
    """

@app.post("/upload")
def upload():
    f = request.files.get("file")
    if not f or f.filename == "":
        write_status("No file selected")
        return redirect(url_for("index"))

    name = secure_filename(f.filename)
    in_path = os.path.join(UPLOAD_DIR, name)
    f.save(in_path)

    write_status(f"Uploaded {name}. Processing...")

    trigger_mode = (request.form.get("trigger_mode") or "distance").strip().lower()
    if trigger_mode not in ("distance", "waypoint"):
        trigger_mode = "distance"

    out_path = "/home/pi/output/mission.waypoints"
    cmd = [
        sys.executable,
        str(CONVERTER_SCRIPT),
        in_path,
        out_path,
        "--trigger-mode",
        trigger_mode
    ]

    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
       write_status("Converter error:\n" + (proc.stderr or proc.stdout)[-800:])
       return redirect(url_for("index"))

    send_worker_command({
        "id": next_command_id(),
        "cmd": "upload_mission",
        "path": out_path
    })

    write_status(f"Done. Mission file ready for download. Trigger mode: {trigger_mode}.")
    global has_upload, latest_output_path
    has_upload = True
    latest_output_path = out_path
    return redirect(url_for("index"))

@app.post("/set_camera")
def set_camera():
    camera_mode = (request.form.get("camera_mode") or DEFAULT_CAMERA_MODE).strip().lower()
    if camera_mode not in ("auto", "manual"):
        camera_mode = DEFAULT_CAMERA_MODE

    exposure_time = request.form.get("exposure_time") or str(DEFAULT_EXPOSURE_TIME)
    analogue_gain = request.form.get("analogue_gain") or str(DEFAULT_ANALOGUE_GAIN)
    try:
        exposure_time = int(float(exposure_time))
        analogue_gain = float(analogue_gain)
        if exposure_time <= 0 or analogue_gain <= 0:
            raise ValueError
    except ValueError:
        write_status("Invalid camera settings. Exposure time and gain must be > 0.")
        return redirect(url_for("index"))

    send_worker_command({
        "id": next_command_id(),
        "cmd": "set_camera",
        "mode": camera_mode,
        "exposure_time": exposure_time,
        "analogue_gain": analogue_gain,
    })

    write_status(f"Camera settings sent. Mode: {camera_mode}.")
    return redirect(url_for("index"))

@app.post("/start_flight")
def start_flight():
    send_worker_command({
        "id": next_command_id(),
        "cmd": "start_flight"
    })
    write_status("Start flight command sent.")
    return redirect(url_for("index"))

@app.get("/download")
def download():
    global latest_output_path
    if not latest_output_path:
        return "Not ready yet", 409
    return send_file(latest_output_path, as_attachment=True, download_name="mission.waypoints")

@app.post("/finish")
def finish():
    send_worker_command({
        "id": next_command_id(),
        "cmd": "finish"
    })

    with open(DONE_FLAG, "w") as f:
        f.write(str(time.time()))
    write_status("Finishing... You can close this page.")
    return redirect(url_for("index"))

if __name__ == "__main__":
    write_status("Ready. Upload a CSV.")
    if os.path.exists(DONE_FLAG):
        os.remove(DONE_FLAG)
    app.run(host="0.0.0.0", port=8000, debug=False)
