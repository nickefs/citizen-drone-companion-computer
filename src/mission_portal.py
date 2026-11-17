"""
Mission portal (fixed copy). See CHANGES.md for what changed compared to Upload/mission_portal.py.
"""
from flask import Flask, request, redirect, url_for, send_file, jsonify
import html
import itertools
import json
import os
import subprocess
import sys
import threading
import time
from werkzeug.utils import secure_filename

import mission_info   # mission_info.py next to this file (src/)

UPLOAD_DIR = "/home/pi/uploads"
OUT_DIR = "/home/pi/output"
STATUS_FILE = "/home/pi/status.txt"
DONE_FLAG = "/home/pi/done.flag"
WORKER_COMMAND_DIR = "/home/pi/worker_commands"   # queue: one <id>.json per command
WORKER_STATUS_FILE = "/home/pi/worker_status.json"

VENV_PY = sys.executable                                   # same Python that runs the portal
CONVERTER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "converter_to_mission.py")

MISSION_INFO_FILE = "/home/pi/output/mission_info.json"
SETTINGS_FILE = "/home/pi/portal_settings.json"      # e.g. the drone's flight time
TEST_PHOTO_FULL = "/home/pi/test_photos/test_full.jpg"
TEST_PHOTO_PREVIEW = "/home/pi/test_photos/test_preview.jpg"

# Battery-use colours for the "% of flight time" bar
FLIGHT_PCT_OK = 60
FLIGHT_PCT_WARN = 80

WORKER_STALE_S = 15.0            # worker writes status every ~1 s; older than this = not responding
FLIGHT_STATES = ("starting", "running", "flight_mode", "link_lost")
STARTABLE_STATES = ("ready_to_start", "mission_complete", "idle", "connected")

os.makedirs(UPLOAD_DIR, exist_ok=True)
os.makedirs(OUT_DIR, exist_ok=True)
os.makedirs(WORKER_COMMAND_DIR, exist_ok=True)

app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = 5 * 1024 * 1024   # waypoint CSVs are tiny
latest_output_path = None

DEFAULT_CAMERA_MODE = "auto"
DEFAULT_EXPOSURE_TIME = 1200
DEFAULT_ANALOGUE_GAIN = 1.0

_id_lock = threading.Lock()
_id_counter = itertools.count()


def _read_json(path, default):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def _write_json(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f)
    os.replace(tmp, path)


def read_settings():
    return _read_json(SETTINGS_FILE, {})


def read_mission_info():
    return _read_json(MISSION_INFO_FILE, None)


def flight_time_usage(info, settings):
    """Return (percent or None, tone) for the mission vs the drone's flight time."""
    try:
        max_min = float(settings.get("max_flight_min") or 0)
        est_s = float((info or {}).get("est_flight_time_s") or 0)
    except (TypeError, ValueError):
        return None, "idle"
    if max_min <= 0 or est_s <= 0:
        return None, "idle"
    pct = round(100.0 * est_s / (max_min * 60.0))
    tone = "ok" if pct <= FLIGHT_PCT_OK else ("busy" if pct <= FLIGHT_PCT_WARN else "bad")
    return pct, tone


def fmt_duration(sec):
    sec = int(round(sec or 0))
    return f"{sec // 60}:{sec % 60:02d}"


def fmt_length(m):
    m = float(m or 0)
    return f"{m / 1000:.2f} km" if m >= 1000 else f"{m:.0f} m"


def write_status(s: str):
    with open(STATUS_FILE, "w") as f:
        f.write(s)


def next_command_id():
    # Unique and increasing even for two clicks in the same millisecond.
    with _id_lock:
        return time.time_ns() * 1000 + (next(_id_counter) % 1000)


def send_worker_command(cmd_dict: dict):
    """Queue a command; the worker processes them oldest-first, none are overwritten."""
    cmd_id = cmd_dict["id"]
    final = os.path.join(WORKER_COMMAND_DIR, f"{cmd_id:024d}.json")
    tmp = final + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(cmd_dict, f)
    os.replace(tmp, final)


def pending_commands():
    try:
        return len([n for n in os.listdir(WORKER_COMMAND_DIR) if n.endswith(".json")])
    except Exception:
        return 0


def read_worker_data():
    if not os.path.exists(WORKER_STATUS_FILE):
        return {}
    try:
        with open(WORKER_STATUS_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def worker_is_stale(data: dict) -> bool:
    ts = data.get("timestamp")
    if not isinstance(ts, (int, float)):
        return True
    return (time.time() - ts) > WORKER_STALE_S


def worker_state(data: dict) -> str:
    # Even a stale status is used: a stale "flight_mode" still means "might be flying".
    return (data or {}).get("state", "unknown")


def read_worker_status_text(data: dict):
    if not data:
        return "No status from worker yet"
    try:
        camera = data.get("camera_settings") or {}
        camera_text = ""
        mode = camera.get("mode", "")
        if mode == "manual":
            camera_text = (
                f' | Camera: manual, {camera.get("exposure_time", "")} us, '
                f'gain {camera.get("analogue_gain", "")}'
            )
        elif mode == "auto":
            camera_text = " | Camera: auto"
        text = f'{data.get("state", "")} | {data.get("message", "")}{camera_text}'
        if worker_is_stale(data):
            age = time.time() - float(data.get("timestamp") or 0)
            text = f"WORKER NOT RESPONDING (last update {age:.0f}s ago) | " + text
        return text
    except Exception:
        return "Worker status unavailable"


def can_start_flight(data: dict):
    """Return (allowed, reason)."""
    if not data or worker_is_stale(data):
        return False, "Worker is not responding"
    if not data.get("connected"):
        return False, "Worker is not connected to the flight controller"
    if not data.get("mission_uploaded"):
        return False, "No mission confirmed on the flight controller yet"
    if data.get("state") not in STARTABLE_STATES:
        return False, f"Worker is busy ({data.get('state')})"
    if pending_commands() > 0:
        return False, "Worker still has queued commands"
    return True, ""


STATE_LABELS = {
    "booting": ("Starting up", "busy"),
    "connecting_fc": ("Connecting to flight controller", "busy"),
    "connected": ("Connected", "busy"),
    "camera_setup": ("Setting up camera", "busy"),
    "idle": ("Ready - upload a mission", "idle"),
    "uploading": ("Uploading mission to drone", "busy"),
    "ready_to_start": ("Mission on drone - ready to fly", "ok"),
    "starting": ("Arming and starting", "fly"),
    "running": ("Mission started", "fly"),
    "flight_mode": ("Flying - taking photos", "fly"),
    "link_lost": ("No signal from flight controller", "bad"),
    "mission_complete": ("Mission complete", "ok"),
    "error": ("Error", "bad"),
    "finished": ("Finished", "idle"),
    "stopped": ("Worker stopped", "bad"),
    "unknown": ("Waiting for worker", "busy"),
}


def status_payload():
    data = read_worker_data()
    allowed, reason = can_start_flight(data)
    portal_status = ""
    if os.path.exists(STATUS_FILE):
        with open(STATUS_FILE, "r") as f:
            portal_status = f.read()
    state = worker_state(data)
    label, tone = STATE_LABELS.get(state, (state, "busy"))
    stale = bool(data) and worker_is_stale(data)
    if stale:
        label, tone = "Worker not responding", "bad"
    camera = data.get("camera_settings") or {}
    if camera.get("mode") == "manual":
        camera_text = f'Manual · {camera.get("exposure_time", "")} µs · gain {camera.get("analogue_gain", "")}'
    else:
        camera_text = "Auto exposure"
    test = data.get("test_photo") or None
    test_text = ""
    if test:
        parts = []
        if test.get("exposure_us"):
            parts.append(f'{int(test["exposure_us"])} µs')
        if test.get("analogue_gain"):
            parts.append(f'gain {test["analogue_gain"]}')
        if test.get("lux"):
            parts.append(f'~{int(test["lux"])} lux')
        taken = time.strftime("%H:%M:%S", time.localtime(test.get("time") or 0))
        test_text = f"Taken {taken} ({test.get('camera_mode', '')})" + (": " + ", ".join(parts) if parts else "")
    return {
        "test_photo_ver": int(1000 * ((test or {}).get("time") or 0)),
        "last_command_id": str(data.get("last_command_id")),   # string: too big for a JS number
        "test_photo_text": test_text,
        "test_allowed": (not stale) and bool(data) and state not in FLIGHT_STATES and state != "uploading",
        "portal_status": portal_status,
        "worker_text": read_worker_status_text(data),
        "worker_message": data.get("message", ""),
        "worker_state": state,
        "state_label": label,
        "state_tone": tone,
        "stale": stale,
        "camera_text": camera_text,
        "photos": data.get("photos_this_flight", 0),
        "capture_errors": data.get("capture_errors", 0),
        "metadata_errors": data.get("metadata_errors", 0),
        "mission_count": data.get("mission_count", 0),
        "last_error": data.get("last_error", ""),
        "start_allowed": allowed,
        "start_reason": reason,
        "in_flight": state in FLIGHT_STATES,
        "download_ready": latest_output_path is not None,
    }


@app.get("/api/status")
def api_status():
    return jsonify(status_payload())


PAGE_CSS = """
:root{color-scheme:light;--bg:#f4f4f5;--panel:#fff;--soft:#f0f0f2;--text:#111113;--muted:#585860;--line:#e4e4e7;
--border:#c3c3ca;--primary:#111113;--primary-hover:#2b2b2f;--on-primary:#fff;
--ok:#1a7f37;--fly:#0b5cad;--bad:#c62828;--busy:#8a5a00;--idle:#585860}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){color-scheme:dark;--bg:#121417;--panel:#1b1e22;--soft:#25292f;
--text:#f4f4f5;--muted:#b7bcc4;--line:#343943;--border:#545b67;--primary:#f4f4f5;--primary-hover:#d8dbe0;--on-primary:#111113;
--ok:#4ade80;--fly:#93c5fd;--bad:#ff9a9a;--busy:#fcd34d;--idle:#b7bcc4}}
:root[data-theme="dark"]{color-scheme:dark;--bg:#121417;--panel:#1b1e22;--soft:#25292f;
--text:#f4f4f5;--muted:#b7bcc4;--line:#343943;--border:#545b67;--primary:#f4f4f5;--primary-hover:#d8dbe0;--on-primary:#111113;
--ok:#4ade80;--fly:#93c5fd;--bad:#ff9a9a;--busy:#fcd34d;--idle:#b7bcc4}
*{box-sizing:border-box}
html,body{margin:0;min-height:100%}
body{background:var(--bg);color:var(--text);
font:15px/1.5 Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;-webkit-text-size-adjust:100%}
button,input{font:inherit}
button:focus-visible,input:focus-visible,a:focus-visible{outline:3px solid var(--text);outline-offset:2px}
main{max-width:560px;min-height:100dvh;margin:0 auto;padding:27px 28px 40px;background:var(--panel);
box-shadow:0 0 32px rgba(0,0,0,.10)}
.brand{display:flex;align-items:center;gap:10px;font-size:12px;font-weight:700;letter-spacing:.13em}
.brand-mark{display:grid;place-items:center;width:30px;height:30px;border:1px solid var(--border)}
.brand-mark svg{width:24px;height:24px;display:block}
.prefs{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-top:18px}
.prefs-label{color:var(--muted);font-size:11px;font-weight:700}
.opts{display:flex}
.opts button{min-height:31px;padding:5px 9px;border:1px solid var(--border);background:var(--panel);color:var(--text);
font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.035em;cursor:pointer}
.opts button+button{border-left:0}
.opts button[aria-pressed="true"]{background:var(--primary);color:var(--on-primary);border-color:var(--primary)}
.heading{margin:28px 0 20px}
.eyebrow{color:var(--muted);font-size:11px;font-weight:750;letter-spacing:.16em}
h1{margin:9px 0 10px;font-size:31px;font-weight:600;line-height:1.1;letter-spacing:-.04em}
.heading p{margin:0;color:var(--muted);font-size:13px;line-height:1.55}
.status{position:sticky;top:0;z-index:5;margin:0 0 6px;padding:13px 14px 13px 16px;background:var(--soft);
border:1px solid var(--line);border-left:4px solid var(--idle);box-shadow:0 6px 14px rgba(0,0,0,.06)}
.status .big{display:flex;align-items:center;gap:8px;font-size:17px;font-weight:700;letter-spacing:-.01em}
.status .big::before{content:"";width:9px;height:9px;flex:none;background:currentColor}
.status .small{font-size:13px;margin-top:2px;color:var(--muted);word-break:break-word}
.status.tone-ok{border-left-color:var(--ok)} .status.tone-ok .big{color:var(--ok)}
.status.tone-fly{border-left-color:var(--fly)} .status.tone-fly .big{color:var(--fly)}
.status.tone-bad{border-left-color:var(--bad)} .status.tone-bad .big{color:var(--bad)}
.status.tone-busy{border-left-color:var(--busy)} .status.tone-busy .big{color:var(--busy)}
.status.tone-idle .big{color:var(--text)}
.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:1px;margin-top:11px;background:var(--line);border:1px solid var(--line)}
.stat{background:var(--panel);padding:7px 9px}
.stat b{display:block;font-size:19px;line-height:1.2}
.stat span{color:var(--muted);font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.08em}
.card{border-top:1px solid var(--line);padding:21px 0}
.section-heading{display:flex;align-items:center;gap:10px;margin-bottom:10px}
.step-number{display:inline-grid;place-items:center;width:26px;height:26px;flex:none;background:var(--soft);
font-size:11px;font-weight:800}
h2{margin:0 0 10px;font-size:17px;letter-spacing:-.02em}
.section-heading h2{margin:0}
.msg{white-space:pre-wrap;font-size:12px;color:var(--muted);margin:10px 0 0;word-break:break-word}
.err{color:var(--bad);font-size:12px;margin:8px 0 0;word-break:break-word}
.err:empty,.msg:empty{display:none}
.hint{font-size:12px;color:var(--muted);margin:8px 0 0;line-height:1.5}
.btn{display:flex;align-items:center;justify-content:space-between;gap:10px;width:100%;min-height:50px;margin-top:14px;
padding:12px 15px;border:1px solid var(--primary);background:var(--primary);color:var(--on-primary);cursor:pointer;
font-size:14px;font-weight:750;text-transform:uppercase;letter-spacing:.035em;text-align:left}
.btn:hover:not(:disabled){background:var(--primary-hover)}
.btn:disabled{opacity:.42;cursor:not-allowed}
.btn .arrow{font-size:20px;line-height:1}
.btn.alt{background:var(--panel);color:var(--text);border-color:var(--border);font-size:13px;font-weight:700}
.btn.alt:hover:not(:disabled){background:var(--soft)}
.btn.stop{background:var(--panel);color:var(--bad);border-color:var(--bad)}
.btn.stop:hover:not(:disabled){background:var(--soft)}
.file{display:block;margin-top:4px;padding:13px;border:1px dashed var(--border);background:var(--soft);
font-size:13px;font-weight:700}
.file input{display:block;width:100%;margin-top:9px;font-size:14px;color:var(--text)}
.field-label{display:block;margin:14px 0 7px;font-size:13px;font-weight:700}
.seg{display:flex;margin-top:4px}
.seg label{flex:1;display:block}
.seg input{position:absolute;opacity:0;pointer-events:none}
.seg span{display:flex;align-items:center;justify-content:center;min-height:42px;padding:8px 6px;border:1px solid var(--border);
background:var(--panel);color:var(--text);font-size:12px;font-weight:700;text-transform:uppercase;letter-spacing:.035em;cursor:pointer}
.seg label+label span{border-left:0}
.seg input:checked+span{background:var(--primary);color:var(--on-primary);border-color:var(--primary)}
.seg input:focus-visible+span{outline:3px solid var(--text);outline-offset:2px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
.grid2 label,.inline label{font-size:12px;font-weight:700;color:var(--muted)}
.grid2 input,.inline input{display:block;width:100%;margin-top:6px;min-height:44px;padding:8px 11px;border:1px solid var(--border);
background:var(--panel);color:var(--text);font-size:16px;font-weight:600}
a.dl{display:flex;justify-content:space-between;align-items:center;width:100%;min-height:44px;margin-top:12px;padding:10px 13px;
border:1px solid var(--border);color:var(--text);text-decoration:none;font-size:13px;font-weight:700;
text-transform:uppercase;letter-spacing:.035em}
a.dl:hover{background:var(--soft)}
.facts{display:grid;grid-template-columns:1fr 1fr;gap:12px 14px;padding:13px;border:1px solid var(--line);background:var(--soft)}
.fact span{display:block;color:var(--muted);font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.08em}
.fact b{font-size:16px}
.usage{margin-top:12px;padding:12px 13px;border:1px solid var(--line);border-left:4px solid var(--idle);background:var(--panel)}
.usage .big{font-size:17px;font-weight:700}
.usage .small{font-size:12px;color:var(--muted)}
.usage.tone-ok{border-left-color:var(--ok)} .usage.tone-ok .big{color:var(--ok)}
.usage.tone-busy{border-left-color:var(--busy)} .usage.tone-busy .big{color:var(--busy)}
.usage.tone-bad{border-left-color:var(--bad)} .usage.tone-bad .big{color:var(--bad)}
.meter{height:8px;background:var(--soft);overflow:hidden;margin-top:9px;border:1px solid var(--line)}
.meter i{display:block;height:100%}
.meter .tone-ok{background:var(--ok)} .meter .tone-busy{background:var(--busy)} .meter .tone-bad{background:var(--bad)}
.inline{display:flex;gap:8px;align-items:flex-end;margin-top:14px}
.inline label{flex:1}
.inline .btn{width:auto;margin:0;min-height:44px;padding:8px 16px;background:var(--panel);color:var(--text);border-color:var(--border);font-size:12px}
.inline .btn:hover{background:var(--soft)}
.photo{margin-top:12px}
.photo img{display:block;width:100%;height:auto;border:1px solid var(--line);background:var(--soft)}
.safety-note{margin-top:14px;padding-left:9px;border-left:2px solid var(--border);color:var(--muted);font-size:11px}
@media (max-width:600px){main{padding:22px 20px 34px;box-shadow:none}.heading{margin:20px 0 18px}h1{font-size:29px}}
"""

# Same drone mark as the survey planner (inline, so the portal works without internet)
DRONE_MARK_SVG = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512">
<rect width="512" height="512" fill="#111113"/>
<path d="M220 220 148 148m144 72 72-72M220 292l-72 72m144-72 72 72" fill="none" stroke="#fff" stroke-width="24" stroke-linecap="round"/>
<g fill="#111113" stroke="#fff" stroke-width="22"><circle cx="130" cy="130" r="43"/><circle cx="382" cy="130" r="43"/>
<circle cx="130" cy="382" r="43"/><circle cx="382" cy="382" r="43"/></g>
<rect x="204" y="204" width="104" height="104" rx="28" fill="#fff"/><circle cx="256" cy="256" r="20" fill="#111113"/>
</svg>"""


def summary_html(info, settings):
    """Mission summary card (rendered on the Pi, no internet needed)."""
    esc = html.escape
    max_min = settings.get("max_flight_min") or ""
    form = f"""
    <form class="inline" action="/set_flight_time" method="post">
      <label>Drone flight time (min, full battery)
        <input type="number" inputmode="decimal" min="1" max="120" step="0.5" name="max_flight_min"
               value="{esc(str(max_min))}" placeholder="e.g. 18">
      </label>
      <button class="btn" type="submit">Save</button>
    </form>"""

    if not info:
        return f"""
  <section class="card">
    <h2>Mission summary</h2>
    <p class="hint">Upload a mission to see its length, photos and flight time.</p>
    {form}
  </section>"""
    if info.get("error"):
        return f"""
  <section class="card">
    <h2>Mission summary</h2>
    <p class="err">Could not read the mission: {esc(str(info["error"]))}</p>
    {form}
  </section>"""

    def alt_text():
        lo, hi = info.get("alt_min_m"), info.get("alt_max_m")
        if lo is None:
            return "-"
        return f"{lo:g} m" if lo == hi else f"{lo:g}–{hi:g} m"

    spacing = info.get("trigger_spacing_m") or []
    if info.get("trigger_mode") == "distance" and spacing:
        trig = "every " + ", ".join(f"{s:.1f}" for s in spacing) + " m"
    elif info.get("trigger_mode") == "waypoint":
        trig = "at waypoints"
    else:
        trig = "none"

    facts = [
        ("Path length", fmt_length(info.get("path_length_m"))),
        ("Est. flight time", fmt_duration(info.get("est_flight_time_s")) + " min"),
        ("Photos (approx.)", f'~{info.get("est_photos", 0)}'),
        ("Camera trigger", trig),
        ("Survey altitude", alt_text()),
        ("Takeoff altitude", f'{info["takeoff_alt_m"]:g} m' if info.get("takeoff_alt_m") is not None else "-"),
        ("Speed", f'{info.get("survey_speed_mps", 0):g} m/s'),
        ("Waypoints", str(info.get("waypoints", 0))),
        ("GSD", f'{info.get("gsd_cm_px", 0):.2f} cm/px'),
        ("Photo footprint", f'{info.get("footprint_w_m", 0):.0f} × {info.get("footprint_h_m", 0):.0f} m'),
        ("Survey area", f'{info.get("area_ha", 0):.2f} ha'),
        ("End of mission", "Return home + land" if info.get("end_action") == "RTL" else "Land at last point"),
    ]
    facts_html = "".join(f'<div class="fact"><span>{esc(k)}</span><b>{esc(v)}</b></div>' for k, v in facts)

    pct, tone = flight_time_usage(info, settings)
    if pct is None:
        usage = '<p class="hint">Enter your drone\'s flight time below to see how much of the battery this mission uses.</p>'
    else:
        note = {
            "ok": "Fits comfortably in one battery.",
            "busy": "Tight - little reserve left. Consider splitting the mission.",
            "bad": "Too long for one battery with a safe reserve. Split the mission or fly higher.",
        }[tone]
        usage = f"""
    <div class="usage tone-{tone}">
      <div class="big">{pct}% of flight time</div>
      <div class="meter"><i class="tone-{tone}" style="width:{min(pct, 100)}%"></i></div>
      <div class="small" style="margin-top:6px">{esc(note)} ({fmt_duration(info.get("est_flight_time_s"))} of {float(max_min):g} min)</div>
    </div>"""

    return f"""
  <section class="card">
    <h2>Mission summary</h2>
    <div class="facts">{facts_html}</div>
    {usage}
    {form}
    <p class="hint">Estimate only. It assumes take-off near the first survey point, uses the flight controller's
    climb/descent/landing speeds, and is on the safe side (slower) for the return leg. Wind is not included.</p>
  </section>"""


@app.get("/")
def index():
    st = status_payload()
    worker_data = read_worker_data()
    camera = worker_data.get("camera_settings") or {}
    camera_mode = camera.get("mode", DEFAULT_CAMERA_MODE)
    exposure_time = camera.get("exposure_time", DEFAULT_EXPOSURE_TIME)
    analogue_gain = camera.get("analogue_gain", DEFAULT_ANALOGUE_GAIN)
    esc = html.escape
    manual = camera_mode == "manual"

    return f"""<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="theme-color" content="#111113">
<title>Drone Mission</title>
<script>
  try {{ const t = localStorage.getItem("portalTheme"); if (t === "light" || t === "dark") document.documentElement.dataset.theme = t; }} catch (e) {{}}
</script>
<style>{PAGE_CSS}</style>
</head><body><main>
  <div class="brand"><span class="brand-mark" aria-hidden="true">{DRONE_MARK_SVG}</span><span>MISSION <strong>PORTAL</strong></span></div>
  <div class="prefs">
    <span class="prefs-label">Appearance</span>
    <div class="opts" role="group" aria-label="Appearance">
      <button type="button" data-theme-choice="system">Auto</button><button type="button" data-theme-choice="light">Light</button><button type="button" data-theme-choice="dark">Dark</button>
    </div>
  </div>
  <div class="heading">
    <span class="eyebrow">COMPANION COMPUTER</span>
    <h1>Fly your survey</h1>
    <p>Upload the planner's waypoint CSV, check the camera, then start the flight.</p>
  </div>

  <div id="statusBar" class="status tone-{esc(st["state_tone"])}">
    <div class="big" id="stateLabel">{esc(st["state_label"])}</div>
    <div class="small" id="workerMsg">{esc(st["worker_message"])}</div>
    <div class="stats">
      <div class="stat"><b id="photos">{st["photos"]}</b><span>photos</span></div>
      <div class="stat"><b id="items">{st["mission_count"] or "-"}</b><span>mission items</span></div>
      <div class="stat"><b id="errors">{st["capture_errors"] + st["metadata_errors"]}</b><span>photo errors</span></div>
    </div>
  </div>

  <section class="card">
    <div class="section-heading"><span class="step-number">01</span><h2>Mission</h2></div>
    <form action="/upload" method="post" enctype="multipart/form-data">
      <label class="file">Waypoint CSV from the planner
        <input id="fileInput" type="file" name="file" accept=".csv,.txt,text/csv,text/plain">
      </label>
      <span class="field-label">Camera trigger</span>
      <div class="seg" role="radiogroup" aria-label="Camera trigger">
        <label><input type="radio" name="trigger_mode" value="distance" checked><span>By distance</span></label>
        <label><input type="radio" name="trigger_mode" value="waypoint"><span>At waypoints</span></label>
      </div>
      <button class="btn" type="submit" id="uploadBtn" {"disabled" if st["in_flight"] else ""}>Upload &amp; send to drone <span class="arrow" aria-hidden="true">→</span></button>
    </form>
    <p class="msg" id="portalStatus">{esc(st["portal_status"])}</p>
    <a class="dl" id="downloadLink" href="/download" style="{"" if st["download_ready"] else "display:none"}">Download mission file <span aria-hidden="true">↓</span></a>
  </section>
{summary_html(read_mission_info() if st["download_ready"] else None, read_settings())}

  <section class="card">
    <div class="section-heading"><span class="step-number">02</span><h2>Camera</h2></div>
    <form action="/set_camera" method="post">
      <span class="field-label">Exposure</span>
      <div class="seg" role="radiogroup" aria-label="Exposure">
        <label><input type="radio" name="camera_mode" value="auto" {"" if manual else "checked"}><span>Auto</span></label>
        <label><input type="radio" name="camera_mode" value="manual" {"checked" if manual else ""}><span>Manual</span></label>
      </div>
      <div class="grid2">
        <label>Exposure (µs)
          <input type="number" inputmode="numeric" min="1" step="1" name="exposure_time" value="{esc(str(exposure_time))}">
        </label>
        <label>Analogue gain
          <input type="number" inputmode="decimal" min="0.1" step="0.1" name="analogue_gain" value="{esc(str(analogue_gain))}">
        </label>
      </div>
      <p class="hint">Now: <span id="cameraText">{esc(st["camera_text"])}</span>. Values are used only in Manual.</p>
      <button class="btn" type="submit">Apply camera settings <span class="arrow" aria-hidden="true">→</span></button>
    </form>
    <button class="btn alt" id="testBtn" type="button" {"" if st["test_allowed"] else "disabled"}>Take test photo <span class="arrow" aria-hidden="true">◎</span></button>
    <p class="hint" id="testText">{esc(st["test_photo_text"]) or "Takes one photo with the current settings (on the ground only)."}</p>
    <div class="photo" id="testPhotoBox" style="{"" if st["test_photo_ver"] else "display:none"}">
      <a id="testFullLink" href="/test_photo_full.jpg?v={st["test_photo_ver"]}" target="_blank">
        <img id="testImg" alt="Test photo" src="{"/test_photo.jpg?v=" + str(st["test_photo_ver"]) if st["test_photo_ver"] else ""}">
      </a>
      <p class="hint">Tap the photo to open it in full resolution.</p>
    </div>
  </section>

  <section class="card">
    <div class="section-heading"><span class="step-number">03</span><h2>Fly</h2></div>
    <form action="/start_flight" method="post"
          onsubmit="return confirm('ARM the motors and start the mission?\\n\\nProps clear, area safe, RC in hand?');">
      <button class="btn go" id="startBtn" type="submit" {"" if st["start_allowed"] else "disabled"}>Start flight <span class="arrow" aria-hidden="true">→</span></button>
    </form>
    <p class="hint" id="startReason">{esc(st["start_reason"])}</p>
    <p class="err" id="lastError">{esc(st["last_error"])}</p>
    <p class="safety-note">Props clear, area safe and RC in hand before you start. The drone arms as soon as you confirm.</p>
  </section>

  <section class="card">
    <div class="section-heading"><span class="step-number">04</span><h2>Finish</h2></div>
    <form action="/finish" method="post" onsubmit="return confirm('Stop the worker and turn off the hotspot?');">
      <button class="btn stop" id="finishBtn" type="submit" {"disabled" if st["in_flight"] else ""}>Finish &amp; close hotspot</button>
    </form>
    <p class="hint">Disabled while the drone is flying, so photo capture is never stopped mid-air.</p>
  </section>
</main>
<script>
  (function () {{
    const root = document.documentElement;
    const btns = document.querySelectorAll("[data-theme-choice]");
    function show(choice) {{
      btns.forEach((b) => b.setAttribute("aria-pressed", b.dataset.themeChoice === choice ? "true" : "false"));
    }}
    show(root.dataset.theme || "system");
    btns.forEach((b) => b.addEventListener("click", () => {{
      const choice = b.dataset.themeChoice;
      if (choice === "system") delete root.dataset.theme; else root.dataset.theme = choice;
      try {{ if (choice === "system") localStorage.removeItem("portalTheme"); else localStorage.setItem("portalTheme", choice); }} catch (e) {{}}
      show(choice);
    }}));
  }})();
  const $ = (id) => document.getElementById(id);
  const fileInput = $("fileInput");
  if (fileInput) fileInput.addEventListener("change", () => {{ $("downloadLink").style.display = "none"; }});
  let testVer = {st["test_photo_ver"]};
  let testWaiting = false;
  let testId = null;
  $("testBtn").addEventListener("click", async () => {{
    $("testBtn").disabled = true;
    $("testText").textContent = "Taking test photo...";
    testWaiting = true;
    try {{
      const r = await fetch("/test_photo", {{method: "POST"}});
      const j = await r.json();
      testId = j.id || null;
      if (!j.ok) {{ testWaiting = false; $("testText").textContent = j.error || "Could not take a test photo"; }}
    }} catch (e) {{ testWaiting = false; $("testText").textContent = "Lost connection to the Pi"; }}
  }});
  async function refresh() {{
    try {{
      const r = await fetch("/api/status", {{cache: "no-store"}});
      const s = await r.json();
      $("statusBar").className = "status tone-" + s.state_tone;
      $("stateLabel").textContent = s.state_label;
      $("workerMsg").textContent = s.worker_message || "";
      $("photos").textContent = s.photos;
      $("items").textContent = s.mission_count || "-";
      $("errors").textContent = (s.capture_errors || 0) + (s.metadata_errors || 0);
      $("portalStatus").textContent = s.portal_status || "";
      $("cameraText").textContent = s.camera_text;
      $("startBtn").disabled = !s.start_allowed;
      $("startReason").textContent = s.start_reason || "";
      $("lastError").textContent = s.last_error || "";
      $("finishBtn").disabled = s.in_flight;
      $("uploadBtn").disabled = s.in_flight;
      $("downloadLink").style.display = s.download_ready ? "" : "none";
      if (!testWaiting) $("testBtn").disabled = !s.test_allowed;
      if (s.test_photo_ver && s.test_photo_ver !== testVer) {{
        testVer = s.test_photo_ver;
        testWaiting = false;
        $("testImg").src = "/test_photo.jpg?v=" + testVer;
        $("testFullLink").href = "/test_photo_full.jpg?v=" + testVer;
        $("testPhotoBox").style.display = "";
        $("testBtn").disabled = !s.test_allowed;
      }}
      if (s.test_photo_text && !testWaiting) $("testText").textContent = s.test_photo_text;
      // worker finished our command but no new photo appeared -> it failed
      if (testWaiting && testId && s.last_command_id === testId && s.test_photo_ver === testVer) {{
        testWaiting = false;
        $("testText").textContent = s.last_error || "Test photo failed";
        $("testBtn").disabled = !s.test_allowed;
      }}
    }} catch (e) {{
      $("statusBar").className = "status tone-bad";
      $("stateLabel").textContent = "Lost connection to the Pi";
    }}
  }}
  setInterval(refresh, 2000);
</script>
</body></html>
"""


@app.post("/upload")
def upload():
    global latest_output_path
    f = request.files.get("file")
    if not f or f.filename == "":
        write_status("No file selected")
        return redirect(url_for("index"))

    if worker_state(read_worker_data()) in FLIGHT_STATES:
        write_status("Cannot upload a mission while a flight is in progress.")
        return redirect(url_for("index"))

    name = secure_filename(f.filename) or "upload.csv"
    in_path = os.path.join(UPLOAD_DIR, name)
    f.save(in_path)

    write_status(f"Uploaded {name}. Processing...")

    trigger_mode = (request.form.get("trigger_mode") or "distance").strip().lower()
    if trigger_mode not in ("distance", "waypoint"):
        trigger_mode = "distance"

    out_path = os.path.join(OUT_DIR, "mission.waypoints")
    cmd = [
        VENV_PY,
        CONVERTER,
        in_path,
        out_path,
        "--trigger-mode",
        trigger_mode
    ]

    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        write_status("Converter timed out")
        return redirect(url_for("index"))
    if proc.returncode != 0:
        latest_output_path = None
        write_status("Converter error:\n" + (proc.stderr or proc.stdout)[-800:])
        return redirect(url_for("index"))

    send_worker_command({
        "id": next_command_id(),
        "cmd": "upload_mission",
        "path": out_path
    })

    # Mission summary for the portal (read-only, does not touch the mission)
    try:
        info = mission_info.summarize(out_path)
    except Exception as e:
        info = {"error": str(e)}
    try:
        _write_json(MISSION_INFO_FILE, info)
    except Exception:
        pass

    warnings = (proc.stderr or "").strip()
    msg = (
        f"Converted ({trigger_mode} trigger). Uploading to the flight controller - "
        f"wait for Worker: ready_to_start before flying."
    )
    if warnings:
        msg += "\nConverter warnings:\n" + warnings[-600:]
    write_status(msg)
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
    allowed, reason = can_start_flight(read_worker_data())
    if not allowed:
        write_status(f"Start flight refused: {reason}")
        return redirect(url_for("index"))

    send_worker_command({
        "id": next_command_id(),
        "cmd": "start_flight"
    })
    write_status("Start flight command sent.")
    return redirect(url_for("index"))


@app.post("/test_photo")
def test_photo():
    """Ask the worker for one test shot. Refused while flying (JSON reply for the page's script)."""
    st = status_payload()
    if not st["test_allowed"]:
        reason = "Not while flying" if st["in_flight"] else "Worker is busy or not responding"
        return jsonify({"ok": False, "error": reason})
    cmd_id = next_command_id()
    send_worker_command({"id": cmd_id, "cmd": "test_photo"})
    return jsonify({"ok": True, "id": str(cmd_id)})


def _send_no_cache(path, **kw):
    if not os.path.exists(path):
        return "No test photo yet", 404
    resp = send_file(path, mimetype="image/jpeg", **kw)
    resp.headers["Cache-Control"] = "no-store"
    return resp


@app.get("/test_photo.jpg")
def test_photo_preview():
    return _send_no_cache(TEST_PHOTO_PREVIEW)


@app.get("/test_photo_full.jpg")
def test_photo_full():
    return _send_no_cache(TEST_PHOTO_FULL)


@app.post("/set_flight_time")
def set_flight_time():
    raw = (request.form.get("max_flight_min") or "").strip()
    settings = read_settings()
    if raw == "":
        settings.pop("max_flight_min", None)
    else:
        try:
            val = float(raw)
            if not (1 <= val <= 120):
                raise ValueError
        except ValueError:
            write_status("Flight time must be a number of minutes between 1 and 120.")
            return redirect(url_for("index"))
        settings["max_flight_min"] = val
    _write_json(SETTINGS_FILE, settings)
    return redirect(url_for("index"))


@app.get("/download")
def download():
    if not latest_output_path or not os.path.exists(latest_output_path):
        return "Not ready yet", 409
    return send_file(latest_output_path, as_attachment=True, download_name="mission.waypoints")


@app.post("/finish")
def finish():
    if worker_state(read_worker_data()) in FLIGHT_STATES:
        write_status("Finish refused: the drone may be flying. Wait until it has landed and disarmed.")
        return redirect(url_for("index"))

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
