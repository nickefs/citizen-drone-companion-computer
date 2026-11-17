# What changed vs the first release

This release brings in the fixed companion code (developed as `Upload_fixed`). In this repository the files keep the
repo layout: Python code in `src/`, scripts in `scripts/`, the XMP config in `config/`. Paths below that say
`/home/pi/<file>` refer to the original flat layout on the Pi; here they map to:

| Original (flat, `/home/pi/`) | This repository |
|---|---|
| `fc_worker.py`, `mission_portal.py`, `mission_info.py`, `converter_to_mission.py` | `src/` |
| `drone_xmp.config` | `config/drone_xmp.config` |
| `setup_drone.sh` | `scripts/setup_rpi.sh` (hotspot password from `HOTSPOT_PASSWORD`) |
| `local_script.sh` | `scripts/run_local.sh` |
| `mission_script2.sh` | `scripts/mission_script2.sh` |

Runtime data (photos, uploads, status and command files) still lives under `/home/pi/`.

**Kept exactly as before (your tuning):**
- **The mission sent to the drone.** Same overlap (25 %), speed (2 m/s), trigger distance, takeoff and RTL.
  The fixed converter's output was compared byte-for-byte with the original's on 21 input/option combinations, and all were identical.
  **One approved exception since then:** camera-ON placement in distance mode (see the next section).
- **XMP writing.** `write_xmp_drone()` and `drone_xmp.config` are unchanged.
- **What the Pi sends to the flight controller.** Same 4 Hz stream request, same two AUTO-mode requests, same STABILIZE request after landing,
  same arm and MISSION_START commands.
- **Photo file names.** Still `photo_YYYYMMDD_HHMMSS_uuuuuu.jpg`.
- **Camera resolution and exposure handling.**

---

## Approved mission change (2026-11-17): camera ON after SurveyEntry, with a photo at the line start

Approved by the owner. Distance mode only; waypoint mode is unchanged.

| | Before | Now |
|---|---|---|
| Where `DO_SET_CAM_TRIGG_DIST` (ON) sits | Before each SurveyEntry (lines 2+); after it only on line 1 | **After** every SurveyEntry, the same on every line |
| When ArduPilot switches the camera on | When the drone reaches the previous waypoint (the Turnaround, or the previous SurveyExit if there's no turnaround), so photos could be taken in the turn or on the move between lines | When the drone reaches SurveyEntry, so only along the survey line |
| param3 ("trigger once immediately") | 0: first photo one trigger distance after ON | **1**: one photo exactly at SurveyEntry, then one every trigger distance |

Camera OFF (after SurveyExit), trigger distance, overlap and everything else are unchanged.

- Checked on a 3-line sample mission: the only differences from the old output are the ON items moving after SurveyEntry and
  param3 0 → 1. `mission_info.py` needed no change: it now counts photos on the survey lines only (9 instead of 11 on the sample).
- **Use a turnaround distance of about 8–10 m in the planner.** With turnaround 0, SurveyEntry is the corner where the drone
  turns, so the photo at SurveyEntry would be taken mid-turn.
- **Not yet flown.** Check the first flight's photos: one should be at the start edge of every line, and none in the turns.

---

## Can any change affect the drone while it flies?

**Short answer: no.** The mission runs on the flight controller. During the flight the Pi only *listens* (CAMERA_FEEDBACK and
heartbeats) and saves photos. Neither the original nor the fixed worker sends anything to the flight controller between
MISSION_START and landing. The only exception is the STABILIZE request after it has landed and disarmed, which is unchanged.

Here is every change that touches the drone, the flight controller or the flight loop, and what it means:

| Change | When it acts | Effect on the drone |
|---|---|---|
| Arm command now waits for the FC's reply (COMMAND_ACK) and shows the reason if arming is refused | On the ground, before takeoff | None. Same arm command. If the FC says no, you now see why (e.g. "PreArm: GPS 1: Bad fix") instead of a timeout |
| MISSION_START now waits up to 3 s for the FC's reply | On the ground, right after AUTO | None. Same command. ArduCopter answers instantly. If there's no reply, the worker continues as before |
| If start fails after arming, **no disarm is sent (same as the original)** | Only if AUTO is not confirmed or MISSION_START is refused, i.e. still on the ground | None. Same as before: the drone stays armed on the ground until ArduPilot disarms it itself (DISARM_DELAY) or you disarm it |
| Mission upload rounds lat/lon to the nearest 1e-7° instead of cutting off the last digit | Mission upload | The FC now stores exactly the coordinates in the .waypoints file. Before, some came out 1e-7° (~1 cm) lower |
| Flight loop ignores heartbeats that are not from the autopilot (GCS, gimbal, other devices) | During flight | Only affects when the Pi *stops taking photos*. Before, a forwarded GCS heartbeat could make the Pi think the drone had landed, and it would stop photographing mid-mission. The drone itself was never affected |
| One failed photo no longer ends photo capture | During flight | Only the Pi. Before, one error (exiftool, disk, camera) silently stopped all remaining photos |
| EXIF/XMP written in a background thread | During flight | Only the Pi. The loop goes straight back to listening, so triggers aren't missed or delayed. On a Pi 3, exiftool now runs while the next photo is taken (more CPU load, nothing sent to the FC) |
| Flight loop no longer blocks forever on a dead serial link. It shows "No signal from flight controller" and keeps waiting | During flight | Only the Pi. Photos resume when the link returns |
| Portal: **Finish / Upload / Start are locked while flying** | During flight | Protects the flight. Before, pressing Finish mid-air killed the worker, so photos stopped |
| `mission_script2.sh` waits while the worker reports a flight before stopping it | During flight | Same as above, as a second safety layer |
| Mission upload clears the "uploaded" flag first | On the ground | Start Flight can no longer fly an old or half-uploaded mission after a failed upload |
| Commands go through a queue folder instead of one file | On the ground | Clicks are no longer lost or overwritten |
| **Take test photo** button | On the ground only | None. The worker reads commands only *outside* the flight loop, and the portal also refuses the request while flying. Nothing is sent to the FC |
| **Camera starts first, FC connects in the background** (owner's request) | Start-up, on the ground | None. Same heartbeat check, same stream request, same 10 s wait before any other FC command. Upload and Start are refused ("Flight controller is not connected yet") until that wait is over, so nothing reaches the FC earlier than before |
| Mission summary / flight-time % | When a mission is uploaded | None. It only *reads* the generated mission file. The mission sent to the drone is unchanged |

**Things that are still the same as before (on purpose):**
- The hotspot stays on during the flight.
- The 4 Hz telemetry streams stay on the 57600 link.
- Restarting a mission after landing resumes the way ArduPilot did before (no forced restart from item 1).

---

## Converter (`converter_to_mission.py`)

The mission it produces is unchanged. Only *reading the CSV* and *refusing bad input* changed:

| Before | Now |
|---|---|
| An altitude column named `alt` was ignored, so every waypoint silently got **10 m** (confirmed by running the original) | `alt_m`, `alt`, `altitude`, `altitude_m`, `height`, `height_m` are accepted. A missing altitude is an **error** (no silent 10 m / 50 m) |
| `SurveyEntry` had to match case exactly. With no match, the mission had **no photos** and no warning | Case-insensitive (`surveyentry`, `Survey_Entry`). Unknown types are an error. Error if distance mode has no SurveyEntry or waypoint mode has no InteriorTrigger |
| No checks on coordinates or altitude | Error for altitude ≤ 0, lat/lon out of range, or 0,0. **Warning only** (mission still made) above 120 m or for photo intervals under 1.5 s |
| Errors were Python tracebacks | `ERROR: …` message, shown in the portal |

Your mapdraw CSVs (`lat,lon,alt_m,type`) give exactly the same mission as before.

**(Fixed 2026-11-17 with the owner's approval; see "Approved mission change" above.)** One thing I noticed but did NOT change at the time: in distance mode, `DO_SET_CAM_TRIGG_DIST` (camera ON) is placed *before*
each SurveyEntry waypoint after the first one. ArduPilot runs it when the drone *leaves the previous Turnaround*, so photos are also taken
along the short Turnaround → SurveyEntry leg. If that's intended (or harmless for you), leave it.

## Worker (`fc_worker.py`) — other fixes (Pi side only)

- **Test photo works without the flight controller** (owner's request). Before, the worker waited forever for the FC
  before even opening the camera, so with the FC unplugged no command (test photo, camera settings) was ever handled.
  Now the camera opens first and the FC is connected from the main loop, retrying (including a missing serial port)
  until it answers. While waiting, the portal shows "Connecting to flight controller" and the test photo / camera
  settings work. Tested with a fake camera and FC: test photo with no FC, upload refused, then FC connects, same
  stream request sent once, ready after the settle time.
- Absolute path to `drone_xmp.config`, so exiftool finds it from any folder.
- EXIF is merged into the camera's own EXIF instead of replacing it. Adds DateTimeOriginal, GPSTimeStamp, GPSVersionID, GPSMapDatum,
  FocalLengthIn35mmFilm. GPS values are the same as before.
- If writing metadata fails for a photo, a `photo_….jpg.json` file with its position is saved next to it.
- When the FC has no GPS time (`time_usec = 0`), the Pi clock is used, so photos are no longer all called
  `photo_19700101_000000_000000.jpg` and overwritten. If a name already exists, `_1`, `_2`… is added.
- Changing camera settings no longer hides the "ready to fly" state.
- Status is refreshed every second, so the portal can tell when the worker has died.

## Portal (`mission_portal.py`) — new phone layout

- Made for a phone on the `Pi-Upload` Wi-Fi. Everything is served by the Pi (no internet needed). Big touch buttons.
- Same look as the Survey Planner: black and white, square corners, numbered steps (01–04), the same drone logo,
  and Auto / Light / Dark buttons (remembered on the phone). Only the look changed: every button, form and
  status update works exactly as before.
- Status bar at the top, always visible, with a coloured edge: green = ready / complete, blue = flying, yellow = busy, red = problem.
  It shows photos taken, mission items and photo errors, and updates every 2 s without reloading the page.
- Steps in order: **01 Mission → 02 Camera → 03 Fly → 04 Finish**.
- **Start flight** is enabled only when the drone has confirmed the mission, and asks for confirmation before arming.
- **Upload and Finish are locked while the drone is flying.**
- Shows the FC's reason when something fails (e.g. a pre-arm error).

### Take test photo (Camera card)
- Takes one photo with the **current camera settings** and shows it in the page. Tap it to open the full 3280×2464 image.
- Shows what the camera actually used: exposure time (µs), gain and brightness (lux). In Auto, this is a quick way to choose
  good Manual values.
- Saved to `/home/pi/test_photos/` (only the latest one is kept), never in `/home/pi/photos/`, so test shots don't end up in your survey.
- The phone gets a small 1280 px preview so it loads quickly over the hotspot (needs `python3-pil`, which `setup_drone.sh` now installs.
  Without it, the full image is shown).
- Disabled while flying.

### Mission summary card (after upload)
Calculated on the Pi from the generated `mission.waypoints` (`mission_info.py`):
- Path length, estimated flight time, approximate photo count, camera trigger spacing
- Survey and takeoff altitude, speed, number of waypoints
- GSD (cm/pixel), photo footprint on the ground, survey area (hectares), end of mission (RTL / land)

**Drone flight time %:** enter your drone's flight time on a full battery (minutes) once. It's saved on the Pi in
`/home/pi/portal_settings.json`. The card then shows how much of it the mission uses:
green up to 60 %, yellow 60–80 %, red above 80 % ("split the mission").

How the time is estimated:
- Legs at the mission speed (2 m/s), including acceleration.
- Climb 2.5 m/s, descent 1.5 m/s, last 10 m of landing at 0.5 m/s, 5 s loiter before landing, plus 3 s per corner and 10 s for arm/takeoff.
  These values are from your parameter file and sit at the top of `mission_info.py`.
- The return home is assumed at survey speed (ArduPilot may fly it faster), so the estimate is on the safe side.
- Home is assumed to be at the first survey point. If you take off far from the survey area, add that distance.
- Wind and battery ageing are not included. Use the % as a guide and keep your usual reserve.
- Photo count models where ArduPilot actually switches the camera on and off, so it's approximate (±1–2 per line).

## Shell scripts

- **setup_drone.sh**: same short layout as the original, with only these changes:
  - installs `piexif` (it installed `pyexif`, the wrong package) and `exiftool` (needed by fc_worker.py, was missing);
  - no `apt full-upgrade` (it upgraded the whole system, ~220 MB, and corrupt downloads broke setup);
  - `apt install --no-install-recommends` (otherwise picamera2's extras pull in mesa/libllvm19, which failed to install);
  - no `pip install picamera2` (the apt version is the one that matches libcamera);
  - **starts the mission on boot** (added at the owner's request): installs and enables a systemd service
    `drone-mission` that runs `/home/pi/mission_script2.sh` at every boot (hotspot → worker → portal, unchanged).
    It is only enabled, not started, so setup over SSH isn't cut off; it first runs after `sudo reboot`.
    After Finish the service ends and stays off until the next boot (or `sudo systemctl start drone-mission`).
    Turn off with `sudo systemctl disable drone-mission`.
  The hotspot password comes from the `HOTSPOT_PASSWORD` environment variable (not stored in the repo). Run it as `pi`, not with sudo.
- **mission_script2.sh**: clears old commands at start, waits while flying, and gives the worker 30 s to finish writing photo metadata before stopping it.
- **local_script.sh**: only stops the portal and worker (was `pkill -f python`, which killed every Python process).

## Deploying

Update the whole repository on the Pi **together** (the new portal needs the new worker and `src/mission_info.py`).

If your Pi is already set up, you only need the extra packages:

```bash
sudo apt install -y libimage-exiftool-perl python3-pil
```

```bash
/home/pi/venvs/drone/bin/pip install piexif
```

For a fresh Pi:

Clone the repository on the Pi, then from its folder:

```bash
HOTSPOT_PASSWORD='choose-a-password' ./scripts/setup_rpi.sh
```

```bash
sudo reboot
```

The hotspot and portal come up on their own after the reboot.

Keep the old files so you can switch back.

## How it was tested

- Converter: the original and fixed versions were run on 3 CSVs × 7 option sets, and all 21 missions were **byte-identical**. Bad inputs (alt header,
  no SurveyEntry, 300 m) were checked separately.
- Worker: 20/20 checks against a simulated ArduCopter, with exiftool missing and a camera error mid-flight. Includes the test photo
  (saved in its own folder, exposure info reported, camera released, survey photos untouched).
- Mission summary: checked by hand on a test survey (292.5 m path, 13 photos matching the ON/OFF trigger points).
- Portal: rendered at 375 px phone width with the summary card (green and red cases) and a real test photo. Buttons lock correctly
  in flight, and there's no sideways scrolling.
- **Not yet tested on the real Pi/drone.** Do one props-off bench run (upload → start → fake trigger → disarm) before flying.
