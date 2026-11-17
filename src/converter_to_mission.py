"""
Waypoint CSV -> QGC WPL 110 mission (fixed copy).

The generated mission matches Upload/converter_to_mission.py for every input the
original read correctly (same overlap, speed, trigger distance, takeoff, RTL),
with one approved difference in distance mode: camera ON (DO_SET_CAM_TRIGG_DIST)
is placed AFTER each SurveyEntry with param3=1 (one photo at the line start).
Everything else changed only input reading and safety checks: see CHANGES.md.
"""
import csv
import argparse
import sys


# MAVLink / Mission Planner command IDs
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206
MAV_CMD_DO_DIGICAM_CONTROL = 203
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20

# Unchanged tuning values
DEFAULT_SENSOR_HEIGHT_MM = 2.76
DEFAULT_FOCAL_LENGTH_MM = 3.04
DEFAULT_IMAGE_HEIGHT_PX = 2464
DEFAULT_FRONT_OVERLAP_PCT = 25.0
MISSION_SPEED_MPS = 2           # DO_CHANGE_SPEED value, same as before

# Only used for warnings (never change the mission)
WARN_MAX_ALT_M = 120.0
WARN_MIN_PHOTO_INTERVAL_S = 1.5

# Waypoint types are matched case-insensitively; '_', '-' and spaces are ignored
WP_TYPES = {
    "surveyentry": "SurveyEntry",
    "interiortrigger": "InteriorTrigger",
    "surveyexit": "SurveyExit",
    "turnaround": "Turnaround",
}

LAT_KEYS = ("lat", "latitude")
LON_KEYS = ("lon", "lng", "long", "longitude")
ALT_KEYS = ("alt_m", "alt", "altitude", "altitude_m", "height", "height_m")
TYPE_KEYS = ("type", "wp_type", "waypoint_type")


def warn(text):
    print(f"WARNING: {text}", file=sys.stderr)


def normalize_type(raw, where):
    key = "".join(ch for ch in str(raw).strip().lower() if ch not in "_- ")
    if not key:
        raise ValueError(f"Κενό type ({where}).")
    if key not in WP_TYPES:
        raise ValueError(
            f"Άγνωστο type {raw!r} ({where}). Επιτρεπτά: {', '.join(WP_TYPES.values())}"
        )
    return WP_TYPES[key]


def compute_trigger_distance_from_camera(
    altitude_agl_m,
    sensor_height_mm=DEFAULT_SENSOR_HEIGHT_MM,
    focal_length_mm=DEFAULT_FOCAL_LENGTH_MM,
    image_height_px=DEFAULT_IMAGE_HEIGHT_PX,
    front_overlap_pct=DEFAULT_FRONT_OVERLAP_PCT,
):
    # Unchanged (same formula, so the same number down to the last digit)
    altitude_agl_m = float(altitude_agl_m)
    sensor_height_mm = float(sensor_height_mm)
    focal_length_mm = float(focal_length_mm)
    image_height_px = int(image_height_px)
    front_overlap_pct = float(front_overlap_pct)

    if altitude_agl_m <= 0:
        raise ValueError("--altitude-agl or first waypoint altitude must be > 0")
    if sensor_height_mm <= 0:
        raise ValueError("--sensor-height-mm must be > 0")
    if focal_length_mm <= 0:
        raise ValueError("--focal-length-mm must be > 0")
    if image_height_px <= 0:
        raise ValueError("--image-height-px must be > 0")
    if not (0 <= front_overlap_pct < 100):
        raise ValueError("--front-overlap must be >= 0 and < 100")

    gsd_h = (altitude_agl_m * sensor_height_mm / focal_length_mm) / image_height_px
    footprint_h = gsd_h * image_height_px
    return footprint_h * (1.0 - front_overlap_pct / 100.0)


def _looks_like_header(row):
    """A header row is one whose first two cells are not numbers."""
    if len(row) < 2:
        return False
    for cell in row[:2]:
        try:
            float(cell.strip())
            return False
        except ValueError:
            pass
    return True


def read_waypoints(input_file):
    """
    Διαβάζει waypoints από CSV/TXT αρχείο.
    Υποστηρίζει:
    - header με lat/latitude, lon/lng/longitude, alt_m/alt/altitude, type
    - χωρίς header: lat,lon,alt,type
    Το altitude είναι υποχρεωτικό (πριν έμπαινε σιωπηλά 10 m ή 50 m).
    """
    waypoints = []

    with open(input_file, "r", encoding="utf-8-sig", newline="") as f:
        rows = [r for r in csv.reader(f) if r and any(c.strip() for c in r)]

    if not rows:
        raise ValueError("Δεν βρέθηκαν waypoints στο input αρχείο.")

    def parse_alt(raw, where):
        raw = (raw or "").strip()
        if not raw:
            raise ValueError(f"Λείπει altitude ({where}).")
        return float(raw)

    if _looks_like_header(rows[0]):
        header = [h.strip().lower() for h in rows[0]]

        def find(keys):
            for k in keys:
                if k in header:
                    return header.index(k)
            return None

        lat_i, lon_i, alt_i, type_i = find(LAT_KEYS), find(LON_KEYS), find(ALT_KEYS), find(TYPE_KEYS)
        if lat_i is None or lon_i is None:
            raise ValueError(f"Δεν βρέθηκαν στήλες lat/lon στο αρχείο (header: {rows[0]}).")
        if type_i is None:
            raise ValueError("Δεν βρέθηκε στήλη type στο αρχείο (πχ SurveyEntry / InteriorTrigger / SurveyExit / Turnaround).")
        if alt_i is None:
            raise ValueError(f"Δεν βρέθηκε στήλη altitude (αναμένεται μία από {', '.join(ALT_KEYS)}).")

        for line_no, row in enumerate(rows[1:], start=2):
            where = f"γραμμή {line_no}"

            def cell(i):
                return row[i] if i < len(row) else ""

            try:
                lat = float(cell(lat_i))
                lon = float(cell(lon_i))
                alt = parse_alt(cell(alt_i), where)
            except ValueError as e:
                raise ValueError(f"Λάθος format στη {where}: {row} ({e})")
            wptype = normalize_type(cell(type_i), where)
            waypoints.append({"lat": lat, "lon": lon, "alt": alt, "type": wptype})
    else:
        for line_no, row in enumerate(rows, start=1):
            where = f"γραμμή {line_no}"
            if len(row) < 4:
                raise ValueError(f"Λείπει type στη {where}. Περιμένω 4 στήλες: lat,lon,alt,type")
            try:
                lat = float(row[0].strip())
                lon = float(row[1].strip())
                alt = parse_alt(row[2], where)
            except ValueError as e:
                raise ValueError(f"Λάθος format στη {where}: {row} ({e})")
            wptype = normalize_type(row[3], where)
            waypoints.append({"lat": lat, "lon": lon, "alt": alt, "type": wptype})

    if not waypoints:
        raise ValueError("Δεν βρέθηκαν waypoints στο input αρχείο.")

    return waypoints


def validate_waypoints(waypoints):
    """Reject clearly broken input; only warn about things that may be intended."""
    for i, wp in enumerate(waypoints, start=1):
        if not (-90 <= wp["lat"] <= 90) or not (-180 <= wp["lon"] <= 180):
            raise ValueError(f"Waypoint {i}: invalid lat/lon {wp['lat']}, {wp['lon']}")
        if wp["lat"] == 0 and wp["lon"] == 0:
            raise ValueError(f"Waypoint {i}: lat/lon is 0,0")
        if wp["alt"] <= 0:
            raise ValueError(f"Waypoint {i}: altitude {wp['alt']} m must be > 0")
        if wp["alt"] > WARN_MAX_ALT_M:
            warn(f"Waypoint {i}: altitude {wp['alt']} m is above {WARN_MAX_ALT_M:.0f} m")


def make_mission_lines(
    waypoints,
    trigger_distance=None,
    trigger_mode="distance",  # distance | waypoint
    add_takeoff=True,
    takeoff_alt_m=None,
    land_at_last=False,
    stop_trigger_at_end=True
):
    """
    Δημιουργεί QGC WPL 110 mission (ίδιο με το αρχικό):
    - HOME (seq 0, required for Mission Planner)
    - TAKEOFF (προαιρετικό)
    - WP0: πρώτο waypoint
    - Trigger modes:
      - waypoint: MAV_CMD_DO_DIGICAM_CONTROL σε κάθε InteriorTrigger waypoint (απαιτεί type)
      - distance: MAV_CMD_DO_SET_CAM_TRIGG_DIST, ON μετά το SurveyEntry (p3=1) / OFF μετά το SurveyExit
    - LAND ή RTL (προαιρετικό)
    """

    lines = ["QGC WPL 110"]
    seq = 0

    def add_item(current, frame, command, p1, p2, p3, p4, lat, lon, alt, autocontinue=1):
        nonlocal seq
        line = (
            f"{seq}\t{current}\t{frame}\t{command}\t"
            f"{p1}\t{p2}\t{p3}\t{p4}\t"
            f"{lat}\t{lon}\t{alt}\t{autocontinue}"
        )
        lines.append(line)
        seq += 1

    wps = [
        {
            "lat": float(wp["lat"]),
            "lon": float(wp["lon"]),
            "alt": float(wp["alt"]),
            "type": normalize_type(wp["type"], f"waypoint {i}"),
        }
        for i, wp in enumerate(waypoints, start=1)
    ]
    trigger_mode = (trigger_mode or "distance").strip().lower()
    if trigger_mode not in ("distance", "waypoint"):
        raise ValueError(f"Άγνωστο --trigger-mode: {trigger_mode!r}. Επιτρεπτά: distance|waypoint")
    effective_mode = trigger_mode
    if effective_mode == "distance":
        if trigger_distance is None:
            raise ValueError("Distance trigger mode requires a trigger distance")
        trigger_distance = float(trigger_distance)
        if trigger_distance <= 0:
            raise ValueError("--trigger-distance πρέπει να είναι > 0")
        wps = [wp for wp in wps if wp["type"] != "InteriorTrigger"]
        if not wps:
            raise ValueError("Distance trigger mode needs at least one non-InteriorTrigger waypoint")
        if not any(wp["type"] == "SurveyEntry" for wp in wps):
            raise ValueError(
                "Distance mode: no SurveyEntry waypoint, so the camera would never be switched on. "
                "Check the 'type' column."
            )
        interval_s = trigger_distance / MISSION_SPEED_MPS
        if interval_s < WARN_MIN_PHOTO_INTERVAL_S:
            warn(f"One photo every {interval_s:.2f} s at {MISSION_SPEED_MPS} m/s - the Pi may miss some.")
    else:
        if not any(wp["type"] == "InteriorTrigger" for wp in wps[1:]):
            raise ValueError("Waypoint mode: no InteriorTrigger waypoint, so no photo would be taken.")

    # ---- From here on: same mission structure as the original ----

    # Πρώτο waypoint (για altitude reference)
    first_lat, first_lon, first_alt = wps[0]["lat"], wps[0]["lon"], wps[0]["alt"]

    # Mission Planner expects seq=0 to be "HOME" (not a real mission item).
    add_item(
        current=1,
        frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=MAV_CMD_NAV_WAYPOINT,
        p1=0, p2=0, p3=0, p4=0,
        lat=first_lat, lon=first_lon, alt=first_alt
    )

    # 1) TAKEOFF ως πρώτο actionable mission item (seq=1)
    if add_takeoff:
        to_alt = float(first_alt if takeoff_alt_m is None else takeoff_alt_m)
        add_item(
            current=0,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=MAV_CMD_NAV_TAKEOFF,
            p1=0, p2=0, p3=0, p4=0,
            lat=0, lon=0, alt=to_alt
        )

    # 2) Αλλαγή ταχύτητας αμέσως μετά (unchanged: 2 m/s)
    add_item(
        current=0,
        frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=MAV_CMD_DO_CHANGE_SPEED,
        p1=0, p2=MISSION_SPEED_MPS, p3=0, p4=0,
        lat=0, lon=0, alt=0
    )

    # 3) Waypoints
    add_item(
        current=0,
        frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=MAV_CMD_NAV_WAYPOINT,
        p1=0, p2=0, p3=0, p4=0,
        lat=first_lat, lon=first_lon, alt=first_alt
    )

    # Camera ON is placed right AFTER each SurveyEntry waypoint (owner's decision,
    # 2026-11-17), so ArduPilot switches it on when the drone reaches the start of
    # the line, never during the turn or the move from the previous line.
    # p3=1 = "trigger once immediately": one photo exactly at SurveyEntry, then one
    # every trigger_distance.
    def add_trigger_on():
        add_item(
            current=0,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=MAV_CMD_DO_SET_CAM_TRIGG_DIST,
            p1=trigger_distance, p2=0, p3=1, p4=0,
            lat=0, lon=0, alt=0
        )

    trigger_enabled = False
    if effective_mode == "distance":
        if wps[0]["type"] == "SurveyEntry":
            add_trigger_on()
            trigger_enabled = True

    # Υπόλοιπα waypoints
    for wp in wps[1:]:
        wptype = wp["type"]

        add_item(
            current=0,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=MAV_CMD_NAV_WAYPOINT,
            p1=0, p2=0, p3=0, p4=0,
            lat=wp["lat"], lon=wp["lon"], alt=wp["alt"]
        )

        if effective_mode == "distance" and (wptype == "SurveyEntry") and (not trigger_enabled):
            add_trigger_on()
            trigger_enabled = True

        if effective_mode == "distance" and (wptype == "SurveyExit") and trigger_enabled:
            add_item(
                current=0,
                frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=MAV_CMD_DO_SET_CAM_TRIGG_DIST,
                p1=0, p2=0, p3=0, p4=0,
                lat=0, lon=0, alt=0
            )
            trigger_enabled = False

        if effective_mode == "waypoint" and wptype == "InteriorTrigger":
            add_item(
                current=0,
                frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=MAV_CMD_DO_DIGICAM_CONTROL,
                p1=1, p2=0, p3=0, p4=0,
                lat=1, lon=0, alt=0
            )

    if effective_mode == "distance" and stop_trigger_at_end:
        if trigger_enabled:
            add_item(
                current=0,
                frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=MAV_CMD_DO_SET_CAM_TRIGG_DIST,
                p1=0, p2=0, p3=0, p4=0,
                lat=0, lon=0, alt=0
            )

    # End action: either LAND at last waypoint, or RTL (return + auto-land)
    if land_at_last:
        land_lat, land_lon, land_alt = wps[-1]["lat"], wps[-1]["lon"], wps[-1]["alt"]
        add_item(
            current=0,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=MAV_CMD_NAV_LAND,
            p1=0, p2=0, p3=0, p4=0,
            lat=land_lat, lon=land_lon, alt=float(land_alt)
        )
    else:
        add_item(
            current=0,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=MAV_CMD_NAV_RETURN_TO_LAUNCH,
            p1=0, p2=0, p3=0, p4=0,
            lat=0, lon=0, alt=0
        )

    return lines


def write_mission(output_file, mission_lines):
    with open(output_file, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(mission_lines))
        f.write("\n")


def main():
    parser = argparse.ArgumentParser(
        description="Μετατροπή waypoint αρχείου σε Mission Planner survey mission με φωτογραφίες."
    )
    parser.add_argument("input", help="Input CSV/TXT με lat,lon,alt,type")
    parser.add_argument("output", help="Output .waypoints file")
    parser.add_argument(
        "--trigger-mode",
        choices=["distance", "waypoint"],
        default="distance",
        help="distance: DO_SET_CAM_TRIGG_DIST (ON μετά το SurveyEntry με φωτο στην αρχή / OFF μετά το SurveyExit) | waypoint: DO_DIGICAM_CONTROL στα InteriorTrigger"
    )
    parser.add_argument("--trigger-distance", type=float, default=None,
                        help="Photo spacing in meters. If omitted, it is computed from camera parameters.")
    parser.add_argument("--altitude-agl", type=float, default=None,
                        help="Altitude AGL for camera trigger calculation. Default: first waypoint altitude.")
    parser.add_argument("--sensor-height-mm", type=float, default=DEFAULT_SENSOR_HEIGHT_MM,
                        help=f"Camera sensor height in mm (default: {DEFAULT_SENSOR_HEIGHT_MM})")
    parser.add_argument("--focal-length-mm", type=float, default=DEFAULT_FOCAL_LENGTH_MM,
                        help=f"Camera focal length in mm (default: {DEFAULT_FOCAL_LENGTH_MM})")
    parser.add_argument("--image-height-px", type=int, default=DEFAULT_IMAGE_HEIGHT_PX,
                        help=f"Image height in pixels (default: {DEFAULT_IMAGE_HEIGHT_PX})")
    parser.add_argument("--front-overlap", type=float, default=DEFAULT_FRONT_OVERLAP_PCT,
                        help=f"Front overlap percent for trigger distance calculation (default: {DEFAULT_FRONT_OVERLAP_PCT})")
    parser.add_argument("--no-takeoff", action="store_true", help="Μην προσθέσεις TAKEOFF στην αρχή")
    parser.add_argument("--takeoff-alt", type=float, default=None,
                        help="TAKEOFF altitude (relative, meters). Default: use first waypoint altitude")
    parser.add_argument("--land-at-last", action="store_true",
                        help="LAND στο τελευταίο waypoint (default: RTL για επιστροφή στο arming/home position)")
    parser.add_argument("--keep-trigger-on", action="store_true", help="Μην κλείσεις το trigger στο τέλος")

    args = parser.parse_args()

    try:
        waypoints = read_waypoints(args.input)
        validate_waypoints(waypoints)

        trigger_distance = args.trigger_distance
        if args.trigger_mode == "distance" and trigger_distance is None:
            altitude_agl_m = args.altitude_agl
            if altitude_agl_m is None:
                altitude_agl_m = waypoints[0]["alt"]
            trigger_distance = compute_trigger_distance_from_camera(
                altitude_agl_m=altitude_agl_m,
                sensor_height_mm=args.sensor_height_mm,
                focal_length_mm=args.focal_length_mm,
                image_height_px=args.image_height_px,
                front_overlap_pct=args.front_overlap,
            )

        mission_lines = make_mission_lines(
            waypoints=waypoints,
            trigger_distance=trigger_distance,
            trigger_mode=args.trigger_mode,
            add_takeoff=not args.no_takeoff,
            takeoff_alt_m=args.takeoff_alt,
            land_at_last=args.land_at_last,
            stop_trigger_at_end=not args.keep_trigger_on
        )
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(2)

    write_mission(args.output, mission_lines)

    print(f"Mission ready: {args.output}")
    print(f"Waypoints: {len(waypoints)}")
    print(f"Trigger mode: {args.trigger_mode}")
    if args.trigger_mode == "distance":
        print(f"Trigger distance: {trigger_distance:.3f} m")


if __name__ == "__main__":
    main()
