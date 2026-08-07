import csv
import argparse
from pathlib import Path


# MAVLink / Mission Planner command IDs
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206
MAV_CMD_DO_DIGICAM_CONTROL = 203
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20

DEFAULT_SENSOR_HEIGHT_MM = 2.76
DEFAULT_FOCAL_LENGTH_MM = 3.04
DEFAULT_IMAGE_HEIGHT_PX = 2464
DEFAULT_FRONT_OVERLAP_PCT = 25.0


def compute_trigger_distance_from_camera(
    altitude_agl_m,
    sensor_height_mm=DEFAULT_SENSOR_HEIGHT_MM,
    focal_length_mm=DEFAULT_FOCAL_LENGTH_MM,
    image_height_px=DEFAULT_IMAGE_HEIGHT_PX,
    front_overlap_pct=DEFAULT_FRONT_OVERLAP_PCT,
):
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


def read_waypoints(input_file):
    """
    Διαβάζει waypoints από CSV/TXT αρχείο.
    Υποστηρίζει:
    - lat,lon,alt
    - latitude,longitude,altitude
    - lat,lon,alt_m,type (πχ SurveyEntry / InteriorTrigger / SurveyExit / Turnaround)
    - χωρίς header
    """
    waypoints = []

    with open(input_file, "r", encoding="utf-8") as f:
        sample = f.read(2048)
        f.seek(0)

        has_header = csv.Sniffer().has_header(sample)
        reader = csv.reader(f)

        if has_header:
            f.seek(0)
            dict_reader = csv.DictReader(f)
            for row in dict_reader:
                keys = {k.strip().lstrip("\ufeff").lower(): k for k in row.keys()}

                lat_key = keys.get("lat") or keys.get("latitude")
                lon_key = keys.get("lon") or keys.get("lng") or keys.get("longitude")
                alt_key = keys.get("alt_m") or keys.get("altitude")
                type_key = keys.get("type") or keys.get("wp_type") or keys.get("waypoint_type")

                if not lat_key or not lon_key:
                    raise ValueError("Δεν βρέθηκαν στήλες lat/lon στο αρχείο.")
                if not type_key:
                    raise ValueError("Δεν βρέθηκε στήλη type στο αρχείο (πχ SurveyEntry / InteriorTrigger / SurveyExit / Turnaround).")

                lat = float(row[lat_key])
                lon = float(row[lon_key])
                alt = float(row[alt_key]) if alt_key else 10.0
                wptype = (row.get(type_key) or "").strip()
                if not wptype:
                    raise ValueError("Η στήλη type είναι κενή σε κάποια γραμμή.")

                waypoints.append({"lat": lat, "lon": lon, "alt": alt, "type": str(wptype).strip()})
        else:
            for i, row in enumerate(reader, start=1):
                if not row or len(row) < 2:
                    continue
                if len(row) < 4:
                    raise ValueError(f"Λείπει type στη γραμμή {i}. Περιμένω 4 στήλες: lat,lon,alt,type")

                try:
                    lat = float(row[0].strip())
                    lon = float(row[1].strip())
                    alt = float(row[2].strip()) if len(row) > 2 and row[2].strip() else 50.0
                    wptype = row[3].strip() if len(row) > 3 and row[3].strip() else ""
                    if not wptype:
                        raise ValueError(f"Κενό type στη γραμμή {i}.")
                    waypoints.append({"lat": lat, "lon": lon, "alt": alt, "type": str(wptype).strip()})
                except ValueError:
                    raise ValueError(f"Λάθος format στη γραμμή {i}: {row}")

    if not waypoints:
        raise ValueError("Δεν βρέθηκαν waypoints στο input αρχείο.")

    return waypoints


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
    Δημιουργεί QGC WPL 110 mission:
    - HOME (seq 0, required for Mission Planner)
    - TAKEOFF (προαιρετικό)
    - WP0: πρώτο waypoint
    - Trigger modes:
      - waypoint: MAV_CMD_DO_DIGICAM_CONTROL σε κάθε InteriorTrigger waypoint (απαιτεί type)
      - distance: MAV_CMD_DO_SET_CAM_TRIGG_DIST, OFF στο SurveyExit / ON στο SurveyEntry
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
            "type": str(wp["type"]).strip(),
        }
        for wp in waypoints
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
        wps = [wp for wp in wps if (wp.get("type") or "").strip() != "InteriorTrigger"]
        if not wps:
            raise ValueError("Distance trigger mode needs at least one non-InteriorTrigger waypoint")

    # Πρώτο waypoint (για altitude reference)
    first_lat, first_lon, first_alt = wps[0]["lat"], wps[0]["lon"], wps[0]["alt"]

    # Mission Planner expects seq=0 to be "HOME" (not a real mission item).
    # If TAKEOFF is seq=0, MP may treat it as HOME and hide it.
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

    # 2) Αλλαγή ταχύτητας αμέσως μετά
    add_item(
        current=0,
        frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=MAV_CMD_DO_CHANGE_SPEED,
        p1=0, p2=2, p3=0, p4=0,
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

    trigger_enabled = False
    if effective_mode == "distance":
        # Ξεκινάμε OFF και ανοίγουμε στο πρώτο SurveyEntry
        trigger_enabled = False
        if (wps[0].get("type") or "").strip() == "SurveyEntry":
            add_item(
                current=0,
                frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=MAV_CMD_DO_SET_CAM_TRIGG_DIST,
                p1=trigger_distance, p2=0, p3=0, p4=0,
                lat=0, lon=0, alt=0
            )
            trigger_enabled = True

    # Υπόλοιπα waypoints
    for wp in wps[1:]:
        wptype = (wp.get("type") or "").strip()

        if effective_mode == "distance" and (wptype == "SurveyEntry") and (not trigger_enabled):
            add_item(
                current=0,
                frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=MAV_CMD_DO_SET_CAM_TRIGG_DIST,
                p1=trigger_distance, p2=0, p3=0, p4=0,
                lat=0, lon=0, alt=0
            )
            trigger_enabled = True

        add_item(
            current=0,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=MAV_CMD_NAV_WAYPOINT,
            p1=0, p2=0, p3=0, p4=0,
            lat=wp["lat"], lon=wp["lon"], alt=wp["alt"]
        )

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
            # Trigger one photo when reaching this waypoint
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
    parser.add_argument("input", help="Input CSV/TXT με lat,lon,alt")
    parser.add_argument("output", help="Output .waypoints file")
    parser.add_argument(
        "--trigger-mode",
        choices=["distance", "waypoint"],
        default="distance",
        help="distance: DO_SET_CAM_TRIGG_DIST (OFF στο SurveyExit / ON στο SurveyEntry) | waypoint: DO_DIGICAM_CONTROL στα InteriorTrigger"
    )
    parser.add_argument(
        "--trigger-distance",
        type=float,
        default=None,
        help="Photo spacing in meters. If omitted, it is computed from camera parameters."
    )
    parser.add_argument(
        "--altitude-agl",
        type=float,
        default=None,
        help="Altitude AGL for camera trigger calculation. Default: first waypoint altitude."
    )
    parser.add_argument(
        "--sensor-height-mm",
        type=float,
        default=DEFAULT_SENSOR_HEIGHT_MM,
        help=f"Camera sensor height in mm (default: {DEFAULT_SENSOR_HEIGHT_MM})"
    )
    parser.add_argument(
        "--focal-length-mm",
        type=float,
        default=DEFAULT_FOCAL_LENGTH_MM,
        help=f"Camera focal length in mm (default: {DEFAULT_FOCAL_LENGTH_MM})"
    )
    parser.add_argument(
        "--image-height-px",
        type=int,
        default=DEFAULT_IMAGE_HEIGHT_PX,
        help=f"Image height in pixels (default: {DEFAULT_IMAGE_HEIGHT_PX})"
    )
    parser.add_argument(
        "--front-overlap",
        type=float,
        default=DEFAULT_FRONT_OVERLAP_PCT,
        help=f"Front overlap percent for trigger distance calculation (default: {DEFAULT_FRONT_OVERLAP_PCT})"
    )
    parser.add_argument(
        "--no-takeoff",
        action="store_true",
        help="Μην προσθέσεις TAKEOFF στην αρχή"
    )
    parser.add_argument(
        "--takeoff-alt",
        type=float,
        default=None,
        help="TAKEOFF altitude (relative, meters). Default: use first waypoint altitude"
    )
    parser.add_argument(
        "--land-at-last",
        action="store_true",
        help="LAND στο τελευταίο waypoint (default: RTL για επιστροφή στο arming/home position)"
    )
    parser.add_argument(
        "--keep-trigger-on",
        action="store_true",
        help="Μην κλείσεις το trigger στο τέλος"
    )

    args = parser.parse_args()

    waypoints = read_waypoints(args.input)
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
    write_mission(args.output, mission_lines)

    print(f"Mission ready: {args.output}")
    print(f"Waypoints: {len(waypoints)}")
    print(f"Trigger mode: {args.trigger_mode}")
    if args.trigger_mode == "distance":
        print(f"Trigger distance: {trigger_distance:.3f} m")


if __name__ == "__main__":
    main()
