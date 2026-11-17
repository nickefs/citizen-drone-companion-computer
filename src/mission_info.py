"""
Mission summary for the portal: path length, estimated photos, altitudes,
GSD / photo footprint, survey area and an estimated flight time.

It only READS the generated mission.waypoints file. It never changes the
mission or talks to the drone.

Run by hand:  python mission_info.py /home/pi/output/mission.waypoints
"""
import json
import math
import sys

# ---- Flight-controller values used for the time estimate ----
# Taken from Drone Parameters/23_2_2026_params.param. Update if you change them on the FC.
CLIMB_SPEED_MPS = 2.5        # WPNAV_SPEED_UP 250
DESCENT_SPEED_MPS = 1.5      # WPNAV_SPEED_DN 150
LAND_SPEED_MPS = 0.5         # LAND_SPEED 50
LAND_ALT_LOW_M = 10.0        # LAND_ALT_LOW 1000
RTL_ALT_M = 15.0             # RTL_ALT 1500
RTL_LOITER_S = 5.0           # RTL_LOIT_TIME 5000
ACCEL_MPS2 = 2.5             # WPNAV_ACCEL 250
DEFAULT_SPEED_MPS = 10.0     # WPNAV_SPEED 1000 (used only until DO_CHANGE_SPEED)
CORNER_EXTRA_S = 3.0         # slow-down + yaw at each waypoint corner (conservative guess)
ARM_TAKEOFF_EXTRA_S = 10.0   # arming, spool-up, takeoff settling

# ---- Camera (IMX219, same values as the converter) ----
FOCAL_LENGTH_MM = 3.04
PIXEL_SIZE_UM = 1.12
IMAGE_W_PX = 3280
IMAGE_H_PX = 2464

# MAVLink command ids
NAV_WAYPOINT = 16
NAV_RTL = 20
NAV_LAND = 21
NAV_TAKEOFF = 22
DO_CHANGE_SPEED = 178
DO_DIGICAM_CONTROL = 203
DO_SET_CAM_TRIGG_DIST = 206

EARTH_R = 6371008.8


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(min(1.0, math.sqrt(a)))


STRAIGHT_DEG = 20.0          # heading change below this = flies straight through the waypoint


def bearing_deg(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(x, y)) % 360.0


def read_mission(path):
    items = []
    with open(path, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()
    if not lines or not lines[0].startswith("QGC WPL"):
        raise ValueError("Not a QGC WPL mission file")
    for line in lines[1:]:
        p = line.split()
        if len(p) < 12:
            continue
        items.append({
            "cmd": int(p[3]),
            "p1": float(p[4]), "p2": float(p[5]), "p3": float(p[6]),
            "lat": float(p[8]), "lon": float(p[9]), "alt": float(p[10]),
        })
    if not items:
        raise ValueError("Mission file is empty")
    return items


def _leg_time_s(dist_m, speed_mps):
    """Time for a straight leg starting and ending at rest (trapezoid speed profile)."""
    if dist_m <= 0:
        return 0.0
    accel_dist = speed_mps ** 2 / ACCEL_MPS2      # accelerate + decelerate
    if dist_m >= accel_dist:
        return dist_m / speed_mps + speed_mps / ACCEL_MPS2
    return 2 * math.sqrt(dist_m / ACCEL_MPS2)


def _convex_hull_area_m2(points):
    """Area (m²) of the convex hull of (lat, lon) points, using a local flat projection."""
    if len(points) < 3:
        return 0.0
    lat0 = sum(p[0] for p in points) / len(points)
    lon0 = sum(p[1] for p in points) / len(points)
    k = math.cos(math.radians(lat0))
    xy = sorted({(math.radians(lon - lon0) * EARTH_R * k, math.radians(lat - lat0) * EARTH_R) for lat, lon in points})
    if len(xy) < 3:
        return 0.0

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in xy:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(xy):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    area = 0.0
    for i in range(len(hull)):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % len(hull)]
        area += x1 * y2 - x2 * y1
    return abs(area) / 2


def summarize(path):
    """
    Walk through the mission the way ArduCopter flies it and add up distances,
    photos and time. 'Home' is taken as the mission's item 0 (the first survey
    waypoint): the real home is wherever you arm, so if you take off far from
    the survey, the real numbers will be larger.
    """
    items = read_mission(path)
    home = items[0]
    lat, lon, alt = home["lat"], home["lon"], 0.0

    speed = DEFAULT_SPEED_MPS
    survey_speed = None
    trig_dist = 0.0          # >0 = camera distance trigger ON
    seg_len = 0.0            # length flown in the current ON segment
    photos = 0
    trig_spacings = set()

    horiz = 0.0
    climb = 0.0
    time_s = ARM_TAKEOFF_EXTRA_S
    nav_alts = []
    nav_points = []
    corners = 0
    prev_brg = None
    takeoff_alt = None
    end_action = "none"
    trigger_mode = "none"

    def close_segment():
        nonlocal seg_len, photos
        if trig_dist > 0:
            photos += int(seg_len // trig_dist) + 1
        seg_len = 0.0

    for it in items[1:]:
        c = it["cmd"]
        if c == NAV_TAKEOFF:
            takeoff_alt = it["alt"]
            climb += max(0.0, it["alt"] - alt)
            time_s += max(0.0, it["alt"] - alt) / CLIMB_SPEED_MPS
            alt = it["alt"]
        elif c == DO_CHANGE_SPEED:
            if it["p2"] > 0:
                speed = it["p2"]
                survey_speed = it["p2"]
        elif c == DO_SET_CAM_TRIGG_DIST:
            if trig_dist > 0:
                close_segment()
            trig_dist = it["p1"]
            if trig_dist > 0:
                trig_spacings.add(round(trig_dist, 2))
                trigger_mode = "distance"
            seg_len = 0.0
        elif c == DO_DIGICAM_CONTROL:
            photos += 1
            trigger_mode = "waypoint"
        elif c == NAV_WAYPOINT:
            d = haversine_m(lat, lon, it["lat"], it["lon"])
            dz = it["alt"] - alt
            horiz += d
            if dz > 0:
                climb += dz
                time_s += dz / CLIMB_SPEED_MPS
            elif dz < 0:
                time_s += -dz / DESCENT_SPEED_MPS
            if d > 0.5:
                brg = bearing_deg(lat, lon, it["lat"], it["lon"])
                turn = 180.0 if prev_brg is None else abs((brg - prev_brg + 180.0) % 360.0 - 180.0)
                if turn < STRAIGHT_DEG:
                    # same line as the previous leg: the copter passes through without stopping
                    time_s += d / speed
                else:
                    time_s += _leg_time_s(d, speed)
                    corners += 1
                prev_brg = brg
            if trig_dist > 0:
                seg_len += d
            lat, lon, alt = it["lat"], it["lon"], it["alt"]
            nav_alts.append(alt)
            nav_points.append((lat, lon))
        elif c in (NAV_RTL, NAV_LAND):
            if trig_dist > 0:
                close_segment()
                trig_dist = 0.0
            if c == NAV_RTL:
                end_action = "RTL"
                if alt < RTL_ALT_M:
                    climb += RTL_ALT_M - alt
                    time_s += (RTL_ALT_M - alt) / CLIMB_SPEED_MPS
                    alt = RTL_ALT_M
                d = haversine_m(lat, lon, home["lat"], home["lon"])
                horiz += d
                # conservative: return assumed at survey speed (RTL may fly faster)
                time_s += _leg_time_s(d, speed) + RTL_LOITER_S
                lat, lon = home["lat"], home["lon"]
            else:
                end_action = "LAND"
            fast = max(0.0, alt - LAND_ALT_LOW_M)
            time_s += fast / DESCENT_SPEED_MPS + min(alt, LAND_ALT_LOW_M) / LAND_SPEED_MPS
            alt = 0.0

    if trig_dist > 0:
        close_segment()

    time_s += corners * CORNER_EXTRA_S

    survey_alt = min(nav_alts) if nav_alts else 0.0
    gsd_m = survey_alt * (PIXEL_SIZE_UM * 1e-6) / (FOCAL_LENGTH_MM * 1e-3) if survey_alt > 0 else 0.0

    return {
        "path_length_m": round(horiz, 1),
        "est_flight_time_s": round(time_s),
        "est_photos": photos,
        "trigger_mode": trigger_mode,
        "trigger_spacing_m": sorted(trig_spacings),
        "survey_speed_mps": survey_speed if survey_speed is not None else speed,
        "takeoff_alt_m": takeoff_alt,
        "alt_min_m": min(nav_alts) if nav_alts else None,
        "alt_max_m": max(nav_alts) if nav_alts else None,
        "waypoints": len(nav_points),
        "mission_items": len(items),
        "gsd_cm_px": round(gsd_m * 100, 2),
        "footprint_w_m": round(gsd_m * IMAGE_W_PX, 1),
        "footprint_h_m": round(gsd_m * IMAGE_H_PX, 1),
        "area_ha": round(_convex_hull_area_m2(nav_points) / 10000, 2),
        "end_action": end_action,
    }


if __name__ == "__main__":
    print(json.dumps(summarize(sys.argv[1]), indent=2))
