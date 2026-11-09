#!/usr/bin/env python3
"""QVIX R-module flight grid (room-local, 2-D) for human <-> planner communication.

Geometry was measured 2026-10-19 by slicing repo_export/gz/models/qvix_r_module/meshes/qvix_r_full.stl
(see worklog/2026-10-19.md).  The module is ONE mesh: racks have no names in gz, so everything here is
derived from those slices.  Frames:
  model frame : mesh origin (x 0..5.8, y 0..22.4, z 0..5.0, metres)
  world frame : model + (0.5, -12.0, 0.15)   (qvix.sdf include pose)
  PX4 NED     : N = world +y, E = world +x, D = -world z  (EKF origin = world origin in SITL)

Naming:  <room><column><row>, e.g. R1C10
  room   : R1 = south room (model y 0.1..10.1), R2 = north room (12.3..22.3). Vestibule between.
  column : aisle A..E, west -> east (A along the west wall, C the central aisle in line with the doors)
  row    : 1.0 m bands, row 1 at the room's south end; row number increases northward (= compass N)
  compass: N/E/S/W are the room's own labels; N = +y model = world N; yaw deg N 0, E 90, S 180, W 270
           (same convention as path_flight_phase1_v13.py --yaw).

Python 3.6 compatible (runs in starling-flight).
"""
import argparse
import json
import math
import sys
from collections import OrderedDict

# ---- measured geometry (model frame, metres) -------------------------------------------------
MODEL_TO_WORLD = (0.5, -12.0, 0.15)
WALL = 0.1
X_MAX, Y_MAX, Z_TOP = 5.8, 22.4, 5.0
RACKS_X = [(0.75, 1.25), (2.0, 2.55), (3.3, 3.8), (4.55, 5.05)]          # 4 rack lines, 0.5 m wide
AISLES = OrderedDict([("A", (0.1, 0.75)), ("B", (1.25, 2.0)), ("C", (2.55, 3.3)),
                      ("D", (3.8, 4.55)), ("E", (5.05, 5.7))])
ROOMS = OrderedDict([
    ("R1", {"y": (0.1, 10.1), "racks_y": (0.6, 8.9)}),     # south room
    ("R2", {"y": (12.3, 22.3), "racks_y": (13.5, 21.8)}),  # north room
])
VESTIBULE_Y = (10.2, 12.2)
PARTITIONS_Y = (10.1, 12.2)          # 0.1 m thick walls at these y (south face)
PARTITION_DOOR_X = (2.45, 3.45)      # full-height opening, in line with aisle C
EXT_DOOR_Y = (10.75, 11.65)          # east wall, vestibule
SHELF_Z = [0.3, 0.9, 1.5, 2.1, 2.7, 3.3, 3.8]   # tray bottoms (coarse, 2026-10-19)
# Exact tray spans measured 2026-10-27 (10 mm slices of qvix_r_racks.stl): 7 trays, each 0.20 m thick; between them only
# 4 corner posts (50 x 50 mm) -> the gaps are empty and can be photographed from the aisle.
TRAY_Z = [(0.29, 0.49), (0.87, 1.07), (1.46, 1.66), (2.04, 2.24), (2.63, 2.83), (3.21, 3.41), (3.79, 3.99)]
GAP_Z = [(TRAY_Z[i][1], TRAY_Z[i + 1][0]) for i in range(len(TRAY_Z) - 1)]            # 6 gaps, 0.38-0.39 m tall
GAP_CENTRE_Z = [round(0.5 * (a + b), 3) for a, b in GAP_Z]                            # 0.68 1.265 1.85 2.435 3.02 3.6 (model z = height above floor)
FLOOR_TO_BODY_AT_REST = 0.03   # body origin sits 0.03 m above the floor when landed (legs); flight script altitude U = height above floor - 0.03
ROW_PITCH = 1.0

# ---- vehicle --------------------------------------------------------------------------------
# rotor hubs (+-0.066, +-0.088) + rotor collision radius 0.06 (repo_export/gz/README.md)
ROTOR_RADIUS_FWD = 0.066 + 0.06      # extent along body x
ROTOR_RADIUS_SIDE = 0.088 + 0.06     # extent along body y
SWEPT_RADIUS = math.hypot(0.066, 0.088) + 0.06   # worst case while yawing  (0.170)
CROSS_MIN_BAND = 0.8                 # open band (m) needed to fly across aisles

COMPASS_YAW = OrderedDict([("N", 0.0), ("E", 90.0), ("S", 180.0), ("W", 270.0)])
COMPASS_VEC = {"N": (0.0, 1.0), "E": (1.0, 0.0), "S": (0.0, -1.0), "W": (-1.0, 0.0)}  # (dx, dy) model


def model_to_world(x, y, z=0.0):
    return (x + MODEL_TO_WORLD[0], y + MODEL_TO_WORLD[1], z + MODEL_TO_WORLD[2])


def world_to_model(x, y, z=0.0):
    return (x - MODEL_TO_WORLD[0], y - MODEL_TO_WORLD[1], z - MODEL_TO_WORLD[2])


def rows(room):
    y0, y1 = ROOMS[room]["y"]
    n = int(round((y1 - y0) / ROW_PITCH))
    return [(i + 1, (y0 + i * ROW_PITCH, y0 + (i + 1) * ROW_PITCH)) for i in range(n)]


def open_band(room, yr):
    """Width of rack-free band inside row yr (m) and its centre y, else (0, None)."""
    ry0, ry1 = ROOMS[room]["racks_y"]
    y0, y1 = yr
    if y1 <= ry0 + 1e-9:            # south of racks
        return (y1 - y0, 0.5 * (y0 + y1))
    if y0 >= ry1 - 1e-9:            # north of racks
        return (y1 - y0, 0.5 * (y0 + y1))
    if y0 < ry0 < y1:               # row straddles the south rack end: open part is below ry0
        return (ry0 - y0, 0.5 * (y0 + ry0))
    if y0 < ry1 < y1:
        return (y1 - ry1, 0.5 * (ry1 + y1))
    return (0.0, None)


def cells(room):
    out = []
    for col, (x0, x1) in AISLES.items():
        for r, (y0, y1) in rows(room):
            band, band_y = open_band(room, (y0, y1))
            cx = 0.5 * (x0 + x1)
            cy = 0.5 * (y0 + y1)
            wx, wy, _ = model_to_world(cx, cy)
            width = x1 - x0
            out.append(OrderedDict([
                ("name", "%s%s%d" % (room, col, r)),
                ("room", room), ("col", col), ("row", r),
                ("model_xy", (round(cx, 3), round(cy, 3))),
                ("world_xy", (round(wx, 3), round(wy, 3))),
                ("ned_ne", (round(wy, 3), round(wx, 3))),
                ("x_range", (x0, x1)), ("y_range", (round(y0, 3), round(y1, 3))),
                ("aisle_width", round(width, 3)),
                ("side_clearance", round(0.5 * width - ROTOR_RADIUS_SIDE, 3)),   # flying along the aisle
                ("yaw_clearance", round(0.5 * width - SWEPT_RADIUS, 3)),         # while turning in place
                ("open_band", round(band, 3)),
                ("cross_ok", band >= CROSS_MIN_BAND),
                ("cross_y_model", None if band_y is None or band < CROSS_MIN_BAND else round(band_y, 3)),
            ]))
    return out


def cell(name):
    for c in cells(name[:2]):
        if c["name"] == name:
            return c
    raise KeyError(name)


def locate(world_x, world_y):
    """Cell name containing a world (x, y), or None."""
    mx, my, _ = world_to_model(world_x, world_y)
    for room in ROOMS:
        for c in cells(room):
            x0, x1 = c["x_range"]
            y0, y1 = c["y_range"]
            if x0 <= mx <= x1 and y0 <= my <= y1:
                return c["name"]
    return None


# ---- outputs ---------------------------------------------------------------------------------
def table_md(room):
    lines = ["# QVIX grid %s (south room = R1, north = R2)" % room, "",
             "Frames: world = model + (0.5, -12, 0.15); NED N = world y, E = world x.",
             "Compass yaw: N 0, E 90, S 180, W 270 (= script --yaw). Row number increases toward N.",
             "Rotor side radius %.3f m, swept radius %.3f m. cross_ok = open band >= %.1f m (lateral moves between aisles)."
             % (ROTOR_RADIUS_SIDE, SWEPT_RADIUS, CROSS_MIN_BAND),
             "Shelf tray bottoms (model z): %s  -> world z +0.15" % SHELF_Z, "",
             "| cell | world x | world y | NED N | NED E | aisle w | side clr | yaw clr | open band | cross |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for c in cells(room):
        lines.append("| %s | %.3f | %.3f | %.3f | %.3f | %.2f | %.3f | %.3f | %.2f | %s |" % (
            c["name"], c["world_xy"][0], c["world_xy"][1], c["ned_ne"][0], c["ned_ne"][1],
            c["aisle_width"], c["side_clearance"], c["yaw_clearance"], c["open_band"],
            "yes (y=%.2f)" % c["cross_y_model"] if c["cross_ok"] else "no"))
    return "\n".join(lines) + "\n"


def ascii_map(room):
    """Compact top view: columns A..E, rows top = north."""
    rs = rows(room)
    out = ["      " + "   ".join("%-4s" % k for k in AISLES)]
    for r, yr in reversed(rs):
        band, _ = open_band(room, yr)
        mark = "=" if band >= CROSS_MIN_BAND else ("-" if band > 0 else " ")
        out.append("%3d  |" % r + "|".join(" %s%d%s " % (k, r, mark) for k in AISLES) + "|")
    out.append("      (= : row open across all aisles, crossing allowed; - : partly open, no crossing)")
    return "\n".join(out)


def marker_cmds(room, z_model=0.02, clear=False, text_z=0.25):
    """gz CLI commands (run inside starling-gz) that draw the grid in the Gazebo GUI via /marker."""
    cmds = []
    base = "gz service -s /marker --reqtype gz.msgs.Marker --reptype gz.msgs.Empty --timeout 2000 --req "
    if clear:
        return [base + "'action: DELETE_ALL, ns: \"grid\"'"]
    mid = 1
    for c in cells(room):
        x0, x1 = c["x_range"]
        y0, y1 = c["y_range"]
        wx, wy, wz = model_to_world(0.5 * (x0 + x1), 0.5 * (y0 + y1), z_model)
        if c["cross_ok"]:
            rgb = (0.2, 0.9, 0.3)
        else:
            rgb = (0.3, 0.6, 1.0)
        # per-cell box (slightly inset so edges show as gaps)
        cmds.append(base + "'action: ADD_MODIFY, ns: \"grid\", id: %d, type: BOX, visibility: GUI, "
                    "pose: {position: {x: %.3f, y: %.3f, z: %.3f}}, scale: {x: %.3f, y: %.3f, z: 0.01}, "
                    "material: {ambient: {r: %.2f, g: %.2f, b: %.2f, a: 0.35}, diffuse: {r: %.2f, g: %.2f, b: %.2f, a: 0.35}}'"
                    % (mid, wx, wy, wz, (x1 - x0) - 0.04, (y1 - y0) - 0.04, rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2]))
        mid += 1
        cmds.append(base + "'action: ADD_MODIFY, ns: \"grid\", id: %d, type: TEXT, visibility: GUI, text: \"%s%d\", "
                    "pose: {position: {x: %.3f, y: %.3f, z: %.3f}}, scale: {x: 0.18, y: 0.18, z: 0.18}, "
                    "material: {ambient: {r: 1, g: 1, b: 1, a: 1}, diffuse: {r: 1, g: 1, b: 1, a: 1}}'"
                    % (mid, c["col"], c["row"], wx - 0.12, wy - 0.08, wz + text_z))
        mid += 1
    return cmds


def draw_map(room, path_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    y0, y1 = ROOMS[room]["y"]
    ry0, ry1 = ROOMS[room]["racks_y"]
    fig, ax = plt.subplots(figsize=(6, 9))
    # walls
    ax.add_patch(Rectangle((0, y0 - WALL), X_MAX, WALL, color="#444"))
    ax.add_patch(Rectangle((0, y1), X_MAX, WALL, color="#444"))
    ax.add_patch(Rectangle((0, y0 - WALL), WALL, y1 - y0 + 2 * WALL, color="#444"))
    ax.add_patch(Rectangle((X_MAX - WALL, y0 - WALL), WALL, y1 - y0 + 2 * WALL, color="#444"))
    # door in the partition (R1 north wall / R2 south wall)
    dy = y1 if room == "R1" else y0 - WALL
    ax.add_patch(Rectangle((PARTITION_DOOR_X[0], dy), PARTITION_DOOR_X[1] - PARTITION_DOOR_X[0], WALL, color="white"))
    ax.text(0.5 * sum(PARTITION_DOOR_X), dy + 0.05, "door", ha="center", va="center", fontsize=7)
    # racks
    for (x0, x1) in RACKS_X:
        ax.add_patch(Rectangle((x0, ry0), x1 - x0, ry1 - ry0, color="#9a7b4f", alpha=0.8))
    # cells
    for c in cells(room):
        xa, xb = c["x_range"]
        ya, yb = c["y_range"]
        fc = "#bff5c6" if c["cross_ok"] else "#dbe9ff"
        ax.add_patch(Rectangle((xa, ya), xb - xa, yb - ya, facecolor=fc, edgecolor="#6b8fb3", lw=0.6, alpha=0.9))
        ax.text(0.5 * (xa + xb), 0.5 * (ya + yb), "%s%d" % (c["col"], c["row"]), ha="center", va="center", fontsize=7)
    ax.set_xlim(-0.2, X_MAX + 0.2)
    ax.set_ylim(y0 - 0.5, y1 + 0.6)
    ax.set_aspect("equal")
    ax.set_xlabel("model x (m)  [world x = +0.5]   E ->")
    ax.set_ylabel("model y (m)  [world y = -12]   N ^")
    ax.set_title("QVIX %s grid (top view, north up). green = crossing row" % room, fontsize=10)
    for k, (xa, xb) in AISLES.items():
        ax.text(0.5 * (xa + xb), y1 + 0.35, k, ha="center", fontsize=9, weight="bold")
    fig.tight_layout()
    fig.savefig(path_png, dpi=150)
    return path_png


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--room", default="R1", choices=list(ROOMS))
    p.add_argument("--table", action="store_true", help="print markdown table")
    p.add_argument("--json", action="store_true", help="print cells as JSON")
    p.add_argument("--ascii", action="store_true", help="print ASCII top view")
    p.add_argument("--map", metavar="PNG", help="draw top-view PNG")
    p.add_argument("--markers", action="store_true", help="print gz marker commands (run inside starling-gz)")
    p.add_argument("--clear-markers", action="store_true")
    p.add_argument("--cell", help="print one cell")
    p.add_argument("--locate", nargs=2, type=float, metavar=("WX", "WY"), help="world x y -> cell")
    a = p.parse_args(argv)
    if a.table:
        sys.stdout.write(table_md(a.room))
    if a.json:
        print(json.dumps(cells(a.room), indent=1))
    if a.ascii:
        print(ascii_map(a.room))
    if a.map:
        print(draw_map(a.room, a.map))
    if a.markers or a.clear_markers:
        print("\n".join(marker_cmds(a.room, clear=a.clear_markers)))
    if a.cell:
        print(json.dumps(cell(a.cell), indent=1))
    if a.locate:
        print(locate(*a.locate))


if __name__ == "__main__":
    main()
