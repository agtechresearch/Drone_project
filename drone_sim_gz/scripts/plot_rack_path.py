#!/usr/bin/env python3
"""Rack-face elevation view of a grid flight: what the vehicle's front camera faces, with the whole rack and the whole path.
Draws one rack face (trays, gaps, end posts, rows) as seen from the aisle and overlays the PLANNED path (dashed, numbered
legs with arrows) and the FLOWN ground-truth path (solid, colour = time: blue at the start -> orange at the end).

  python3 plot_rack_path.py <plan.json> <truth.txt> <out.png> [--aisle A] [--face E] [--title "..."]

Horizontal axis = along the aisle as the camera sees it (facing E: north on the LEFT, row 10 .. row 1; facing W: mirrored),
vertical axis = height of the vehicle body above the floor (m). Only truth samples inside the chosen aisle are drawn.
Uses PIL only (the host matplotlib is broken against numpy 2). Geometry from qvix_grid.py (TRAY_Z, racks_y, rows).
"""
import argparse, json, os, re, sys
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import qvix_grid as G  # noqa: E402

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_B = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
S = 2                                   # supersampling


def font(sz, bold=False):
    try:
        return ImageFont.truetype(FONT_B if bold else FONT, sz * S, index=1)      # index 1 = KR
    except OSError:
        return ImageFont.load_default()


def load_truth(path):
    out = []
    for line in open(path):
        m = re.match(r"(\d+):(\d+):([\d.]+)\s+\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]", line)
        if m:
            t = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))
            yaw = re.search(r"\]\s+\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]", line)
            out.append((t, float(m.group(4)), float(m.group(5)), float(m.group(6)), 0, 0, float(yaw.group(3)) if yaw else 0.0))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("plan"); ap.add_argument("truth"); ap.add_argument("out")
    ap.add_argument("--aisle", default=None); ap.add_argument("--face", default=None); ap.add_argument("--title", default="")
    ap.add_argument("--room", default="R1")
    ap.add_argument("--only-face", action="store_true",
                    help="draw only the stages flown facing --face (two-face survey flights: one figure per face); truth samples by yaw")
    a = ap.parse_args()
    plan = json.load(open(a.plan))
    stages = plan["stages"]
    aisle = a.aisle or plan["spawn"]["cell"][2]
    face = a.face or next((s["heading"] for s in stages if s.get("heading") in ("E", "W") and s.get("cell", "")[2:3] == aisle), "E")
    room = a.room
    W, H = 1400, 760
    L, R, T, B = 90, 60, 96, 120                                   # plot margins (px, before supersampling)
    img = Image.new("RGB", (W * S, H * S), (250, 251, 249)); d = ImageDraw.Draw(img)
    y0, y1 = G.ROOMS[room]["y"]; ry0, ry1 = G.ROOMS[room]["racks_y"]
    zmax = 4.4
    mx = G.MODEL_TO_WORLD

    def X(model_y):                                                # facing E: north (large y) on the left
        f = (y1 - model_y) / (y1 - y0) if face == "E" else (model_y - y0) / (y1 - y0)
        return (L + f * (W - L - R)) * S

    def Y(h):
        return (T + (1 - h / zmax) * (H - T - B)) * S

    # floor and walls
    d.rectangle([X(y1) if face == "E" else X(y0), Y(0), X(y0) if face == "E" else X(y1), Y(-0.08)], fill=(150, 150, 145))
    # rack: tray bands, gaps, end posts
    for zl, zh in G.TRAY_Z:
        d.rectangle([min(X(ry0), X(ry1)), Y(zh), max(X(ry0), X(ry1)), Y(zl)], fill=(70, 118, 82))
    for i, (zl, zh) in enumerate(G.GAP_Z):
        d.rectangle([min(X(ry0), X(ry1)), Y(zh), max(X(ry0), X(ry1)), Y(zl)], fill=(222, 232, 222))
        zc = G.GAP_CENTRE_Z[i]
        for xx in range(int(min(X(ry0), X(ry1))), int(max(X(ry0), X(ry1))), 18 * S):
            d.line([xx, Y(zc), xx + 8 * S, Y(zc)], fill=(150, 170, 150), width=S)
        d.text((max(X(ry0), X(ry1)) + 8 * S, Y(zc) - 9 * S), "틈 %d  %.2f m" % (i + 1, zc), font=font(13), fill=(60, 90, 65))
    for py in (ry0, ry1 - 0.05):
        d.rectangle([min(X(py), X(py + 0.05)), Y(G.TRAY_Z[-1][1]), max(X(py), X(py + 0.05)), Y(0)], fill=(45, 80, 55))
    # height axis
    for h in [0, 1, 2, 3, 4]:
        d.line([L * S - 6 * S, Y(h), L * S, Y(h)], fill=(90, 90, 90), width=S)
        d.text((L * S - 52 * S, Y(h) - 9 * S), "%.0f m" % h, font=font(13), fill=(70, 70, 70))
    # rows
    for r, (ya, yb) in G.rows(room):
        xc = X(0.5 * (ya + yb))
        d.text((xc - 12 * S, Y(-0.08) + 10 * S), "%d행" % r, font=font(13), fill=(70, 70, 70))
        d.line([X(ya), Y(-0.08), X(ya), Y(-0.08) + 6 * S], fill=(120, 120, 120), width=S)
    left, right = ("북 (N)", "남 (S)") if face == "E" else ("남 (S)", "북 (N)")
    d.text((L * S, Y(-0.08) + 40 * S), "← " + left, font=font(14, True), fill=(50, 50, 50))
    d.text(((W - R) * S - 80 * S, Y(-0.08) + 40 * S), right + " →", font=font(14, True), fill=(50, 50, 50))

    # planned path (cell rows -> y, alt -> height above floor)
    def cell_y(name):
        return G.cell(name)["model_xy"][1]
    pts = []
    cur_alt = plan["alt"]; cur_cell = plan["spawn"]["cell"]
    if not a.only_face:
        pts.append((cell_y(cur_cell), 0.03)); pts.append((cell_y(cur_cell), cur_alt + 0.03))
    for s in stages:
        if s["kind"] == "goto" or s["kind"] == "alt":
            prev = (cur_cell, cur_alt)
            cur_cell = s["cell"]; cur_alt = s.get("alt", cur_alt)
            if cur_cell[2] != aisle or (a.only_face and s.get("heading") != face):
                continue
            if not pts and prev[0][2] == aisle:                  # face starts here: include the leg's start point
                pts.append((cell_y(prev[0]), prev[1] + 0.03))
            pts.append((cell_y(cur_cell), cur_alt + 0.03))
    if not a.only_face:
        pts.append((pts[-1][0], 0.03))
    for i in range(len(pts) - 1):
        (ya, za), (yb, zb) = pts[i], pts[i + 1]
        xa, xb, ya_, yb_ = X(ya), X(yb), Y(za), Y(zb)
        n = max(1, int(((xb - xa) ** 2 + (yb_ - ya_) ** 2) ** 0.5 / (14 * S)))
        for k in range(0, n, 2):
            f0, f1 = k / n, min(1, (k + 1) / n)
            d.line([xa + (xb - xa) * f0, ya_ + (yb_ - ya_) * f0, xa + (xb - xa) * f1, ya_ + (yb_ - ya_) * f1], fill=(40, 40, 40), width=2 * S)
    # flown path (truth), coloured by time
    T_ = [p for p in load_truth(a.truth)]
    xr = (G.AISLES[aisle][0] + mx[0], G.AISLES[aisle][1] + mx[0])
    T_ = [p for p in T_ if xr[0] - 0.05 <= p[1] <= xr[1] + 0.05]
    if a.only_face:                                               # gz yaw: E = 0, W = pi
        import math
        want = 0.0 if face == "E" else math.pi
        T_ = [p for p in T_ if abs(math.atan2(math.sin(p[6] - want), math.cos(p[6] - want))) < 0.35]
    if T_:
        t0, t1 = T_[0][0], T_[-1][0]

        def col(t):
            f = (t - t0) / max(1e-6, t1 - t0)
            c0, c1 = (35, 110, 220), (235, 110, 20)
            return tuple(int(c0[i] + (c1[i] - c0[i]) * f) for i in range(3))
        for p, q in zip(T_, T_[1:]):
            d.line([X(p[2] - mx[1]), Y(p[3] - mx[2]), X(q[2] - mx[1]), Y(q[3] - mx[2])], fill=col(p[0]), width=4 * S)
    # leg numbers + arrows on the plan
    import math
    leg = 0
    for i in (range(0, len(pts) - 1) if a.only_face else range(1, len(pts) - 2)):
        (ya, za), (yb, zb) = pts[i], pts[i + 1]
        if abs(ya - yb) < 1e-6 and abs(za - zb) < 1e-6:
            continue
        leg += 1
        xa, xb, ya_, yb_ = X(ya), X(yb), Y(za), Y(zb)
        xm, ym = (xa + xb) / 2, (ya_ + yb_) / 2
        ang = math.atan2(yb_ - ya_, xb - xa)
        ah = 12 * S
        tip = (xm + math.cos(ang) * ah, ym + math.sin(ang) * ah)
        d.polygon([tip, (xm + math.cos(ang + 2.6) * ah, ym + math.sin(ang + 2.6) * ah),
                   (xm + math.cos(ang - 2.6) * ah, ym + math.sin(ang - 2.6) * ah)], fill=(20, 20, 20))
        off = (-22 * S, -14 * S) if abs(ya - yb) > 1e-6 else (12 * S, -10 * S)
        r = 11 * S; cx, cy = xm + off[0], ym + off[1]
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(255, 255, 255), outline=(20, 20, 20), width=S)
        f = font(12, True); tw = d.textlength(str(leg), font=f)
        d.text((cx - tw / 2, cy - 9 * S), str(leg), font=f, fill=(20, 20, 20))
    # title + legend
    d.text((L * S, 22 * S), a.title or "%s%s 통로, %s쪽 재배단 면" % (room, aisle, "동" if face == "E" else "서"), font=font(20, True), fill=(25, 30, 28))
    lx, ly = L * S, 60 * S
    for k in range(0, 34 * S, 4 * S):
        if (k // (4 * S)) % 2 == 0:
            d.line([lx + k, ly + 8 * S, lx + k + 4 * S, ly + 8 * S], fill=(40, 40, 40), width=2 * S)
    d.text((lx + 42 * S, ly - 2 * S), "계획 경로 (번호 = 비행 순서)", font=font(13), fill=(40, 40, 40))
    lx2 = lx + 270 * S
    for k in range(40):
        f = k / 39
        c = tuple(int((35, 110, 220)[i] + ((235, 110, 20)[i] - (35, 110, 220)[i]) * f) for i in range(3))
        d.line([lx2 + k * S, ly + 8 * S, lx2 + (k + 1) * S, ly + 8 * S], fill=c, width=4 * S)
    d.text((lx2 + 48 * S, ly - 2 * S), "실제 비행 (gz 진실값, 파랑=시작 → 주황=끝)", font=font(13), fill=(40, 40, 40))
    lx3 = lx2 + 390 * S
    d.rectangle([lx3, ly + 1 * S, lx3 + 26 * S, ly + 15 * S], fill=(70, 118, 82))
    d.text((lx3 + 32 * S, ly - 2 * S), "트레이", font=font(13), fill=(40, 40, 40))
    d.rectangle([lx3 + 90 * S, ly + 1 * S, lx3 + 116 * S, ly + 15 * S], fill=(222, 232, 222), outline=(150, 170, 150))
    d.text((lx3 + 122 * S, ly - 2 * S), "틈 (촬영 높이)", font=font(13), fill=(40, 40, 40))
    img.resize((W, H), Image.LANCZOS).save(a.out)
    print("wrote %s (aisle %s face %s, %d planned points, %d truth samples)" % (a.out, aisle, face, len(pts), len(T_)))


if __name__ == "__main__":
    main()
