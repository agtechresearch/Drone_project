#!/usr/bin/env python3
"""QVIX vertical-farm map figures for the wall-keep report (2026-10-29). Run with the cad venv:
  ~/starling_sim/cad/venv/bin/python wk_map.py <out_dir>
  map_top.png  : plan view (north up) of room R1 with the whole module as an inset: walls, 4 rack lines, aisles A-E,
                 rows 1-10, the 8 rack faces named <aisle><side> (e.g. CW = aisle C's west face), takeoff cell row 9,
                 the survey route (C aisle, facing W), front-camera field of view and the chase-camera position
  map_side.png : elevation of one rack face as the front camera sees it (heading W from aisle C: south on the left,
                 north on the right): 7 trays, 6 open gaps, gap 1 (0.68 m) and gap 2 (1.265 m) = the two survey legs,
                 takeoff / climb / landing
Geometry from qvix_grid.py (v2 copy: rack faces ray-cast from the mesh).
"""
import math, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon, FancyArrowPatch, Circle

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import qvix_grid as G  # noqa: E402

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False
RACK = "#b9d6b9"; RACK_E = "#5f8f5f"; WALLC = "#777777"; FACE = "#d9480f"; ROUTE = "#1f5fbf"; CAM = "#e8a400"
out = sys.argv[1] if len(sys.argv) > 1 else "."
os.makedirs(out, exist_ok=True)
y0, y1 = G.ROOMS["R1"]["y"]; ry0, ry1 = G.ROOMS["R1"]["racks_y"]
FACES = [("AE", 0, "W"), ("BW", 0, "E"), ("BE", 1, "W"), ("CW", 1, "E"), ("CE", 2, "W"), ("DW", 2, "E"), ("DE", 3, "W"), ("EW", 3, "E")]


def top():
    fig = plt.figure(figsize=(14, 9.2), dpi=110)
    ax = fig.add_axes([0.20, 0.05, 0.79, 0.88])
    ins = fig.add_axes([0.02, 0.10, 0.15, 0.78])
    # ---- whole module inset
    ins.add_patch(Rectangle((0, 0), G.X_MAX, G.Y_MAX, fc="#f4f4f4", ec="#444", lw=1.2))
    for room, yy in (("R1", G.ROOMS["R1"]), ("R2", G.ROOMS["R2"])):
        for a, b in G.RACK_UNITS_Y[room]:
            for rx0, rx1 in G.RACKS_X:
                ins.add_patch(Rectangle((rx0, a), rx1 - rx0, b - a, fc=RACK, ec=RACK_E, lw=0.6))
        ins.text(G.X_MAX / 2, 0.5 * sum(yy["y"]), room, ha="center", va="center", fontsize=13, fontweight="bold", color="#333",
                 bbox=dict(fc="white", ec="none", alpha=0.8))
    for py in G.PARTITIONS_Y:
        ins.plot([0, G.PARTITION_DOOR_X[0]], [py, py], color="#444", lw=1.5)
        ins.plot([G.PARTITION_DOOR_X[1], G.X_MAX], [py, py], color="#444", lw=1.5)
    ins.text(G.X_MAX / 2, 0.5 * sum(G.VESTIBULE_Y), "전실", ha="center", va="center", fontsize=9, color="#555")
    ins.add_patch(Rectangle((0, y0 - 0.1), G.X_MAX, y1 - y0 + 0.2, fc="none", ec=FACE, lw=2.5))
    ins.text(G.X_MAX / 2, -1.1, "QVIX 모듈 전체\n(빨간 테두리 = 오른쪽 R1)", ha="center", va="top", fontsize=9)
    ins.set_xlim(-0.3, G.X_MAX + 0.3); ins.set_ylim(-2.6, G.Y_MAX + 0.6); ins.set_aspect("equal"); ins.axis("off")
    ins.annotate("", xy=(G.X_MAX + 0.2, G.Y_MAX - 0.3), xytext=(G.X_MAX + 0.2, G.Y_MAX - 2.3), arrowprops=dict(arrowstyle="-|>", lw=1.5))
    ins.text(G.X_MAX + 0.2, G.Y_MAX + 0.1, "N", ha="center", fontsize=10, fontweight="bold")
    # ---- R1 plan
    ax.add_patch(Rectangle((0, y0 - G.WALL), G.X_MAX, y1 - y0 + 2 * G.WALL, fc="#fafafa", ec=WALLC, lw=3))
    for r in range(1, 11):
        ax.axhline(y0 + r - 1, color="#e3e3e3", lw=0.8, zorder=0)
        ax.text(-0.12, y0 + r - 0.5, "row %d" % r, ha="right", va="center", fontsize=9.5, color="#555")
    for ua, ub in G.RACK_UNITS_Y["R1"]:
        for rx0, rx1 in G.RACKS_X:
            ax.add_patch(Rectangle((rx0, ua), rx1 - rx0, ub - ua, fc=RACK, ec=RACK_E, lw=1))
    for k, (rx0, rx1) in enumerate(G.RACKS_X):
        for ua, ub in G.RACK_UNITS_Y["R1"]:
            ax.text(0.5 * (rx0 + rx1), 0.5 * (ua + ub), "재배단 %d" % (k + 1), rotation=90, ha="center", va="center", fontsize=9.5, color="#3c6b3c")
    ga, gb = G.RACK_UNITS_Y["R1"][0][1], G.RACK_UNITS_Y["R1"][1][0]
    for col, (a, b) in G.AISLES.items():
        ax.text(0.5 * (a + b), y1 + 0.35, "%s통로" % col, ha="center", va="bottom", fontsize=12, fontweight="bold")
        ax.plot(0.5 * (a + b), y0 + 8.5, "o", ms=11, mfc="none", mec="black", mew=1.8, zorder=5)    # takeoff cell row 9
    for name, k, side in FACES:                  # face = the rack side seen from the named aisle
        rx0, rx1 = G.RACKS_X[k]
        x = rx0 if side == "W" else rx1
        for ua, ub in G.RACK_UNITS_Y["R1"]:
            ax.plot([x, x], [ua, ub], color=FACE, lw=3, solid_capstyle="butt", zorder=4)
        if side == "W":                          # west side of a rack line -> label under the rack, east side -> above it
            ax.text(x, ry0 - 0.12, name, ha="center", va="top", fontsize=10.5, fontweight="bold", color=FACE)
        else:
            ax.text(x, ry1 + 0.1, name, ha="center", va="bottom", fontsize=10.5, fontweight="bold", color=FACE)
    # example: survey of CW from aisle C
    cx = 0.5 * sum(G.AISLES["C"])
    ax.add_patch(FancyArrowPatch((cx - 0.09, y0 + 8.5), (cx - 0.09, y0 + 0.5), arrowstyle="-|>", mutation_scale=18, lw=2.2, color=ROUTE, zorder=6))
    ax.add_patch(FancyArrowPatch((cx + 0.09, y0 + 0.5), (cx + 0.09, y0 + 8.5), arrowstyle="-|>", mutation_scale=18, lw=2.2, color=ROUTE, ls="--", zorder=6))
    # front camera FOV (68 deg) facing W at row 5 and ToF FOV (106.5 deg)
    vy = y0 + 4.5
    for fov, col, lab in ((106.5, "#f2c14e", "ToF 106.5°"), (68.0, CAM, "정면 카메라 68°")):
        L = cx - G.AISLES["C"][0]
        h = L * math.tan(math.radians(fov / 2))
        ax.add_patch(Polygon([(cx, vy), (G.AISLES["C"][0], vy - h), (G.AISLES["C"][0], vy + h)], closed=True, fc=col, ec=col, alpha=0.35, zorder=3))
    ax.add_patch(Circle((cx, vy), 0.09, fc="#222", zorder=7))
    CX = G.X_MAX + 0.9
    box = dict(fc="white", ec="#bbb")
    ax.plot(cx, vy + 1.6, marker="s", ms=9, color="#7a3fbf", zorder=7)
    ax.annotate("○ 이륙 위치 = 각 통로 row 9 (바닥)", (0.5 * sum(G.AISLES["E"]) + 0.12, y0 + 8.5), xytext=(CX, y0 + 9.3), fontsize=10,
                arrowprops=dict(arrowstyle="->"), ha="left", va="center", bbox=box)
    ax.annotate("전지적 시점 카메라: 통로 중앙에서\n기체 1.6 m 북쪽, 0.35 m 위에서 따라감", (cx + 0.08, vy + 1.6), xytext=(CX, y0 + 7.4), fontsize=10,
                arrowprops=dict(arrowstyle="->", color="#7a3fbf"), color="#5a2f8f", ha="left", va="center", bbox=dict(fc="white", ec="#c8b3e6"))
    ax.annotate("예: C통로에서 CW 면 촬영\n① 0.68 m 높이로 북→남 (실선)\n② 1.265 m 높이로 남→북 (점선)", (cx + 0.12, y0 + 2.4), xytext=(CX, y0 + 2.5),
                fontsize=10, arrowprops=dict(arrowstyle="->", color=ROUTE), color=ROUTE, ha="left", va="center", bbox=dict(fc="white", ec="#b7c8ea"))
    ax.annotate("기체 헤딩 W: 정면 카메라(68°, 진한 노랑)와\nToF(106.5°, 연한 노랑)가 CW 면을 정면으로 봄", (cx + 0.05, vy), xytext=(CX, y0 + 5.7), fontsize=10,
                arrowprops=dict(arrowstyle="->", color="#333"), ha="left", va="center", bbox=box)
    ax.annotate("재배단 한 줄 = 4 m 유닛 2개\n사이 0.3 m는 비어 있음 (row 5 부근)", (G.RACKS_X[3][1] + 0.02, 0.5 * (ga + gb)), xytext=(CX, y0 + 4.1),
                fontsize=10, arrowprops=dict(arrowstyle="->", color="#3c6b3c"), color="#3c6b3c", ha="left", va="center", bbox=dict(fc="white", ec="#a9c9a9"))
    ax.text(CX, y0 + 1.0, "면 이름 = <통로><방향>\n예) CW = C통로에서 본 서쪽 재배단 면\n     CE = C통로에서 본 동쪽 재배단 면\nA통로 서쪽·E통로 동쪽은 벽이라 면 없음",
            fontsize=10, ha="left", va="center", color=FACE, bbox=dict(fc="white", ec="#f0b49a"))
    ax.text(G.X_MAX / 2, y0 - 0.75, "회색 띠(row 1 남쪽, row 9~10 북쪽) = 재배단이 없는 통로 끝  ·  통로 폭 0.65~0.75 m, 재배단 폭 0.51 m",
            ha="center", va="top", fontsize=9, color="#555")
    ax.axhspan(y0, ry0, color="#ececec", zorder=0); ax.axhspan(ry1, y1, color="#ececec", zorder=0)
    ax.annotate("", xy=(G.X_MAX + 0.45, y1 + 0.4), xytext=(G.X_MAX + 0.45, y1 - 0.8), arrowprops=dict(arrowstyle="-|>", lw=2))
    ax.text(G.X_MAX + 0.45, y1 + 0.5, "N", ha="center", va="bottom", fontsize=12, fontweight="bold")
    ax.set_xlim(-0.9, G.X_MAX + 5.0); ax.set_ylim(y0 - 1.2, y1 + 1.1); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("R1 평면도 (위에서 본 모습, 북쪽이 위)", fontsize=14, pad=16)
    fig.savefig(os.path.join(out, "map_top.png")); plt.close(fig)


def side():
    fig, ax = plt.subplots(figsize=(13, 6.2), dpi=110)
    fig.subplots_adjust(left=0.07, right=0.98, top=0.88, bottom=0.12)
    ax.axhspan(-0.05, 0, color="#bbb")
    units = G.RACK_UNITS_Y["R1"]
    for i, (za, zb) in enumerate(G.TRAY_Z):
        for ua, ub in units:
            ax.add_patch(Rectangle((ua, za), ub - ua, zb - za, fc=RACK, ec=RACK_E, lw=1))
        ax.text(ry1 + 0.12, 0.5 * (za + zb), "트레이 %d" % (i + 1), va="center", fontsize=8.5, color="#3c6b3c")
    for ua, ub in units:                         # 50 mm posts at every unit end
        for x in (ua + 0.025, ub - 0.025):
            ax.add_patch(Rectangle((x - 0.025, 0), 0.05, G.TRAY_Z[-1][1], fc="#6d8f6d", ec="none"))
    for k, (ua, ub) in enumerate(units):
        ax.text(0.5 * (ua + ub), G.TRAY_Z[-1][1] + 0.08, "재배단 유닛 %d (4.0 m)" % (k + 1), ha="center", va="bottom", fontsize=10, color="#3c6b3c")
    ga, gb = units[0][1], units[1][0]
    ax.annotate("유닛 사이 빈 공간 0.3 m", (0.5 * (ga + gb), 2.95), xytext=(0.5 * (ga + gb), 4.35), ha="center", fontsize=9.5, color="#555",
                arrowprops=dict(arrowstyle="->", color="#777"))
    for i, ((a, b), c) in enumerate(zip(G.GAP_Z, G.GAP_CENTRE_Z)):
        ax.text(ry0 - 0.12, c, "틈 %d  %s m" % (i + 1, ("%.3f" % c).rstrip("0").rstrip(".")), ha="right", va="center", fontsize=9,
                color=ROUTE if i < 2 else "#777", fontweight="bold" if i < 2 else "normal")
    # flight: takeoff at row 9 (floor) -> 0.68 m, leg 1 to row 1, climb to 1.265 m, leg 2 back to row 9, land
    r9, r1 = y0 + 8.5, y0 + 0.5
    h1, h2 = G.GAP_CENTRE_Z[0], G.GAP_CENTRE_Z[1]
    ax.plot([r9, r9], [0.03, h1], color="#555", lw=2, ls=":")
    ax.add_patch(FancyArrowPatch((r9, h1), (r1, h1), arrowstyle="-|>", mutation_scale=20, lw=2.6, color=ROUTE))
    ax.add_patch(FancyArrowPatch((r1, h1), (r1, h2), arrowstyle="-|>", mutation_scale=16, lw=2, color="#555"))
    ax.add_patch(FancyArrowPatch((r1, h2), (r9, h2), arrowstyle="-|>", mutation_scale=20, lw=2.6, color=ROUTE, ls="--"))
    ax.plot([r9, r9], [h2, 0.03], color="#555", lw=2, ls=":")
    ax.plot(r9, 0.03, "o", ms=13, mfc="none", mec="black", mew=2, zorder=6, clip_on=False)
    ax.text(r9 + 0.15, 0.12, "○ 이륙·착륙 (row 9 바닥)", fontsize=9.5, va="bottom")
    ax.text(0.5 * (r1 + r9), h1 - 0.08, "① 틈 1 높이 0.68 m, 북→남 (8 m)", color=ROUTE, ha="center", va="top", fontsize=10, fontweight="bold")
    ax.text(0.5 * (r1 + r9), h2 + 0.08, "② 틈 2 높이 1.265 m, 남→북", color=ROUTE, ha="center", va="bottom", fontsize=10, fontweight="bold")
    ax.text(r1 - 0.12, 0.5 * (h1 + h2), "상승", ha="right", va="center", fontsize=9, color="#555")
    ax.set_xticks([y0 + r - 0.5 for r in range(1, 11)]); ax.set_xticklabels(["row %d" % r for r in range(1, 11)])
    ax.set_xlim(y0 - 1.2, y1 + 0.9); ax.set_ylim(-0.05, 4.55)
    ax.set_ylabel("바닥에서 높이 (m)"); ax.set_xlabel("← 남쪽            통로 방향 (row)            북쪽 →")
    ax.grid(alpha=0.25)
    ax.set_title("재배단 정면도 — C통로에서 서쪽(CW 면)을 바라본 모습 = 정면 카메라가 보는 방향", fontsize=13)
    fig.savefig(os.path.join(out, "map_side.png")); plt.close(fig)


top(); side()
print("maps ->", out)
