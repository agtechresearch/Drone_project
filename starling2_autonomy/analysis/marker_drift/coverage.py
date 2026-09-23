"""마커 커버리지 점검: 비행 경로상 각 위치에서 화각 안에 온전히 들어오는 태그가 몇 개인지.

마커 크기·간격·이격 거리를 정할 때 쓴다(docs/09 §3.2, §4.2). 카메라 파일이 없으면 화각으로 가상 카메라를 만든다.

  python -m marker_drift coverage --fov 120x93.5 --size 1280x800 --standoff 0.5 --altitude 0.6
  python -m marker_drift coverage --intrinsics hires.yaml --layout my_layout.yaml --standoff 0.5 --altitude 2.0

판정: 코너 4개가 모두 이미지 안(여백 margin_px)이고 깊이 > 0 이며 가장 짧은 변이 min_edge_px 이상.
"""

import math

import numpy as np

from marker_drift import geometry
from marker_drift.intrinsics import Intrinsics
from marker_drift.markers import Layout


def intrinsics_from_fov(hfov_deg, vfov_deg, width, height):
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = (height / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)
    return Intrinsics.simple(fx, fy, width / 2.0, height / 2.0, width, height)


def camera_facing_wall(yaw_deg=0.0, pitch_deg=0.0):
    """벽(+y)을 보는 카메라 R_M_C. yaw 양수 = +x 쪽, pitch 양수 = 위."""
    return geometry.rot_z(math.radians(-yaw_deg)) @ geometry.rot_x(math.radians(pitch_deg)) @ geometry.R_M_T_UPRIGHT


def visible_tags(R_M_C, p_M_C, layout, intr, margin_px=8, min_edge_px=30):
    """(보이는 태그 id 리스트, 그중 가장 짧은 변 px)"""
    W, H = intr.width, intr.height
    ids = []
    min_edge = float("inf")
    for tid in layout.ids():
        R_M_T, p_M_T = layout.pose(tid)
        px, depth = geometry.project_tag_corners(R_M_C, p_M_C, R_M_T, p_M_T, layout.size(tid), intr.K, intr.dist)
        if (depth <= 0.05).any():
            continue
        if px[:, 0].min() < margin_px or px[:, 0].max() > W - margin_px or px[:, 1].min() < margin_px or px[:, 1].max() > H - margin_px:
            continue
        edges = [np.linalg.norm(px[i] - px[(i + 1) % 4]) for i in range(4)]
        if min(edges) < min_edge_px:
            continue
        ids.append(tid)
        min_edge = min(min_edge, min(edges))
    return ids, (min_edge if ids else float("nan"))


def coverage_profile(layout, intr, standoff_m, altitude_m, yaw_deg=0.0, pitch_deg=0.0,
                     x_start=0.0, x_end=None, step_m=0.05, **kw):
    """x 를 따라 (x, n_visible, ids, min_edge_px) 목록."""
    if x_end is None:
        x_end = max(m["x"] for m in layout.markers.values())
    R = camera_facing_wall(yaw_deg, pitch_deg)
    rows = []
    for x in np.arange(x_start, x_end + 1e-9, step_m):
        p = np.array([x, -abs(standoff_m), altitude_m])
        ids, edge = visible_tags(R, p, layout, intr, **kw)
        rows.append((float(x), len(ids), ids, edge))
    return rows


def summarize(rows):
    n = len(rows)
    ge1 = sum(1 for r in rows if r[1] >= 1) / n
    ge2 = sum(1 for r in rows if r[1] >= 2) / n
    gaps = []
    start = None
    for i, r in enumerate(rows):
        if r[1] == 0 and start is None:
            start = r[0]
        if (r[1] > 0 or i == n - 1) and start is not None:
            gaps.append((start, r[0]))
            start = None
    edges = [r[3] for r in rows if r[1] > 0]
    return {"frac_ge1": ge1, "frac_ge2": ge2, "gaps": gaps,
            "min_edge_px": min(edges) if edges else float("nan"),
            "max_gap_m": max((b - a for a, b in gaps), default=0.0)}


def run(args):
    if args.intrinsics:
        intr = Intrinsics.load(args.intrinsics)
        if intr.width is None or intr.height is None:
            raise SystemExit("intrinsics 에 width/height 가 필요")
    else:
        h, v = [float(x) for x in args.fov.lower().split("x")]
        w, hh = [int(x) for x in args.size.lower().split("x")]
        intr = intrinsics_from_fov(h, v, w, hh)
    layout = Layout.load(args.layout) if args.layout else Layout.default()
    if args.tag_size:
        for m in layout.markers.values():
            m["size_m"] = args.tag_size
    rows = coverage_profile(layout, intr, args.standoff, args.altitude, args.yaw, args.pitch,
                            step_m=args.step, margin_px=args.margin, min_edge_px=args.min_edge)
    s = summarize(rows)
    print("camera: fx={:.0f} fy={:.0f} {}x{}  standoff={:.2f} m  altitude={:.2f} m  yaw={:.1f}  tag={:.3f} m".format(
        intr.K[0, 0], intr.K[1, 1], intr.width, intr.height, args.standoff, args.altitude, args.yaw,
        next(iter(layout.markers.values()))["size_m"]))
    print("path with >=1 tag : {:5.1f} %".format(s["frac_ge1"] * 100))
    print("path with >=2 tags: {:5.1f} %".format(s["frac_ge2"] * 100))
    print("smallest tag edge : {:.0f} px (limit {} px)".format(s["min_edge_px"], args.min_edge))
    if s["gaps"]:
        print("gaps (no tag)     : " + ", ".join("{:.2f}-{:.2f} m".format(a, b) for a, b in s["gaps"]) +
              "   max {:.2f} m".format(s["max_gap_m"]))
    else:
        print("gaps (no tag)     : none")
    if args.verbose:
        for x, n, ids, edge in rows:
            print("  x={:5.2f}  n={}  ids={}  min_edge={:.0f}px".format(x, n, ids, edge if ids else float("nan")))


def add_args(p):
    p.add_argument("--intrinsics", default=None)
    p.add_argument("--fov", default="120x93.5", help="가로x세로 화각(도). intrinsics 없을 때. hires 기본값")
    p.add_argument("--size", default="1280x800", help="가로x세로 픽셀. intrinsics 없을 때")
    p.add_argument("--layout", default=None)
    p.add_argument("--tag-size", type=float, default=None, help="배치의 태그 크기를 덮어씀 [m]")
    p.add_argument("--standoff", type=float, required=True, help="벽과의 이격 [m]")
    p.add_argument("--altitude", type=float, required=True, help="카메라 고도 [m]")
    p.add_argument("--yaw", type=float, default=0.0)
    p.add_argument("--pitch", type=float, default=0.0)
    p.add_argument("--step", type=float, default=0.05)
    p.add_argument("--margin", type=int, default=8)
    p.add_argument("--min-edge", type=int, default=30)
    p.add_argument("-v", "--verbose", action="store_true")
    p.set_defaults(func=run)
