#!/usr/bin/env python3
"""
VIO 용 현수막 무늬 생성기 — 불규칙 점·얼룩 패턴 (docs/09 전방 현수막 대용)

왜 이 무늬인가
  - 단색: 카메라가 기억할 특징점이 없어 VIO 가 무너진다.
  - 규칙 격자: 특징점은 많지만 전부 같아 보여 한 칸씩 어긋나는 착오가 난다(반복 질감).
  - 불규칙 얼룩: 크기·간격·모양이 제각각이라 어느 자리인지 구분된다. 드론 연구실의 "VIO 벽지".

크기 설계 (tracking_front 1280x800, 이격 50 cm 기준 약 1.3 mm/px)
  얼룩 지름 1~12 cm 를 로그 균등으로 섞는다. 작은 것은 가까이서, 큰 것은 멀리서/흐릿할 때 잡힌다.
  검정 비율 약 30 %. 겹치지 않게 최소 간격을 둔다(겹치면 모양이 뭉개져 큰 덩어리가 된다).

출력 (print 업체용)
  <out>/banner_pattern_6000x2000mm_scale10.pdf   벡터, 1/10 축소 (600x200 mm). "10배 확대 출력" 요청.
  <out>/banner_pattern_6000x2000mm_2pxmm.png     래스터, 2 px/mm (12000x4000), 이미지로 받는 업체용
  <out>/banner_pattern_preview.jpg               화면 확인용
  <out>/banner_pattern_blobs.csv                 얼룩 좌표(m)·반지름·톤 (재현·검증용)

하단 가장자리에 마커 판 붙일 x 위치 눈금(태그 0~11, 0.5 m 간격)을 작게 넣는다. 가장자리 2 cm 안이라 VIO 에는 영향 없다.

사용
  .venv/Scripts/python analysis/banner_pattern.py                       # 6 m x 2 m, seed 7
  .venv/Scripts/python analysis/banner_pattern.py --width 7 --height 2.5 --seed 3 --no-ticks
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

# ---------------------------------------------------------------------------

def sample_blobs(width_m, height_m, rng, coverage=0.30, r_min=0.005, r_max=0.06,
                 gap=0.006, margin=0.03, max_attempts=4_000_000):
    """겹치지 않는 얼룩(중심, 반지름)을 목표 검정 비율까지 뿌린다. 반지름은 로그 균등."""
    from scipy.spatial import cKDTree

    target_area = coverage * width_m * height_m
    centers, radii = [], []
    area = 0.0
    attempts = 0
    tree = None
    rebuild_every = 500
    pending = 0

    while area < target_area and attempts < max_attempts:
        attempts += 1
        r = math.exp(rng.uniform(math.log(r_min), math.log(r_max)))
        x = rng.uniform(margin + r, width_m - margin - r)
        y = rng.uniform(margin + r, height_m - margin - r)
        if centers:
            if tree is None or pending >= rebuild_every:
                tree = cKDTree(np.array(centers))
                pending = 0
            # 최근 추가분(트리에 없음)은 따로 본다
            idx = tree.query_ball_point((x, y), r + r_max + gap)
            ok = True
            for i in idx:
                if math.hypot(x - centers[i][0], y - centers[i][1]) < r + radii[i] + gap:
                    ok = False
                    break
            if ok:
                for i in range(len(centers) - pending, len(centers)):
                    if math.hypot(x - centers[i][0], y - centers[i][1]) < r + radii[i] + gap:
                        ok = False
                        break
            if not ok:
                continue
        centers.append((x, y))
        radii.append(r)
        pending += 1
        area += math.pi * r * r
    return np.array(centers), np.array(radii), area / (width_m * height_m), attempts


def blob_polygon(cx, cy, r, rng, n=20, wobble=0.28):
    """원을 저주파 사인으로 흔든 얼룩 외곽선 (m 단위 꼭짓점)."""
    th = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    k1, k2 = rng.integers(2, 4), rng.integers(4, 7)
    p1, p2 = rng.uniform(0, 2 * math.pi, size=2)
    a1, a2 = rng.uniform(0.0, wobble), rng.uniform(0.0, wobble * 0.5)
    rr = r * (1.0 + a1 * np.sin(k1 * th + p1) + a2 * np.sin(k2 * th + p2))
    rot = rng.uniform(0, 2 * math.pi)
    return np.stack([cx + rr * np.cos(th + rot), cy + rr * np.sin(th + rot)], axis=1)


def tag_tick_positions(width_m, n_tags=12, spacing=0.5):
    """태그 0~11 중심 x (m). 전체 폭 가운데 정렬."""
    span = (n_tags - 1) * spacing
    x0 = (width_m - span) / 2.0
    return [(i, x0 + i * spacing) for i in range(n_tags)]


# ---------------------------------------------------------------------------

def write_pdf(path, width_m, height_m, polys, tones, ticks, scale=10.0):
    """벡터 PDF, 1/scale 축소. matplotlib 로 그린다 (좌표계: mm, y 위쪽)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from matplotlib.collections import PatchCollection

    w_mm, h_mm = width_m * 1000.0 / scale, height_m * 1000.0 / scale
    fig = plt.figure(figsize=(w_mm / 25.4, h_mm / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, width_m)
    ax.set_ylim(0, height_m)
    ax.set_axis_off()
    ax.set_facecolor("white")
    patches = [Polygon(p, closed=True) for p in polys]
    pc = PatchCollection(patches, facecolors=[(t, t, t) for t in tones], edgecolors="none",
                         antialiased=True)
    ax.add_collection(pc)
    for tag_id, x in ticks:
        ax.plot([x, x], [0.0, 0.02], color=(0.45, 0.45, 0.45), linewidth=0.6, solid_capstyle="butt")
        ax.text(x, 0.024, "{}".format(tag_id), ha="center", va="bottom", fontsize=3.5,
                color=(0.45, 0.45, 0.45))
    fig.savefig(path, format="pdf", facecolor="white")
    plt.close(fig)


def write_png(path, width_m, height_m, polys, tones, ticks, px_per_mm=2.0):
    import cv2
    W, H = int(round(width_m * 1000 * px_per_mm)), int(round(height_m * 1000 * px_per_mm))
    img = np.full((H, W), 255, np.uint8)
    s = 1000.0 * px_per_mm
    for p, t in zip(polys, tones):
        pts = np.round(np.stack([p[:, 0] * s, (height_m - p[:, 1]) * s], axis=1)).astype(np.int32)
        cv2.fillPoly(img, [pts], int(round(t * 255)), lineType=cv2.LINE_AA)
    for tag_id, x in ticks:
        xp = int(round(x * s))
        cv2.line(img, (xp, H - 1), (xp, H - int(20 * px_per_mm)), 115, max(1, int(px_per_mm)))
        cv2.putText(img, str(tag_id), (xp - int(6 * px_per_mm), H - int(24 * px_per_mm)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35 * px_per_mm, 115, max(1, int(px_per_mm)), cv2.LINE_AA)
    cv2.imwrite(path, img, [cv2.IMWRITE_PNG_COMPRESSION, 6])
    return img


def write_preview(path, img, max_w=2400):
    import cv2
    h, w = img.shape[:2]
    f = min(1.0, max_w / float(w))
    small = cv2.resize(img, (int(w * f), int(h * f)), interpolation=cv2.INTER_AREA)
    cv2.imwrite(path, small, [cv2.IMWRITE_JPEG_QUALITY, 85])


# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description="VIO 용 불규칙 얼룩 현수막 무늬 생성")
    ap.add_argument("--width", type=float, default=6.0, help="가로 (m)")
    ap.add_argument("--height", type=float, default=2.0, help="세로 (m)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--coverage", type=float, default=0.30, help="검정 면적 비율 목표")
    ap.add_argument("--r-min", type=float, default=0.005, help="최소 반지름 (m)")
    ap.add_argument("--r-max", type=float, default=0.06, help="최대 반지름 (m)")
    ap.add_argument("--gray-frac", type=float, default=0.25, help="진회색으로 찍을 얼룩 비율")
    ap.add_argument("--no-ticks", action="store_true", help="하단 태그 위치 눈금 생략")
    ap.add_argument("--px-per-mm", type=float, default=2.0)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "banner_out"))
    args = ap.parse_args(argv)

    os.makedirs(args.out, exist_ok=True)
    rng = np.random.default_rng(args.seed)

    centers, radii, cov, attempts = sample_blobs(args.width, args.height, rng, args.coverage,
                                                 args.r_min, args.r_max)
    n = len(radii)
    tones = np.where(rng.uniform(size=n) < args.gray_frac, 0.30, 0.0)
    polys = [blob_polygon(cx, cy, r, rng) for (cx, cy), r in zip(centers, radii)]
    ticks = [] if args.no_ticks else tag_tick_positions(args.width)

    stem = "banner_pattern_{:.0f}x{:.0f}mm".format(args.width * 1000, args.height * 1000)
    pdf = os.path.join(args.out, stem + "_scale10.pdf")
    png = os.path.join(args.out, stem + "_{:g}pxmm.png".format(args.px_per_mm))
    prev = os.path.join(args.out, "banner_pattern_preview.jpg")
    csvp = os.path.join(args.out, "banner_pattern_blobs.csv")

    write_pdf(pdf, args.width, args.height, polys, tones, ticks)
    img = write_png(png, args.width, args.height, polys, tones, ticks, args.px_per_mm)
    write_preview(prev, img)
    with open(csvp, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x_m", "y_m", "r_m", "tone"])
        for (cx, cy), r, t in zip(centers, radii, tones):
            w.writerow(["{:.4f}".format(cx), "{:.4f}".format(cy), "{:.4f}".format(r), "{:.2f}".format(t)])

    d_cm = radii * 200.0
    print("크기        : {:.1f} x {:.1f} m, seed {}".format(args.width, args.height, args.seed))
    print("얼룩 수     : {}  (시도 {})".format(n, attempts))
    print("검정 비율   : {:.1f} %".format(cov * 100))
    print("지름 분포   : 최소 {:.1f} cm / 중앙 {:.1f} cm / 최대 {:.1f} cm".format(
        d_cm.min(), np.median(d_cm), d_cm.max()))
    print("지름 1~3 cm {:.0f} %, 3~6 cm {:.0f} %, 6~12 cm {:.0f} %".format(
        100 * np.mean(d_cm < 3), 100 * np.mean((d_cm >= 3) & (d_cm < 6)), 100 * np.mean(d_cm >= 6)))
    for p in (pdf, png, prev, csvp):
        print("  {:>10.1f} KB  {}".format(os.path.getsize(p) / 1024.0, p))
    return 0


if __name__ == "__main__":
    sys.exit(main())
