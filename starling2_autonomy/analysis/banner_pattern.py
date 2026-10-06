#!/usr/bin/env python3
"""
VIO 용 현수막 무늬 생성기 (docs/09 전방 현수막)

세 가지 무늬를 같은 크기·같은 파일 형식으로 뽑는다. `--style` 로 고른다.
  blobs  불규칙 얼룩 (기본). 크기·간격·모양이 제각각이라 어느 자리인지 구분된다. 드론 연구실의 "VIO 벽지".
  grid   불규칙 간격 격자. 선 간격·굵기·톤이 제각각이라 "어느 칸인지" 구분된다. `--regular` 를 주면
         균일 격자(`--grid-spacing`, `--grid-width`). 2026-10-06 결정: 50 cm × 5 m 균일 격자.
  bed    재배단 그림. 3단 선반 + 상추 포기 + 선반 기둥 + LED 바 + 점적 호스. 현장(실내 재배실) 과 비슷하게.

왜 단색은 안 되는가: 카메라가 기억할 특징점이 없어 VIO 가 무너진다. 왜 균일 격자는 애매한가: 점은
많은데 전부 같아 보여 한 칸 어긋나는 착오가 난다(반복 질감).

크기 설계 (tracking_front 1280x800, 이격 50 cm 기준 약 1.3 mm/px)
  얼룩 지름 1~12 cm 를 로그 균등으로 섞는다. 작은 것은 가까이서, 큰 것은 멀리서/흐릿할 때 잡힌다.

출력 (<out>/, 인쇄 업체용)
  banner_<style>_6000x2000mm_scale10.pdf   벡터, 1/10 축소 (600x200 mm). "10배 확대 출력" 요청.
  banner_<style>_6000x2000mm_2pxmm.png     래스터, 2 px/mm (12000x4000). 이미지로 받는 업체용
  banner_<style>_preview.jpg               화면 확인용
  banner_blobs_blobs.csv                   (blobs 만) 얼룩 좌표(m)·반지름·톤

하단 가장자리에 마커 판 붙일 x 위치 눈금(태그 0~11, 0.5 m 간격)을 작게 넣는다. 가장자리 2 cm 안이라 VIO 에는 영향 없다.

사용
  .venv/Scripts/python analysis/banner_pattern.py                       # blobs, 6 m x 2 m, seed 7
  .venv/Scripts/python analysis/banner_pattern.py --style grid
  .venv/Scripts/python analysis/banner_pattern.py --style grid --regular
  .venv/Scripts/python analysis/banner_pattern.py --style bed
  .venv/Scripts/python analysis/banner_pattern.py --width 7 --height 2.5 --seed 3 --no-ticks
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

TICK_COLOR = (0.45, 0.45, 0.45)


# ---------------------------------------------------------------------------
# 그리기 추상화 — 같은 장면을 래스터(cv2) 와 벡터(matplotlib PDF) 두 벌로 그린다
# 좌표는 m, 원점은 왼쪽 아래, 색은 RGB 0~1.
# ---------------------------------------------------------------------------

class CvCanvas(object):
    def __init__(self, width_m, height_m, px_per_mm, bg):
        import cv2
        self.cv2 = cv2
        self.w, self.h = width_m, height_m
        self.s = 1000.0 * px_per_mm
        self.W, self.H = int(round(width_m * self.s)), int(round(height_m * self.s))
        self.img = np.empty((self.H, self.W, 3), np.uint8)
        self.img[:] = self._c(bg)

    @staticmethod
    def _c(rgb):
        return (int(round(rgb[2] * 255)), int(round(rgb[1] * 255)), int(round(rgb[0] * 255)))

    def _pt(self, x, y):
        return (int(round(x * self.s)), int(round((self.h - y) * self.s)))

    def polygon(self, pts, color):
        arr = np.array([self._pt(x, y) for x, y in pts], np.int32)
        self.cv2.fillPoly(self.img, [arr], self._c(color), lineType=self.cv2.LINE_AA)

    def ellipse(self, cx, cy, rx, ry, angle_deg, color):
        self.cv2.ellipse(self.img, self._pt(cx, cy), (int(round(rx * self.s)), int(round(ry * self.s))),
                         -angle_deg, 0, 360, self._c(color), -1, self.cv2.LINE_AA)

    def text(self, x, y, s, size_m, color):
        scale = size_m * self.s / 22.0            # HERSHEY_SIMPLEX 글자 높이 ≈ 22 px @ scale 1
        thick = max(1, int(round(size_m * self.s / 12.0)))
        (tw, th), _ = self.cv2.getTextSize(s, self.cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
        px, py = self._pt(x, y)
        self.cv2.putText(self.img, s, (px - tw // 2, py), self.cv2.FONT_HERSHEY_SIMPLEX, scale,
                         self._c(color), thick, self.cv2.LINE_AA)

    def save(self, path):
        self.cv2.imwrite(path, self.img, [self.cv2.IMWRITE_PNG_COMPRESSION, 6])

    def save_preview(self, path, max_w=2400):
        f = min(1.0, max_w / float(self.W))
        small = self.cv2.resize(self.img, (int(self.W * f), int(self.H * f)), interpolation=self.cv2.INTER_AREA)
        self.cv2.imwrite(path, small, [self.cv2.IMWRITE_JPEG_QUALITY, 85])


class MplCanvas(object):
    def __init__(self, width_m, height_m, scale, bg):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon, Ellipse
        self._Polygon, self._Ellipse = Polygon, Ellipse
        self.plt = plt
        self.w, self.h = width_m, height_m
        w_mm, h_mm = width_m * 1000.0 / scale, height_m * 1000.0 / scale
        self.fig = plt.figure(figsize=(w_mm / 25.4, h_mm / 25.4))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, width_m)
        self.ax.set_ylim(0, height_m)
        self.ax.set_axis_off()
        self.fig.patch.set_facecolor(bg)
        self.ax.set_facecolor(bg)
        self.bg = bg
        self.h_in = h_mm / 25.4
        self._patches, self._colors = [], []

    def polygon(self, pts, color):
        self._patches.append(self._Polygon(np.asarray(pts), closed=True))
        self._colors.append(color)

    def ellipse(self, cx, cy, rx, ry, angle_deg, color):
        self._patches.append(self._Ellipse((cx, cy), 2 * rx, 2 * ry, angle=angle_deg))
        self._colors.append(color)

    def text(self, x, y, s, size_m, color):
        self._flush()
        pt = size_m / self.h * self.h_in * 72.0
        self.ax.text(x, y, s, ha="center", va="bottom", fontsize=pt, color=color, family="DejaVu Sans")

    def _flush(self):
        if not self._patches:
            return
        from matplotlib.collections import PatchCollection
        pc = PatchCollection(self._patches, facecolors=self._colors, edgecolors="none", antialiased=True)
        self.ax.add_collection(pc)
        self._patches, self._colors = [], []

    def save(self, path):
        self._flush()
        self.fig.savefig(path, format="pdf", facecolor=self.bg)
        self.plt.close(self.fig)


class MultiCanvas(object):
    """같은 호출을 여러 캔버스에 전달."""

    def __init__(self, canvases):
        self.c = canvases

    def polygon(self, pts, color):
        for c in self.c:
            c.polygon(pts, color)

    def ellipse(self, cx, cy, rx, ry, angle_deg, color):
        for c in self.c:
            c.ellipse(cx, cy, rx, ry, angle_deg, color)

    def text(self, x, y, s, size_m, color):
        for c in self.c:
            c.text(x, y, s, size_m, color)


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def tag_tick_positions(width_m, n_tags=12, spacing=0.5, margin=0.25):
    """태그 0~11 중심 x (m). 전체 폭 가운데 정렬. 폭이 모자라면 들어가는 개수만."""
    n_fit = int((width_m - 2 * margin) / spacing) + 1
    n_tags = max(0, min(n_tags, n_fit))
    span = (n_tags - 1) * spacing
    x0 = (width_m - span) / 2.0
    return [(i, x0 + i * spacing) for i in range(n_tags)]


def draw_ticks(cv, width_m):
    for tag_id, x in tag_tick_positions(width_m):
        cv.polygon(rect(x - 0.0015, 0.0, x + 0.0015, 0.02), TICK_COLOR)
        cv.text(x, 0.024, str(tag_id), 0.012, TICK_COLOR)


# ---------------------------------------------------------------------------
# 스타일 1: 불규칙 얼룩
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


def style_blobs(cv, args, rng, out_dir):
    centers, radii, cov, attempts = sample_blobs(args.width, args.height, rng, args.coverage,
                                                 args.r_min, args.r_max)
    n = len(radii)
    tones = np.where(rng.uniform(size=n) < args.gray_frac, 0.30, 0.0)
    for (cx, cy), r, t in zip(centers, radii, tones):
        cv.polygon(blob_polygon(cx, cy, r, rng), (t, t, t))
    with open(os.path.join(out_dir, "banner_blobs_blobs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x_m", "y_m", "r_m", "tone"])
        for (cx, cy), r, t in zip(centers, radii, tones):
            w.writerow(["{:.4f}".format(cx), "{:.4f}".format(cy), "{:.4f}".format(r), "{:.2f}".format(t)])
    d_cm = radii * 200.0
    return ["얼룩 수     : {}  (시도 {})".format(n, attempts),
            "검정 비율   : {:.1f} %".format(cov * 100),
            "지름 분포   : 최소 {:.1f} cm / 중앙 {:.1f} cm / 최대 {:.1f} cm".format(d_cm.min(), np.median(d_cm), d_cm.max()),
            "지름 1~3 cm {:.0f} %, 3~6 cm {:.0f} %, 6~12 cm {:.0f} %".format(
                100 * np.mean(d_cm < 3), 100 * np.mean((d_cm >= 3) & (d_cm < 6)), 100 * np.mean(d_cm >= 6))]


# ---------------------------------------------------------------------------
# 스타일 2: 격자 (불규칙 간격 / 균일)
# ---------------------------------------------------------------------------

def _positions(length, lo, hi, rng, regular=None):
    if regular:
        return [regular * i for i in range(1, int(length / regular))]
    xs, x = [], 0.0
    while True:
        x += rng.uniform(lo, hi)
        if x >= length - lo * 0.5:
            break
        xs.append(x)
    return xs


def style_grid(cv, args, rng, out_dir):
    reg = args.grid_spacing if args.regular else None
    xs = _positions(args.width, 0.18, 0.55, rng, reg)
    ys = _positions(args.height, 0.15, 0.45, rng, reg)
    n_lines = 0
    for x in xs:
        w = args.grid_width if reg else rng.uniform(0.006, 0.028)
        tone = 0.0 if (reg or rng.uniform() < 0.7) else 0.4
        cv.polygon(rect(x - w / 2, 0.0, x + w / 2, args.height), (tone, tone, tone))
        n_lines += 1
    for y in ys:
        w = args.grid_width if reg else rng.uniform(0.006, 0.028)
        tone = 0.0 if (reg or rng.uniform() < 0.7) else 0.4
        cv.polygon(rect(0.0, y - w / 2, args.width, y + w / 2), (tone, tone, tone))
        n_lines += 1
    # 불규칙 격자에는 교차점 일부에 작은 사각 점을 더 찍어 칸 구분을 돕는다
    n_dots = 0
    if not reg:
        for x in xs:
            for y in ys:
                if rng.uniform() < 0.25:
                    r = rng.uniform(0.015, 0.035)
                    cv.polygon(rect(x - r, y - r, x + r, y + r), (0.0, 0.0, 0.0))
                    n_dots += 1
    kind = ("균일 {:.0f} cm 간격, 굵기 {:.1f} cm".format(reg * 100, args.grid_width * 100) if reg
            else "불규칙 (세로선 간격 18~55 cm, 가로선 15~45 cm, 굵기 0.6~2.8 cm)")
    return ["격자        : {}".format(kind),
            "선 수       : 세로 {} + 가로 {} = {}, 교차점 점 {}".format(len(xs), len(ys), n_lines, n_dots)]


# ---------------------------------------------------------------------------
# 스타일 3: 재배단 (실내 재배실 선반 측면)
# ---------------------------------------------------------------------------

GREENS = [(0.33, 0.58, 0.20), (0.45, 0.68, 0.25), (0.56, 0.76, 0.30), (0.26, 0.48, 0.17),
          (0.62, 0.80, 0.36), (0.40, 0.63, 0.22), (0.20, 0.40, 0.14)]
REDS = [(0.45, 0.18, 0.20), (0.55, 0.25, 0.25), (0.38, 0.14, 0.18), (0.62, 0.32, 0.28), (0.30, 0.12, 0.14)]


def draw_lettuce(cv, cx, cy, r, rng, red=False):
    """상추 포기: 잎 타원 여러 장을 뒤(어두움)→앞(밝음) 순으로 겹친다."""
    pal = REDS if red else GREENS
    n_leaf = int(rng.integers(10, 17))
    leaves = []
    for _ in range(n_leaf):
        ang = rng.uniform(0, 360)
        d = rng.uniform(0.15, 0.6) * r
        lx = cx + d * math.cos(math.radians(ang))
        ly = cy + d * math.sin(math.radians(ang))
        rx = r * rng.uniform(0.35, 0.6)
        ry = r * rng.uniform(0.55, 0.9)
        leaves.append((d, lx, ly, rx, ry, ang + rng.uniform(-25, 25)))
    leaves.sort(key=lambda l: -l[0])         # 바깥 잎부터
    for i, (d, lx, ly, rx, ry, ang) in enumerate(leaves):
        base = pal[int(rng.integers(len(pal)))]
        shade = 0.75 + 0.35 * (i / float(n_leaf))     # 안쪽 잎이 밝다
        col = tuple(min(1.0, c * shade) for c in base)
        cv.ellipse(lx, ly, rx, ry, ang, col)
        # 잎맥 느낌의 어두운 가는 타원
        if rng.uniform() < 0.5:
            dark = tuple(c * 0.6 for c in col)
            cv.ellipse(lx, ly, rx * 0.12, ry * 0.8, ang, dark)


def style_bed(cv, args, rng, out_dir):
    W, H = args.width, args.height
    # 배경: 밝은 벽 + 약한 얼룩(완전 균일 면을 피한다)
    for _ in range(int(W * H * 25)):
        x, y = rng.uniform(0, W), rng.uniform(0, H)
        r = rng.uniform(0.05, 0.25)
        t = rng.uniform(0.86, 0.94)
        cv.ellipse(x, y, r, r * rng.uniform(0.4, 1.0), rng.uniform(0, 180), (t, t, t))

    shelf_y = [0.33, 1.03, 1.73] if H >= 1.9 else [0.33, 1.03]
    shelf_h = 0.08
    n_plants = 0
    # 선반 기둥 (수직)
    posts = _positions(W, 1.0, 1.5, rng)
    for x in [0.06] + posts + [W - 0.06]:
        pw = rng.uniform(0.04, 0.06)
        cv.polygon(rect(x - pw / 2, 0.0, x + pw / 2, H), (0.30, 0.31, 0.33))
        for y in np.arange(0.1, H, 0.1):     # 볼트 구멍
            cv.ellipse(x, y, 0.008, 0.008, 0, (0.15, 0.15, 0.16))
    # 위쪽 수평 파이프
    cv.polygon(rect(0.0, H - 0.09, W, H - 0.05), (0.80, 0.80, 0.78))
    cv.polygon(rect(0.0, H - 0.06, W, H - 0.05), (0.62, 0.62, 0.60))

    for k, ys in enumerate(shelf_y):
        # 선반 앞판 (트레이 턱) + 아래 그림자 띠
        cv.polygon(rect(0.0, ys - 0.02, W, ys), (0.25, 0.25, 0.27))
        cv.polygon(rect(0.0, ys, W, ys + shelf_h), (0.84, 0.84, 0.82))
        cv.polygon(rect(0.0, ys + shelf_h - 0.012, W, ys + shelf_h), (0.70, 0.70, 0.68))
        # 트레이 칸 구분선 (불규칙 간격)
        for x in _positions(W, 0.45, 0.75, rng):
            cv.polygon(rect(x - 0.004, ys + 0.005, x + 0.004, ys + shelf_h - 0.012), (0.66, 0.66, 0.64))
        # 라벨 (작은 흰 사각 + 글자)
        for x in _positions(W, 0.9, 1.8, rng):
            cv.polygon(rect(x - 0.045, ys + 0.018, x + 0.045, ys + 0.05), (0.98, 0.98, 0.98))
            cv.polygon(rect(x - 0.045, ys + 0.018, x + 0.045, ys + 0.021), (0.3, 0.3, 0.3))
            cv.text(x, ys + 0.026, "{}-{:02d}".format("ABC"[k], int(rng.integers(1, 40))), 0.016, (0.2, 0.2, 0.25))
        # 점적 호스 (검은 가는 선) + 상추
        top = ys + shelf_h
        cv.polygon(rect(0.0, top + 0.012, W, top + 0.02), (0.08, 0.08, 0.08))
        red_run = False
        x = rng.uniform(0.10, 0.22)
        while x < W - 0.08:
            if rng.uniform() < 0.08:          # 빈 자리
                x += rng.uniform(0.18, 0.26)
                continue
            if rng.uniform() < 0.12:
                red_run = not red_run
            r = rng.uniform(0.075, 0.125)
            draw_lettuce(cv, x, top + r * 0.75, r, rng, red=red_run)
            n_plants += 1
            x += rng.uniform(0.17, 0.26)
        # 이 단을 비추는 LED 바 (바로 위 선반 아래쪽)
        if k + 1 < len(shelf_y):
            yl = shelf_y[k + 1] - 0.14
        else:
            yl = H - 0.16
        cv.polygon(rect(0.0, yl - 0.012, W, yl + 0.012), (0.20, 0.20, 0.22))
        cv.polygon(rect(0.0, yl - 0.006, W, yl + 0.006), (0.95, 0.80, 0.92))
        for xl in np.arange(0.05, W, 0.05):   # LED 소자
            cv.ellipse(xl + rng.uniform(-0.004, 0.004), yl, 0.006, 0.004, 0, (1.0, 0.95, 1.0))
    # 수직 급수 파이프 하나
    xp = rng.uniform(0.3, W - 0.3)
    cv.polygon(rect(xp - 0.02, 0.0, xp + 0.02, H), (0.90, 0.90, 0.88))
    cv.polygon(rect(xp + 0.008, 0.0, xp + 0.02, H), (0.72, 0.72, 0.70))
    return ["선반        : {}단 (높이 {})".format(len(shelf_y), ", ".join("{:.2f} m".format(y) for y in shelf_y)),
            "기둥        : {}개, 상추 {}포기".format(len(posts) + 2, n_plants)]


STYLES = {"blobs": style_blobs, "grid": style_grid, "bed": style_bed}


# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description="VIO 용 현수막 무늬 생성 (blobs / grid / bed)")
    ap.add_argument("--style", choices=sorted(STYLES), default="blobs")
    ap.add_argument("--width", type=float, default=6.0, help="가로 (m)")
    ap.add_argument("--height", type=float, default=2.0, help="세로 (m)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--coverage", type=float, default=0.30, help="[blobs] 검정 면적 비율 목표")
    ap.add_argument("--r-min", type=float, default=0.005, help="[blobs] 최소 반지름 (m)")
    ap.add_argument("--r-max", type=float, default=0.06, help="[blobs] 최대 반지름 (m)")
    ap.add_argument("--gray-frac", type=float, default=0.25, help="[blobs] 진회색 얼룩 비율")
    ap.add_argument("--regular", action="store_true", help="[grid] 균일 격자")
    ap.add_argument("--grid-spacing", type=float, default=0.10, help="[grid --regular] 격자 간격 (m)")
    ap.add_argument("--grid-width", type=float, default=0.012, help="[grid --regular] 선 굵기 (m)")
    ap.add_argument("--no-ticks", action="store_true", help="하단 태그 위치 눈금 생략")
    ap.add_argument("--px-per-mm", type=float, default=2.0)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "banner_out"))
    args = ap.parse_args(argv)

    os.makedirs(args.out, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    bg = (0.92, 0.92, 0.91) if args.style == "bed" else (1.0, 1.0, 1.0)
    name = args.style + ("_regular" if (args.style == "grid" and args.regular) else "")
    stem = "banner_{}_{:.0f}x{:.0f}mm".format(name, args.width * 1000, args.height * 1000)
    pdf = os.path.join(args.out, stem + "_scale10.pdf")
    png = os.path.join(args.out, stem + "_{:g}pxmm.png".format(args.px_per_mm))
    prev = os.path.join(args.out, "banner_{}_preview.jpg".format(name))

    raster = CvCanvas(args.width, args.height, args.px_per_mm, bg)
    vector = MplCanvas(args.width, args.height, 10.0, bg)
    cv = MultiCanvas([raster, vector])

    lines = STYLES[args.style](cv, args, rng, args.out)
    ticks = [] if args.no_ticks else tag_tick_positions(args.width)
    if ticks:
        draw_ticks(cv, args.width)
        lines.append("태그 눈금   : {}개 (ID 0~{}), x = {:.2f} m 부터 0.5 m 간격".format(
            len(ticks), ticks[-1][0], ticks[0][1]))

    vector.save(pdf)
    raster.save(png)
    raster.save_preview(prev)

    print("스타일      : {}  크기 {:.1f} x {:.1f} m, seed {}".format(name, args.width, args.height, args.seed))
    for ln in lines:
        print(ln)
    for p in (pdf, png, prev):
        print("  {:>10.1f} KB  {}".format(os.path.getsize(p) / 1024.0, p))
    return 0


if __name__ == "__main__":
    sys.exit(main())
