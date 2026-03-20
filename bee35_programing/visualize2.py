#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
드론 정렬 로그 시각화 스크립트

단일 비행:   logs/ 폴더의 최신 파일 자동 선택
다중 비교:   logs_p/ 폴더에 비교할 CSV 파일들을 넣고 실행

- 필요 패키지: pip install pandas matplotlib
"""

import sys
import glob
import os
import re
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import platform
import math 

if platform.system() == "Windows":
    matplotlib.rc("font", family="Malgun Gothic")
elif platform.system() == "Darwin":
    matplotlib.rc("font", family="AppleGothic")
matplotlib.rcParams["axes.unicode_minus"] = False

C_BG    = "#0d1117"
C_PANEL = "#161b22"
C_TEXT  = "#e6edf3"
C_GRID  = "#21262d"
C_ALIGN = "#06d6a0"

# 세트별 색상 — x / pitch
COLORS = [
    {"x": "#00cfff", "pitch": "#ffd166"},
    {"x": "#c77dff", "pitch": "#90e0ef"},
    {"x": "#06d6a0", "pitch": "#ef476f"},
    {"x": "#f4d35e", "pitch": "#f95738"},
    {"x": "#a8dadc", "pitch": "#457b9d"},
    {"x": "#b7e4c7", "pitch": "#9b5de5"},
]
BAR_COLOR = ["#00cfff", "#c77dff", "#06d6a0", "#f4d35e", "#a8dadc", "#b7e4c7"]

LOG_DIR     = "logs"
COMPARE_DIR = "logs_p"
VISUAL_DIR  = "visual"

# ─────────────────────────────────────────
# 유틸
# ─────────────────────────────────────────
def find_latest(pattern):
    files = sorted(glob.glob(pattern))
    return files[-1] if files else None

def find_summary_for(rt_path):
    dirname  = os.path.dirname(rt_path)
    basename = os.path.basename(rt_path)
    m = re.search(r"realtime_(\d{8}_\d{6})", basename)
    if not m:
        return None
    ts         = m.group(1) + ".csv"
    candidates = glob.glob(os.path.join(dirname, f"*summary_{ts}"))
    return candidates[0] if candidates else None

def short_label(path, idx):
    base  = os.path.splitext(os.path.basename(path))[0]
    parts = base.split("_")
    ts    = parts[-1] if parts else str(idx)
    return f"F{ts}"

def load_pair(rt_path, idx):
    rt       = pd.read_csv(rt_path)
    sum_path = find_summary_for(rt_path)
    sumdf    = pd.read_csv(sum_path) if sum_path else None
    label    = short_label(rt_path, idx)
    fullname = os.path.splitext(os.path.basename(rt_path))[0]
    return label, fullname, rt, sumdf

def make_ax(fig, spec, row, col=None, colspan=False):
    ax = fig.add_subplot(spec[row, :] if colspan else spec[row, col])
    ax.set_facecolor(C_PANEL)
    ax.tick_params(colors=C_TEXT, labelsize=8)
    ax.xaxis.label.set_color(C_TEXT)
    ax.yaxis.label.set_color(C_TEXT)
    ax.title.set_color(C_TEXT)
    for spine in ax.spines.values():
        spine.set_edgecolor(C_GRID)
    ax.grid(color=C_GRID, linewidth=0.5, alpha=0.7)
    return ax

def shade_aligned(ax, rt):
    mask     = rt["aligned"] == 1
    in_block = False
    for i, v in enumerate(mask):
        if v and not in_block:
            x0 = rt["elapsed_s"].iloc[i]; in_block = True
        elif not v and in_block:
            ax.axvspan(x0, rt["elapsed_s"].iloc[i], alpha=0.10, color=C_ALIGN, zorder=0)
            in_block = False
    if in_block:
        ax.axvspan(x0, rt["elapsed_s"].iloc[-1], alpha=0.10, color=C_ALIGN, zorder=0)

# ─────────────────────────────────────────
# 데이터 로드
# ─────────────────────────────────────────
os.makedirs(VISUAL_DIR, exist_ok=True)

if os.path.isdir(COMPARE_DIR):
    rt_files = sorted(glob.glob(os.path.join(COMPARE_DIR, "*realtime*.csv")))
else:
    rt_files = []

if rt_files:
    datasets = [load_pair(f, i) for i, f in enumerate(rt_files)]
    mode     = "compare"
    print(f"[비교 모드] {len(datasets)}개 세트")
    for label, fullname, rt, sumdf in datasets:
        print(f"  [{label}] {fullname}")
else:
    rt_path = find_latest(os.path.join(LOG_DIR, "realtime_*.csv")) or \
              find_latest(os.path.join(LOG_DIR, "camtest_realtime_*.csv"))
    if not rt_path:
        print("[ERROR] logs/ 또는 logs_p/ 폴더에 CSV 파일이 없습니다.")
        sys.exit(1)
    datasets = [load_pair(rt_path, 0)]
    mode     = "single"
    print(f"[단일 모드] {rt_path}")

compare = mode == "compare"
n       = len(datasets)

has_imu = all("actual_roll_deg" in d[2].columns for d in datasets)
has_vib = all("vib_x"           in d[2].columns for d in datasets)

# ─────────────────────────────────────────
# 레이아웃
# ─────────────────────────────────────────
n_rows  = 2                          # x 오차, pitch 오차
n_rows += 1 if has_imu else 0        # roll/pitch IMU
n_rows += 1 if has_vib else 0        # 진동 시계열
n_rows += 1                          # 소요시간 + 누적오차
n_rows += 1                          # rms_vib 막대 + 정렬비율
n_rows += 1                          # 오차 분포 (히스토그램)
if compare:
    n_rows += n - 1                  # 비교 모드 히스토그램 추가 행

fig = plt.figure(figsize=(16, 4.5 * n_rows), facecolor=C_BG)
gs  = gridspec.GridSpec(n_rows, 2, figure=fig, hspace=0.6, wspace=0.32)
row = 0

# ── 상단 제목 ─────────────────────────────
if compare:
    mapping = "  |  ".join(f"[{d[0]}] {d[1]}" for d in datasets)
    fig.text(0.5, 0.998, "비교 분석", ha="center", va="top",
             fontsize=15, color=C_TEXT, fontweight="bold")
    fig.text(0.5, 0.993, mapping, ha="center", va="top",
             fontsize=7.5, color="#8b949e")
else:
    fig.text(0.5, 0.998, "드론 AprilTag 정렬 분석", ha="center", va="top",
             fontsize=15, color=C_TEXT, fontweight="bold")
    fig.text(0.5, 0.993, datasets[0][1], ha="center", va="top",
             fontsize=8, color="#8b949e")

# ── 1. 오차 시계열: x ─────────────────────
ax1 = make_ax(fig, gs, row, colspan=True); row += 1
for i, (label, _, rt, __) in enumerate(datasets):
    p   = COLORS[i % len(COLORS)]
    sfx = f" [{label}]" if compare else ""
    ax1.plot(rt["elapsed_s"], rt["err_x_px"], color=p["x"], lw=0.9, label=f"err_x{sfx}")
    shade_aligned(ax1, rt)
ax1.axhline(0, color=C_TEXT, lw=0.4, ls="--", alpha=0.5)
ax1.set_title("오차 시계열 — 좌우(x)  (px)", fontsize=10)
ax1.set_xlabel("elapsed (s)"); ax1.set_ylabel("pixel error")
ax1.legend(facecolor=C_PANEL, labelcolor=C_TEXT, fontsize=8)

# ── 2. 오차 시계열: pitch ─────────────────
ax2 = make_ax(fig, gs, row, colspan=True); row += 1
for i, (label, _, rt, __) in enumerate(datasets):
    p   = COLORS[i % len(COLORS)]
    sfx = f" [{label}]" if compare else ""
    ax2.plot(rt["elapsed_s"], rt["err_pitch_px"], color=p["pitch"], lw=0.9, label=f"err_pitch{sfx}")
    shade_aligned(ax2, rt)
ax2.axhline(0, color=C_TEXT, lw=0.4, ls="--", alpha=0.5)
ax2.set_title("오차 시계열 — 피치/거리  (marker px − target px)", fontsize=10)
ax2.set_xlabel("elapsed (s)"); ax2.set_ylabel("pixel error")
ax2.legend(facecolor=C_PANEL, labelcolor=C_TEXT, fontsize=8)

# ── 3. 목표 vs 실제 롤/피치 ──────────────
if has_imu:
    ax3l = make_ax(fig, gs, row, 0)
    ax3r = make_ax(fig, gs, row, 1); row += 1
    for i, (label, _, rt, __) in enumerate(datasets):
        p   = COLORS[i % len(COLORS)]
        sfx = f" [{label}]" if compare else ""
        ax3l.plot(rt["elapsed_s"], rt["target_roll_us"],   color=p["x"],     lw=0.8, ls="--", label=f"목표{sfx}")
        ax3l.plot(rt["elapsed_s"], rt["actual_roll_deg"],  color=p["pitch"], lw=0.8,          label=f"실제{sfx}")
        ax3r.plot(rt["elapsed_s"], rt["target_pitch_us"],  color=p["x"],     lw=0.8, ls="--", label=f"목표{sfx}")
        ax3r.plot(rt["elapsed_s"], rt["actual_pitch_deg"], color=p["pitch"], lw=0.8,          label=f"실제{sfx}")
        shade_aligned(ax3l, rt); shade_aligned(ax3r, rt)
    ax3l.set_title("목표 vs 실제 Roll", fontsize=10); ax3l.set_xlabel("elapsed (s)")
    ax3r.set_title("목표 vs 실제 Pitch", fontsize=10); ax3r.set_xlabel("elapsed (s)")
    ax3l.legend(facecolor=C_PANEL, labelcolor=C_TEXT, fontsize=8)
    ax3r.legend(facecolor=C_PANEL, labelcolor=C_TEXT, fontsize=8)

# ── 4. IMU 진동 시계열 ────────────────────
if has_vib:
    ax4 = make_ax(fig, gs, row, colspan=True); row += 1
    for i, (label, _, rt, __) in enumerate(datasets):
        p   = COLORS[i % len(COLORS)]
        sfx = f" [{label}]" if compare else ""
        ax4.plot(rt["elapsed_s"], rt["vib_x"], color=p["x"],     lw=0.8, label=f"vib_x{sfx}")
        ax4.plot(rt["elapsed_s"], rt["vib_y"], color="#ff6b6b",  lw=0.8, label=f"vib_y{sfx}", alpha=0.7)
        ax4.plot(rt["elapsed_s"], rt["vib_z"], color=p["pitch"], lw=0.8, label=f"vib_z{sfx}")
        shade_aligned(ax4, rt)
    ax4.set_title("IMU 진동 시계열", fontsize=10)
    ax4.set_xlabel("elapsed (s)"); ax4.set_ylabel("vibration")
    ax4.legend(facecolor=C_PANEL, labelcolor=C_TEXT, fontsize=8, ncol=3)

# ── 5. 소요시간 + 누적 오차 ──────────────
ax5 = make_ax(fig, gs, row, 0)
ax6 = make_ax(fig, gs, row, 1); row += 1

bar_w = min(0.6 / max(n, 1), 0.35)
for i, (label, _, rt, sumdf) in enumerate(datasets):
    if sumdf is None or len(sumdf) == 0:
        continue
    offset = (i - (n - 1) / 2) * bar_w
    x      = sumdf["attempt"]

    ax5.bar(x + offset, sumdf["align_time_s"], width=bar_w * 0.9,
            color=BAR_COLOR[i % len(BAR_COLOR)], alpha=0.85,
            label=label, edgecolor=C_GRID, lw=0.5)

    ax6.bar(x + offset, sumdf["total_abs_err_px"], width=bar_w * 0.9,
            color=BAR_COLOR[i % len(BAR_COLOR)], alpha=0.85,
            label=label, edgecolor=C_GRID, lw=0.5)

ax5.set_title("시도별 정렬 소요시간", fontsize=10)
ax5.set_xlabel("attempt #"); ax5.set_ylabel("seconds")
ax5.legend(facecolor=C_PANEL, labelcolor=C_TEXT, fontsize=8)
ax6.set_title("시도별 누적 절댓값 오차 (x + pitch)", fontsize=10)
ax6.set_xlabel("attempt #"); ax6.set_ylabel("total abs error (px)")
ax6.legend(facecolor=C_PANEL, labelcolor=C_TEXT, fontsize=8)

# ── 6. RMS 진동 막대 + 정렬 비율 ─────────
ax_vib  = make_ax(fig, gs, row, 0)
ax_rate = make_ax(fig, gs, row, 1); row += 1

# RMS 진동 막대
for i, (label, _, rt, sumdf) in enumerate(datasets):
    if sumdf is None or len(sumdf) == 0 or "rms_vib" not in sumdf.columns:
        continue
    offset = (i - (n - 1) / 2) * bar_w
    x      = sumdf["attempt"]
    ax_vib.bar(x + offset, sumdf["rms_vib"], width=bar_w * 0.9,
               color=BAR_COLOR[i % len(BAR_COLOR)], alpha=0.85,
               label=label, edgecolor=C_GRID, lw=0.5)
ax_vib.set_title("시도별 RMS 진동", fontsize=10)
ax_vib.set_xlabel("attempt #"); ax_vib.set_ylabel("RMS vibration")
ax_vib.legend(facecolor=C_PANEL, labelcolor=C_TEXT, fontsize=8)

# 정렬 비율
if not compare:
    _, _, rt, _ = datasets[0]
    mask = rt["aligned"] == 1
    ax_rate.pie([mask.sum(), (~mask).sum()],
                labels=["ALIGNED", "NOT ALIGNED"],
                colors=[C_ALIGN, "#ff6b6b"],
                autopct="%1.1f%%",
                textprops={"color": C_TEXT, "fontsize": 9},
                startangle=90)
    ax_rate.set_title("전체 구간 정렬 비율", fontsize=10)
else:
    lbls = [d[0] for d in datasets]
    pcts = [(d[2]["aligned"] == 1).mean() * 100 for d in datasets]
    bars = ax_rate.bar(lbls, pcts,
                       color=[BAR_COLOR[i % len(BAR_COLOR)] for i in range(n)],
                       edgecolor=C_GRID, width=0.45)
    ax_rate.set_ylim(0, 110); ax_rate.set_ylabel("%")
    ax_rate.set_title("전체 구간 정렬 비율 비교 (%)", fontsize=10)
    for bar, pct in zip(bars, pcts):
        ax_rate.text(bar.get_x() + bar.get_width() / 2, pct + 2,
                     f"{pct:.1f}%", ha="center", color=C_TEXT, fontsize=11, fontweight="bold")

# ── 7. 오차 분포 히스토그램 ──────────────
if not compare:
    ax8 = make_ax(fig, gs, row, 0)
    ax9 = make_ax(fig, gs, row, 1); row += 1
    _, _, rt, _ = datasets[0]
    bins = 40
    ax8.hist(rt["err_x_px"],     bins=bins, color=COLORS[0]["x"],     alpha=0.7, label="err_x",     density=True)
    ax8.set_title("오차 분포 — 좌우(x)", fontsize=10)
    ax8.set_xlabel("pixel error"); ax8.set_ylabel("density")
    ax8.legend(facecolor=C_PANEL, labelcolor=C_TEXT, fontsize=8)

    ax9.hist(rt["err_pitch_px"], bins=bins, color=COLORS[0]["pitch"], alpha=0.7, label="err_pitch", density=True)
    ax9.set_title("오차 분포 — 피치/거리", fontsize=10)
    ax9.set_xlabel("pixel error"); ax9.set_ylabel("density")
    ax9.legend(facecolor=C_PANEL, labelcolor=C_TEXT, fontsize=8)
else:
    for i, (label, _, rt, __) in enumerate(datasets):
        p    = COLORS[i % len(COLORS)]
        bins = 40
        col  = i % 2
        if col == 0:
            cur_row = row + i // 2
        ax_h = make_ax(fig, gs, cur_row, col)
        ax_h.hist(rt["err_x_px"],     bins=bins, color=p["x"],     alpha=0.65, label="err_x",     density=True)
        ax_h.hist(rt["err_pitch_px"], bins=bins, color=p["pitch"], alpha=0.65, label="err_pitch", density=True)
        ax_h.set_title(f"오차 분포  [{label}]", fontsize=10)
        ax_h.set_xlabel("pixel error"); ax_h.set_ylabel("density")
        ax_h.legend(facecolor=C_PANEL, labelcolor=C_TEXT, fontsize=8)
    row += math.ceil(n / 2)

# ─────────────────────────────────────────
# 저장
# ─────────────────────────────────────────
import math as _math
if compare:
    names_joined = "_vs_".join(d[0] for d in datasets)
    out_path = os.path.join(VISUAL_DIR, f"compare_{names_joined}.png")
else:
    out_path = os.path.join(VISUAL_DIR, f"{datasets[0][1]}.png")

plt.savefig(out_path, dpi=150, bbox_inches="tight", facecolor=C_BG)
print(f"[SAVED] {out_path}")
plt.show()