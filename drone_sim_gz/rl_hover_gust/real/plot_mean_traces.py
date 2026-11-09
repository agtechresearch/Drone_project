#!/usr/bin/env python3
"""README figure (2026-11-09): real fan comparison traces, fan ON = 0, from real/ctl_20261105/mean_traces.json
(grid s, px4/ppo = 4-flight mean, *_all = each flight; tag lateral position, cm, same smoothed metric as ctl_judge).
  venv/bin/python real/plot_mean_traces.py <out.png>"""
import json, os, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
HERE = os.path.dirname(os.path.abspath(__file__))
d = json.load(open(os.path.join(HERE, "ctl_20261105/mean_traces.json"))); g = d["grid"]
lo = min(min(x) for k in ("px4_all", "ppo_all") for x in d[k]); hi = max(max(x) for k in ("px4_all", "ppo_all") for x in d[k])
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"  # Noto Sans CJK (Korean glyphs)
if os.path.exists(FONT):
    font_manager.fontManager.addfont(FONT); plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
INK, MUTED, GRID, BG = "#1b2430", "#6a7179", "#e3e6ea", "#fcfcfb"
C = {"px4": "#d9542b", "ppo": "#2a78d6"}; LBL = {"px4": "PX4 단독", "ppo": "PPO ④"}
fig, ax = plt.subplots(figsize=(9, 4.6), dpi=160); fig.patch.set_facecolor(BG); ax.set_facecolor(BG)
ax.axvspan(0, 15, color="#eef2f6", zorder=0); ax.text(7.5, hi + 0.9, "선풍기 ON (15 s)", ha="center", color=MUTED, fontsize=10)
ax.axvspan(15, 20, color="#f6f3ee", zorder=0); ax.text(17.5, hi + 0.9, "OFF + 5 s", ha="center", color=MUTED, fontsize=10)
ax.axhline(0, color=MUTED, lw=0.8)
for k in ("px4", "ppo"):
    for tr in d[k + "_all"]:
        ax.plot(g, tr, color=C[k], lw=1, alpha=0.28)
    ax.plot(g, d[k], color=C[k], lw=2.2, label=LBL[k] + " (4회 평균)")
    i = max(range(len(g)), key=lambda j: abs(d[k][j]) if 0 <= g[j] <= 20 else -1)
    ax.annotate("%s 평균 최대 %.1f cm" % (LBL[k], abs(d[k][i])), (g[i], d[k][i]), xytext=(8, 6), textcoords="offset points", color=INK, fontsize=9.5)
ax.set_xlim(g[0], g[-1]); ax.set_ylim(lo - 0.5, hi + 1.6)
ax.set_xlabel("선풍기 켠 뒤 시간 (s)", color=INK); ax.set_ylabel("좌우 밀림 (cm, 태그 기준)", color=INK)
ax.grid(axis="y", color=GRID, lw=0.8); ax.set_axisbelow(True)
for s in ("top", "right"): ax.spines[s].set_visible(False)
for s in ("left", "bottom"): ax.spines[s].set_color(MUTED)
ax.tick_params(colors=MUTED); ax.legend(loc="upper left", bbox_to_anchor=(0, 1.13), ncol=2, frameon=False, labelcolor=INK)
fig.tight_layout(); fig.savefig(sys.argv[1], facecolor=BG); print("->", sys.argv[1])
