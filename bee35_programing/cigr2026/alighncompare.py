#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
논문 초록 시각화 자료 생성
사용법: python abstract_figures.py          # logs_/ 폴더 자동
        python abstract_figures.py [경로]
출력:   figures/ 폴더에 PNG 3장
"""

import csv, os, sys, math
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyArrowPatch
import matplotlib.font_manager as fm

LOG_DIR    = sys.argv[1] if len(sys.argv) > 1 else "logs_"
OUT_DIR    = "figures"
os.makedirs(OUT_DIR, exist_ok=True)

# ── 폰트 ──────────────────────────────────────────────────
ko = [f.name for f in fm.fontManager.ttflist
      if any(k in f.name for k in ['Malgun','NanumGothic','AppleGothic','Noto Sans CJK'])]
plt.rcParams['font.family'] = ko[0] if ko else 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False

# ── 팔레트 ────────────────────────────────────────────────
CA   = '#1D4ED8'   # Align — 딥 블루
CL   = '#B91C1C'   # LOITER — 딥 레드
CA_L = '#BFDBFE'
CL_L = '#FECACA'
BG   = '#FAFAFA'
DARK = '#0F172A'
MID  = '#475569'
LINE = '#CBD5E1'

# ── 데이터 로드 ────────────────────────────────────────────
def fix_nodetect(rows):
    lx, lp = None, None
    for r in rows:
        if r['err_x_px'] == '0' and r['err_pitch_px'] == '0':
            if lx: r['err_x_px'] = lx; r['err_pitch_px'] = lp
        else:
            lx, lp = r['err_x_px'], r['err_pitch_px']
    return rows

align_rt, loiter_rt = [], []
align_sum_dfs, loiter_sum_dfs = [], []

for f in sorted(os.listdir(LOG_DIR)):
    p = os.path.join(LOG_DIR, f)
    if not f.endswith('.csv'): continue
    if 'realtime' in f:
        with open(p, newline='', encoding='utf-8') as fh:
            rows = fix_nodetect(list(csv.DictReader(fh)))
        (loiter_rt if 'loiter' in f else align_rt).extend(rows)
    elif 'summary' in f:
        (loiter_sum_dfs if 'loiter' in f else align_sum_dfs).append(pd.read_csv(p))

AS = pd.concat(align_sum_dfs,  ignore_index=True)
LS = pd.concat(loiter_sum_dfs, ignore_index=True)

A_ex  = np.array([abs(float(r['err_x_px'])) for r in align_rt])
L_ex  = np.array([abs(float(r['err_x_px'])) for r in loiter_rt])
A_aln = np.array([int(r['aligned'])          for r in align_rt])
L_aln = np.array([int(r['aligned'])          for r in loiter_rt])

a_rate = A_aln.mean() * 100
l_rate = L_aln.mean() * 100
a_time = AS['align_time_s'].mean()
l_time = LS['align_time_s'].mean()
a_time_std = AS['align_time_s'].std()
l_time_std = LS['align_time_s'].std()
a_err  = A_ex.mean()
l_err  = L_ex.mean()

# ═══════════════════════════════════════════════════════════
# Fig 1 — 핵심 3지표 비교 (논문 제출용 단일 패널)
# ═══════════════════════════════════════════════════════════
fig = plt.figure(figsize=(14, 5), facecolor=BG)
fig.patch.set_facecolor(BG)

gs = gridspec.GridSpec(1, 3, figure=fig,
                       left=0.06, right=0.97,
                       top=0.82, bottom=0.14,
                       wspace=0.38)

def spine_clean(ax):
    ax.set_facecolor(BG)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color(LINE)
    ax.spines['bottom'].set_color(LINE)
    ax.tick_params(colors=MID, labelsize=10)
    ax.yaxis.label.set_color(MID)
    ax.grid(axis='y', color=LINE, linewidth=0.8, zorder=0)

# ── (a) 정렬 성공률 ────────────────────────────────────────
ax1 = fig.add_subplot(gs[0])
spine_clean(ax1)

bars = ax1.bar(['Align', 'LOITER'], [a_rate, l_rate],
               color=[CA, CL], width=0.48, zorder=3,
               edgecolor='white', linewidth=1.2)

for bar, v in zip(bars, [a_rate, l_rate]):
    ax1.text(bar.get_x() + bar.get_width()/2,
             bar.get_height() + 1.2,
             f'{v:.1f}%', ha='center', va='bottom',
             fontsize=13, fontweight='bold', color=DARK)

ax1.set_ylim(0, 60)
ax1.set_ylabel('Alignment Rate (%)', fontsize=10)
ax1.set_title('(a) 정렬 성공률', fontsize=12, fontweight='bold',
              color=DARK, pad=10)

diff_rate = a_rate - l_rate
ax1.annotate('', xy=(1, l_rate+1), xytext=(0, a_rate+1),
             arrowprops=dict(arrowstyle='->', color=CA, lw=1.5,
                             connectionstyle='arc3,rad=-0.25'))
ax1.text(0.5, max(a_rate, l_rate) * 0.62,
         f'+{diff_rate:.1f}%p', ha='center', fontsize=10,
         color=CA, fontweight='bold')

# ── (b) 평균 정렬 시간 ────────────────────────────────────
ax2 = fig.add_subplot(gs[1])
spine_clean(ax2)

bars2 = ax2.bar(['Align', 'LOITER'], [a_time, l_time],
                color=[CA, CL], width=0.48, zorder=3,
                yerr=[a_time_std, l_time_std],
                error_kw=dict(elinewidth=2, capsize=7,
                              ecolor='#64748B', capthick=2),
                edgecolor='white', linewidth=1.2)

for bar, v, s in zip(bars2, [a_time, l_time], [a_time_std, l_time_std]):
    ax2.text(bar.get_x() + bar.get_width()/2,
             bar.get_height() + s + 0.12,
             f'{v:.2f}s', ha='center', va='bottom',
             fontsize=13, fontweight='bold', color=DARK)

ax2.set_ylim(0, max(a_time, l_time) + max(a_time_std, l_time_std) + 1.5)
ax2.set_ylabel('Mean Alignment Time (s)', fontsize=10)
ax2.set_title('(b) 평균 정렬 소요 시간', fontsize=12, fontweight='bold',
              color=DARK, pad=10)

ax2.text(0.5, 0.94, f'Align σ={a_time_std:.2f}s  |  LOITER σ={l_time_std:.2f}s',
         transform=ax2.transAxes, ha='center', fontsize=8.5,
         color=MID, style='italic')

# ── (c) 좌우 오차 감소 ────────────────────────────────────
ax3 = fig.add_subplot(gs[2])
spine_clean(ax3)

bars3 = ax3.bar(['Align', 'LOITER'], [a_err, l_err],
                color=[CA, CL], width=0.48, zorder=3,
                edgecolor='white', linewidth=1.2)

for bar, v in zip(bars3, [a_err, l_err]):
    ax3.text(bar.get_x() + bar.get_width()/2,
             bar.get_height() + 1.5,
             f'{v:.1f}px', ha='center', va='bottom',
             fontsize=13, fontweight='bold', color=DARK)

ax3.set_ylim(0, l_err * 1.35)
ax3.set_ylabel('Mean |err_x| (px)', fontsize=10)
ax3.set_title('(c) 좌우 오차', fontsize=12, fontweight='bold',
              color=DARK, pad=10)

reduction = (l_err - a_err) / l_err * 100
ax3.annotate('', xy=(0, a_err), xytext=(1, l_err),
             arrowprops=dict(arrowstyle='->', color='#0EA5E9', lw=2))
ax3.text(0.5, (a_err + l_err) / 2,
         f'▼ {reduction:.1f}%', ha='center', fontsize=11,
         color='#0EA5E9', fontweight='bold',
         bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                   edgecolor='#0EA5E9', linewidth=1.2))

# 범례
legend = [mpatches.Patch(color=CA, label='Align Algorithm'),
          mpatches.Patch(color=CL, label='LOITER Only')]
fig.legend(handles=legend, loc='upper center',
           ncol=2, fontsize=11, frameon=False,
           bbox_to_anchor=(0.5, 0.97),
           handlelength=1.4, handleheight=0.9)

fig.text(0.5, 0.01,
         'Fig. 1  Align Algorithm vs LOITER Mode — Key Performance Metrics',
         ha='center', fontsize=9, color=MID, style='italic')

out1 = os.path.join(OUT_DIR, 'fig1_key_metrics.png')
plt.savefig(out1, dpi=200, bbox_inches='tight', facecolor=BG)
plt.close()
print(f"저장: {out1}")

# ═══════════════════════════════════════════════════════════
# Fig 2 — 정렬 시간 분포 (박스플롯 + 개별점)
# ═══════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(8, 6), facecolor=BG)
ax.set_facecolor(BG)

a_t = AS['align_time_s'].values
l_t = LS['align_time_s'].values

# 박스플롯
bp = ax.boxplot([a_t, l_t],
                positions=[1, 2], widths=0.35,
                patch_artist=True,
                tick_labels=['Align\nAlgorithm', 'LOITER\nOnly'],
                medianprops=dict(color='white', linewidth=2.5),
                whiskerprops=dict(color=MID, linewidth=1.5, linestyle='--'),
                capprops=dict(color=MID, linewidth=2),
                flierprops=dict(marker='o', markersize=4,
                                markerfacecolor=MID, alpha=0.4))
bp['boxes'][0].set_facecolor(CA); bp['boxes'][0].set_alpha(0.75)
bp['boxes'][1].set_facecolor(CL); bp['boxes'][1].set_alpha(0.75)

# 개별 데이터 점 (jitter)
rng = np.random.default_rng(42)
for i, (data, c) in enumerate([(a_t, CA), (l_t, CL)], 1):
    jitter = rng.uniform(-0.08, 0.08, len(data))
    ax.scatter(i + jitter, data, color=c, alpha=0.35, s=22, zorder=4)

# 평균 표시
for i, (data, c) in enumerate([(a_t, CA), (l_t, CL)], 1):
    ax.scatter(i, np.mean(data), marker='D', s=70,
               color='white', edgecolors=c, linewidths=2.5, zorder=5)

# 통계 주석
for i, data in enumerate([a_t, l_t], 1):
    ax.text(i, data.max() + 0.4,
            f'μ={np.mean(data):.2f}s\nσ={np.std(data):.2f}s\nmed={np.median(data):.2f}s',
            ha='center', fontsize=9, color=MID,
            bbox=dict(boxstyle='round,pad=0.4', facecolor='white',
                      edgecolor=LINE, linewidth=1))

ax.set_ylabel('Alignment Time (s)', fontsize=11, color=MID)
ax.set_title('Fig. 2  Alignment Time Distribution\n'
             'Align Algorithm vs LOITER Mode',
             fontsize=12, fontweight='bold', color=DARK, pad=12)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['left'].set_color(LINE)
ax.spines['bottom'].set_color(LINE)
ax.tick_params(colors=MID, labelsize=11)
ax.grid(axis='y', color=LINE, linewidth=0.8)
ax.set_xlim(0.5, 2.5)

diamond = mpatches.Patch(facecolor='white', edgecolor=MID,
                         linewidth=1.5, label='Mean (◆)')
ax.legend(handles=[diamond], loc='upper right', fontsize=9,
          frameon=True, edgecolor=LINE)

fig.text(0.5, 0.01,
         '개별 점: 각 시도 / ◆: 평균 / 박스: IQR / 수염: 1.5×IQR',
         ha='center', fontsize=8.5, color=MID, style='italic')

out2 = os.path.join(OUT_DIR, 'fig2_time_boxplot.png')
plt.savefig(out2, dpi=200, bbox_inches='tight', facecolor=BG)
plt.close()
print(f"저장: {out2}")

# ═══════════════════════════════════════════════════════════
# Fig 3 — 좌우 오차 누적분포 (CDF) + 임계선
# ═══════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(8, 6), facecolor=BG)
ax.set_facecolor(BG)

for data, c, label in [(A_ex, CA, f'Align  (μ={a_err:.1f}px)'),
                        (L_ex, CL, f'LOITER (μ={l_err:.1f}px)')]:
    sd = np.sort(data)
    cdf = np.arange(1, len(sd)+1) / len(sd)
    ax.plot(sd, cdf, color=c, lw=2.5, label=label)
    ax.axvline(np.mean(data), color=c, ls='--', lw=1.5, alpha=0.7)

# 50px 임계선
for thr, lbl in [(50, '50px'), (100, '100px')]:
    ax.axvline(thr, color='#94A3B8', ls=':', lw=1.5)
    ax.text(thr+2, 0.05, lbl, fontsize=8.5, color='#94A3B8')

# 오차 감소 강조 화살표
y_mark = 0.55
ax_a = np.interp(y_mark, np.arange(1, len(np.sort(A_ex))+1)/len(A_ex), np.sort(A_ex))
ax_l = np.interp(y_mark, np.arange(1, len(np.sort(L_ex))+1)/len(L_ex), np.sort(L_ex))
ax.annotate('', xy=(ax_a, y_mark), xytext=(ax_l, y_mark),
            arrowprops=dict(arrowstyle='->', color='#0EA5E9', lw=2))
ax.text((ax_a+ax_l)/2, y_mark+0.03,
        f'▼{reduction:.1f}%', ha='center', fontsize=10,
        color='#0EA5E9', fontweight='bold')

ax.set_xlabel('|err_x| (px)', fontsize=11, color=MID)
ax.set_ylabel('Cumulative Probability', fontsize=11, color=MID)
ax.set_title('Fig. 3  Lateral Error CDF\nAlign Algorithm vs LOITER Mode',
             fontsize=12, fontweight='bold', color=DARK, pad=12)
ax.legend(fontsize=10, frameon=True, edgecolor=LINE, loc='lower right')
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['left'].set_color(LINE)
ax.spines['bottom'].set_color(LINE)
ax.tick_params(colors=MID)
ax.grid(alpha=0.35, color=LINE)
ax.set_xlim(0, np.percentile(np.concatenate([A_ex, L_ex]), 98))

fig.text(0.5, 0.01,
         '세로 점선: 각 분포의 평균값 / 수직선: 오차 임계값',
         ha='center', fontsize=8.5, color=MID, style='italic')

out3 = os.path.join(OUT_DIR, 'fig3_error_cdf.png')
plt.savefig(out3, dpi=200, bbox_inches='tight', facecolor=BG)
plt.close()
print(f"저장: {out3}")

# ── 콘솔 확인 ─────────────────────────────────────────────
print("\n▶ 초록 수치 확인")
print(f"  정렬 성공률  : Align {a_rate:.1f}%  /  LOITER {l_rate:.1f}%")
print(f"  평균 정렬 시간: Align {a_time:.2f}s ± {a_time_std:.2f}  /  LOITER {l_time:.2f}s ± {l_time_std:.2f}")
print(f"  좌우 오차     : Align {a_err:.1f}px  /  LOITER {l_err:.1f}px  (감소 {reduction:.1f}%)")
print(f"\n  → figures/ 폴더에 PNG 3장 저장 완료")