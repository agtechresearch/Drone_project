#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
정렬 비행 vs 로이터 비행 비교 분석
- 5개씩 평균내서 비교
- logs_/ 폴더 기준

사용법:
  python compare.py           # logs_/ 폴더 자동
  python compare.py [경로]
"""

import csv, os, sys, math
import matplotlib
# matplotlib.use('Agg')  # Agg는 Linux 전용
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# ── 설정 ──────────────────────────────────
LOG_DIR    = sys.argv[1] if len(sys.argv) > 1 else "logs_"
OUTPUT_DIR = "visual"
os.makedirs(OUTPUT_DIR, exist_ok=True)

matplotlib.rcParams['font.family'] = 'DejaVu Sans'

# ── 미감지 선행값 유지 ─────────────────────
def fix_nodetect(rows):
    last_x, last_p = None, None
    for row in rows:
        ex, ep = row['err_x_px'], row['err_pitch_px']
        if ex == '0' and ep == '0':
            if last_x is not None:
                row['err_x_px']     = last_x
                row['err_pitch_px'] = last_p
        else:
            last_x, last_p = ex, ep
    return rows

def load_realtime(path):
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    return fix_nodetect(rows)

def analyze(rows):
    err_x   = [abs(float(r['err_x_px']))    for r in rows]
    err_p   = [abs(float(r['err_pitch_px'])) for r in rows]
    aligned = [int(r['aligned'])             for r in rows]
    vib     = [math.sqrt(float(r['vib_x'])**2 +
                         float(r['vib_y'])**2 +
                         float(r['vib_z'])**2) for r in rows]
    t       = [float(r['elapsed_s'])         for r in rows]
    return {
        'err_x':       err_x,
        'err_p':       err_p,
        'aligned':     aligned,
        'vib':         vib,
        't':           t,
        'mean_err_x':  np.mean(err_x),
        'mean_err_p':  np.mean(err_p),
        'std_err_x':   np.std(err_x),
        'std_err_p':   np.std(err_p),
        'align_ratio': np.mean(aligned) * 100,
        'mean_vib':    np.mean(vib),
        'std_vib':     np.std(vib),
    }

# ── 파일 로드 ──────────────────────────────
align_stats, loiter_stats = [], []
align_rows_all, loiter_rows_all = [], []

for f in sorted(os.listdir(LOG_DIR)):
    path = os.path.join(LOG_DIR, f)
    if 'realtime' not in f or not f.endswith('.csv'):
        continue
    rows = load_realtime(path)
    if rows is None:
        continue
    s = analyze(rows)
    if 'loiter' in f:
        loiter_stats.append(s)
        loiter_rows_all.extend(rows)
    else:
        align_stats.append(s)
        align_rows_all.extend(rows)

if not align_stats or not loiter_stats:
    print("[ERROR] 파일이 부족합니다.")
    sys.exit(1)

print(f"정렬 비행: {len(align_stats)}개 / 로이터 비행: {len(loiter_stats)}개")

# ── 그룹 평균 계산 ─────────────────────────
def gmean(stats, key): return np.mean([s[key] for s in stats])
def gstd(stats, key):  return np.std([s[key]  for s in stats])

metrics = {
    'mean_err_x':  'Mean Lateral Error (px)',
    'mean_err_p':  'Mean Fwd/Bwd Error (px)',
    'align_ratio': 'Alignment Rate (%)',
    'mean_vib':    'Mean Vibration (RMS)',
}

C_ALIGN  = '#2196F3'   # 파랑 — 정렬 비행
C_LOITER = '#F44336'   # 빨강 — 로이터 비행

# ════════════════════════════════════════════
# 그래프 1: 그룹 평균 막대 비교 (4개 지표)
# ════════════════════════════════════════════
fig, axes = plt.subplots(1, 4, figsize=(16, 5))
fig.suptitle('Alignment Control vs LOITER Only — Group Average', fontsize=14, fontweight='bold')

for ax, (key, label) in zip(axes, metrics.items()):
    a_mean = gmean(align_stats,  key)
    l_mean = gmean(loiter_stats, key)
    a_std  = gstd(align_stats,   key)
    l_std  = gstd(loiter_stats,  key)

    bars = ax.bar(['Align', 'LOITER'], [a_mean, l_mean],
                  color=[C_ALIGN, C_LOITER],
                  yerr=[a_std, l_std], capsize=6, width=0.5,
                  error_kw={'elinewidth': 1.5})

    # 값 표시
    for bar, val in zip(bars, [a_mean, l_mean]):
        ax.text(bar.get_x() + bar.get_width()/2,
                bar.get_height() + a_std * 0.1,
                f'{val:.1f}', ha='center', va='bottom', fontsize=10, fontweight='bold')

    ax.set_title(label, fontsize=11)
    ax.set_ylabel(label.split('(')[-1].replace(')', '') if '(' in label else '')
    ax.grid(axis='y', alpha=0.3)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

plt.tight_layout()
out1 = os.path.join(OUTPUT_DIR, 'compare_bar.png')
plt.savefig(out1, dpi=150, bbox_inches='tight')
plt.close()
print(f"저장: {out1}")

# ════════════════════════════════════════════
# 그래프 2: 개별 비행 산점도 (Align Rate vs 좌우오차)
# ════════════════════════════════════════════
fig, axes = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle('Individual Flight Distribution', fontsize=13, fontweight='bold')

for ax, (x_key, y_key, x_lbl, y_lbl) in zip(axes, [
    ('mean_err_x',  'align_ratio', 'Mean Lateral Error (px)', 'Alignment Rate (%)'),
    ('mean_err_p',  'align_ratio', 'Mean Fwd/Bwd Error (px)', 'Alignment Rate (%)'),
]):
    for i, s in enumerate(align_stats):
        ax.scatter(s[x_key], s[y_key], color=C_ALIGN,  s=80, zorder=3,
                   label='Align' if i == 0 else '')
    for i, s in enumerate(loiter_stats):
        ax.scatter(s[x_key], s[y_key], color=C_LOITER, s=80, marker='s', zorder=3,
                   label='LOITER' if i == 0 else '')

    ax.set_xlabel(x_lbl); ax.set_ylabel(y_lbl)
    ax.legend(); ax.grid(alpha=0.3)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

plt.tight_layout()
out2 = os.path.join(OUTPUT_DIR, 'compare_scatter.png')
plt.savefig(out2, dpi=150, bbox_inches='tight')
plt.close()
print(f"저장: {out2}")

# ════════════════════════════════════════════
# 그래프 3: 오차 분포 히스토그램 (좌우 / 앞뒤)
# ════════════════════════════════════════════
fig, axes = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle('Error Distribution Histogram', fontsize=13, fontweight='bold')

all_align_x  = [abs(float(r['err_x_px']))     for r in align_rows_all]
all_loiter_x = [abs(float(r['err_x_px']))     for r in loiter_rows_all]
all_align_p  = [abs(float(r['err_pitch_px'])) for r in align_rows_all]
all_loiter_p = [abs(float(r['err_pitch_px'])) for r in loiter_rows_all]

for ax, a_data, l_data, title in zip(axes,
    [all_align_x,  all_align_p],
    [all_loiter_x, all_loiter_p],
    ['Lateral Error |err_x| (px)', 'Fwd/Bwd Error |err_pitch| (px)']):

    bins = np.linspace(0, max(max(a_data), max(l_data)), 50)
    ax.hist(a_data, bins=bins, alpha=0.6, color=C_ALIGN,  label='Align',  density=True)
    ax.hist(l_data, bins=bins, alpha=0.6, color=C_LOITER, label='LOITER', density=True)
    ax.axvline(np.mean(a_data), color=C_ALIGN,  linestyle='--', linewidth=1.5,
               label=f'Align mean={np.mean(a_data):.1f}')
    ax.axvline(np.mean(l_data), color=C_LOITER, linestyle='--', linewidth=1.5,
               label=f'LOITER mean={np.mean(l_data):.1f}')
    ax.set_title(title); ax.set_xlabel('Error (px)'); ax.set_ylabel('Density')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

plt.tight_layout()
out3 = os.path.join(OUTPUT_DIR, 'compare_hist.png')
plt.savefig(out3, dpi=150, bbox_inches='tight')
plt.close()
print(f"저장: {out3}")

# ════════════════════════════════════════════
# 콘솔 요약 출력
# ════════════════════════════════════════════
print("\n" + "="*55)
print(f"{'지표':<22} {'정렬 비행':>12} {'로이터 비행':>12}")
print("-"*55)
for key, label in metrics.items():
    a = gmean(align_stats,  key)
    l = gmean(loiter_stats, key)
    diff = ((a - l) / l * 100) if l != 0 else 0
    print(f"  {label:<20} {a:>10.2f} {l:>10.2f}   ({diff:+.1f}%)")
print("="*55)