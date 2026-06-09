#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
실험군(align) vs 대조군(loiter) 요약 결과 비교 분석.

사용법:
  python compare_transit.py
    → logs/ 폴더에서 align_transit_summary_*.csv, loiter_transit_summary_*.csv
       파일을 자동으로 모아 비교.

출력:
  - 콘솔: 그룹별 평균/표준편차, t-test 결과
  - figures/compare_transit_*.png: 막대그래프, 박스플롯
"""

import os
import glob
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

LOGS_DIR = "logs"
FIG_DIR  = "figures"

# 비교할 지표 (요약 CSV 컬럼명)
METRICS = [
    ("goal_mean_err_x",     "도착지 평균 |좌우 오차| (px)"),
    ("goal_mean_err_pitch", "도착지 평균 |거리 오차| (px)"),
    ("goal_rms_err_x",      "도착지 좌우 RMS 오차 (px)"),
    ("goal_rms_err_pitch",  "도착지 거리 RMS 오차 (px)"),
    ("goal_aligned_ratio",  "측정 구간 정렬 비율"),
    ("goal_rms_vib",        "측정 구간 진동 RMS"),
]


def load_group(group_name):
    """logs/{group_name}_transit_summary_*.csv 파일을 모두 모아 DataFrame으로."""
    pattern = os.path.join(LOGS_DIR, f"{group_name}_transit_summary_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"[WARN] '{group_name}' 그룹의 요약 파일을 찾지 못함: {pattern}")
        return pd.DataFrame()

    dfs = []
    for f in files:
        try:
            df = pd.read_csv(f)
            df["source_file"] = os.path.basename(f)
            dfs.append(df)
        except Exception as e:
            print(f"[WARN] {f} 읽기 실패: {e}")

    return pd.concat(dfs, ignore_index=True) if dfs else pd.DataFrame()


def print_summary(label, df):
    print(f"\n=== {label} (n={len(df)}, 성공={int(df['success'].sum())}) ===")
    if len(df) == 0:
        return

    # 성공한 실험만 통계
    succ = df[df["success"] == 1]
    if len(succ) == 0:
        print("  성공 사례 없음 — 통계 생략")
        return

    for col, label_kor in METRICS:
        if col not in succ.columns:
            continue
        vals = pd.to_numeric(succ[col], errors="coerce").dropna()
        if len(vals) == 0:
            continue
        print(f"  {label_kor:30s} : mean={vals.mean():7.2f}  "
              f"std={vals.std():6.2f}  median={vals.median():6.2f}  n={len(vals)}")


def plot_comparison(df_align, df_loiter, out_path):
    """그룹별 평균 막대그래프 + 표준편차 에러바."""
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    axes = axes.flatten()

    succ_a = df_align[df_align["success"] == 1] if len(df_align) else df_align
    succ_l = df_loiter[df_loiter["success"] == 1] if len(df_loiter) else df_loiter

    for ax, (col, label_kor) in zip(axes, METRICS):
        a_vals = pd.to_numeric(succ_a[col], errors="coerce").dropna() if col in succ_a.columns else pd.Series(dtype=float)
        l_vals = pd.to_numeric(succ_l[col], errors="coerce").dropna() if col in succ_l.columns else pd.Series(dtype=float)

        means = [a_vals.mean() if len(a_vals) else 0,
                 l_vals.mean() if len(l_vals) else 0]
        stds  = [a_vals.std()  if len(a_vals) > 1 else 0,
                 l_vals.std()  if len(l_vals) > 1 else 0]

        x = np.arange(2)
        colors = ["#2E86DE", "#EE5A24"]
        bars = ax.bar(x, means, yerr=stds, capsize=8,
                      color=colors, alpha=0.85, edgecolor="black")
        ax.set_xticks(x)
        ax.set_xticklabels([f"Align\n(n={len(a_vals)})", f"Loiter\n(n={len(l_vals)})"])
        ax.set_title(label_kor, fontsize=11)
        ax.grid(axis="y", alpha=0.3)

        # 막대 위에 값 표시
        for bar, val in zip(bars, means):
            if val > 0 or val < 0:
                ax.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
                        f"{val:.2f}", ha="center", va="bottom", fontsize=9)

    plt.tight_layout()
    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    print(f"\n[FIG] 저장: {out_path}")


def plot_boxplots(df_align, df_loiter, out_path):
    """주요 지표 박스플롯."""
    succ_a = df_align[df_align["success"] == 1] if len(df_align) else df_align
    succ_l = df_loiter[df_loiter["success"] == 1] if len(df_loiter) else df_loiter

    plot_metrics = [m for m in METRICS if m[0] in
                    ("goal_mean_err_x", "goal_rms_err_x", "goal_aligned_ratio", "goal_rms_vib")]

    fig, axes = plt.subplots(1, len(plot_metrics), figsize=(4 * len(plot_metrics), 5))
    if len(plot_metrics) == 1:
        axes = [axes]

    for ax, (col, label_kor) in zip(axes, plot_metrics):
        a_vals = pd.to_numeric(succ_a[col], errors="coerce").dropna() if col in succ_a.columns else pd.Series(dtype=float)
        l_vals = pd.to_numeric(succ_l[col], errors="coerce").dropna() if col in succ_l.columns else pd.Series(dtype=float)

        data = []
        labels = []
        if len(a_vals):
            data.append(a_vals.values); labels.append(f"Align (n={len(a_vals)})")
        if len(l_vals):
            data.append(l_vals.values); labels.append(f"Loiter (n={len(l_vals)})")

        if data:
            bp = ax.boxplot(data, labels=labels, patch_artist=True, widths=0.5)
            for patch, color in zip(bp["boxes"], ["#2E86DE", "#EE5A24"]):
                patch.set_facecolor(color)
                patch.set_alpha(0.6)
        ax.set_title(label_kor, fontsize=11)
        ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    print(f"[FIG] 저장: {out_path}")


def ttest_metrics(df_align, df_loiter):
    """t-test (Welch's). scipy 없으면 skip."""
    try:
        from scipy import stats
    except ImportError:
        print("\n[INFO] scipy가 없어 t-test 생략 (pip install scipy)")
        return

    succ_a = df_align[df_align["success"] == 1]
    succ_l = df_loiter[df_loiter["success"] == 1]

    print(f"\n=== Welch's t-test (Align vs Loiter) ===")
    for col, label_kor in METRICS:
        a = pd.to_numeric(succ_a[col], errors="coerce").dropna() if col in succ_a.columns else pd.Series(dtype=float)
        l = pd.to_numeric(succ_l[col], errors="coerce").dropna() if col in succ_l.columns else pd.Series(dtype=float)
        if len(a) < 2 or len(l) < 2:
            continue
        t, p = stats.ttest_ind(a, l, equal_var=False)
        sig = "**" if p < 0.01 else ("*" if p < 0.05 else "")
        print(f"  {label_kor:30s} : t={t:+6.2f}  p={p:.4f}  {sig}")


def main():
    df_align  = load_group("align")
    df_loiter = load_group("loiter")

    if len(df_align) == 0 and len(df_loiter) == 0:
        print("[ERROR] 어떤 요약 파일도 찾지 못함. logs/ 폴더와 파일명을 확인하세요.")
        sys.exit(1)

    print_summary("[Align] 실험군", df_align)
    print_summary("[Loiter] 대조군", df_loiter)
    ttest_metrics(df_align, df_loiter)

    if len(df_align) > 0 and len(df_loiter) > 0:
        os.makedirs(FIG_DIR, exist_ok=True)
        plot_comparison(df_align, df_loiter, os.path.join(FIG_DIR, "compare_transit_bars.png"))
        plot_boxplots  (df_align, df_loiter, os.path.join(FIG_DIR, "compare_transit_boxes.png"))


if __name__ == "__main__":
    main()
