"""그림과 조건 비교.

  plot_flight(ts, metrics, out_png)        한 비행: 평면 궤적 + 진행거리 대 드리프트 + 시간 대 yaw
  compare(summary_csv_paths, out_dir)      여러 비행 요약 -> 조건별 상자그림, Welch t 검정, 표본 수 산정
"""

import os

import numpy as np
import pandas as pd

from marker_drift.drift import sample_size_two_group


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    # 설치된 한글 글꼴만 고른다(없는 이름을 넣으면 findfont 경고가 프레임마다 쏟아진다)
    installed = {f.name for f in font_manager.fontManager.ttflist}
    family = [n for n in ("Malgun Gothic", "AppleGothic", "NanumGothic") if n in installed]
    plt.rcParams["font.family"] = family + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    return plt


def plot_flight(ts, metrics, out_png, layout=None, title=""):
    plt = _plt()
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    ax = axes[0, 0]
    ax.plot(ts["x"], ts["y"], ".-", ms=3, label="tag (P_tag)")
    ax.plot(ts["vio_x"], ts["vio_y"], ".-", ms=3, label="VIO -> M")
    if "cmd_x" in ts.columns:
        ax.plot(ts["cmd_x"], ts["cmd_y"], "--", lw=1, label="cmd")
    if layout is not None:
        for mid, mk in layout.markers.items():
            ax.plot(mk["x"], mk["y"], "ks", ms=4)
            ax.annotate(str(mid), (mk["x"], mk["y"]), textcoords="offset points", xytext=(0, 4), ha="center", fontsize=7)
    ax.set_xlabel("x [m] (progress)")
    ax.set_ylabel("y [m] (toward wall)")
    ax.set_title("top view")
    ax.axis("equal")
    ax.legend(fontsize=8)

    ax = axes[0, 1]
    for c, lab in (("e_x", "x"), ("e_y", "y"), ("e_z", "z")):
        ax.plot(ts["dist_m"], ts[c] * 100, label=lab)
    ax.plot(ts["dist_m"], ts["e_h"] * 100, "k", lw=1.5, label="|horizontal|")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_xlabel("distance traveled [m]")
    ax.set_ylabel("drift e = VIO - tag [cm]")
    ax.set_title("drift vs distance  (E_h = {:.1f} cm)".format(metrics.get("E_h", float("nan")) * 100))
    ax.legend(fontsize=8)

    ax = axes[1, 0]
    ax.plot(ts["t"] - ts["t"].iloc[0], ts["yaw_deg"], label="tag yaw (0 = square to wall)")
    ax.plot(ts["t"] - ts["t"].iloc[0], ts["vio_yaw_deg"], label="VIO yaw -> M")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("deg")
    ax.set_title("yaw")
    ax.legend(fontsize=8)

    ax = axes[1, 1]
    if "d_h" in ts.columns:
        ax.plot(ts["t"] - ts["t"].iloc[0], ts["d_h"] * 100, label="path deviation |tag - cmd|")
    ax.plot(ts["t"] - ts["t"].iloc[0], ts["e_h"] * 100, label="drift |VIO - tag|")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("cm")
    ax.set_title("horizontal errors")
    ax.legend(fontsize=8)

    fig.suptitle(title or os.path.basename(out_png))
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def load_summaries(paths):
    frames = []
    for p in paths:
        df = pd.read_csv(p)
        frames.append(df)
    return pd.concat(frames, ignore_index=True)


def compare(summary_df, out_dir, baseline="A", metrics=("E_h", "E_x", "E_y", "E_z", "dev_rms_h_m"),
            delta_m=0.03, alpha=0.05, power=0.8):
    """summary_df 에 condition 컬럼과 지표 컬럼이 있어야 한다. 결과: 표(CSV) + 상자그림(PNG)."""
    os.makedirs(out_dir, exist_ok=True)
    from scipy import stats
    conds = sorted(summary_df["condition"].dropna().unique().tolist())
    rows = []
    for met in metrics:
        if met not in summary_df.columns:
            continue
        for c in conds:
            v = pd.to_numeric(summary_df.loc[summary_df["condition"] == c, met], errors="coerce").dropna()
            row = {"metric": met, "condition": c, "n": int(len(v)),
                   "mean": v.mean() if len(v) else np.nan, "std": v.std(ddof=1) if len(v) > 1 else np.nan,
                   "median": v.median() if len(v) else np.nan}
            if c != baseline and baseline in conds:
                b = pd.to_numeric(summary_df.loc[summary_df["condition"] == baseline, met], errors="coerce").dropna()
                if len(v) > 1 and len(b) > 1:
                    t, p = stats.ttest_ind(v, b, equal_var=False)
                    row["welch_t_vs_{}".format(baseline)] = t
                    row["welch_p_vs_{}".format(baseline)] = p
                    pooled_sd = float(np.sqrt((v.var(ddof=1) + b.var(ddof=1)) / 2))
                    row["n_per_group_for_delta_{}m".format(delta_m)] = sample_size_two_group(pooled_sd, delta_m, alpha, power)
            rows.append(row)
    table = pd.DataFrame(rows)
    table.to_csv(os.path.join(out_dir, "compare_table.csv"), index=False)

    plt = _plt()
    avail = [m for m in metrics if m in summary_df.columns]
    fig, axes = plt.subplots(1, len(avail), figsize=(4 * len(avail), 4), squeeze=False)
    for ax, met in zip(axes[0], avail):
        data = [pd.to_numeric(summary_df.loc[summary_df["condition"] == c, met], errors="coerce").dropna().to_numpy() * 100 for c in conds]
        try:
            ax.boxplot(data, tick_labels=conds, showmeans=True)   # matplotlib >= 3.9
        except TypeError:
            ax.boxplot(data, labels=conds, showmeans=True)
        for i, dvals in enumerate(data):
            ax.plot(np.full(len(dvals), i + 1) + np.random.uniform(-0.08, 0.08, len(dvals)), dvals, "k.", alpha=0.6)
        ax.set_title(met)
        ax.set_ylabel("cm")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "compare_box.png"), dpi=130)
    plt.close(fig)
    return table
