#!/usr/bin/env python3
"""Compare all lateral controllers on the surrogate evaluation grid (v3_marker, 2026-10-29).
  venv/bin/python experiments/eval_all.py   -> results/eval.json, results/eval.md, results/eval_*.png
Controllers: C0 none, C1 estimator (v2), C2 rule (default), C2r rule_rel, C3 rule_opt (optimised), C4a mlp residual,
C4b mlp pure. Sensing: tof (continuous face distance) and marker (AprilTags on unit-end posts only).
Evaluation grid = surrogate.scenario_grid (12 scenarios x 10 seeds), disjoint from the optimisation/RL training draws."""
import json, os
import numpy as np
from multiprocessing import Pool
from common import ROOT, S, C
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams["font.family"] = "Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"] = False
R = os.path.join(ROOT, "results")


def load(p):
    p = os.path.join(R, p)
    return json.load(open(p)) if os.path.exists(p) else None


def controllers(sensing):
    out = [("C0 보상 없음", "none", {}), ("C1 연속 추정(v2)", "estimator", {}), ("C2 정적 규칙", "rule", {}),
           ("C2r 상대 규칙", "rule_rel", {})]
    ro = load("rule_opt_%s.json" % sensing)
    if ro:
        out.append(("C3 최적화 규칙", "rule", ro["best"]))
    rt = load("rule_opt_tof.json")
    if sensing == "tof":
        if os.path.exists(os.path.join(R, "policy_residual.npz")):
            out.append(("C4a 규칙+강화학습", "mlp", {"weights": os.path.join(R, "policy_residual.npz"), "residual": True,
                                                  "rule_params": rt["best"] if rt else {}}))
        if os.path.exists(os.path.join(R, "policy_pure.npz")):
            out.append(("C4b 강화학습 단독", "mlp", {"weights": os.path.join(R, "policy_pure.npz")}))
    return out


def job(arg):
    label, name, kw, sc = arg
    m, _ = S.run(sc, C.make(name, **kw))
    m.update(ctrl=label, scen=sc.name, sensing=sc.sensing, seed=sc.seed)
    return m


if __name__ == "__main__":
    rows = []
    for sensing in ("tof", "marker"):
        scs = S.scenario_grid(range(10))
        for sc in scs:
            sc.sensing = sensing
        jobs = [(lab, nm, kw, sc) for lab, nm, kw in controllers(sensing) for sc in scs]
        with Pool(24) as p:
            rows += p.map(job, jobs, chunksize=8)
    json.dump(rows, open(os.path.join(R, "eval.json"), "w"))
    agg = {}
    for r in rows:
        agg.setdefault((r["sensing"], r["ctrl"]), []).append(r)
    lines = ["| 센서 | 제어 | 최대 오차 평균 cm | 최대 오차 95% cm | RMS 평균 cm | ±3 cm 유지율 | 접촉률 | 보상 횟수 | 방향 바뀜 | 총 이동 m |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    summ = {}
    for (sn, cl), rs in agg.items():
        mx = np.array([r["max_all"] for r in rs]); s = {
            "max_mean": mx.mean() * 100, "max_p95": np.percentile(mx, 95) * 100, "rms": np.mean([r["rms"] for r in rs]) * 100,
            "within3": np.mean([r["within3"] for r in rs]), "contact": np.mean([r["contact"] for r in rs]),
            "n_corr": np.mean([r["n_corr"] for r in rs]), "rev": np.mean([r["reversals"] for r in rs]), "travel": np.mean([r["travel"] for r in rs])}
        summ["%s|%s" % (sn, cl)] = s
        lines.append("| %s | %s | %.1f | %.1f | %.2f | %.0f%% | %.0f%% | %.0f | %.0f | %.2f |" % (
            "ToF 연속" if sn == "tof" else "마커만", cl, s["max_mean"], s["max_p95"], s["rms"], s["within3"] * 100, s["contact"] * 100,
            s["n_corr"], s["rev"], s["travel"]))
    # per scenario (tof)
    scen_names = sorted({r["scen"] for r in rows})
    lines += ["", "ToF 연속, 시나리오별 최대 오차 평균 cm", "", "| 제어 | " + " | ".join(scen_names) + " |", "|---|" + "---|" * len(scen_names)]
    for lab, _, _ in controllers("tof"):
        vals = []
        for sn in scen_names:
            rs = [r for r in rows if r["sensing"] == "tof" and r["ctrl"] == lab and r["scen"] == sn]
            vals.append("%.1f%s" % (np.mean([r["max_all"] for r in rs]) * 100, "✕" if any(r["contact"] for r in rs) else ""))
        lines.append("| %s | %s |" % (lab, " | ".join(vals)))
    open(os.path.join(R, "eval.md"), "w").write("\n".join(lines) + "\n")
    json.dump(summ, open(os.path.join(R, "eval_summary.json"), "w"), indent=1, ensure_ascii=False)
    print("\n".join(lines))
    # accuracy vs over-reaction
    fig, axs = plt.subplots(1, 2, figsize=(13, 5), dpi=110)
    for k, sn in enumerate(("tof", "marker")):
        ax = axs[k]
        for key, s in summ.items():
            s2, cl = key.split("|")
            if s2 != sn:
                continue
            ax.scatter(s["n_corr"], s["max_mean"], s=60)
            ax.annotate(cl, (s["n_corr"], s["max_mean"]), xytext=(5, 4), textcoords="offset points", fontsize=9)
        ax.set_xscale("symlog"); ax.set_xlabel("비행당 보상 횟수 (적을수록 덜 과민)"); ax.set_ylabel("최대 오차 평균 (cm)")
        ax.set_title("ToF 연속 탐지" if sn == "tof" else "마커만 (기둥 4 m 간격)"); ax.grid(alpha=.3)
    fig.suptitle("정확도 vs 과반응 — 대리 모델, 12 시나리오 × 10 시드"); fig.tight_layout(); fig.savefig(os.path.join(R, "eval_tradeoff.png"))
