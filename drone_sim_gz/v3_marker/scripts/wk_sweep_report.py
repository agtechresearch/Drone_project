#!/usr/bin/env python3
"""Error-magnitude sweep summary (2026-10-29). Run with the cad venv after wk_report.py has written metrics.json per run:
  ~/starling_sim/cad/venv/bin/python wk_sweep_report.py <out_dir> <list> [<list> ...]
<list> lines: "<face> <cond> <off|on> <run dir>" (cond yaw<NN> = NN/10 deg, drift<NN> = NN/1000 m/s, step<NN> = NN cm, none)
Writes sweep_summary.png (3 panels: heading / drift / step; max error during legs and over the whole survey, rotor
clearance; wall-keep off vs on), sweep.json and sweep.md.
"""
import json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
rows = []; not_eval = []       # 2026-10-31 (codex missing-metric-status): planned runs that produced nothing stay visible
for lst in sys.argv[2:]:
    for line in open(lst):
        p = line.split()
        if len(p) < 4 or p[3] == "MISSING":
            not_eval.append({"line": line.strip(), "status": "not run / start failed"}); continue
        f, c, w, d = p
        mp = os.path.join(d, "metrics.json")
        if not os.path.exists(mp):
            not_eval.append({"line": line.strip(), "dir": d, "status": "no metrics.json (evaluator failed or not run)"}); continue
        m = json.load(open(mp))
        if c == "none":
            kinds = [("yaw", 0.0), ("drift", 0.0), ("step", 0.0)]
        elif c.startswith("yaw"):
            kinds = [("yaw", int(c[3:]) / 10.0)]
        elif c.startswith("drift"):
            kinds = [("drift", int(c[5:]) / 10.0)]          # cm/s
        elif c.startswith("step"):
            kinds = [("step", float(int(c[4:])))]          # cm
        else:
            continue
        cl = [m[k] for k in ("rotor_clear_min_front", "rotor_clear_min_back") if m.get(k) is not None]
        for kind, mag in kinds:
            rows.append({"kind": kind, "mag": mag, "wk": w, "dir": d, "verdict": m.get("grid_verify"),
                         "err_legs": m.get("err_max_abs_legs"), "err_all": m.get("err_max_abs"),
                         "clear": min(cl) if cl else None,          # was min(..., 9): a missing value looked like 9 m of clearance
                         "clear_missing": None if cl else "no rotor clearance in metrics.json",
                         "status": "evaluated" if (m.get("err_max_abs") is not None and cl) else "partly evaluated",
                         "final_c": m.get("final_c"), "updates": m.get("wk_updates")})
rows.sort(key=lambda r: (r["kind"], r["mag"], r["wk"]))
json.dump(rows, open(os.path.join(out, "sweep.json"), "w"), indent=1, ensure_ascii=False)
json.dump({"evaluated_rows": sum(r["status"] == "evaluated" for r in rows),
           "partly_evaluated_rows": sum(r["status"] != "evaluated" for r in rows), "not_evaluated": not_eval},
          open(os.path.join(out, "sweep_status.json"), "w"), indent=1, ensure_ascii=False)
print("sweep: %d rows evaluated, %d partly, %d planned runs not evaluated" % (
    sum(r["status"] == "evaluated" for r in rows), sum(r["status"] != "evaluated" for r in rows), len(not_eval)))
KIND = {"yaw": ("VIO 헤딩 오정렬", "헤딩 오차 (°)"), "drift": ("VIO 드리프트", "드리프트 속도 (cm/s)"), "step": ("레그 중 계단 오차", "위치 점프 (cm)")}
fig, axs = plt.subplots(2, 3, figsize=(15, 8.2), dpi=110, sharey="row")
for k, kind in enumerate(("yaw", "drift", "step")):
    ax, ax2 = axs[0][k], axs[1][k]
    WL = {"off": "끔", "on": "켬", "onrot": "켬+회전 보정"}
    for w, col, mk in (("off", "#c0392b", "s"), ("on", "#2e86c1", "o"), ("onrot", "#27ae60", "D")):
        rr = [r for r in rows if r["kind"] == kind and r["wk"] == w and r["err_all"] is not None and r["clear"] is not None]
        if not rr:
            continue
        x = [r["mag"] for r in rr]
        ax.plot(x, [r["err_all"] * 100 for r in rr], marker=mk, color=col, lw=2, label="보정 %s · 전체 최대" % WL[w])
        if w in ("on", "onrot"):
            ax.plot(x, [(r["err_legs"] if r["err_legs"] is not None else r["err_all"]) * 100 for r in rr], marker=mk, color=col, lw=1.4, ls="--", mfc="white", label="보정 %s · 레그 중 최대" % WL[w])
        ax2.plot(x, [r["clear"] * 100 for r in rr], marker=mk, color=col, lw=2, label="보정 %s" % WL[w])
        for r in rr:
            if r["verdict"] != "PASS":
                ax2.annotate("FAIL", (r["mag"], r["clear"] * 100), xytext=(0, -14), textcoords="offset points", ha="center", fontsize=8, color=col)
    ax.set_title(KIND[kind][0], fontsize=13); ax.grid(alpha=0.3)
    ax2.axhline(0, color="#d62728", lw=1.2); ax2.axhspan(-15, 0, color="#f7d9d6", zorder=0)
    ax2.set_xlabel(KIND[kind][1]); ax2.grid(alpha=0.3)
    if k == 0:
        ax.set_ylabel("통로 중앙 대비 최대 오차 (cm)"); ax2.set_ylabel("가까운 쪽 프로펠러 여유 최소 (cm)\n(0 미만 = 재배단 접촉)")
    ax.legend(fontsize=8.5, loc="upper left"); ax2.legend(fontsize=8.5, loc="lower left")
axs[1][0].set_ylim(-15, 30)
fig.suptitle("오차 크기 스윕 — C통로 서쪽 면 (진실값)", fontsize=14)
fig.tight_layout(rect=(0, 0, 1, 0.96)); fig.savefig(os.path.join(out, "sweep_summary.png")); plt.close(fig)
with open(os.path.join(out, "sweep.md"), "w") as f:
    f.write("| 오차 | 크기 | 보정 | 레그 중 최대 cm | 전체 최대 cm | 프로펠러 여유 최소 cm | 판정 |\n|---|---|---|---|---|---|---|\n")
    # 2026-10-31 codex missing-metric-status reply-2: a missing number stays visible as "없음" (never 9 m or 0), so a row
    # without a clearance is not read as a big safety margin; the verdict column then says why
    cm = lambda v: "%.1f" % (v * 100) if v is not None else "없음"   # noqa: E731
    for r in rows:
        f.write("| %s | %g | %s | %s | %s | %s | %s |\n" % (KIND[r["kind"]][0], r["mag"], r["wk"], cm(r["err_legs"]), cm(r["err_all"]),
                cm(r["clear"]), (r["verdict"] or "판정 없음") if (r["err_all"] is not None and r["clear"] is not None) else "%s (값 없음: 미평가)" % (r["verdict"] or "판정 없음")))
print(open(os.path.join(out, "sweep.md")).read())
