#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
파일럿 비행 결과에서 조건별 드리프트 통계와 반복 수 n 을 계산한다 (docs/09 §5.4). 표준 라이브러리만 쓴다.

    python3 sim/pilot_stats.py logs/20261110_19*_i*            # 결과 폴더들 (각각 md_*_summary.json 포함)
    python3 sim/pilot_stats.py --delta 0.03 --metric out logs/...
    python3 sim/pilot_stats.py --csv pilot.csv logs/...         # 비행별 표를 CSV 로도 저장

지표(--metric):
    out    = legs_settled[out_settled].drift_along   (끝 태그(9) 앞 최종 드리프트 E, 기본)
    back   = legs_settled[back_settled].drift_along  (복귀 후 루프 폐합 드리프트)
    loop   = out − back                               (왕복 누적)
    standoff = out_settled.drift_standoff
    seg_out / seg_back = drift_log.csv 에서 진행 구간 [--seg-from, --seg-to] 태그 사이의 err_along 증가분
               (기본 2→6: 통로 조건 B 의 후방 장애물이 마커 2~6 구간에만 있으므로, 2026-11-10). 구간 밖은 A 와 같은 환경.
n ≈ 16·(s/Δ)²  (두 표본 t 검정, α 0.05, 검정력 0.8 근사). s 는 조건 안 비행 간 표본표준편차.
시뮬 결과의 s 는 VIO 가 아니라 코드·제어기 재현성("바닥 잡음")이므로 실기 n 산정에는 쓰지 않는다 (docs/12 2026-11-10).
"""
import argparse
import glob
import json
import math
import os
import sys


def load_drift_log(folder):
    """*_drift_log.csv → [(leg, truth_along, err_along)] (err 가 있는 행만)."""
    import csv
    fs = sorted(glob.glob(os.path.join(folder, "*_drift_log.csv")))
    rows = []
    if not fs:
        return rows
    with open(fs[-1], encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append((r.get("leg", ""), float(r["truth_along"]), float(r["err_along"])))
            except (ValueError, KeyError, TypeError):
                continue
    return rows


def segment_drift(rows, leg, from_m, to_m, window_m=0.25):
    """leg 구간에서 truth_along 이 from_m / to_m 주변(±window_m)인 프레임들의 err_along 평균 차 (to − from). 프레임이 없으면 None."""
    pts = [(t, e) for (l, t, e) in rows if l == leg]
    if len(pts) < 2:
        return None
    # 이동 중 단일 프레임은 피드백 지연 잡음(±2 cm)이 커서, 각 지점 ±window 안 프레임의 err 평균을 쓴다
    wa = [e for (t, e) in pts if abs(t - from_m) <= window_m]
    wb = [e for (t, e) in pts if abs(t - to_m) <= window_m]
    if not wa or not wb:
        return None
    return sum(wb) / len(wb) - sum(wa) / len(wa)


def load_run(folder):
    js = sorted(glob.glob(os.path.join(folder, "*_summary.json")))
    if not js:
        return None
    with open(js[-1], encoding="utf-8") as f:
        d = json.load(f)
    seq = d.get("sequence", {})
    p = seq.get("params") or d.get("params") or {}
    settled = dict((s.get("leg"), s) for s in seq.get("legs_settled", []))
    out = settled.get("out_settled", {})
    back = settled.get("back_settled", {})
    inj = {}
    ij = os.path.join(folder, "inject.json")
    if os.path.isfile(ij):
        try:
            inj = json.load(open(ij))
        except Exception:
            inj = {"raw": open(ij).read().strip()}
    return {
        "folder": os.path.basename(folder.rstrip("/\\")),
        "condition": str(p.get("condition", "?")),
        "run": p.get("run"),
        "altitude": p.get("altitude"),
        "completed": bool(seq.get("completed")),
        "landed_in_place": bool(seq.get("landed_in_place")),
        "align": (seq.get("align") or {}).get("state"),
        "out": out.get("drift_along"),
        "back": back.get("drift_along"),
        "standoff": out.get("drift_standoff"),
        "inject": inj,
        "spacing": float(p.get("tag_spacing", 0.5) or 0.5),
        "drift_rows": load_drift_log(folder),
    }


SEG = {"from": 2, "to": 6}     # main() 에서 --seg-from/--seg-to 로 덮어쓴다


def metric_value(r, metric):
    if metric in ("seg_out", "seg_back"):
        a, b = SEG["from"] * r["spacing"], SEG["to"] * r["spacing"]
        return segment_drift(r["drift_rows"], "out" if metric == "seg_out" else "back", a, b)
    if metric == "out":
        return r["out"]
    if metric == "back":
        return r["back"]
    if metric == "standoff":
        return r["standoff"]
    if metric == "loop":
        if r["out"] is None or r["back"] is None:
            return None
        return r["out"] - r["back"]
    raise ValueError(metric)


def mean_sd(xs):
    n = len(xs)
    if n == 0:
        return float("nan"), float("nan")
    m = sum(xs) / n
    if n < 2:
        return m, float("nan")
    return m, math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1))


def welch(a, b):
    """두 조건 평균 차의 Welch t 와 자유도 (p 값은 scipy 없이는 생략)."""
    ma, sa = mean_sd(a)
    mb, sb = mean_sd(b)
    if len(a) < 2 or len(b) < 2 or (sa == 0 and sb == 0):
        return float("nan"), float("nan"), mb - ma
    va, vb = sa * sa / len(a), sb * sb / len(b)
    t = (mb - ma) / math.sqrt(va + vb)
    df = (va + vb) ** 2 / (va * va / (len(a) - 1) + vb * vb / (len(b) - 1))
    return t, df, mb - ma


def fmt(x, w=7, p=3):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "-".rjust(w)
    return ("{:+." + str(p) + "f}").format(x).rjust(w)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--delta", type=float, default=0.03, help="검출하려는 최소 차이 Δ m (잠정 3 cm)")
    ap.add_argument("--metric", default="out", choices=["out", "back", "loop", "standoff", "seg_out", "seg_back"])
    ap.add_argument("--seg-from", type=int, default=2, help="seg_* 구간 시작 태그 (기본 2: 통로 B 장애물 시작)")
    ap.add_argument("--seg-to", type=int, default=6, help="seg_* 구간 끝 태그 (기본 6)")
    ap.add_argument("--csv", default="", help="비행별 표를 이 CSV 로 저장")
    ap.add_argument("--include-failed", action="store_true", help="completed 가 아닌 비행도 통계에 넣는다")
    a = ap.parse_args()
    SEG["from"], SEG["to"] = a.seg_from, a.seg_to

    folders = []
    for pat in a.folders:
        hits = sorted(glob.glob(pat))
        folders.extend(hits if hits else [pat])
    runs = []
    for f in folders:
        r = load_run(f)
        if r is None:
            print("요약 없음: {}".format(f), file=sys.stderr)
            continue
        runs.append(r)
    if not runs:
        raise SystemExit("읽은 비행이 없다")
    runs.sort(key=lambda r: (r["condition"], r["run"] if r["run"] is not None else 0, r["folder"]))

    print("비행별 ({}; seg = 태그 {}→{} 구간 err_along 증가분):".format(a.metric, a.seg_from, a.seg_to))
    print("  {:<22} {:>4} {:>3} {:>5} {:>9} {:>7}  {:>7} {:>7} {:>7} {:>7} {:>7} {:>7}  inject".format(
        "folder", "cond", "run", "alt", "completed", "align", "out", "back", "loop", "stoff", "seg_out", "seg_bk"))
    for r in runs:
        print("  {:<22} {:>4} {:>3} {:>5} {:>9} {:>7}  {} {} {} {} {} {}  {}".format(
            r["folder"], r["condition"], r["run"] if r["run"] is not None else "-",
            "{:.1f}".format(r["altitude"]) if r["altitude"] is not None else "-",
            "yes" if r["completed"] else ("LAND" if r["landed_in_place"] else "NO"),
            r["align"] or "-", fmt(r["out"]), fmt(r["back"]), fmt(metric_value(r, "loop")), fmt(r["standoff"]),
            fmt(metric_value(r, "seg_out")), fmt(metric_value(r, "seg_back")),
            json.dumps(r["inject"], ensure_ascii=False) if r["inject"] else ""))

    by = {}
    for r in runs:
        if not r["completed"] and not a.include_failed:
            continue
        v = metric_value(r, a.metric)
        if v is None:
            continue
        by.setdefault(r["condition"], []).append(v)

    print("\n조건별 ({}; Δ = {:.3f} m):".format(a.metric, a.delta))
    print("  {:>4} {:>3} {:>8} {:>8} {:>8} {:>8}  {:>6}".format("cond", "n", "mean", "s", "min", "max", "n_req"))
    for c in sorted(by):
        xs = by[c]
        m, s = mean_sd(xs)
        nreq = 16.0 * (s / a.delta) ** 2 if not math.isnan(s) and a.delta > 0 else float("nan")
        print("  {:>4} {:>3} {} {} {} {}  {}".format(c, len(xs), fmt(m, 8), fmt(s, 8), fmt(min(xs), 8), fmt(max(xs), 8),
                                                     "-" if math.isnan(nreq) else "{:.1f} -> {}".format(nreq, max(2, int(math.ceil(nreq))))))
    conds = sorted(by)
    if len(conds) >= 2:
        print("\n조건 간 비교 (Welch t; |t| 가 약 2 이상이면 의미 있는 차이 후보, 정확한 p 는 scipy 로):")
        base = "A" if "A" in by else conds[0]
        for c in conds:
            if c == base:
                continue
            t, df, diff = welch(by[base], by[c])
            print("  {} vs {}: diff {} m  t {}  df {}".format(base, c, fmt(diff, 7), fmt(t, 6, 2), fmt(df, 5, 1)))
    print("\n주의: 시뮬 VIO 는 진실값 릴레이라 여기의 s 는 코드·제어기 재현성이다. 실기 n 은 실기 파일럿 s 로 정한다.")

    if a.csv:
        import csv
        with open(a.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["folder", "condition", "run", "altitude", "completed", "landed_in_place", "align",
                        "drift_out", "drift_back", "drift_loop", "drift_standoff", "seg_out", "seg_back", "inject"])
            for r in runs:
                w.writerow([r["folder"], r["condition"], r["run"], r["altitude"], r["completed"], r["landed_in_place"],
                            r["align"], r["out"], r["back"], metric_value(r, "loop"), r["standoff"],
                            metric_value(r, "seg_out"), metric_value(r, "seg_back"),
                            json.dumps(r["inject"], ensure_ascii=False)])
        print("CSV 저장: {}".format(a.csv))


if __name__ == "__main__":
    main()
