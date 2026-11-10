#!/usr/bin/env python3
"""run_sim_drift.sh 산출 폴더 하나를 요약한다 (서버 호스트 python3, 표준 라이브러리만).
  python3 summarize_run.py logs/<시각>_iN [...]
출력: 주입값, 완료/그 자리 착륙, 정렬·센터링, 레그 정지 사유, legs_settled 드리프트, drift_log 레그별 err 통계(평균·sd·끝값),
      events 의 geofence/경고, 진실값(gt.csv) 이동 범위. 여러 폴더를 주면 차례로."""
import csv
import glob
import json
import os
import statistics as st
import sys


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def summarize(d):
    print("=" * 100)
    print(os.path.basename(d.rstrip("/")))
    args = open(os.path.join(d, "args.txt")).read().strip().splitlines() if os.path.exists(os.path.join(d, "args.txt")) else []
    print("args  :", args[0] if args else "?")
    inj = os.path.join(d, "inject.json")
    print("inject:", open(inj).read().strip() if os.path.exists(inj) else "{}")
    sums = glob.glob(os.path.join(d, "md_*_summary.json"))
    if not sums:
        print("!! summary.json 없음 (비행 코드가 끝까지 못 감). mission.log 끝:")
        ml = os.path.join(d, "mission.log")
        if os.path.exists(ml):
            print("".join(open(ml, errors="replace").readlines()[-8:]))
        return
    s = json.load(open(sums[0]))
    seq = s.get("sequence", {})
    print("result: completed=%s landed_in_place=%s exception=%s initial_yaw=%s(%s)" % (
        seq.get("completed"), seq.get("landed_in_place"), seq.get("exception"),
        s.get("initial_yaw_deg"), s.get("initial_yaw_source")))
    a = seq.get("align") or {}
    print("align : %s  err %.2f deg  %s" % (a.get("state"), fnum(a.get("final_yaw_error_deg")), a.get("reason")))
    c = seq.get("center") or {}
    print("center: ok=%s iter=%s dx=%.3f standoff=%.3f %s" % (c.get("ok"), c.get("iterations"), fnum(c.get("dx")),
                                                             fnum(c.get("standoff")), c.get("reason")))
    for l in seq.get("legs", []):
        print("leg %-5s %-8s own_along %.3f truth %.3f -> drift_along %+.3f  standoff own %.3f truth %.3f  | %s" % (
            l.get("leg"), l.get("status"), fnum(l.get("own_along_b")), fnum(l.get("truth_along")),
            fnum(l.get("drift_along")), fnum(l.get("own_standoff")), fnum(l.get("truth_standoff")), l.get("reason")))
    for l in seq.get("legs_settled", []):
        print("settled %-13s drift_along %+.3f  drift_standoff %+.3f  (truth_along %.3f, ids %s)" % (
            l.get("leg"), fnum(l.get("drift_along")), fnum(l.get("drift_standoff")), fnum(l.get("truth_along")),
            l.get("truth_ids")))
    # 스테이지 표
    for stg in s.get("stages", []) if isinstance(s.get("stages"), list) else []:
        print("stage %-16s %-8s" % (stg.get("name"), stg.get("status")))
    # drift_log
    dl = glob.glob(os.path.join(d, "md_*_drift_log.csv"))
    if dl:
        rows = list(csv.DictReader(open(dl[0])))
        legs = []
        for r in rows:
            if r["leg"] not in legs:
                legs.append(r["leg"])
        print("drift_log rows %d  legs %s" % (len(rows), legs))
        for leg in legs:
            rr = [r for r in rows if r["leg"] == leg]
            e = [fnum(r["err_along"]) for r in rr if r["err_along"] not in ("", "nan")]
            es = [fnum(r["err_standoff"]) for r in rr if r["err_standoff"] not in ("", "nan")]
            yd = [fnum(r["yaw_drift_deg"]) for r in rr if r["yaw_drift_deg"] not in ("", "nan")]
            if not e:
                print("  %-13s n=%3d (유효 err 없음)" % (leg, len(rr)))
                continue
            print("  %-13s n=%3d err_along first %+.3f last %+.3f mean %+.3f sd %.3f | standoff err last %+.3f mean %+.3f | yaw_drift mean %+.2f" % (
                leg, len(e), e[0], e[-1], st.mean(e), st.pstdev(e) if len(e) > 1 else 0.0,
                es[-1] if es else float("nan"), st.mean(es) if es else float("nan"), st.mean(yd) if yd else float("nan")))
    # events
    ev = glob.glob(os.path.join(d, "md_*_events.csv"))
    if ev:
        rows = list(csv.DictReader(open(ev[0])))
        keys = [k for k in rows[0].keys()] if rows else []
        ecol = "event" if "event" in keys else (keys[1] if len(keys) > 1 else None)
        interesting = [r for r in rows if ecol and any(w in (r.get(ecol) or "").lower() for w in
                                                       ("geofence", "jump", "abort", "emergency", "timeout", "max_dist", "stale", "land_here", "unstable", "warn"))]
        print("events %d, 주목 %d:" % (len(rows), len(interesting)))
        for r in interesting[:12]:
            print("   ", {k: r[k] for k in keys[:4]})
    # gt
    gt = os.path.join(d, "gt.csv")
    if os.path.exists(gt):
        rows = list(csv.DictReader(open(gt)))
        if rows:
            xs = [fnum(r["x"]) for r in rows]
            ys = [fnum(r["y"]) for r in rows]
            zs = [fnum(r["z"]) for r in rows]
            print("gt    : x(E) %.2f..%.2f  y(N) %.2f..%.2f  z %.2f..%.2f  (%d rows, %.0f s)" % (
                min(xs), max(xs), min(ys), max(ys), min(zs), max(zs), len(rows), fnum(rows[-1]["t_sim"]) - fnum(rows[0]["t_sim"])))
    tl = os.path.join(d, "tag_detect.log")
    if os.path.exists(tl):
        last = open(tl, errors="replace").read().strip().splitlines()
        last = [x for x in last if x.startswith("[sim_tag_detect] stopped")]
        if last:
            print("tags  :", last[-1])


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for d in sys.argv[1:]:
        summarize(d)
