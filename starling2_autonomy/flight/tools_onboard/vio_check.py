#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""이륙 전 VIO 상태 점검 (기체에서 실행, python3.6, 표준 라이브러리만). 2026-11-10 사고 대책 P5.

    python3 vio_check.py [--seconds 20] [--pipe ov] [--quiet-sec 60]

1) open-vins 저널 최근 --quiet-sec 초의 BAD 전이·리셋 줄을 센다 (있으면 "기다렸다가 다시").
2) voxl-inspect-vio -n <pipe> 를 --seconds 동안 읽어 quality/features/state 통계와 위치 드리프트를 낸다.
   사용 순서: 기체를 바닥에 둔 채 1회(정지) → 손으로 천천히 들어 30 cm 정도 움직였다 제자리에 둔 뒤 1회(복귀 확인).
3) px4-listener vehicle_visual_odometry 로 PX4 가 보는 quality 도 한 번 읽는다.
판정: 정지 20 s 동안 quality 최소 >= 50, features 최소 >= 3, 상태 전부 OKAY, 위치 변동 <= 3 cm, 저널 조용 → PASS.
비행 코드의 게이트(s2flight.vehicle.preflight_vio_gate)와 같은 기준이다. 이 스크립트가 FAIL 이면 비행 코드도 arm 을 거부한다.
"""
import argparse
import math
import subprocess
import sys
import time

sys.path.insert(0, "/home/root")
try:
    from s2flight.vio import parse_voxl_inspect_vio_line
except ImportError:
    parse_voxl_inspect_vio_line = None


def journal_events(seconds):
    try:
        out = subprocess.check_output(["journalctl", "-u", "voxl-open-vins-server", "--since", "-%ds" % int(seconds),
                                       "--no-pager", "-o", "short"], stderr=subprocess.DEVNULL, timeout=8)
    except Exception as e:  # noqa: BLE001
        return None, str(e)
    lines = out.decode("utf-8", errors="replace").splitlines()
    keys = ("BAD", "RESET", "reset", "quality was bad", "not stationary")
    return [l for l in lines if any(k in l for k in keys)], None


def sample_vio(pipe, seconds):
    proc = subprocess.Popen(["voxl-inspect-vio", "-n", pipe], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    t0 = time.time()
    rows = []
    try:
        while time.time() - t0 < seconds:
            line = proc.stdout.readline()
            if not line:
                break
            parsed = parse_voxl_inspect_vio_line(line.decode("utf-8", errors="replace"))
            if parsed is not None:
                parsed["t"] = time.time() - t0
                rows.append(parsed)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except Exception:  # noqa: BLE001
            proc.kill()
    return rows


def px4_ev_quality():
    try:
        out = subprocess.check_output(["px4-listener", "vehicle_visual_odometry", "1"], stderr=subprocess.DEVNULL, timeout=5)
    except Exception as e:  # noqa: BLE001
        return None, str(e)
    for l in out.decode("utf-8", errors="replace").splitlines():
        if "quality" in l:
            try:
                return int(float(l.split(":")[1].strip())), None
            except ValueError:
                pass
    return None, "quality field not found"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--pipe", default="ov")
    ap.add_argument("--quiet-sec", type=float, default=60.0)
    ap.add_argument("--min-quality", type=int, default=50)
    ap.add_argument("--min-features", type=int, default=3)
    ap.add_argument("--max-drift", type=float, default=0.03)
    a = ap.parse_args()
    if parse_voxl_inspect_vio_line is None:
        print("s2flight 패키지를 /home/root 에서 못 찾음 (deploy_pkg.sh 먼저)")
        return 2

    fails = []
    ev, err = journal_events(a.quiet_sec)
    if ev is None:
        print("[journal] skipped ({})".format(err))
    else:
        print("[journal] open-vins BAD/reset lines in last {:.0f}s: {}".format(a.quiet_sec, len(ev)))
        for l in ev[-5:]:
            print("          " + l[-110:])
        if ev:
            fails.append("journal: {} BAD/reset in last {:.0f}s".format(len(ev), a.quiet_sec))

    print("[vio] sampling voxl-inspect-vio -n {} for {:.0f}s ... (keep the vehicle still)".format(a.pipe, a.seconds))
    rows = sample_vio(a.pipe, a.seconds)
    if not rows:
        print("[vio] no samples!")
        fails.append("no VIO samples")
    else:
        qs = [r["quality"] for r in rows]
        fs = [r["features"] for r in rows if r["features"] >= 0]
        states = [r["state"] for r in rows]
        not_ok = sum(1 for st in states if st not in ("", "OKAY"))
        pos = [r["pos"] for r in rows if r["pos"] is not None]
        drift = 0.0
        if len(pos) > 1:
            drift = max(max(p[i] for p in pos) - min(p[i] for p in pos) for i in range(3))
        rate = len(rows) / max(1e-6, rows[-1]["t"] - rows[0]["t"]) if len(rows) > 1 else 0.0
        print("[vio] n={} ({:.1f} Hz)  quality min {} mean {:.0f} max {}  features min {} mean {:.1f}  not-OKAY {}  pos drift {:.3f} m".format(
            len(rows), rate, min(qs), sum(qs) / float(len(qs)), max(qs), (min(fs) if fs else -1),
            (sum(fs) / float(len(fs)) if fs else -1), not_ok, drift))
        bad = [r for r in rows if r["quality"] < 1 or r["state"] not in ("", "OKAY")]
        if bad:
            print("[vio] first bad sample at t={:.1f}s: {}".format(bad[0]["t"], bad[0]["raw"]))
        if min(qs) < a.min_quality:
            fails.append("quality min {} < {}".format(min(qs), a.min_quality))
        if fs and min(fs) < a.min_features:
            fails.append("features min {} < {} (featureless view? move the vehicle / add texture)".format(min(fs), a.min_features))
        if not_ok:
            fails.append("{} samples not OKAY".format(not_ok))
        if drift > a.max_drift:
            fails.append("position drifted {:.3f} m while still (> {:.2f})".format(drift, a.max_drift))

    q, err = px4_ev_quality()
    print("[px4] vehicle_visual_odometry.quality = {}{}".format(q, "" if err is None else " ({})".format(err)))
    if q is not None and q < 1:
        fails.append("PX4 sees EV quality {}".format(q))

    print("=" * 60)
    if fails:
        print("RESULT: FAIL — do NOT fly")
        for f in fails:
            print("  - " + f)
        print("  → 30~60 s 기다린 뒤 다시 실행. 계속 실패하면 이륙 지점 전방 시야(텍스처)·조명·카메라 렌즈를 확인.")
        return 1
    print("RESULT: PASS — VIO stable for {:.0f}s. 다음: 기체를 손으로 들어 움직였다 내려놓고 한 번 더 실행해 복귀를 확인.".format(a.seconds))
    return 0


if __name__ == "__main__":
    sys.exit(main())
