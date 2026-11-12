#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""이륙 전 VIO 상태 점검 (기체에서 실행, python3.6, 표준 라이브러리만). 2026-11-10 사고 대책 P5.

    python3 vio_check.py [--seconds 20] [--pipe ov] [--quiet-sec 60]

1) open-vins 저널 최근 --quiet-sec 초의 BAD 전이·리셋 줄을 센다 (있으면 "기다렸다가 다시").
2) voxl-inspect-vio -n <pipe> 를 --seconds 동안 읽어 quality/features/state 통계와 위치 드리프트를 낸다.
   사용 순서: 기체를 바닥에 둔 채 1회(정지) → 손으로 천천히 들어 30 cm 정도 움직였다 제자리에 둔 뒤 1회(복귀 확인).
3) px4-listener vehicle_visual_odometry 로 PX4 가 보는 quality 도 한 번 읽는다.
4) (v0.2.2, 2026-11-12) 부팅 경과 >= --min-uptime, EKF 자이로 바이어스(estimator_sensor_bias) 축별 <= --max-gyro-bias deg/s,
   창 동안 EKF 헤딩(vehicle_local_position.heading) 변동 <= --max-heading-drift deg/s. 부팅 31 s 만에 arm 한 비행에서
   z 바이어스 -3.5 deg/s → 정지 상태에서 헤딩이 돌아 좌측 밀림·좌회전이 났다. FAIL 이면 재부팅 후 2~3 분 가만히 두고 재검사.
판정: 정지 20 s 동안 quality 최소 >= 50, features 최소 >= 3, 상태 전부 OKAY, 위치 변동 <= 3 cm, 저널 조용, 헤딩·바이어스 양호 → PASS.
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


def uptime_s():
    try:
        with open("/proc/uptime") as f:
            return float(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        return None


def px4_listener(topic):
    try:
        out = subprocess.check_output(["px4-listener", topic, "-n", "1"], stderr=subprocess.DEVNULL, timeout=6)
    except Exception as e:  # noqa: BLE001
        return None, str(e)
    return out.decode("utf-8", errors="replace"), None


def px4_sensor_bias():
    """(gyro deg/s tuple or None, accel m/s² tuple or None, err)"""
    txt, err = px4_listener("estimator_sensor_bias")
    if txt is None:
        return None, None, err
    g = a = None
    for l in txt.splitlines():
        if "[" not in l:
            continue
        try:
            vals = [float(v) for v in l.split("[", 1)[1].split("]", 1)[0].split(",")]
        except ValueError:
            continue
        if l.strip().startswith("gyro_bias:"):
            g = tuple(math.degrees(v) for v in vals[:3])
        elif l.strip().startswith("accel_bias:"):
            a = tuple(vals[:3])
    return g, a, (None if g is not None else "gyro_bias not found")


def px4_heading_deg():
    txt, err = px4_listener("vehicle_local_position")
    if txt is None:
        return None, err
    for l in txt.splitlines():
        ls = l.strip()
        if ls.startswith("heading:"):
            try:
                return math.degrees(float(ls.split(":", 1)[1].strip())), None
            except ValueError:
                break
    return None, "heading not found"


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


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
    ap.add_argument("--min-features", type=int, default=8, help="2026-11-12: 6 개 지점에서 이륙 순간 VIO 상실 → 천장 충돌")
    ap.add_argument("--min-rate", type=float, default=20.0, help="voxl-inspect-vio 출력률 하한 Hz (정상 30)")
    ap.add_argument("--max-accel-bias", type=float, default=0.3, help="EKF 가속도계 바이어스 축별 상한 m/s² (PX4 한계 0.4; 0 = 끔)")
    ap.add_argument("--max-drift", type=float, default=0.03)
    ap.add_argument("--min-uptime", type=float, default=120.0, help="부팅 뒤 이 초 전이면 FAIL (0 = 끔)")
    ap.add_argument("--max-gyro-bias", type=float, default=1.0, help="EKF 자이로 바이어스 축별 상한 deg/s (0 = 끔)")
    ap.add_argument("--max-heading-drift", type=float, default=0.3, help="정지 EKF 헤딩 드리프트 상한 deg/s (0 = 끔)")
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

    up = uptime_s()
    print("[boot] uptime {} s".format("?" if up is None else "{:.0f}".format(up)))
    if up is not None and a.min_uptime > 0 and up < a.min_uptime:
        fails.append("booted only {:.0f}s ago (< {:.0f}) — EKF gyro bias not converged; wait, keep still".format(up, a.min_uptime))
    gb, ab, err = px4_sensor_bias()
    if gb is None:
        print("[ekf] gyro bias: skipped ({})".format(err))
    else:
        print("[ekf] gyro bias x {:+.2f} y {:+.2f} z {:+.2f} deg/s (limit {:.1f})".format(gb[0], gb[1], gb[2], a.max_gyro_bias))
        if a.max_gyro_bias > 0 and max(abs(b) for b in gb) > a.max_gyro_bias:
            fails.append("EKF gyro bias {:.2f} deg/s > {:.1f} — heading will spin while hovering. REBOOT, leave still 2-3 min, re-check".format(
                max(abs(b) for b in gb), a.max_gyro_bias))
    if ab is not None:
        print("[ekf] accel bias x {:+.2f} y {:+.2f} z {:+.2f} m/s2 (limit {:.2f}; PX4 clamps at 0.40)".format(ab[0], ab[1], ab[2], a.max_accel_bias))
        if a.max_accel_bias > 0 and max(abs(b) for b in ab) > a.max_accel_bias:
            fails.append("EKF accel bias {:.2f} m/s2 > {:.2f} — if VIO drops at liftoff the vehicle will think it is falling and climb (2026-11-12 ceiling). REBOOT".format(
                max(abs(b) for b in ab), a.max_accel_bias))
    h0, herr = px4_heading_deg()
    t_h0 = time.time()

    print("[vio] sampling voxl-inspect-vio -n {} for {:.0f}s ... (keep the vehicle still)".format(a.pipe, a.seconds))
    rows = sample_vio(a.pipe, a.seconds)
    h1, herr1 = px4_heading_deg()
    if h0 is None or h1 is None:
        print("[ekf] heading drift: skipped ({})".format(herr or herr1))
    else:
        dt = max(1e-3, time.time() - t_h0)
        rate = wrap180(h1 - h0) / dt
        print("[ekf] heading {:+.1f} -> {:+.1f} deg over {:.0f}s = {:+.2f} deg/s (limit {:.1f})".format(h0, h1, dt, rate, a.max_heading_drift))
        if a.max_heading_drift > 0 and abs(rate) > a.max_heading_drift:
            fails.append("EKF heading drifting {:+.2f} deg/s while still (> {:.1f}) — REBOOT, leave still 2-3 min, re-check".format(rate, a.max_heading_drift))
        yaws = [r["rpy"][2] for r in rows if r.get("rpy") is not None]
        if len(yaws) > 1:
            rel_y = [wrap180(y - yaws[0]) for y in yaws]
            print("[vio] VIO yaw range {:.2f} deg over window; EKF-VIO yaw offset now {:+.1f} deg".format(max(rel_y) - min(rel_y), wrap180(h1 - yaws[-1])))
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
            fails.append("features min {} < {} — camera sees no texture: put the tag board / boxes 1-2 m ahead, paper on the floor, or move the takeoff spot".format(min(fs), a.min_features))
        if a.min_rate > 0 and rate < a.min_rate:
            fails.append("VIO rate {:.1f} Hz < {:.0f} (tracker struggling — same fix as features)".format(rate, a.min_rate))
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
        print("  → 헤딩/바이어스 FAIL 은 기다려도 안 낫는다: 기체 재부팅 → 바닥에 둔 채 손대지 않고 2~3 분 → 다시 실행.")
        return 1
    print("RESULT: PASS — VIO stable for {:.0f}s. 다음: 기체를 손으로 들어 움직였다 내려놓고 한 번 더 실행해 복귀를 확인.".format(a.seconds))
    return 0


if __name__ == "__main__":
    sys.exit(main())
