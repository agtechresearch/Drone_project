#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""카메라-기체 yaw 편향 손 시험 (비행 없음, 기체에서 실행). 2026-11-12 A1: 정렬 뒤 우측 이동 중 이격이 0.17 m/m 커짐(≈10°).

    python3 yaw_bias_test.py            # 1단계: 정지 3 s 동안 태그 정렬 각(psi) 측정
                                        # 2단계: 안내가 뜨면 기체를 바닥 격자선을 따라(현수막과 평행하게, 몸체 우측으로) 1.5~2 m 천천히 민다
                                        #        → EKF 위치가 그리는 이동 방향 vs EKF 헤딩+90° 의 차이를 잰다

판정:
  psi(태그 직선 기울기) ≈ 0 인데 '이동 방향 - (헤딩+90)' ≈ ±10° → VIO/EKF 헤딩이 위치 프레임과 어긋남 (카메라-IMU 외부보정)
  psi ≈ ±10° (기체를 격자에 맞춰 직각으로 둔 상태에서)          → 카메라가 기체에 대해 물리적으로 돌아감
둘 다 작으면 비행 중 다른 원인.
"""
import io
import math
import re
import subprocess
import sys
import time

# 기체 터미널 로케일이 ASCII 여도 한글이 나오게 (2026-11-13 UnicodeEncodeError)
if (getattr(sys.stdout, "encoding", "") or "").lower().replace("-", "") != "utf8":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace", line_buffering=True)

sys.path.insert(0, "/home/root")
from s2flight.align import measure_alignment          # noqa: E402
from s2flight.tags import MpaTagFeed                  # noqa: E402
import asyncio                                        # noqa: E402


def vio_yaw_deg(seconds=1.2):
    """voxl-inspect-vio -n ov 에서 yaw(deg) 샘플 평균. 실패하면 None. 2026-11-13: EKF 헤딩 vs VIO yaw 오프셋을 같이 찍기 위해."""
    try:
        out = subprocess.run(["timeout", str(seconds + 1), "voxl-inspect-vio", "-n", "ov"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=seconds + 3).stdout
    except Exception:  # noqa: BLE001
        return None
    ys = []
    text = re.sub(r"\x1b\[[0-9;?]*[a-zA-Z]", "", out.decode("utf-8", errors="replace"))   # ANSI 색/커서 코드 제거
    for l in text.replace("\r", "\n").splitlines():
        parts = [c.strip() for c in l.split("|")]
        if len(parts) >= 4 and parts[0].replace(".", "", 1).isdigit():
            try:
                ys.append(float(parts[2].split()[2]))
            except (ValueError, IndexError):
                pass
    if not ys:
        return None
    return math.degrees(math.atan2(sum(math.sin(math.radians(y)) for y in ys), sum(math.cos(math.radians(y)) for y in ys)))


def px4_pose():
    try:
        out = subprocess.check_output(["px4-listener", "vehicle_local_position", "-n", "1"], stderr=subprocess.DEVNULL, timeout=4)
    except Exception:  # noqa: BLE001
        return None
    d = {}
    for l in out.decode("utf-8", errors="replace").splitlines():
        ls = l.strip()
        for k in ("x:", "y:", "heading:"):
            if ls.startswith(k):
                try:
                    d[k[:-1]] = float(ls.split(":", 1)[1].strip())
                except ValueError:
                    pass
    return d if len(d) == 3 else None


async def sample_psi(seconds):
    feed = MpaTagFeed()
    feed.start(asyncio.get_event_loop())
    psis, n = [], 0
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            frames = feed.poll(time.time())
            for fr in (frames or []):
                m = measure_alignment(fr.tags)
                if m.valid:
                    psis.append(m.yaw_error_deg)
                n += 1
            await asyncio.sleep(0.05)
    finally:
        feed.stop()
    return psis, n


def main():
    print("[1/2] 기체를 바닥 격자에 맞춰 현수막에 직각으로 놓고 가만히 두세요. 태그 정렬각 3 s 측정...")
    psis, n = asyncio.get_event_loop().run_until_complete(sample_psi(3.0))
    if psis:
        mean = sum(psis) / len(psis)
        sd = (sum((p - mean) ** 2 for p in psis) / len(psis)) ** 0.5
        print("      psi(태그 직선 기울기) = {:+.2f} deg (sd {:.2f}, n={})  [+ = 기체 코가 법선 우측]".format(mean, sd, len(psis)))
    else:
        print("      태그 측정 없음 (frames {}) — 태그 2장 이상 보여야 함".format(n))
    p0 = px4_pose()
    if p0 is None:
        print("px4-listener 실패"); return 2
    print("      EKF heading {:+.1f} deg, pos ({:.2f}, {:.2f})".format(math.degrees(p0["heading"]), p0["x"], p0["y"]))
    vy = vio_yaw_deg()
    if vy is None:
        print("      VIO yaw: 읽기 실패 (voxl-inspect-vio)")
    else:
        off = (math.degrees(p0["heading"]) - vy + 180.0) % 360.0 - 180.0
        print("      VIO yaw {:+.1f} deg -> EKF heading - VIO yaw = {:+.1f} deg  [0 근처면 어긋남은 VIO 안, 크면 PX4 쪽 오프셋]".format(vy, off))
    try:
        input("[2/2] 준비되면 Enter → 그때부터 기록. 기체 헤딩을 바꾸지 말고 격자선을 따라 현수막과 평행하게 몸체 '오른쪽'으로 1.5~2 m 민 뒤 다시 Enter. ")
    except EOFError:
        pass
    import threading
    pts, hdg = [], []
    stop = threading.Event()

    def _sampler():
        while not stop.is_set():
            p = px4_pose()
            if p:
                pts.append((p["x"], p["y"])); hdg.append(p["heading"])
            time.sleep(0.3)
    th = threading.Thread(target=_sampler); th.daemon = True; th.start()
    try:
        input("      기록 중... 다 밀었으면 Enter ")
    except EOFError:
        time.sleep(12.0)
    stop.set(); th.join(timeout=2.0)
    if len(pts) < 3:
        print("위치 샘플 부족"); return 2
    dx = pts[-1][0] - pts[0][0]; dy = pts[-1][1] - pts[0][1]
    dist = math.hypot(dx, dy)
    motion = math.degrees(math.atan2(dy, dx))
    h_mean = math.degrees(math.atan2(sum(math.sin(h) for h in hdg), sum(math.cos(h) for h in hdg)))
    right = (h_mean + 90.0 + 180.0) % 360.0 - 180.0
    diff = (motion - right + 180.0) % 360.0 - 180.0
    h_range = max(math.degrees(h) for h in hdg) - min(math.degrees(h) for h in hdg)
    print("      이동 {:.2f} m, 이동 방향(NED) {:+.1f} deg, EKF 헤딩 평균 {:+.1f} deg (변동 {:.1f}), 헤딩+90 = {:+.1f}".format(dist, motion, h_mean, h_range, right))
    print("      ==> 이동 방향 - (헤딩+90) = {:+.1f} deg   [+ = 이동이 우측 기준보다 시계방향(뒤쪽으로)]".format(diff))
    if dist < 0.8:
        print("      이동이 짧다 — 1.5 m 이상 밀어 주세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
