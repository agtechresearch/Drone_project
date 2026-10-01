#!/usr/bin/env python3
"""tag_align 단위 테스트 — 정렬 측정(직선 맞춤 yaw) 과 폐루프 상태기계.

기체 없이 실행한다. 벽에 붙은 태그를 '정답을 아는 가짜 카메라' 로 찍어
(yaw·피치·좌우 위치를 정해 좌표를 만들어) 측정값이 정답을 되찾는지 채점한다.
컨트롤러는 가짜 기체(명령한 만큼 정확히 도는 기체)에 붙여 수렴·시간초과·튐 처리를 본다.

    python analysis/test_tag_align.py
"""
import math
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "flight"))

import mpa_tag_detections as mtd     # noqa: E402
import tag_align as ta               # noqa: E402

if hasattr(sys.stdout, "reconfigure"):          # Windows 콘솔(cp949)에서 기호가 깨지지 않게
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

results = []


def ok(name, cond):
    results.append((name, bool(cond)))
    print(("  PASS  " if cond else "  FAIL  ") + name)


def _raises(fn, exc):
    try:
        fn()
    except exc:
        return True
    except Exception:
        return False
    return False


# ---------------------------------------------------------------------------
# 3x3 행렬 도우미 (numpy 없이)
# ---------------------------------------------------------------------------

def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def matvec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rot_y(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
FLIP_Z = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))   # 태그 z 가 카메라 쪽을 보는 관례


def wall_tags(ids, yaw_deg, pitch_deg=0.0, standoff=0.5, lateral=0.0, spacing=0.5,
              row_y=None, ts_ns=1000, recv_time=0.0, normal_out=False, noise=None):
    """벽에 붙은 태그들을 '카메라가 yaw_deg 만큼 우회전, pitch_deg 만큼 숙인' 상태로 찍는다.

    정면·수평 좌표계: 태그 i 중심 = (lateral + (i - id0)·spacing, y_row, standoff).
    카메라 좌표 = Rx(pitch) · Ry(−yaw) · 정면좌표. (우회전 ψ>0 → 정면 점이 화면 왼쪽으로)
    """
    rot = matmul(rot_x(pitch_deg), rot_y(-yaw_deg))
    out = []
    for k, tag_id in enumerate(ids):
        row = ta.row_of(tag_id)
        y = row_y if row_y is not None else (0.0 if row == 0 else -1.4)
        base = ids[0] if row == ta.row_of(ids[0]) else [i for i in ids if ta.row_of(i) == row][0]
        p = (lateral + (tag_id - base) * spacing, y, standoff)
        if noise is not None:
            p = (p[0] + noise[k][0], p[1] + noise[k][1], p[2] + noise[k][2])
        t_cam = matvec(rot, p)
        r_cam = matmul(rot, FLIP_Z) if normal_out else rot
        out.append(mtd.TagDetection(tag_id, 0.07, ts_ns, "", 0, t_cam, r_cam,
                                    (0, 0, 0), IDENTITY, "tracking_front", recv_time=recv_time))
    return out


# ---------------------------------------------------------------------------
print("[1] 기본 기하")
ok("fold: 95 → -85", abs(ta.fold_line_angle_deg(95.0) + 85.0) < 1e-9)
ok("fold: -90 → 90", abs(ta.fold_line_angle_deg(-90.0) - 90.0) < 1e-9)
ok("fold: 180 → 0", abs(ta.fold_line_angle_deg(180.0)) < 1e-9)
ok("row_of: 0~19 하단, 20~39 상단, 그 밖 None",
   ta.row_of(0) == 0 and ta.row_of(11) == 0 and ta.row_of(20) == 1 and ta.row_of(31) == 1
   and ta.row_of(40) is None and ta.row_of(-1) is None)

ang, cx, cz, rms, span = ta.fit_line_xz([(-0.25, 0.5), (0.25, 0.5)])
ok("두 점 수평선: 각 0, 중심 (0, 0.5), 폭 0.5", abs(ang) < 1e-9 and abs(cx) < 1e-9
   and abs(cz - 0.5) < 1e-9 and rms < 1e-12 and abs(span - 0.5) < 1e-9)
ang, _, _, rms, _ = ta.fit_line_xz([(-0.25, 0.5 - 0.25 * math.tan(math.radians(10))),
                                    (0.25, 0.5 + 0.25 * math.tan(math.radians(10)))])
ok("두 점 10° 기울기", abs(ang - 10.0) < 1e-9)
pts = [(x, 0.5 + x * math.tan(math.radians(-6.0))) for x in (-0.5, 0.0, 0.5)]
ang, _, _, rms, span = ta.fit_line_xz(pts)
ok("세 점 -6°, 잔차 0, 폭 1.0/cos6°", abs(ang + 6.0) < 1e-9 and rms < 1e-12
   and abs(span - 1.0 / math.cos(math.radians(6.0))) < 1e-6)
ang, _, _, rms, _ = ta.fit_line_xz([(-0.25, 0.5), (0.0, 0.52), (0.25, 0.5)])
ok("가운데 점이 2 cm 튀면 잔차 ≈ 9.4 mm", abs(rms - 0.02 * math.sqrt(2.0) / 3.0) < 1e-6)
ok("점 하나는 ValueError", _raises(lambda: ta.fit_line_xz([(0.0, 0.5)]), ValueError))

# ---------------------------------------------------------------------------
print("[2] 법선 기준 yaw (진단)")
for psi in (0.0, 12.0, -7.5, 30.0):
    r_in = rot_y(-psi)
    r_out = matmul(rot_y(-psi), FLIP_Z)
    ok("법선 yaw ψ={:+.1f}: z 안쪽 관례".format(psi), abs(ta.yaw_from_normal_deg(r_in) - psi) < 1e-9)
    ok("법선 yaw ψ={:+.1f}: z 바깥쪽 관례도 같다".format(psi), abs(ta.yaw_from_normal_deg(r_out) - psi) < 1e-9)

# ---------------------------------------------------------------------------
print("[3] 측정 — 정면·회전·부호")
m = ta.measure_alignment(wall_tags([0, 1], 0.0))
ok("정면: valid, yaw 0, 거리 0.5, 중심 x 0.25, 폭 0.5",
   m.valid and abs(m.yaw_error_deg) < 1e-9 and abs(m.range_m - 0.5) < 1e-9
   and abs(m.center_x_m - 0.25) < 1e-9 and abs(m.baseline_m - 0.5) < 1e-9)
ok("정면: ids / row / n_tags / reason", m.ids == [0, 1] and m.row == 0 and m.n_tags == 2 and m.reason == "ok")

tags = wall_tags([0, 1], 12.0)
m = ta.measure_alignment(tags)
ok("우회전 12°: 오른쪽 태그(1)가 더 멀다", tags[1].t_cam[2] > tags[0].t_cam[2])
ok("우회전 12°: yaw_error +12, correction -12",
   abs(m.yaw_error_deg - 12.0) < 1e-9 and abs(m.yaw_correction_deg + 12.0) < 1e-9)
ok("우회전 12°: 법선 yaw 도 +12 (정답 자세이므로)", abs(m.normal_yaw_deg - 12.0) < 1e-9)

m = ta.measure_alignment(wall_tags([0, 1], -7.0))
ok("좌회전 7°: yaw_error -7", abs(m.yaw_error_deg + 7.0) < 1e-9)

m_a = ta.measure_alignment(wall_tags([0, 1], 9.0))
m_b = ta.measure_alignment(wall_tags([1, 0], 9.0, spacing=-0.5))     # 태그 1 이 왼쪽에
ok("ID 순서가 뒤집혀도 같은 yaw", abs(m_a.yaw_error_deg - m_b.yaw_error_deg) < 1e-9)

m = ta.measure_alignment(wall_tags([0, 1], 9.0, lateral=-0.6))
ok("좌우 치우침은 yaw 에 영향 없음, 중심 x 반영", abs(m.yaw_error_deg - 9.0) < 1e-9 and m.center_x_m < -0.3)

m = ta.measure_alignment(wall_tags([0, 1], 5.0, pitch_deg=10.0))
ok("피치 10° 에서 ψ=5°: 0.3° 이내 (문서대로 약간 작게)", abs(m.yaw_error_deg - 5.0) < 0.3 and m.yaw_error_deg < 5.0)
m = ta.measure_alignment(wall_tags([0, 1], 0.0, pitch_deg=15.0))
ok("피치 15° 라도 정면이면 정확히 0", abs(m.yaw_error_deg) < 1e-9)

m = ta.measure_alignment(wall_tags([0, 1, 2], 4.0))
ok("태그 셋: 모두 사용, 폭 1.0", m.n_tags == 3 and abs(m.baseline_m - 1.0) < 1e-6 and abs(m.yaw_error_deg - 4.0) < 1e-9)

m = ta.measure_alignment(wall_tags([0, 1], 3.0, normal_out=True))
ok("법선 관례가 반대여도 line yaw·법선 yaw 모두 +3", abs(m.yaw_error_deg - 3.0) < 1e-9 and abs(m.normal_yaw_deg - 3.0) < 1e-9)

# ---------------------------------------------------------------------------
print("[4] 측정 — 열 선택·부족·오검출")
mixed = wall_tags([0, 1], 6.0) + wall_tags([20], 6.0, row_y=-1.4)
m = ta.measure_alignment(mixed)
ok("하단 2 + 상단 1 → 하단열만", m.row == 0 and m.ids == [0, 1] and abs(m.yaw_error_deg - 6.0) < 1e-9)
m = ta.measure_alignment(wall_tags([20, 21, 22], 6.0, row_y=-1.4) + wall_tags([0], 6.0))
ok("상단 3 + 하단 1 → 상단열", m.row == 1 and m.ids == [20, 21, 22])
m = ta.measure_alignment(wall_tags([0, 1], 2.0) + wall_tags([20, 21], 2.0, row_y=-1.4))
ok("2:2 동률이면 하단열", m.row == 0)

m = ta.measure_alignment(wall_tags([0], 8.0))
ok("태그 하나: invalid 'single_tag', 법선 yaw 는 채움",
   not m.valid and m.reason == "single_tag" and abs(m.normal_yaw_deg - 8.0) < 1e-9
   and math.isnan(m.yaw_error_deg) and m.n_tags == 1)
m = ta.measure_alignment([])
ok("빈 목록: 'no_tags'", not m.valid and m.reason == "no_tags" and m.n_tags == 0)
m = ta.measure_alignment(wall_tags([40, 41], 0.0, row_y=0.0))
ok("배치 밖 ID 만 있으면 'no_tags'", not m.valid and m.reason == "no_tags")
m = ta.measure_alignment(wall_tags([0, 1], 0.0), allowed_ids={0})
ok("allowed_ids 로 거르면 하나 남아 single_tag", m.reason == "single_tag")
m = ta.measure_alignment(wall_tags([0, 1], 0.0, standoff=4.0))
ok("4 m 너머 태그는 버린다 (max_range 3 m)", m.reason == "no_tags")
m = ta.measure_alignment(wall_tags([0, 1], 0.0, standoff=4.0), max_range_m=5.0)
ok("max_range 를 늘리면 쓴다", m.valid)

bad = wall_tags([0, 1, 2], 0.0)
x, y, z = bad[1].t_cam
bad[1].t_cam = (x, y, z + 0.08)          # 가운데 태그가 8 cm 튄 오검출
m = ta.measure_alignment(bad)
ok("한 태그가 직선에서 벗어나면 'line_fit_poor'", not m.valid and m.reason == "line_fit_poor" and m.line_rms_m > 0.03)
m = ta.measure_alignment(bad, line_rms_max_m=0.1)
ok("잔차 한계를 올리면 통과", m.valid)

tup = [(0, (-0.25, 0.0, 0.5), IDENTITY, 77, 1.5), (1, (0.25, 0.0, 0.5), IDENTITY, 77, 1.5)]
m = ta.measure_alignment(tup)
ok("튜플 입력도 받는다 (timestamp·recv_time 전달)", m.valid and m.timestamp_ns == 77 and m.recv_time == 1.5)

noisy = wall_tags([0, 1], 5.0, noise=[(0.002, 0.0, -0.002), (-0.001, 0.0, 0.002)])
m = ta.measure_alignment(noisy)
ok("2 mm 잡음 → 0.6° 이내", abs(m.yaw_error_deg - 5.0) < 0.6)
rp = repr(m)
ok("repr 에 yaw 가 들어간다", "yaw_err" in rp)

# ---------------------------------------------------------------------------
print("[5] 컨트롤러 — 가짜 기체로 수렴")


class Plant(object):
    """명령한 만큼 정확히 도는 기체 + 15 fps 검출. 잡음은 결정적 패턴."""

    def __init__(self, yaw_err0, fps=15.0, noise_deg=0.3, ids=(0, 1)):
        self.err = yaw_err0
        self.dt = 1.0 / fps
        self.noise = noise_deg
        self.ids = list(ids)
        self.k = 0

    def frame(self, now):
        self.k += 1
        n = self.noise * math.sin(self.k * 1.7)
        return wall_tags(self.ids, self.err + n, ts_ns=int(now * 1e9), recv_time=now)


def run(ctl, plant, t_end=20.0, ramp_rate=30.0, single_after=None, lost_between=None,
        outlier_at=None):
    """시뮬레이션. (최종 cmd, 걸린 시간, 명령 목록) 을 돌려준다."""
    now = 0.0
    ctl.start(now)
    cmd = None
    log = []
    while now < t_end:
        if lost_between and lost_between[0] <= now < lost_between[1]:
            meas = ta.measure_alignment([])
        else:
            tags = plant.frame(now)
            if single_after is not None and now >= single_after:
                tags = tags[:1]
            if outlier_at is not None and abs(now - outlier_at) < plant.dt / 2:
                tags = wall_tags(plant.ids, plant.err + 6.0, ts_ns=int(now * 1e9), recv_time=now)
            meas = ta.measure_alignment(tags)
        cmd = ctl.update(meas, now)
        log.append((now, cmd))
        if cmd.yaw_delta_deg is not None:
            plant.err += cmd.yaw_delta_deg            # 기체가 돈 만큼 오차가 줄어든다
            now += abs(cmd.yaw_delta_deg) / ramp_rate + 0.2
            ctl.ack_correction(now)
        if cmd.done:
            return cmd, now, log
        now += plant.dt
    return cmd, now, log


ctl = ta.AlignController(tol_deg=1.5, hold_s=1.0, timeout_s=15.0)
plant = Plant(12.0)
cmd, t, log = run(ctl, plant)
ok("12° 오차 → ALIGNED", cmd.state == ta.ALIGNED and cmd.done)
ok("6 s 안에 정렬", t < 6.0)
ok("최종 오차 < tol", abs(plant.err) < 1.5)
ok("보정 명령은 1~3회, 첫 명령은 -gain·err 방향", 1 <= len(ctl.corrections) <= 3 and ctl.corrections[0][1] < 0)
ok("첫 보정량 ≈ -0.8 × 12 (±잡음)", abs(ctl.corrections[0][1] + 9.6) < 0.5)
ok("t_aligned 기록", ctl.t_aligned is not None and abs(ctl.t_aligned - t) < 1e-9)
ok("ALIGNED 뒤 update 는 그대로 ALIGNED, 명령 없음",
   ctl.update(ta.measure_alignment(plant.frame(t + 1)), t + 1).state == ta.ALIGNED
   and ctl.update(None, t + 2).yaw_delta_deg is None)

ctl = ta.AlignController(tol_deg=1.5)
plant = Plant(-25.0)
cmd, t, log = run(ctl, plant)
ok("-25° (좌로 틀어짐) → ALIGNED, 첫 보정은 우회전(+)", cmd.state == ta.ALIGNED and ctl.corrections[0][1] > 0)

ctl = ta.AlignController(tol_deg=1.5)
plant = Plant(0.4)
cmd, t, log = run(ctl, plant)
ok("처음부터 tol 안이면 보정 없이 ALIGNED", cmd.state == ta.ALIGNED and len(ctl.corrections) == 0)
ok("hold 1.0 s 이상 걸림", t >= 1.0)
states = [c.state for _, c in log]
ok("HOLDING 상태를 거친다", ta.HOLDING in states)

ctl = ta.AlignController(tol_deg=1.5, max_step_deg=15.0)
plant = Plant(40.0, noise_deg=0.0)
cmd, t, log = run(ctl, plant)
ok("40° 는 max_step 15° 로 잘라 여러 번", cmd.state == ta.ALIGNED and abs(ctl.corrections[0][1] + 15.0) < 1e-9
   and len(ctl.corrections) >= 2)

# ---------------------------------------------------------------------------
print("[6] 컨트롤러 — 실패·튐·상실")
ctl = ta.AlignController(timeout_s=15.0)
plant = Plant(5.0)
cmd, t, log = run(ctl, plant, lost_between=(0.0, 100.0))
ok("태그 없음 15 s → FAILED", cmd.state == ta.FAILED and cmd.done and abs(t - 15.0) < 0.1)
ok("실패 이유에 timeout·no_tags", "timeout" in cmd.reason and "no_tags" in cmd.reason)
ok("실패 전까지 WAIT_TAGS", all(c.state == ta.WAIT_TAGS for _, c in log[:-1]))

ctl = ta.AlignController(timeout_s=15.0)
plant = Plant(5.0)
cmd, t, log = run(ctl, plant, single_after=0.0)
ok("태그 하나뿐이면 보정하지 않고 FAILED(single_tag)",
   cmd.state == ta.FAILED and "single_tag" in cmd.reason and len(ctl.corrections) == 0)

ctl = ta.AlignController(timeout_s=15.0, hold_s=1.0)
plant = Plant(0.5)
cmd, t, log = run(ctl, plant, lost_between=(0.5, 100.0))
ok("hold 도중 태그를 잃으면 계속 못 끝내고 FAILED", cmd.state == ta.FAILED and "lost" in cmd.reason)

ctl = ta.AlignController(tol_deg=1.5, hold_s=1.0, window=3)
plant = Plant(0.3, noise_deg=0.1)
cmd, t, log = run(ctl, plant, outlier_at=0.5)
ok("한 프레임 6° 튐은 중앙값이 걸러 보정 없이 ALIGNED", cmd.state == ta.ALIGNED and len(ctl.corrections) == 0)

ctl = ta.AlignController(tol_deg=1.5, window=1)
plant = Plant(0.3, noise_deg=0.1)
cmd, t, log = run(ctl, plant, outlier_at=0.5)
ok("window=1 이면 같은 튐에 보정 명령이 나간다 (중앙값 효과 확인)", len(ctl.corrections) >= 1)

# 시간이 멈춘 뒤 들어온 오래된 측정은 무시
ctl = ta.AlignController(max_age_s=0.6)
ctl.start(0.0)
old = ta.measure_alignment(wall_tags([0, 1], 10.0, recv_time=0.0))
c1 = ctl.update(old, 0.05)
c2 = ctl.update(None, 5.0)
ok("새 측정(0.05 s)은 보정, 5 s 뒤엔 오래돼서 WAIT_TAGS",
   c1.state == ta.CORRECTING and c1.yaw_delta_deg is not None)
ctl.ack_correction(0.5)
c2 = ctl.update(None, 5.0)
ok("ack 후 측정이 없으면 WAIT_TAGS(lost)", c2.state == ta.WAIT_TAGS and "lost" in c2.reason)

# ack 전·정착 중 측정 무시
ctl = ta.AlignController(settle_s=0.4)
ctl.start(0.0)
c = ctl.update(ta.measure_alignment(wall_tags([0, 1], 10.0, recv_time=0.0)), 0.0)
ok("보정 명령 발행", c.state == ta.CORRECTING and c.yaw_delta_deg is not None)
c = ctl.update(ta.measure_alignment(wall_tags([0, 1], 3.0, recv_time=0.3)), 0.3)
ok("ack 전 측정은 버리고 CORRECTING 유지, 새 명령 없음", c.state == ta.CORRECTING and c.yaw_delta_deg is None
   and "ack" in c.reason)
ctl.ack_correction(0.6)
c = ctl.update(ta.measure_alignment(wall_tags([0, 1], 3.0, recv_time=0.7)), 0.7)
ok("정착 시간 안 측정도 무시 (settling)", c.state == ta.WAIT_TAGS and "settling" in c.reason)
c = ctl.update(ta.measure_alignment(wall_tags([0, 1], 3.0, recv_time=1.1)), 1.1)
ok("정착 뒤에는 다시 보정", c.state == ta.CORRECTING and c.yaw_delta_deg is not None and c.yaw_delta_deg < 0)

# 작은 오차는 gain 대신 전량 보정 (min_step)
ctl = ta.AlignController(tol_deg=0.2, gain=0.1, min_step_deg=0.3)
ctl.start(0.0)
c = ctl.update(ta.measure_alignment(wall_tags([0, 1], 1.0, recv_time=0.0)), 0.0)
ok("gain·err 가 min_step 보다 작으면 -err 전량", abs(c.yaw_delta_deg + 1.0) < 1e-9)

ok("잘못된 인자는 ValueError", _raises(lambda: ta.AlignController(tol_deg=0.0), ValueError))
ok("AlignCommand repr", "CORRECTING" in repr(c))

# ---------------------------------------------------------------------------
print("[7] 통합 — 바이트 → 프레임 → 측정 → 명령")
ctl = ta.AlignController()
ctl.start(0.0)
stream = mtd.TagDetectionStream()
asm = mtd.FrameAssembler()
raw = b"".join(mtd.pack_record(d) for d in wall_tags([0, 1], 8.0, ts_ns=1) + wall_tags([0, 1], 8.0, ts_ns=2))
frames = asm.add_many(stream.feed(raw, recv_time=0.0))
ok("두 프레임 바이트 → 닫힌 프레임 1", len(frames) == 1 and frames[0].ids == [0, 1])
m = ta.measure_alignment(frames[0].tags)
ok("float32 왕복 후에도 yaw 8° (1e-4 이내)", m.valid and abs(m.yaw_error_deg - 8.0) < 1e-4)
c = ctl.update(m, 0.0)
ok("명령 -6.4° (gain 0.8)", c.state == ta.CORRECTING and abs(c.yaw_delta_deg + 6.4) < 1e-3)

# ---------------------------------------------------------------------------
print("\n" + "=" * 55)
passed = sum(1 for _n, c in results if c)
total = len(results)
print("결과: {} / {} 통과".format(passed, total))
if passed != total:
    print("\n실패 항목:")
    for n, c in results:
        if not c:
            print("  - " + n)
sys.exit(0 if passed == total else 1)
