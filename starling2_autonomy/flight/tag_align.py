#!/usr/bin/env python3
"""
태그 정렬 계산부 (tracking_front 태그 검출 → yaw 오차 → 폐루프 보정)

드리프트 실험(docs/09 §4 3단계)의 "정밀 정렬"을 담당한다. 미션 코드(v15)는
    1. `tag_detections` 파이프에서 프레임을 받고 (mpa_tag_detections.py)
    2. `measure_alignment(frame.tags)` 로 yaw 오차를 재고
    3. `AlignController.update(측정, now)` 가 내놓는 yaw 보정량을 `go_yaw` 로 실행
    4. ALIGNED 면 t0 확정, FAILED 면 착륙
한다. 이 모듈은 I/O 도 asyncio 도 없고 numpy 도 없다(기체 파이썬 3.6). 기체 없이
전부 테스트할 수 있다(analysis/test_tag_align.py).

정렬 정의 (docs/09 §4 "정면 정렬")
    마커 평면의 법선과 카메라 광축이 수평면에서 이루는 각 = yaw 오차.
    좌우 위치(화면 중앙 정렬)는 요구하지 않는다. 다만 태그가 두 개 이상 보여야 한다.

왜 태그 하나의 자세를 쓰지 않는가
    7 cm 태그를 정면에서 보면 PnP 해가 둘로 갈려(평면 포즈 모호성) 태그 하나의
    yaw 가 프레임마다 ±1~2° 튄다. 반면 태그 **중심 위치**는 안정적이다. 그래서
    같은 열에 있는 태그들의 중심을 카메라 수평면(x–z)에 투영해 **직선을 맞추고**,
    그 직선이 카메라 x 축과 이루는 각을 yaw 오차로 쓴다. 벽이 카메라와 정면이면
    태그들은 x 축에 평행하게 늘어서고(z 가 같음), 틀어지면 한쪽이 멀어진다.
    태그 둘이면 연결선, 셋 이상이면 최소제곱 직선이다. 태그 ID 순서·법선 부호
    관례와 무관하다. 법선으로 구한 yaw 는 진단용으로 같이 내보낸다.

부호 규약 (중요)
    카메라 프레임: x 우, y 하, z 전방. NED yaw 는 시계방향(우회전)이 양수.
      yaw_error_deg      > 0 : 카메라가 정면 대비 **오른쪽**으로 돌아가 있다
                               (화면 오른쪽 태그가 왼쪽 태그보다 멀다)
      yaw_correction_deg = -yaw_error_deg : v14 `go_yaw(relative_delta=...)` 에 넣을 값
    유도: 카메라가 ψ 만큼 우회전하면 정면 좌표 (x, z) 는 (x cosψ − s sinψ, x sinψ + s cosψ)
    가 되어 x 가 큰(오른쪽) 태그의 z 가 커진다. atan2(Δz, Δx) = ψ.

카메라 피치
    tracking_front 가 아래로 θ 기울어 있으면 수평 벡터의 z 성분이 cosθ 배로 줄어
    측정 yaw 가 atan(cosθ·tanψ) 로 약간 작게 나온다(θ=10°, ψ=10° 에서 0.15°).
    폐루프이므로 0 으로 수렴하는 데는 지장이 없다. 보정하지 않는다.

허용오차
    tol_deg 기본 1.5° 는 잠정값이다. 정지 촬영에서 σ_yaw 를 재어 max(1°, 3σ) 로
    바꾼다(docs/09 §5.3). 그때까지는 보수적으로 둔다.
"""

import collections
import math

# ---------------------------------------------------------------------------
# 마커 배치 상수 (docs/09 §3, 2026-09-23 확정)
# ---------------------------------------------------------------------------

ROW_ID_RANGES = ((0, 20), (20, 40))     # 하단열 0~11(여유 ~19), 상단열 20~31(여유 ~39)
TAG_SPACING_M = 0.5
LINE_RMS_MAX_M = 0.03                   # 같은 열 태그가 한 직선에서 이만큼 벗어나면 오검출로 본다
DEFAULT_MAX_RANGE_M = 3.0


def row_of(tag_id):
    """태그 ID → 열 번호 (0 하단, 1 상단). 배치 밖 ID 는 None."""
    for row, (lo, hi) in enumerate(ROW_ID_RANGES):
        if lo <= tag_id < hi:
            return row
    return None


def fold_line_angle_deg(angle_deg):
    """직선의 방향각은 180° 주기다. (-90, 90] 로 접는다."""
    a = math.fmod(angle_deg, 180.0)
    if a > 90.0:
        a -= 180.0
    elif a <= -90.0:
        a += 180.0
    return a


def yaw_from_normal_deg(r_cam):
    """태그 하나의 회전행렬(R_tag_to_cam)에서 법선 기준 yaw 를 구한다 (진단용).

    법선 n = R 의 세 번째 열. z 성분이 음수면(법선이 카메라 쪽) 뒤집어서 항상 벽
    안쪽을 보게 한 뒤, 수평면에서 광축과 이루는 각을 잰다. 벽이 정면이면 n=(0,0,1).
    카메라가 우회전(ψ>0)하면 n=(−sinψ, 0, cosψ) 이므로 atan2(−n_x, n_z) = ψ.
    단일 태그 모호성 때문에 이 값은 믿지 말고 line 기반 값과 비교하는 데만 쓴다.
    """
    nx, ny, nz = r_cam[0][2], r_cam[1][2], r_cam[2][2]
    if nz < 0.0:
        nx, ny, nz = -nx, -ny, -nz
    return math.degrees(math.atan2(-nx, nz))


def fit_line_xz(points):
    """(x, z) 점들에 주축 직선을 맞춘다. 반환 (angle_deg, cx, cz, rms_m, span_m).

    angle 은 직선이 x 축과 이루는 각(z 가 x 와 함께 커지면 양수), (-90, 90].
    2점이면 연결선 그 자체다. 3점 이상이면 2D 공분산의 주축(총최소제곱)이다.
    rms 는 점에서 직선까지 수직거리의 RMS, span 은 직선 위 투영의 최대 폭.
    """
    n = len(points)
    if n < 2:
        raise ValueError("직선을 맞추려면 점이 둘 이상 필요하다")
    cx = sum(p[0] for p in points) / n
    cz = sum(p[1] for p in points) / n
    sxx = sxz = szz = 0.0
    for x, z in points:
        dx, dz = x - cx, z - cz
        sxx += dx * dx
        sxz += dx * dz
        szz += dz * dz
    # 주축 각: 2D 공분산의 최대 고유벡터 방향
    angle = 0.5 * math.atan2(2.0 * sxz, sxx - szz)
    ux, uz = math.cos(angle), math.sin(angle)
    sq = 0.0
    proj = []
    for x, z in points:
        dx, dz = x - cx, z - cz
        along = dx * ux + dz * uz
        perp = -dx * uz + dz * ux
        sq += perp * perp
        proj.append(along)
    rms = math.sqrt(sq / n)
    span = max(proj) - min(proj)
    return fold_line_angle_deg(math.degrees(angle)), cx, cz, rms, span


# ---------------------------------------------------------------------------
# 측정
# ---------------------------------------------------------------------------

class AlignMeasurement(object):
    """프레임 하나에서 나온 정렬 측정값."""

    __slots__ = ("timestamp_ns", "recv_time", "ids", "n_tags", "row",
                 "yaw_error_deg", "yaw_correction_deg", "normal_yaw_deg", "normal_yaws",
                 "center_x_m", "range_m", "baseline_m", "line_rms_m", "valid", "reason")

    def __init__(self):
        self.timestamp_ns = None
        self.recv_time = None
        self.ids = []
        self.n_tags = 0
        self.row = None
        self.yaw_error_deg = float("nan")
        self.yaw_correction_deg = float("nan")
        self.normal_yaw_deg = float("nan")
        self.normal_yaws = {}
        self.center_x_m = float("nan")
        self.range_m = float("nan")
        self.baseline_m = float("nan")
        self.line_rms_m = float("nan")
        self.valid = False
        self.reason = "no_tags"

    def __repr__(self):
        if self.valid:
            return "<Align yaw_err={:+.2f}deg ids={} x={:+.3f} z={:.3f} rms={:.1f}mm>".format(
                self.yaw_error_deg, self.ids, self.center_x_m, self.range_m, self.line_rms_m * 1e3)
        return "<Align invalid: {} ids={}>".format(self.reason, self.ids)


def _as_tag(obj):
    """TagDetection 이든 (id, t_cam, r_cam[, timestamp_ns, recv_time]) 튜플이든 받는다."""
    if hasattr(obj, "t_cam"):
        return (obj.id, obj.t_cam, obj.r_cam,
                getattr(obj, "timestamp_ns", None), getattr(obj, "recv_time", None))
    tag_id, t_cam, r_cam = obj[0], obj[1], obj[2]
    ts = obj[3] if len(obj) > 3 else None
    rt = obj[4] if len(obj) > 4 else None
    return (tag_id, t_cam, r_cam, ts, rt)


def measure_alignment(detections, min_tags=2, allowed_ids=None,
                      max_range_m=DEFAULT_MAX_RANGE_M, line_rms_max_m=LINE_RMS_MAX_M):
    """한 프레임의 검출 목록에서 yaw 오차를 잰다.

    - 배치 밖 ID, allowed_ids 밖 ID, 너무 먼 태그(max_range_m)는 버린다.
    - 하단열·상단열이 섞여 있으면 **태그가 더 많은 열**만 쓴다(열이 다르면 높이가
      달라 피치 때문에 z 가 어긋난다). 같으면 하단열.
    - 남은 태그가 min_tags 미만이면 valid=False (reason: no_tags / single_tag).
      그래도 법선 yaw·중심·거리는 채워서 진단에 쓸 수 있게 한다.
    - 직선 맞춤 잔차가 line_rms_max_m 을 넘으면 valid=False (reason: line_fit_poor).
    """
    m = AlignMeasurement()
    tags = []
    for obj in detections:
        tag_id, t_cam, r_cam, ts, rt = _as_tag(obj)
        if m.timestamp_ns is None and ts is not None:
            m.timestamp_ns = ts
        if m.recv_time is None and rt is not None:
            m.recv_time = rt
        if allowed_ids is not None and tag_id not in allowed_ids:
            continue
        row = row_of(tag_id)
        if row is None:
            continue
        x, y, z = t_cam
        if z <= 0.0 or math.sqrt(x * x + y * y + z * z) > max_range_m:
            continue
        tags.append((tag_id, row, (x, y, z), r_cam))

    if not tags:
        m.reason = "no_tags"
        return m

    counts = collections.Counter(t[1] for t in tags)
    row = min(counts, key=lambda r: (-counts[r], r))
    tags = [t for t in tags if t[1] == row]
    tags.sort(key=lambda t: t[0])

    m.row = row
    m.ids = [t[0] for t in tags]
    m.n_tags = len(tags)
    m.normal_yaws = dict((t[0], yaw_from_normal_deg(t[3])) for t in tags)
    m.normal_yaw_deg = sum(m.normal_yaws.values()) / len(m.normal_yaws)
    m.center_x_m = sum(t[2][0] for t in tags) / len(tags)
    m.range_m = sum(t[2][2] for t in tags) / len(tags)

    if m.n_tags < min_tags:
        m.reason = "single_tag" if m.n_tags == 1 else "too_few_tags"
        return m

    angle, cx, cz, rms, span = fit_line_xz([(t[2][0], t[2][2]) for t in tags])
    m.line_rms_m = rms
    m.baseline_m = span
    m.center_x_m = cx
    m.range_m = cz
    if rms > line_rms_max_m:
        m.reason = "line_fit_poor"
        return m

    m.yaw_error_deg = angle
    m.yaw_correction_deg = -angle
    m.valid = True
    m.reason = "ok"
    return m


# ---------------------------------------------------------------------------
# 폐루프 컨트롤러 (상태기계)
# ---------------------------------------------------------------------------

WAIT_TAGS = "WAIT_TAGS"       # 쓸 만한 측정이 없다 (태그 안 보임 / 하나뿐 / 정착 대기)
CORRECTING = "CORRECTING"     # yaw 보정 명령을 내렸고 미션 루프가 실행 중
HOLDING = "HOLDING"           # 허용오차 안. hold_s 동안 유지되는지 보는 중
ALIGNED = "ALIGNED"           # 완료 (종결)
FAILED = "FAILED"             # 시간 초과 (종결) → 착륙


class AlignCommand(object):
    """update() 의 반환. yaw_delta_deg 가 None 이 아니면 미션 루프가 그만큼 회전시킨다."""

    __slots__ = ("state", "yaw_delta_deg", "yaw_error_deg", "reason", "elapsed_s")

    def __init__(self, state, yaw_delta_deg, yaw_error_deg, reason, elapsed_s):
        self.state = state
        self.yaw_delta_deg = yaw_delta_deg
        self.yaw_error_deg = yaw_error_deg
        self.reason = reason
        self.elapsed_s = elapsed_s

    @property
    def done(self):
        return self.state in (ALIGNED, FAILED)

    def __repr__(self):
        d = "" if self.yaw_delta_deg is None else " delta={:+.2f}".format(self.yaw_delta_deg)
        e = "" if math.isnan(self.yaw_error_deg) else " err={:+.2f}".format(self.yaw_error_deg)
        return "<AlignCommand {} t={:.1f}s{}{} {}>".format(
            self.state, self.elapsed_s, e, d, self.reason)


class AlignController(object):
    """태그 정렬 상태기계. 미션 루프가 주기적으로 update() 를 부른다.

    사용 (의사코드):
        ctl = AlignController(tol_deg=1.5, timeout_s=15.0)
        ctl.start(now)
        while True:
            meas = measure_alignment(frame.tags) if frame else None
            cmd = ctl.update(meas, now)
            if cmd.yaw_delta_deg is not None:
                await go_yaw(current_yaw + cmd.yaw_delta_deg, ...)   # 회전 실행
                ctl.ack_correction(now)                              # 끝났다고 알림
            if cmd.done: break

    규칙
      - 최근 window 개 측정의 **중앙값**으로 판단한다 (한 프레임 튐은 무시).
      - |오차| > tol 이면 보정 명령: delta = clip(gain × (−오차), ±max_step).
        명령 후 ack_correction() 까지는 측정을 버린다(회전 중 영상). ack 후 settle_s
        동안도 버린다(검출 지연·진동).
      - |오차| ≤ tol 이 hold_s 동안, 측정 min_hold_frames 개 이상으로 유지되면 ALIGNED.
      - 어느 상태든 timeout_s 를 넘기면 FAILED. 이유에 마지막 상황을 적는다.
      - 측정이 max_age_s 보다 오래됐으면 없는 것으로 친다.
    """

    def __init__(self, tol_deg=1.5, hold_s=1.0, timeout_s=15.0,
                 gain=0.8, max_step_deg=15.0, min_step_deg=0.3,
                 window=3, max_age_s=0.6, settle_s=0.4, min_hold_frames=3,
                 min_tags=2):
        if tol_deg <= 0 or hold_s < 0 or timeout_s <= 0:
            raise ValueError("tol_deg>0, hold_s>=0, timeout_s>0 이어야 한다")
        self.tol_deg = float(tol_deg)
        self.hold_s = float(hold_s)
        self.timeout_s = float(timeout_s)
        self.gain = float(gain)
        self.max_step_deg = float(max_step_deg)
        self.min_step_deg = float(min_step_deg)
        self.window = int(window)
        self.max_age_s = float(max_age_s)
        self.settle_s = float(settle_s)
        self.min_hold_frames = int(min_hold_frames)
        self.min_tags = int(min_tags)
        self.reset()

    def reset(self):
        self.state = WAIT_TAGS
        self.t_start = None
        self.t_aligned = None
        self.t_last_valid = None
        self.last_invalid_reason = "no_tags"
        self.hold_since = None
        self.hold_frames = 0
        self.settle_until = -float("inf")
        self.awaiting_ack = False
        self.corrections = []                 # (now, delta) 기록
        self.n_measurements = 0
        self._hist = collections.deque(maxlen=self.window)   # (recv_time, yaw_error)

    # -- 외부 호출 -----------------------------------------------------------

    def start(self, now):
        self.reset()
        self.t_start = now

    def ack_correction(self, now):
        """미션 루프가 yaw 회전을 끝냈을 때 부른다. 정착 시간 뒤부터 측정을 다시 받는다."""
        self.awaiting_ack = False
        self.settle_until = now + self.settle_s
        self._hist.clear()
        self.hold_since = None
        self.hold_frames = 0
        if self.state == CORRECTING:
            self.state = WAIT_TAGS

    def update(self, meas, now):
        """측정(없으면 None)을 넣고 명령을 받는다."""
        if self.t_start is None:
            self.start(now)
        elapsed = now - self.t_start

        if self.state in (ALIGNED, FAILED):
            return AlignCommand(self.state, None, self._median_error(now), self._done_reason, elapsed)

        # 측정 접수
        if meas is not None:
            self.n_measurements += 1
            if meas.valid and meas.n_tags >= self.min_tags:
                if not self.awaiting_ack and now >= self.settle_until:
                    rt = meas.recv_time if meas.recv_time is not None else now
                    self._hist.append((rt, meas.yaw_error_deg))
                    self.t_last_valid = now
            else:
                self.last_invalid_reason = meas.reason if meas.reason != "ok" else "too_few_tags"

        # 시간 초과
        if elapsed >= self.timeout_s:
            self.state = FAILED
            self._done_reason = "timeout {:.1f}s ({})".format(elapsed, self._situation(now))
            return AlignCommand(FAILED, None, self._median_error(now), self._done_reason, elapsed)

        if self.awaiting_ack:
            return AlignCommand(CORRECTING, None, float("nan"), "awaiting ack", elapsed)

        if now < self.settle_until:
            return AlignCommand(WAIT_TAGS, None, float("nan"),
                                "settling {:.2f}s".format(self.settle_until - now), elapsed)

        err = self._median_error(now)
        if math.isnan(err):
            self.state = WAIT_TAGS
            self.hold_since = None
            self.hold_frames = 0
            return AlignCommand(WAIT_TAGS, None, err, self._situation(now), elapsed)

        if abs(err) > self.tol_deg:
            delta = -self.gain * err
            if abs(delta) < self.min_step_deg:
                delta = -err
            delta = max(-self.max_step_deg, min(self.max_step_deg, delta))
            self.state = CORRECTING
            self.awaiting_ack = True
            self.hold_since = None
            self.hold_frames = 0
            self.corrections.append((now, delta))
            return AlignCommand(CORRECTING, delta, err,
                                "err {:+.2f} > tol {:.2f}".format(err, self.tol_deg), elapsed)

        # 허용오차 안
        if self.hold_since is None:
            self.hold_since = now
            self.hold_frames = 0
        self.hold_frames += 1
        held = now - self.hold_since
        if held >= self.hold_s and self.hold_frames >= self.min_hold_frames:
            self.state = ALIGNED
            self.t_aligned = now
            self._done_reason = "held {:.2f}s / {} frames, err {:+.2f}".format(held, self.hold_frames, err)
            return AlignCommand(ALIGNED, None, err, self._done_reason, elapsed)
        self.state = HOLDING
        return AlignCommand(HOLDING, None, err,
                            "holding {:.2f}/{:.2f}s ({} frames)".format(held, self.hold_s, self.hold_frames),
                            elapsed)

    # -- 내부 ----------------------------------------------------------------

    _done_reason = ""

    def _median_error(self, now):
        recent = [e for (rt, e) in self._hist if now - rt <= self.max_age_s]
        if not recent:
            return float("nan")
        recent.sort()
        n = len(recent)
        if n % 2:
            return recent[n // 2]
        return 0.5 * (recent[n // 2 - 1] + recent[n // 2])

    def _situation(self, now):
        if self.t_last_valid is None:
            return "no valid measurement yet ({})".format(self.last_invalid_reason)
        return "tags lost {:.1f}s ago ({})".format(now - self.t_last_valid, self.last_invalid_reason)
