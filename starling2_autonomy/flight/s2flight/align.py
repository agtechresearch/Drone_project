"""태그 정렬 계산부 — 태그 중심 연결선 → yaw 오차, 폐루프 상태기계 (flight/tag_align.py 사본).

정렬 정의 (docs/09 §5.2 "정면 정렬")
    마커 평면의 법선과 카메라 광축이 수평면에서 이루는 각 = yaw 오차.
    좌우 위치(화면 중앙 정렬)는 요구하지 않는다. 다만 태그가 두 개 이상 보여야 한다.

왜 태그 하나의 자세를 쓰지 않는가
    7 cm 태그를 정면에서 보면 PnP 해가 둘로 갈려(평면 포즈 모호성) 태그 하나의 yaw 가
    프레임마다 ±1~2° 튄다. 반면 태그 중심 위치는 안정적이다. 그래서 같은 열 태그들의 중심을
    카메라 수평면(x–z)에 투영해 직선을 맞추고, 그 직선이 카메라 x 축과 이루는 각을 yaw 오차로
    쓴다. 태그 ID 순서·법선 부호 관례와 무관하다. 법선 yaw 는 진단용으로 병기한다.

부호 규약 (중요)
    카메라 프레임 x 우, y 하, z 전방. NED yaw 는 시계방향(우회전) 양수.
      yaw_error_deg      > 0 : 카메라가 정면 대비 오른쪽으로 돌아가 있다 (화면 오른쪽 태그가 더 멀다)
      yaw_correction_deg = -yaw_error_deg : Vehicle.turn(delta) 에 넣을 값
    유도: 카메라가 ψ 만큼 우회전하면 정면 좌표 (x, s) 는 (x cosψ − s sinψ, x sinψ + s cosψ) 가
    되어 x 가 큰(오른쪽) 태그의 z 가 커진다. atan2(Δz, Δx) = ψ.

허용오차
    tol_deg 기본 1.5° 는 잠정값. 정지 촬영에서 σ_yaw 를 재어 max(1°, 3σ) 로 바꾼다 (docs/09 §5.3).
"""

import collections
import math

ROW_ID_RANGES = ((0, 20), (20, 40))     # 하단열 0~11(여유 ~19), 상단열 20~31(여유 ~39)
TAG_SPACING_M = 0.5
LINE_RMS_MAX_M = 0.03
DEFAULT_MAX_RANGE_M = 3.0


def row_of(tag_id):
    for row, (lo, hi) in enumerate(ROW_ID_RANGES):
        if lo <= tag_id < hi:
            return row
    return None


def fold_line_angle_deg(angle_deg):
    a = math.fmod(angle_deg, 180.0)
    if a > 90.0:
        a -= 180.0
    elif a <= -90.0:
        a += 180.0
    return a


def yaw_from_normal_deg(r_cam):
    """태그 하나의 R_tag_to_cam 에서 법선 기준 yaw (진단용)."""
    nx, ny, nz = r_cam[0][2], r_cam[1][2], r_cam[2][2]
    if nz < 0.0:
        nx, ny, nz = -nx, -ny, -nz
    return math.degrees(math.atan2(-nx, nz))


def fit_line_xz(points):
    """(x, z) 점들에 주축 직선을 맞춘다. 반환 (angle_deg, cx, cz, rms_m, span_m)."""
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


class AlignMeasurement(object):
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

    def as_row(self):
        return {
            "timestamp_ns": self.timestamp_ns, "recv_time": self.recv_time,
            "ids": " ".join(str(i) for i in self.ids), "n_tags": self.n_tags, "row": self.row,
            "yaw_error_deg": self.yaw_error_deg, "normal_yaw_deg": self.normal_yaw_deg,
            "center_x_m": self.center_x_m, "range_m": self.range_m, "baseline_m": self.baseline_m,
            "line_rms_m": self.line_rms_m, "valid": int(self.valid), "reason": self.reason,
        }


def _as_tag(obj):
    if hasattr(obj, "t_cam"):
        return (obj.id, obj.t_cam, obj.r_cam,
                getattr(obj, "timestamp_ns", None), getattr(obj, "recv_time", None))
    tag_id, t_cam, r_cam = obj[0], obj[1], obj[2]
    ts = obj[3] if len(obj) > 3 else None
    rt = obj[4] if len(obj) > 4 else None
    return (tag_id, t_cam, r_cam, ts, rt)


def measure_alignment(detections, min_tags=2, allowed_ids=None,
                      max_range_m=DEFAULT_MAX_RANGE_M, line_rms_max_m=LINE_RMS_MAX_M):
    """한 프레임의 검출 목록에서 yaw 오차를 잰다 (규칙은 tag_align.py 와 동일)."""
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

WAIT_TAGS = "WAIT_TAGS"
CORRECTING = "CORRECTING"
HOLDING = "HOLDING"
ALIGNED = "ALIGNED"
FAILED = "FAILED"


class AlignCommand(object):
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
        return "<AlignCommand {} t={:.1f}s{}{} {}>".format(self.state, self.elapsed_s, e, d, self.reason)


class AlignController(object):
    """태그 정렬 상태기계. 미션 루프가 주기적으로 update() 를 부른다.

      - 최근 window 개 측정의 중앙값으로 판단 (한 프레임 튐 무시).
      - |오차| > tol 이면 보정 명령 delta = clip(gain × (−오차), ±max_step). 명령 후
        ack_correction() 까지, 그리고 그 뒤 settle_s 동안 측정을 버린다.
      - |오차| ≤ tol 이 hold_s 동안, min_hold_frames 개 이상 유지되면 ALIGNED.
      - timeout_s 를 넘기면 FAILED.
    """

    def __init__(self, tol_deg=1.5, hold_s=1.0, timeout_s=15.0,
                 gain=0.8, max_step_deg=15.0, min_step_deg=0.3,
                 window=3, max_age_s=0.6, settle_s=0.4, min_hold_frames=3, min_tags=2):
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
        self.corrections = []
        self.n_measurements = 0
        self._done_reason = ""
        self._hist = collections.deque(maxlen=self.window)

    def start(self, now):
        self.reset()
        self.t_start = now

    def ack_correction(self, now):
        self.awaiting_ack = False
        self.settle_until = now + self.settle_s
        self._hist.clear()
        self.hold_since = None
        self.hold_frames = 0
        if self.state == CORRECTING:
            self.state = WAIT_TAGS

    def update(self, meas, now):
        if self.t_start is None:
            self.start(now)
        elapsed = now - self.t_start

        if self.state in (ALIGNED, FAILED):
            return AlignCommand(self.state, None, self._median_error(now), self._done_reason, elapsed)

        if meas is not None:
            self.n_measurements += 1
            if meas.valid and meas.n_tags >= self.min_tags:
                if not self.awaiting_ack and now >= self.settle_until:
                    rt = meas.recv_time if meas.recv_time is not None else now
                    self._hist.append((rt, meas.yaw_error_deg))
                    self.t_last_valid = now
            else:
                self.last_invalid_reason = meas.reason if meas.reason != "ok" else "too_few_tags"

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
