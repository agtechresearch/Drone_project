"""태그 기반 실시간 드리프트 기록 + 조건 정지 (마커 드리프트 실험, 2026-11-09 설계 확정).

실험 설계
    이륙 → 좌회전 → 태그 start(0) 정면 정렬·센터링 (= t0) → 우측으로 가면서 보이는 태그마다
    "기체 자기 추정 위치" 와 "태그가 말하는 실제 위치" 의 차이를 기록 → 태그 end(10) 이 카메라 중앙에 오면 정지
    → 호버 → 좌측으로 태그 start 가 중앙에 올 때까지 복귀 → 호버 → 착륙. 정지 조건이 자기 추정이 아니라 마커(실제 세계)다.
    기체 자기 추정은 비행에 쓰지 않고 기록만 한다 — 그 차이가 드리프트다.

좌표
    현수막 좌표 along: 태그 start 를 0 으로 두고 spacing 간격 → along(k) = spacing * (k - start).
    카메라 좌표 (x 우, y 하, z 전) 와 카메라 yaw 오차 ψ(>0 = 카메라가 오른쪽으로 돌아감, align.measure_alignment 와 같은 부호):
        태그의 현수막 along 오프셋  dx = x·cosψ + z·sinψ        (기체 위치에서 태그까지, 오른쪽 +)
        현수막까지 수직 거리       s  = -x·sinψ + z·cosψ
        → 기체 실제 along = along(k) - dx,  실제 standoff = s
    ψ 는 프레임에 태그가 둘 이상이면 연결선 각(정확), 하나면 그 태그의 법선 각(잡음 큼, psi_src=normal).

기체 자기 추정 own_*: t0 에서 피드백(reach 파이프) 위치·정렬 yaw 로 축을 잡고, 그 뒤 피드백 변위를 우측(along)/전방 축에 사영.
    own_along_b = own_along + truth_along(t0)   (t0 에 센터링이 끝난 자리 = 현수막 좌표 원점 근처)
    err_along   = own_along_b - truth_along     (>0: 기체는 실제보다 더 갔다고 생각)
    own_standoff = standoff(t0) - own_fwd,  err_standoff = own_standoff - truth_standoff

DriftMonitor 는 TagFeed 인터페이스(poll)를 그대로 제공하므로 align_stage.run_alignment 에 feed 로 넘길 수 있고(그동안도 기록),
t0 뒤에는 start(loop) 로 백그라운드 폴링을 돌려 호버·이동 중에도 기록한다. 정지 판정은 centered_stop(...) 가 돌려주는
predicate 로 Vehicle.move_until 에 넘긴다.
"""

import asyncio
import collections
import math

from . import align as al
from .tags import TagFeed
from .util import angle_diff_deg, body_delta_to_ned, log, yaw_to_360


class BannerLayout(object):
    """태그 id → 현수막 along 좌표 (m). start..end 만 유효."""

    def __init__(self, start_tag=0, end_tag=9, spacing_m=0.5):      # 2026-11-10 실험장: 태그 0~9 (4.5 m)
        if end_tag <= start_tag:
            raise ValueError("end_tag 는 start_tag 보다 커야 한다")
        self.start_tag = int(start_tag)
        self.end_tag = int(end_tag)
        self.spacing_m = float(spacing_m)

    @property
    def ids(self):
        return set(range(self.start_tag, self.end_tag + 1))

    @property
    def length_m(self):
        return self.spacing_m * (self.end_tag - self.start_tag)

    def along(self, tag_id):
        if tag_id < self.start_tag or tag_id > self.end_tag:
            return None
        return self.spacing_m * (tag_id - self.start_tag)

    def as_dict(self):
        return {"start_tag": self.start_tag, "end_tag": self.end_tag, "spacing_m": self.spacing_m, "length_m": self.length_m}


def banner_coords(x_cam, z_cam, psi_deg):
    """카메라 좌표 → (태그 along 오프셋 dx, 현수막 수직거리 s)."""
    c = math.cos(math.radians(psi_deg))
    s = math.sin(math.radians(psi_deg))
    return x_cam * c + z_cam * s, -x_cam * s + z_cam * c


class FrameEstimate(object):
    """프레임 하나에서 나온 실제 위치 추정."""

    __slots__ = ("timestamp_ns", "recv_time", "ids", "psi_deg", "psi_src", "truth_along", "truth_standoff",
                 "per_tag", "n_tags")

    def __init__(self):
        self.timestamp_ns = None
        self.recv_time = None
        self.ids = []
        self.psi_deg = float("nan")
        self.psi_src = ""
        self.truth_along = float("nan")
        self.truth_standoff = float("nan")
        self.per_tag = {}          # id -> (x_cam, z_cam, dx, s)
        self.n_tags = 0


def estimate_frame(frame, layout, max_range_m=al.DEFAULT_MAX_RANGE_M, max_vertical_m=al.DEFAULT_MAX_VERTICAL_M):
    """TagFrame → FrameEstimate (유효 태그 없으면 n_tags 0)."""
    est = FrameEstimate()
    est.timestamp_ns = frame.timestamp_ns
    est.recv_time = frame.recv_time
    tags = []
    for d in frame.tags:
        if layout.along(d.id) is None:
            continue
        x, y, z = d.t_cam
        if z <= 0.0 or math.sqrt(x * x + y * y + z * z) > max_range_m:
            continue
        if max_vertical_m is not None and abs(y) > max_vertical_m:   # 같은 ID 의 다른 높이 열 배제 (2026-11-10)
            continue
        tags.append(d)
    est.n_tags = len(tags)
    if not tags:
        return est
    tags.sort(key=lambda d: d.id)
    est.ids = [d.id for d in tags]
    if len(tags) >= 2:
        m = al.measure_alignment(tags, min_tags=2, allowed_ids=layout.ids, max_range_m=max_range_m,
                                 max_vertical_m=max_vertical_m)
        if m.valid:
            est.psi_deg = m.yaw_error_deg
            est.psi_src = "line"
    if est.psi_src == "":
        est.psi_deg = sum(al.yaw_from_normal_deg(d.r_cam) for d in tags) / len(tags)
        est.psi_src = "normal"
    alongs = []
    standoffs = []
    for d in tags:
        dx, s = banner_coords(d.t_cam[0], d.t_cam[2], est.psi_deg)
        est.per_tag[d.id] = (d.t_cam[0], d.t_cam[2], dx, s)
        alongs.append(layout.along(d.id) - dx)
        standoffs.append(s)
    est.truth_along = sum(alongs) / len(alongs)
    est.truth_standoff = sum(standoffs) / len(standoffs)
    return est


class DriftMonitor(TagFeed):
    """태그 피드를 소비하며 프레임마다 실제 위치·자기 추정·오차를 기록한다. 정지 판정용 최신 검출도 보관한다."""

    def __init__(self, vehicle, feed, layout, recorder=None, max_range_m=al.DEFAULT_MAX_RANGE_M, stale_s=0.4,
                 table="drift_log", max_vertical_m=al.DEFAULT_MAX_VERTICAL_M):
        self.v = vehicle
        self.feed = feed
        self.layout = layout
        self.rec = recorder
        self.max_range_m = max_range_m
        self.max_vertical_m = max_vertical_m
        self.stale_s = stale_s
        self.table = table
        self.latest = {}                    # tag id -> (x_cam, z_cam, dx, s, recv_time, frame_ns)
        self.last_est = None
        self._last_leg_log = 0.0
        self.leg = "pre"
        self.ref = None                     # 기준(t0): dict
        self.n_frames = 0
        self.n_rows = 0
        self._task = None
        self._recent = collections.deque(maxlen=50)

    # -- TagFeed 인터페이스 ----------------------------------------------------

    def start(self, loop=None, period_s=0.02):
        """백그라운드 폴링 시작 (정렬이 끝난 뒤에만: 정렬 중에는 run_alignment 가 poll 을 부른다)."""
        if self._task is None:
            self._task = asyncio.ensure_future(self._run(period_s))

    async def _run(self, period_s):
        try:
            while True:
                self.poll(self.v.now())
                await asyncio.sleep(period_s)
        except asyncio.CancelledError:
            pass

    def poll(self, now):
        frames = self.feed.poll(now)
        for f in frames:
            self._process(f, now)
        return frames

    def stop(self):
        if self._task is not None:
            self._task.cancel()
            self._task = None

    def describe(self):
        return "DriftMonitor({} tags {}..{} @{:.2f}m)".format(self.feed.describe(), self.layout.start_tag,
                                                             self.layout.end_tag, self.layout.spacing_m)

    @property
    def stats(self):
        return {"frames": self.n_frames, "rows": self.n_rows, "feed": getattr(self.feed, "stats", None)}

    # -- 기준 / 자기 추정 -----------------------------------------------------

    def own_state(self):
        """t0 기준 자기 추정 변위 (along 우측 +, fwd 전방 +, up) 와 yaw. 기준 전이면 None."""
        if self.ref is None:
            return None
        fb = self.v.feedback_state()
        if fb is None:
            return None
        dn = fb["rel_n"] - self.ref["rel_n"]
        de = fb["rel_e"] - self.ref["rel_e"]
        r = self.ref
        along = dn * r["right_n"] + de * r["right_e"]
        fwd = dn * r["fwd_n"] + de * r["fwd_e"]
        up = fb["rel_u"] - r["rel_u"]
        return {"along": along, "fwd": fwd, "up": up, "yaw_deg": fb.get("yaw_deg", float("nan")),
                "along_b": along + r["truth_along"], "standoff": r["truth_standoff"] - fwd}

    def set_reference(self, name="t0"):
        """센터링이 끝난 자리를 기준으로 잡는다: 피드백 위치, 정렬 yaw 축, 그 순간의 태그 실제 좌표."""
        fb = self.v.feedback_state()
        if fb is None:
            raise RuntimeError("reference 를 잡으려면 피드백 위치가 필요하다")
        yaw = self.v.current_target_yaw
        fwd_n, fwd_e = body_delta_to_ned(1.0, 0.0, yaw)
        right_n, right_e = body_delta_to_ned(0.0, 1.0, yaw)
        est = self.fresh_estimate(self.v.now())
        truth_along = est.truth_along if est is not None else 0.0
        truth_standoff = est.truth_standoff if est is not None else float("nan")
        self.ref = {"name": name, "mono_time": self.v.now(), "rel_n": fb["rel_n"], "rel_e": fb["rel_e"], "rel_u": fb["rel_u"],
                    "yaw_deg": yaw, "fwd_n": fwd_n, "fwd_e": fwd_e, "right_n": right_n, "right_e": right_e,
                    "truth_along": truth_along, "truth_standoff": truth_standoff,
                    "ref_from_tags": est is not None, "ref_ids": list(est.ids) if est is not None else []}
        self.v.event("drift_reference", name, truth_along=truth_along, truth_standoff=truth_standoff,
                     yaw_deg=yaw, ref_ids=str(self.ref["ref_ids"]))
        log("[drift] reference '{}': fb_rel=(N{:+.3f},E{:+.3f},U{:+.3f}) yaw={:+.1f} tags {} -> truth along {:+.3f} standoff {:.3f}".format(
            name, fb["rel_n"], fb["rel_e"], fb["rel_u"], yaw, self.ref["ref_ids"], truth_along, truth_standoff))
        return self.ref

    def set_leg(self, leg):
        self.leg = leg
        self.v.set_sync_field("drift_leg", leg)

    # -- 최신 검출 / 정지 판정 -------------------------------------------------

    def fresh(self, tag_id, now, stale_s=None):
        """stale 이 아닌 최신 검출 (x_cam, z_cam, dx, s, recv_time, frame_ns) 또는 None."""
        stale_s = self.stale_s if stale_s is None else stale_s
        d = self.latest.get(tag_id)
        if d is None or now - d[4] > stale_s:
            return None
        return d

    def fresh_estimate(self, now, stale_s=None):
        stale_s = self.stale_s if stale_s is None else stale_s
        e = self.last_est
        if e is None or e.n_tags == 0 or now - e.recv_time > stale_s:
            return None
        return e

    def centered_stop(self, tag_id, direction, tol_m=0.05, stale_s=None):
        """Vehicle.move_until 용 predicate. direction +1 = 우측 이동(태그 x_cam 이 +에서 0 으로 줄어듦), -1 = 좌측.
        태그 along 오프셋 dx 가 direction 기준으로 tol 이하(또는 지나침)면 정지 사유를 돌려준다."""
        sign = 1.0 if direction >= 0 else -1.0

        def _stop(now):
            d = self.fresh(tag_id, now, stale_s)
            if d is None:
                return None
            dx = d[2]
            if sign * dx <= tol_m:
                return "tag {} centered: dx {:+.3f} m (tol {:.3f}, age {:.2f}s)".format(tag_id, dx, tol_m, now - d[4])
            return None
        return _stop

    def standoff_guard(self, min_m, max_m, stale_s=None):
        """Vehicle.move_until 용 predicate (v0.2.10). 태그 진실값 이격이 [min_m, max_m] 을 벗어나면 정지 사유를 돌려준다.
        2026-11-13: 헤딩-위치 프레임 어긋남(±7~10°)으로 우측 이동 중 이격이 0.17 m/m 씩 변한다 → 현수막(또는 통로 B 벽)에 닿기 전에 레그를 끝낸다.
        검출이 없거나 오래되면 None (판단 보류). max_m 에 inf 를 주면 하한만 본다."""
        def _stop(now):
            e = self.fresh_estimate(now, stale_s)
            if e is None or e.truth_standoff != e.truth_standoff:
                return None
            if e.truth_standoff < min_m:
                return "standoff guard: truth standoff {:.2f} m < {:.2f} (tags {})".format(e.truth_standoff, min_m, e.ids)
            if e.truth_standoff > max_m:
                return "standoff guard: truth standoff {:.2f} m > {:.2f} (tags {})".format(e.truth_standoff, max_m, e.ids)
            return None
        return _stop

    # -- 프레임 처리 -----------------------------------------------------------

    def _process(self, frame, now):
        self.n_frames += 1
        est = estimate_frame(frame, self.layout, self.max_range_m, self.max_vertical_m)
        if est.n_tags == 0:
            return
        self.last_est = est
        self._recent.append(est)
        for tid, (x, z, dx, s) in est.per_tag.items():
            self.latest[tid] = (x, z, dx, s, est.recv_time, est.timestamp_ns)
        own = self.own_state()
        row = {
            "mono_time": now, "frame_ns": est.timestamp_ns, "recv_time": est.recv_time, "leg": self.leg,
            "phase": self.v.active_phase, "stage": (self.v.active_stage or {}).get("name", ""),
            "n_tags": est.n_tags, "ids": " ".join(str(i) for i in est.ids),
            "psi_deg": est.psi_deg, "psi_src": est.psi_src,
            "truth_along": est.truth_along, "truth_standoff": est.truth_standoff,
        }
        if own is not None:
            row.update({"own_along": own["along"], "own_along_b": own["along_b"], "own_fwd": own["fwd"], "own_up": own["up"],
                        "own_standoff": own["standoff"], "fb_yaw_deg": own["yaw_deg"],
                        "err_along": own["along_b"] - est.truth_along,
                        "err_standoff": own["standoff"] - est.truth_standoff,
                        "yaw_drift_deg": angle_diff_deg(own["yaw_deg"], self.ref["yaw_deg"]) if own["yaw_deg"] == own["yaw_deg"] else float("nan")})
            self.v.set_sync_field("own_along", own["along_b"])
            self.v.set_sync_field("truth_along", est.truth_along)
            self.v.set_sync_field("err_along", row["err_along"])
            self.v.set_sync_field("err_standoff", row["err_standoff"])
            if self.leg in ("out", "back") and now - self._last_leg_log >= 2.0:     # 비행 중 콘솔: 태그 기준 진실값과 오차 (2 s 마다)
                self._last_leg_log = now
                log("       [drift {}] tags {} psi {:+.1f} | truth along {:.2f} standoff {:.2f} | err along {:+.3f} standoff {:+.3f}".format(
                    self.leg, est.n_tags, est.psi_deg if est.psi_deg == est.psi_deg else float("nan"), est.truth_along, est.truth_standoff,
                    row["err_along"], row["err_standoff"]))
        else:
            row.update({"own_along": "", "own_along_b": "", "own_fwd": "", "own_up": "", "own_standoff": "", "fb_yaw_deg": "",
                        "err_along": "", "err_standoff": "", "yaw_drift_deg": ""})
        self.v.set_sync_field("tag_ids", row["ids"])
        if self.rec is not None:
            self.rec.add_row(self.table, row)
        self.n_rows += 1

    def leg_summary(self, name, status, reason):
        """다리(leg) 끝 요약: 자기 추정 vs 태그 실제 (정지 순간)."""
        own = self.own_state()
        est = self.fresh_estimate(self.v.now(), stale_s=1.0)
        out = {"leg": name, "status": status, "reason": reason,
               "own_along_b": own["along_b"] if own else float("nan"),
               "own_standoff": own["standoff"] if own else float("nan"),
               "truth_along": est.truth_along if est else float("nan"),
               "truth_standoff": est.truth_standoff if est else float("nan"),
               "truth_ids": list(est.ids) if est else []}
        out["drift_along"] = out["own_along_b"] - out["truth_along"]
        out["drift_standoff"] = out["own_standoff"] - out["truth_standoff"]
        self.v.event("leg_end", name, status=status, reason=reason, own_along=out["own_along_b"], truth_along=out["truth_along"],
                     drift_along=out["drift_along"], own_standoff=out["own_standoff"], truth_standoff=out["truth_standoff"],
                     drift_standoff=out["drift_standoff"])
        log("[drift] leg {} {}: own along {:+.3f} vs truth {:+.3f} -> drift {:+.3f} m | standoff own {:.3f} truth {:.3f} -> {:+.3f} m".format(
            name, status, out["own_along_b"], out["truth_along"], out["drift_along"], out["own_standoff"], out["truth_standoff"],
            out["drift_standoff"]))
        return out


def any_stop(*fns):
    """여러 predicate 를 합친다: 먼저 사유를 돌려주는 것이 정지 사유 (v0.2.10)."""
    def _stop(now):
        for f in fns:
            if f is None:
                continue
            r = f(now)
            if r:
                return r
        return None
    return _stop


async def center_on_tag(vehicle, monitor, tag_id, tol_m=0.05, standoff_m=None, standoff_tol_m=0.05, max_iter=3,
                        sample_s=0.6, speed_mps=0.2, name="center"):
    """태그 tag_id 가 카메라 정중앙(along 오프셋 0)에, 그리고 standoff_m 이 주어지면 그 거리에 오도록 작은 이동을 반복한다.

    monitor.poll 을 직접 돌려 sample_s 동안 검출을 모은 뒤 중앙값으로 판단한다. 반환 dict(ok, iterations, dx, standoff).
    """
    v = vehicle
    result = {"ok": False, "iterations": 0, "dx": float("nan"), "standoff": float("nan"), "reason": ""}
    v.event("center_start", name, tag_id=tag_id, tol_m=tol_m, standoff_m=standoff_m if standoff_m is not None else "")
    for it in range(max_iter + 1):
        if v.aborted:
            result["reason"] = "aborted"
            break
        t_end = v.now() + sample_s
        dxs, ss = [], []
        while v.now() < t_end:
            now = v.now()
            monitor.poll(now)
            d = monitor.fresh(tag_id, now, stale_s=0.1)
            if d is not None:
                dxs.append(d[2])
                ss.append(d[3])
            await asyncio.sleep(0.02)
        if not dxs:
            result["reason"] = "tag {} not seen for {:.1f}s".format(tag_id, sample_s)
            log("[center] {}".format(result["reason"]))
            break
        dxs.sort()
        ss.sort()
        dx = dxs[len(dxs) // 2]
        s = ss[len(ss) // 2]
        result.update({"dx": dx, "standoff": s, "iterations": it})
        d_fwd = 0.0 if standoff_m is None else (s - standoff_m)
        log("[center] iter {}: tag {} dx {:+.3f} m standoff {:.3f} m ({} samples)".format(it, tag_id, dx, s, len(dxs)))
        if abs(dx) <= tol_m and abs(d_fwd) <= standoff_tol_m:
            result["ok"] = True
            result["reason"] = "centered"
            break
        if it >= max_iter:
            result["reason"] = "not centered after {} moves (dx {:+.3f}, d_standoff {:+.3f})".format(max_iter, dx, d_fwd)
            break
        ok = await v.move_body(d_fwd, dx, "{}_{}".format(name, it + 1), "CENTER", speed_mps=speed_mps)
        if not ok and v.aborted:
            result["reason"] = "aborted during move"
            break
    v.event("center_end", name, ok=result["ok"], reason=result["reason"], dx=result["dx"], standoff=result["standoff"],
            iterations=result["iterations"])
    return result
