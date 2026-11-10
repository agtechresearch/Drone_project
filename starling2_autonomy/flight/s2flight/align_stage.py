"""정렬 단계 실행기 — Vehicle + TagFeed + AlignController 를 묶어 "정밀 정렬" 을 수행한다 (docs/09 §5.2 3단계).

    result = await run_alignment(vehicle, feed, controller, recorder, params)
    result.state in (ALIGNED, FAILED); result.t0 = 정렬 완료 시각(CLOCK_MONOTONIC 초)

루프 (period 마다)
    1. feed.poll(now) 로 닫힌 프레임을 받는다. 각 프레임을 measure_alignment 로 재고 기록한다.
    2. 마지막 측정을 controller.update() 에 넣는다 (프레임이 없으면 None).
    3. 보정 명령이 나오면 vehicle.turn(delta) 로 회전하고 controller.ack_correction().
    4. ALIGNED → t0 확정. FAILED → 호출자가 착륙시킨다. vehicle.aborted 면 즉시 FAILED.
"""

import asyncio
import math

from . import align as al
from .util import log


class AlignParams(object):
    def __init__(self, tol_deg=1.5, hold_s=1.0, timeout_s=15.0, gain=0.8, max_step_deg=15.0,
                 settle_s=0.4, min_tags=2, max_range_m=al.DEFAULT_MAX_RANGE_M, allowed_ids=None,
                 yaw_speed_deg_s=20.0, period_s=0.05, expected_tag_size_m=0.20):
        self.tol_deg = tol_deg
        self.hold_s = hold_s
        self.timeout_s = timeout_s
        self.gain = gain
        self.max_step_deg = max_step_deg
        self.settle_s = settle_s
        self.min_tags = min_tags
        self.max_range_m = max_range_m
        self.allowed_ids = allowed_ids
        self.yaw_speed_deg_s = yaw_speed_deg_s
        self.period_s = period_s
        self.expected_tag_size_m = expected_tag_size_m

    def make_controller(self):
        return al.AlignController(tol_deg=self.tol_deg, hold_s=self.hold_s, timeout_s=self.timeout_s,
                                  gain=self.gain, max_step_deg=self.max_step_deg, settle_s=self.settle_s,
                                  min_tags=self.min_tags)

    def as_dict(self):
        return dict(self.__dict__)


class AlignResult(object):
    def __init__(self):
        self.state = al.FAILED
        self.reason = ""
        self.t0 = None                  # CLOCK_MONOTONIC 초
        self.t0_frame_ns = None         # 정렬 완료 판정에 쓰인 마지막 프레임의 카메라 타임스탬프
        self.final_yaw_error_deg = float("nan")
        self.final_yaw_cmd_deg = float("nan")
        self.final_yaw_fb_deg = float("nan")
        self.corrections = []
        self.n_frames = 0
        self.n_valid = 0
        self.elapsed_s = 0.0
        self.size_warning = None

    @property
    def ok(self):
        return self.state == al.ALIGNED

    def as_dict(self):
        return dict(self.__dict__)


async def run_alignment(vehicle, feed, params, recorder=None, controller=None, name="align"):
    ctl = controller or params.make_controller()
    res = AlignResult()
    now = vehicle.now()
    ctl.start(now)
    vehicle.event("align_start", name, tol_deg=params.tol_deg, timeout_s=params.timeout_s)
    vehicle.set_sync_field("align_state", ctl.state)
    vehicle.set_sync_field("align_err_deg", "")
    log("[align] start: tol={:.2f}deg hold={:.1f}s timeout={:.1f}s feed={}".format(
        params.tol_deg, params.hold_s, params.timeout_s, feed.describe()))

    last_meas = None
    last_frame_ns = None
    last_print = -1.0
    n_corr = 0
    while True:
        if vehicle.aborted:
            res.state = al.FAILED
            res.reason = "vehicle aborted: {}".format(vehicle.abort_reason)
            break
        now = vehicle.now()
        frames = feed.poll(now)
        meas = None
        for f in frames:
            res.n_frames += 1
            m = al.measure_alignment(f.tags, min_tags=params.min_tags, allowed_ids=params.allowed_ids,
                                     max_range_m=params.max_range_m)
            if m.recv_time is None:
                m.recv_time = f.recv_time
            if m.timestamp_ns is None:
                m.timestamp_ns = f.timestamp_ns
            if m.valid:
                res.n_valid += 1
            if res.size_warning is None and f.tags:
                sz = f.tags[0].size_m
                if abs(sz - params.expected_tag_size_m) > 1e-3:
                    res.size_warning = ("tag_detections size_m={:.3f} but expected {:.3f}: "
                                        "tag_locations.conf 크기 설정이 다르다 (거리 {:.1f}배)").format(
                        sz, params.expected_tag_size_m, sz / params.expected_tag_size_m)
                    log("[align] WARNING {}".format(res.size_warning))
            if recorder is not None:
                row = m.as_row()
                row.update({"mono_time": now, "state": ctl.state, "n_tags_frame": len(f.tags)})
                recorder.add_row("align_log", row)
            meas = m
            last_frame_ns = f.timestamp_ns
        if meas is not None:
            last_meas = meas

        cmd = ctl.update(meas, now)
        vehicle.set_sync_field("align_state", cmd.state)
        vehicle.set_sync_field("align_err_deg", "" if math.isnan(cmd.yaw_error_deg) else cmd.yaw_error_deg)

        if cmd.elapsed_s - last_print >= 1.0 or cmd.yaw_delta_deg is not None or cmd.done:
            last_print = cmd.elapsed_s
            log("[align] t={:4.1f}s {} frames={} valid={} {}".format(
                cmd.elapsed_s, cmd.state, res.n_frames, res.n_valid, cmd.reason))

        if cmd.yaw_delta_deg is not None:
            n_corr += 1
            delta = cmd.yaw_delta_deg
            log("[align] correction #{}: err {:+.2f}deg -> turn {:+.2f}deg".format(n_corr, cmd.yaw_error_deg, delta))
            vehicle.event("align_correction", name, n=n_corr, yaw_error_deg=cmd.yaw_error_deg, delta_deg=delta)
            ok = await vehicle.turn(delta, "{}_corr{}".format(name, n_corr), "ALIGN", yaw_speed_deg_s=params.yaw_speed_deg_s)
            res.corrections.append((now, cmd.yaw_error_deg, delta, ok))
            ctl.ack_correction(vehicle.now())
            if vehicle.aborted:
                continue

        if cmd.done:
            res.state = cmd.state
            res.reason = cmd.reason
            res.elapsed_s = cmd.elapsed_s
            res.final_yaw_error_deg = cmd.yaw_error_deg
            break

        await asyncio.sleep(params.period_s)

    res.final_yaw_cmd_deg = vehicle.current_target_yaw
    res.final_yaw_fb_deg = vehicle.feedback_yaw()
    if res.state == al.ALIGNED:
        res.t0 = vehicle.now()
        res.t0_frame_ns = last_frame_ns
        vehicle.set_sync_field("align_state", al.ALIGNED)
        vehicle.event("aligned", res.reason, t0_mono=res.t0, t0_frame_ns=res.t0_frame_ns,
                      yaw_error_deg=res.final_yaw_error_deg, yaw_cmd_deg=res.final_yaw_cmd_deg,
                      yaw_fb_deg=res.final_yaw_fb_deg, corrections=len(res.corrections))
        log("[align] ALIGNED t0={:.3f} yaw_err={:+.2f}deg yaw_cmd={:+.1f} yaw_fb={:+.1f} corrections={}".format(
            res.t0, res.final_yaw_error_deg, res.final_yaw_cmd_deg, res.final_yaw_fb_deg, len(res.corrections)))
    else:
        vehicle.event("align_failed", res.reason, frames=res.n_frames, valid=res.n_valid)
        log("[align] FAILED: {} (frames={} valid={})".format(res.reason, res.n_frames, res.n_valid))
    return res
