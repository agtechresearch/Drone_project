#!/usr/bin/env python3
"""
마커 기반 드리프트 측정 실험 비행 (docs/09 §5.2 공통 비행 시퀀스). s2flight 패키지 위에서 돈다.

 0. 프리플라이트   MAVSDK 연결, 위치 health, 초기 위치, 정면(arm 직전 heading) 확정, 태그 피드 확인, voxl-logger 시작
 1. 이륙          목표 고도(0.6 / 2.0 m). 현수막은 기체 좌측
 2. 조준 회전      좌회전 90° (--turn-deg, 정면 기준 상대)
 3. 정밀 정렬      tracking_front 태그 2개 연결선 → yaw 오차 폐루프 (s2flight.align_stage). 15 s 안에 실패하면 착륙
 3b. 센터링        태그 start(0) 가 카메라 정중앙·이격 70 cm 에 오도록 작은 이동 (s2flight.drift.center_on_tag) → 여기가 t0
 4. 호버 기준      10 s
 5. 측면 이동      body 우측으로 0.5 m/s. **태그 end(10) 이 카메라 중앙에 오면 정지** (도달 판정 없음, Vehicle.move_until).
                  그동안 보이는 태그마다 "자기 추정 위치 vs 태그 실제 위치" 를 drift_log 에 기록. 끝 태그를 못 찾으면 그 자리 착륙
 6. 도착 호버      5 s
 7. 복귀          좌측으로 태그 start 가 중앙에 올 때까지, 5 s 호버. 못 찾으면 그 자리 착륙
 8. 착륙          preland 하강 → land

기체 자기 추정(이동량)은 비행에 쓰지 않고 기록만 한다. 그것과 태그가 말하는 실제 위치의 차이가 드리프트다 (2026-11-09 설계 확정).
--stop-mode distance 를 주면 옛 방식(자기 추정으로 --distance 만큼, 도달 판정)으로 간다.

기체에서:
    python3 marker_drift_flight.py --altitude 0.6 --condition A --run 1
    python3 marker_drift_flight.py --altitude 2.0 --condition C --run 1 --start-tag 20 --end-tag 30
    python3 marker_drift_flight.py --no-align --distance 0.5 --condition T    # 패키지 호버 점검 비행 (시뮬 통과 후)

시뮬레이션(drone_sim_gz, gz Harmonic SITL — 실기 전에 여기서 먼저 돌린다; sim/marker_drift_gz/README.md):
    python3 marker_drift_flight.py --sim --port 50053 --sim-tag-port 14682 -y      # run_sim_drift.sh 가 이렇게 부른다
    --sim: 포즈 = MAVSDK 텔레메트리(voxl-inspect-pose 대신), 태그 = UDP JSON(sim_tag_detect.py), voxl-logger 생략

산출물 (--out-dir, 기본 /home/root/marker_drift_logs/<시각>_<조건><회차>/)
    <prefix>_sync.csv     1 s 주기 동기 샘플 (mono_time = CLOCK_MONOTONIC = voxl-logger 시계)
    <prefix>_events.csv   단계 시작/끝, aligned(t0), 착륙 등 순간 기록 → 후처리 --hover/--move/--end 창
    <prefix>_align_log.csv 정렬 중 프레임별 yaw 오차
    <prefix>_drift_log.csv 태그 프레임마다: 실제 along/standoff(태그), 자기 추정 own_along/standoff, 오차 err_along/err_standoff, yaw
    <prefix>_stages.csv   단계 중 MAVSDK 갱신마다의 샘플
    <prefix>_summary.json 파라미터·결과·voxl-logger 폴더
    <prefix>_console.log  콘솔 로그 사본

voxl-logger 폴더(/data/voxl-logger/logNNNN)는 summary.json 의 "voxl_logger_dir" 에 적힌다.
후처리: analysis/marker_drift (README 참조).
"""

import argparse
import asyncio
import os
import signal
import sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from s2flight import __version__ as S2_VERSION                     # noqa: E402
from s2flight import align as al                                    # noqa: E402
from s2flight import mpa                                            # noqa: E402
from s2flight.align_stage import AlignParams, run_alignment         # noqa: E402
from s2flight.config import FlightConfig                            # noqa: E402
from s2flight.drift import BannerLayout, DriftMonitor, center_on_tag  # noqa: E402
from s2flight.recorder import FlightRecorder                        # noqa: E402
from s2flight.sim import DEFAULT_TAG_PORT, UdpTagFeed               # noqa: E402
from s2flight.tags import MpaTagFeed                                # noqa: E402
from s2flight.util import log, set_log_file                         # noqa: E402
from s2flight.vehicle import Vehicle                                # noqa: E402
from s2flight.voxl_logger import VoxlLogger                         # noqa: E402

DEFAULT_OUT_ROOT = "/home/root/marker_drift_logs"


class DriftParams(object):
    """실험 시퀀스 파라미터 (docs/09 §5.2 + 2026-11-09 태그 정지 설계)."""

    def __init__(self, altitude=0.6, distance=5.0, speed=0.5, turn_deg=-90.0, turn_speed=20.0,
                 post_turn_hover=2.0, hover_ref=10.0, hover_far=5.0, hover_return=5.0,
                 do_align=True, align=None, condition="A", run=1, standoff=0.70,
                 stop_mode="tag", start_tag=0, end_tag=10, tag_spacing=0.5, center=True, center_tol=0.05,
                 center_standoff=True, max_extra=1.0, tag_stale=0.4):
        self.altitude = altitude
        self.distance = distance              # stop_mode=distance 에서만 비행에 쓰인다
        self.speed = speed
        self.turn_deg = turn_deg
        self.turn_speed = turn_speed
        self.post_turn_hover = post_turn_hover
        self.hover_ref = hover_ref
        self.hover_far = hover_far
        self.hover_return = hover_return
        self.do_align = do_align
        self.align = align or AlignParams()
        self.condition = condition
        self.run = run
        self.standoff = standoff
        self.stop_mode = stop_mode            # "tag": 끝 태그 중앙에서 정지 / "distance": 자기 추정 거리(옛 방식)
        self.start_tag = start_tag
        self.end_tag = end_tag
        self.tag_spacing = tag_spacing
        self.center = center                  # 정렬 뒤 start 태그 센터링
        self.center_tol = center_tol          # 중앙 판정 허용 m (정지 조건에도 쓴다)
        self.center_standoff = center_standoff  # 센터링 때 이격도 standoff 로 맞춘다
        self.max_extra = max_extra            # 태그 모드 안전 상한 = 현수막 길이 + 이 값
        self.tag_stale = tag_stale            # 이보다 오래된 검출은 정지 판정에 안 쓴다 s

    def layout(self):
        return BannerLayout(self.start_tag, self.end_tag, self.tag_spacing)

    def as_dict(self):
        d = dict(self.__dict__)
        d["align"] = self.align.as_dict()
        d["layout"] = self.layout().as_dict()
        return d


async def run_sequence(v, params, feed=None, recorder=None):
    """이륙부터 착륙까지. 반환: 요약 dict. 예외는 호출자가 비상정지로 처리한다."""
    p = params
    layout = p.layout()
    summary = {"params": p.as_dict(), "align": None, "center": None, "legs": [], "legs_settled": [], "completed": False,
               "aborted": False, "landed_in_place": False, "t0": None}
    # legs: 정지 순간(이동 중 피드백 지연 포함) / legs_settled: 뒤따르는 호버가 끝난 정지 상태 — 드리프트 수치는 settled 를 본다
    mon = None
    if feed is not None:
        mon = DriftMonitor(v, feed, layout, recorder=recorder, max_range_m=p.align.max_range_m, stale_s=p.tag_stale)
    if p.stop_mode == "tag" and mon is None:
        raise RuntimeError("stop_mode=tag 인데 태그 피드가 없다 (--stop-mode distance 또는 검출기 기동)")
    v.event("sequence_start", "condition={} run={} stop_mode={} tags {}..{}".format(p.condition, p.run, p.stop_mode,
                                                                                    p.start_tag, p.end_tag))

    async def land_here(why):
        log("!!! {} -> landing here.".format(why))
        summary["landed_in_place"] = True
        if mon is not None:
            mon.set_leg("land")
        await v.preland_and_land()

    try:
        ok = await v.takeoff(p.altitude)
        if v.aborted or not ok:
            log("!!! takeoff did not pass ({}). Landing.".format("aborted" if v.aborted else "not reached"))
            await v.land_and_disarm()
            summary["aborted"] = True
            return summary

        await v.turn(p.turn_deg, "aim_turn", "TURN{:+.0f}".format(p.turn_deg), yaw_speed_deg_s=p.turn_speed)
        await v.hover(p.post_turn_hover, "post_turn")
        if v.aborted:
            await v.land_and_disarm()
            summary["aborted"] = True
            return summary

        if p.do_align:
            if mon is None:
                raise RuntimeError("do_align=True 인데 태그 피드가 없다")
            mon.set_leg("align")
            res = await run_alignment(v, mon, p.align, recorder=recorder)       # mon 이 피드 역할 (기록도 됨)
            summary["align"] = res.as_dict()
            if not res.ok:
                log("!!! alignment failed: {}. Landing per plan (docs/09 §5.2).".format(res.reason))
                await v.preland_and_land()
                summary["aborted"] = True
                return summary
            if p.center:
                mon.set_leg("center")
                c = await center_on_tag(v, mon, p.start_tag, tol_m=p.center_tol,
                                        standoff_m=p.standoff if p.center_standoff else None,
                                        speed_mps=min(0.2, p.speed))
                summary["center"] = c
                if v.aborted:
                    await v.land_and_disarm()
                    summary["aborted"] = True
                    return summary
                if not c["ok"] and "not seen" in c["reason"]:
                    await land_here("start tag {} not visible for centering".format(p.start_tag))
                    return summary
                if not c["ok"]:
                    log("!!! centering not converged ({}); continuing — reference uses measured tag position".format(c["reason"]))
        else:
            v.event("align_skipped", "--no-align")
            summary["align"] = {"state": "SKIPPED", "t0": v.now()}

        if mon is not None:
            ref = mon.set_reference("t0")
            summary["t0"] = ref["mono_time"]
            summary["reference"] = ref
            mon.set_leg("hover_ref")
            mon.start()
        await v.hover(p.hover_ref, "hover_ref")
        if v.aborted:
            await v.land_and_disarm()
            summary["aborted"] = True
            return summary

        if p.stop_mode == "tag":
            max_dist = layout.length_m + p.max_extra
            mon.set_leg("out")
            ok = await v.move_until(0.0, +1.0, "move_out", "RIGHT", mon.centered_stop(p.end_tag, +1, p.center_tol),
                                    p.speed, max_dist)
            r = v.results[-1]
            summary["legs"].append(mon.leg_summary("out", r["status"], r["stop_reason"]))
            if v.aborted:
                await v.land_and_disarm()
                summary["aborted"] = True
                return summary
            if not ok:
                await land_here("end tag {} not centered ({}: {})".format(p.end_tag, r["status"], r["stop_reason"]))
                return summary
            mon.set_leg("hover_far")
            await v.hover(p.hover_far, "hover_far")
            summary["legs_settled"].append(mon.leg_summary("out_settled", "HOVER_END", "after hover_far"))
            mon.set_leg("back")
            ok = await v.move_until(0.0, -1.0, "move_back", "LEFT", mon.centered_stop(p.start_tag, -1, p.center_tol),
                                    p.speed, max_dist)
            r = v.results[-1]
            summary["legs"].append(mon.leg_summary("back", r["status"], r["stop_reason"]))
            if v.aborted:
                await v.land_and_disarm()
                summary["aborted"] = True
                return summary
            if not ok:
                await land_here("start tag {} not centered on return ({}: {})".format(p.start_tag, r["status"], r["stop_reason"]))
                return summary
            mon.set_leg("hover_return")
            await v.hover(p.hover_return, "hover_return")
            summary["legs_settled"].append(mon.leg_summary("back_settled", "HOVER_END", "after hover_return"))
        else:
            await v.move_body(0.0, +p.distance, "move_out", "RIGHT", speed_mps=p.speed)
            if mon is not None:
                summary["legs"].append(mon.leg_summary("out", v.results[-1]["status"], "distance mode"))
            await v.hover(p.hover_far, "hover_far")
            await v.move_body(0.0, -p.distance, "move_back", "LEFT", speed_mps=p.speed)
            if mon is not None:
                summary["legs"].append(mon.leg_summary("back", v.results[-1]["status"], "distance mode"))
            await v.hover(p.hover_return, "hover_return")
        if v.aborted:
            await v.land_and_disarm()
            summary["aborted"] = True
            return summary

        if mon is not None:
            mon.set_leg("land")
        await v.preland_and_land()
        summary["completed"] = not v.aborted
        summary["aborted"] = v.aborted
        return summary
    finally:
        if mon is not None:
            mon.stop()
            summary["drift_stats"] = mon.stats


def build_arg_parser():
    p = argparse.ArgumentParser(description="Starling 2 마커 드리프트 측정 실험 비행 (s2flight {})".format(S2_VERSION))
    e = p.add_argument_group("실험 시퀀스 (docs/09 §5.2)")
    e.add_argument("--altitude", type=float, default=0.6, help="비행 고도 m (조건 A/B 0.6, C 2.0)")
    e.add_argument("--distance", type=float, default=5.0,
                   help="--stop-mode distance 에서의 측면 이동 거리 m. 태그 모드에서는 안 쓰임(현수막 길이 = 간격 x 태그 수)")
    e.add_argument("--standoff", type=float, default=0.70,
                   help="현수막 평면과 기체의 이격 m (2026-11-09 확정 0.70). 비행에는 안 쓰이고 조종자 안내·요약 기록용")
    e.add_argument("--speed", type=float, default=0.5, help="측면 이동 속도 m/s")
    e.add_argument("--turn-deg", type=float, default=-90.0, help="조준 회전 (NED 상대, 음수 = 좌회전)")
    e.add_argument("--turn-speed", type=float, default=20.0, help="조준 회전 속도 deg/s")
    e.add_argument("--post-turn-hover", type=float, default=2.0)
    e.add_argument("--hover-ref", type=float, default=10.0, help="정렬 후 기준 호버 s")
    e.add_argument("--hover-far", type=float, default=5.0, help="도착 호버 s")
    e.add_argument("--hover-return", type=float, default=5.0, help="복귀 호버 s")
    e.add_argument("--condition", default="A", help="조건 표기 A/B/C (파일명·요약에만 쓰임)")
    e.add_argument("--run", type=int, default=1, help="회차 (파일명·요약에만 쓰임)")

    a = p.add_argument_group("정렬 (s2flight.align_stage)")
    a.add_argument("--no-align", action="store_true", help="태그 정렬 생략 (패키지 점검 비행용)")
    a.add_argument("--align-tol", type=float, default=1.5, help="허용오차 deg. 정지 촬영 σ 로 max(1, 3σ) 갱신 (§5.3)")
    a.add_argument("--align-hold", type=float, default=1.0)
    a.add_argument("--align-timeout", type=float, default=15.0, help="초과 시 착륙 (확정)")
    a.add_argument("--align-gain", type=float, default=0.8)
    a.add_argument("--align-max-step", type=float, default=15.0)
    a.add_argument("--align-settle", type=float, default=0.4)
    a.add_argument("--align-yaw-speed", type=float, default=20.0)
    a.add_argument("--align-max-range", type=float, default=al.DEFAULT_MAX_RANGE_M,
                   help="이보다 먼 태그는 무시 m. tag_locations 크기가 0.4 로 남아 있으면 거리가 5.7배로 나와 걸러진다")
    a.add_argument("--tag-size", type=float, default=0.20,
                   help="기대 태그 크기 m (2026-11-09 확정 0.20). tag_locations.conf 와 다르면 첫 프레임에 경고")
    a.add_argument("--tag-pipe", default="tag_detections")
    a.add_argument("--sim-tag-port", type=int, default=DEFAULT_TAG_PORT,
                   help="--sim 일 때 시뮬 태그 검출기(sim/marker_drift_gz/sim_tag_detect.py) UDP 포트")

    t = p.add_argument_group("정지 조건 / 드리프트 기록 (s2flight.drift)")
    t.add_argument("--stop-mode", choices=["tag", "distance"], default="tag",
                   help="tag: 끝 태그가 카메라 중앙에 오면 정지(기본, 도달 판정 없음) / distance: 자기 추정으로 --distance 만큼(옛 방식)")
    t.add_argument("--start-tag", type=int, default=0, help="시작 태그 id (정렬·센터링·복귀 정지). 상단열이면 20")
    t.add_argument("--end-tag", type=int, default=10, help="끝 태그 id (우측 이동 정지). 상단열이면 30")
    t.add_argument("--tag-spacing", type=float, default=0.5, help="태그 간격 m → 실제 위치 환산")
    t.add_argument("--center-tol", type=float, default=0.05, help="'중앙' 판정 허용 m (센터링·정지 공통)")
    t.add_argument("--no-center", action="store_true", help="정렬 뒤 start 태그 센터링 생략")
    t.add_argument("--no-standoff-correct", action="store_true", help="센터링 때 이격(--standoff) 보정 생략, 좌우만")
    t.add_argument("--max-extra", type=float, default=1.0, help="태그 모드 안전 상한 = 현수막 길이 + 이 값 m. 넘으면 그 자리 착륙")
    t.add_argument("--tag-stale", type=float, default=0.4, help="이보다 오래된 검출은 정지 판정에 쓰지 않음 s")

    r = p.add_argument_group("기록")
    r.add_argument("--out-dir", default="", help="산출물 폴더. 기본 {}/<시각>_<조건><회차>".format(DEFAULT_OUT_ROOT))
    r.add_argument("--no-logger", action="store_true", help="voxl-logger 를 띄우지 않는다")
    r.add_argument("--logger-cam", nargs="*", default=["hires_small_color"],
                   help="voxl-logger -c 카메라. 후처리 정밀도는 hires_large_color 권장이나 부하 시험 후 (docs/10)")
    r.add_argument("--logger-max-sec", type=int, default=300, help="voxl-logger -t 상한 s")
    r.add_argument("--logger-required", action="store_true", help="voxl-logger 가 못 뜨면 비행하지 않는다")

    FlightConfig.add_arguments(p)
    p.add_argument("-y", "--yes", action="store_true", help="확인 프롬프트 생략")
    return p


def make_params(args):
    allowed = set(range(args.start_tag, args.end_tag + 1))
    align = AlignParams(tol_deg=args.align_tol, hold_s=args.align_hold, timeout_s=args.align_timeout,
                        gain=args.align_gain, max_step_deg=args.align_max_step, settle_s=args.align_settle,
                        max_range_m=args.align_max_range, allowed_ids=allowed,
                        yaw_speed_deg_s=args.align_yaw_speed, expected_tag_size_m=args.tag_size)
    return DriftParams(altitude=args.altitude, distance=args.distance, speed=args.speed,
                       turn_deg=args.turn_deg, turn_speed=args.turn_speed, post_turn_hover=args.post_turn_hover,
                       hover_ref=args.hover_ref, hover_far=args.hover_far, hover_return=args.hover_return,
                       do_align=not args.no_align, align=align, condition=args.condition, run=args.run,
                       standoff=args.standoff, stop_mode=args.stop_mode, start_tag=args.start_tag, end_tag=args.end_tag,
                       tag_spacing=args.tag_spacing, center=not args.no_center, center_tol=args.center_tol,
                       center_standoff=not args.no_standoff_correct, max_extra=args.max_extra, tag_stale=args.tag_stale)


def print_banner(args, params, cfg, out_dir):
    p = params
    print("=" * 78)
    print("Starling 2 — MARKER DRIFT EXPERIMENT FLIGHT  (s2flight {})".format(S2_VERSION))
    print("  Mode          : {}".format("SITL (--sim: pose = MAVSDK telemetry, tags = UDP, no voxl-logger)" if cfg.sim
                                       else "REAL VEHICLE (VOXL pipes)"))
    print("  Condition/run : {} #{}".format(p.condition, p.run))
    print("  Altitude      : {:.2f} m".format(p.altitude))
    lay = p.layout()
    if p.stop_mode == "tag":
        leg_out = "right @ {:.2f} m/s until tag {} centred (banner {:.1f} m, max {:.1f} m else LAND)".format(
            p.speed, p.end_tag, lay.length_m, lay.length_m + p.max_extra)
        leg_back = "left until tag {} centred".format(p.start_tag)
    else:
        leg_out = "right {:.2f} m @ {:.2f} m/s (own estimate, reach judged)".format(p.distance, p.speed)
        leg_back = "left {:.2f} m".format(p.distance)
    print("  Sequence      : takeoff -> turn {:+.0f} deg -> {}{} -> hover {:.0f}s -> {} -> hover {:.0f}s -> {} -> hover {:.0f}s -> preland -> land".format(
        p.turn_deg, "ALIGN(tol {:.1f} deg, {:.0f}s)".format(p.align.tol_deg, p.align.timeout_s) if p.do_align else "no-align",
        " -> CENTER tag {} (tol {:.2f} m{})".format(p.start_tag, p.center_tol, ", standoff {:.2f}".format(p.standoff) if p.center_standoff else "")
        if (p.do_align and p.center) else "",
        p.hover_ref, leg_out, p.hover_far, leg_back, p.hover_return))
    print("  Drift record  : every tag frame -> drift_log.csv (own estimate vs tag truth); own estimate NOT used for flight")
    mode, val = cfg.initial_yaw_value()
    print("  Heading ref   : {}".format({"current": "CURRENT heading when armed = 'forward' (place the vehicle facing the flight direction)",
                                        "zero": "VIO power-on heading (v14 behaviour)"}.get(mode, "fixed yaw {:+.1f} deg".format(val or 0.0))))
    print("  Reach judge   : {} '{}'  (h_tol {:.2f} m, v_tol {:.2f} m)".format(
        "MAVSDK telemetry as" if cfg.sim else "voxl-inspect-pose -n", cfg.reach_pipe, cfg.h_tol, cfg.v_tol))
    print("  Geofence      : {}".format("OFF" if cfg.no_geofence else "track h<={:.1f} v<={:.1f} m, persist {:.1f}s".format(
        cfg.max_horizontal_dev, cfg.max_alt_deviation, cfg.geofence_violation_sec)))
    print("  Tag source    : {}{}".format("udp://127.0.0.1:{}".format(args.sim_tag_port) if cfg.sim else args.tag_pipe,
                                        "" if p.do_align else " (unused, --no-align)"))
    print("  voxl-logger   : {}".format("OFF (SITL)" if cfg.sim else "OFF" if args.no_logger
                                        else " ".join(args.logger_cam) + " + ov + px4 pose + imu, max {}s".format(args.logger_max_sec)))
    print("  Output        : {}".format(out_dir))
    print("  Takeoff spot : {:.0f} cm from the banner plane, banner on the LEFT, tags 0/1 ahead-left; {:.1f} m+ clear ahead "
          "(becomes 'right' after the turn). Expected tag size {:.2f} m.".format(p.standoff * 100, p.distance + 0.5, p.align.expected_tag_size_m))
    print("  SITL: no RC needed." if cfg.sim else "  RC in OFFBOARD mode, kill switch ready.")
    print("=" * 78)


async def main_async(args, params, cfg, out_dir, prefix):
    rec = FlightRecorder(cfg, out_dir=out_dir, prefix=prefix)
    rec.summary["params"] = params.as_dict()
    rec.summary["config"] = cfg.as_dict()
    rec.summary["s2flight_version"] = S2_VERSION
    rec.summary["sim"] = cfg.sim
    v = Vehicle(cfg, recorder=rec, sim=cfg.sim)
    logger = None
    feed = None
    loop = asyncio.get_event_loop()

    def sigint_handler():
        log("SIGINT received")
        asyncio.ensure_future(v.emergency_stop())

    try:
        loop.add_signal_handler(signal.SIGINT, sigint_handler)
    except (NotImplementedError, RuntimeError):
        pass

    exit_code = 0
    try:
        await v.connect()
        await v.wait_position_stable()
        await v.check_vio_quality_preflight()
        await v.capture_initial_mavsdk_position()

        need_feed = params.do_align or params.stop_mode == "tag"
        if need_feed and cfg.sim:
            feed = UdpTagFeed(port=args.sim_tag_port, size_m=params.align.expected_tag_size_m)
            feed.start(loop)
        elif need_feed:
            if not mpa.pipe_available(args.tag_pipe):
                raise RuntimeError("tag pipe '{}' 가 없다. voxl-tag-detector 를 먼저 띄울 것 "
                                   "(systemctl start voxl-tag-detector) 또는 --no-align".format(args.tag_pipe))
            feed = MpaTagFeed(args.tag_pipe)
            feed.start(loop)
            log("[tags] subscribed {} as {}".format(args.tag_pipe, feed.client.client_name))

        if cfg.sim:
            rec.summary["voxl_logger_started"] = False
        elif not args.no_logger:
            logger = VoxlLogger(name="marker_drift_{}{}".format(params.condition, params.run),
                                cameras=args.logger_cam, max_seconds=args.logger_max_sec)
            started = await logger.start(required=args.logger_required)
            rec.summary["voxl_logger_cmd"] = " ".join(logger.command())
            rec.summary["voxl_logger_started"] = started
            rec.event("logger_start" if started else "logger_not_started", logger.log_dir or "")

        await v.start_monitors_and_offboard()
        seq = await run_sequence(v, params, feed=feed, recorder=rec)
        rec.summary["sequence"] = seq
    except Exception as e:
        log("Exception: {!r}".format(e))
        rec.summary["exception"] = repr(e)
        exit_code = 1
        try:
            await v.emergency_stop()
        except Exception:
            pass
    finally:
        if feed is not None:
            try:
                rec.summary["tag_feed_stats"] = feed.stats
                feed.stop()
            except Exception:
                pass
        if logger is not None:
            try:
                rec.summary["voxl_logger_dir"] = await logger.stop()
            except Exception as e:
                log("[logger] stop error: {}".format(e))
        rec.summary["vehicle"] = v.summary_dict()
        try:
            v.print_summary()
        except Exception:
            pass
        rec.write_all()
    return exit_code


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    if args.distance <= 0 or args.altitude <= 0 or args.speed <= 0:
        parser.error("--distance, --altitude, --speed 는 양수여야 한다")
    cfg = FlightConfig.from_args(args)
    params = make_params(args)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = "md_{}{}_{}".format(params.condition, params.run, stamp)
    # /home/root 는 기체에만 있다. 시뮬 컨테이너·PC 에서는 현재 폴더 아래 (v13 의 --csv auto 폴백과 같은 규칙)
    out_root = DEFAULT_OUT_ROOT if os.path.isdir(os.path.dirname(DEFAULT_OUT_ROOT)) else os.path.join(os.getcwd(), "marker_drift_logs")
    out_dir = args.out_dir or os.path.join(out_root, "{}_{}{}".format(stamp, params.condition, params.run))
    try:
        os.makedirs(out_dir, exist_ok=True)
        set_log_file(os.path.join(out_dir, prefix + "_console.log"))
    except OSError as e:
        print("out_dir 생성 실패 {}: {}".format(out_dir, e))
        sys.exit(2)

    print_banner(args, params, cfg, out_dir)
    if not args.yes:
        input("Type Enter when confirmed...")
    rc = asyncio.get_event_loop().run_until_complete(main_async(args, params, cfg, out_dir, prefix))
    set_log_file(None)
    sys.exit(rc)


if __name__ == "__main__":
    main()
