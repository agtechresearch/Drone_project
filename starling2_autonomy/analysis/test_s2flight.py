#!/usr/bin/env python3
"""s2flight 패키지 + marker_drift_flight 시퀀스 테스트 (기체·mavsdk 없이).

가짜 MAVSDK(명령 setpoint 를 1차 지연으로 따라가는 기체)와 가짜 벽 카메라(기체 yaw 와 벽 방향에서
태그 두 개의 카메라 좌표를 계산)를 붙여 전체 실험 시퀀스를 빠른 파라미터로 끝까지 돌린다.

    python analysis/test_s2flight.py
"""
import asyncio
import csv
import json
import math
import os
import pathlib
import random
import shutil
import socket
import sys
import tempfile
import time

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "flight"))

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

from s2flight import align as al                         # noqa: E402
from s2flight import tags, util                           # noqa: E402
from s2flight.align_stage import AlignParams              # noqa: E402
from s2flight.config import FlightConfig                  # noqa: E402
from s2flight.geofence import Geofence                    # noqa: E402
from s2flight.pose import parse_voxl_inspect_pose_line    # noqa: E402
from s2flight.recorder import FlightRecorder              # noqa: E402
from s2flight import sim as s2sim                         # noqa: E402
from s2flight import drift as s2drift                     # noqa: E402
from s2flight.vehicle import Vehicle                      # noqa: E402
from s2flight import vehicle                              # noqa: E402  (OffboardError fallback for retry test)
from s2flight.voxl_logger import VoxlLogger               # noqa: E402
import marker_drift_flight as mdf                         # noqa: E402

results = []


def ok(name, cond, note=""):
    results.append((name, bool(cond)))
    print(("  PASS  " if cond else "  FAIL  ") + name + ("   " + note if note and not cond else ""))


def near(a, b, tol):
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
# 가짜 MAVSDK
# ---------------------------------------------------------------------------

class _Obj(object):
    def __init__(self, **kw):
        self.__dict__.update(kw)


class FakeDrone(object):
    """setpoint 를 1차 지연으로 따라가는 기체. 포즈 파이프는 vehicle.update_pose 로 직접 밀어넣는다."""

    POSE_OFFSET = (1.0, -2.0, 0.3)     # 파이프 원좌표 = NED + 오프셋 (reach 정렬 검증용)

    def __init__(self, yaw0=0.0, gain=0.35, stuck=False, n0=0.0, e0=0.0):
        self.n = n0
        self.e = e0
        self.d = 0.0
        self.yaw = yaw0
        self.gain = gain
        self.stuck = stuck
        self.target = None
        self.armed = False
        self.in_air_flag = False
        self.landing = False
        self.offboard = _Obj(start=self._offboard_start, set_position_ned=self._set_target, set_velocity_body=self._set_vel_body)
        self.vel_body = None            # (fwd, right, down, yawspeed) — offboard 접지 흉내
        self._vb_t = None
        self.action = _Obj(arm=self._arm, land=self._land, disarm=self._disarm, kill=self._kill)
        self.core = _Obj(connection_state=self._conn)
        self.telemetry = _Obj(health=self._health, position_velocity_ned=self._pos, in_air=self._in_air,
                              attitude_euler=self._att, odometry=self._odom, armed=self._armed_stream,
                              status_text=self._status_text)
        self.status_texts = []          # (type, text) 를 넣어 두면 스트림이 한 번씩 내보낸다
        self.odom_lag_s = 0.0          # odometry time_usec 를 이만큼 과거로 찍어 MAVSDK 소비 지연을 흉내
        self.pos_period = 0.05         # position_velocity_ned 발행 주기 (20 Hz 기본; 60 Hz 로 올려 소비 루프를 시험)
        self.vehicle = None
        self.cmd_log = []
        self.ts_ms = 0.0
        # VIO 상태 (2026-11-10 사고 대책 테스트): 기본 양호. 테스트가 바꾼다
        self.vio_quality = 96
        self.vio_features = 11
        self.vio_state = "OKAY"
        self.vio_feed = True
        self.vio_every_n = 1            # n 번째 push_pose 마다 VIO 샘플 (출력률 저하 흉내)
        self._pose_n = 0
        self.extra_pipe_offset = (0.0, 0.0, 0.0)      # 파이프에만 더해지는 오프셋 → MAVSDK 와 불일치 흉내
        self.ekf_yaw_drift_dps = 0.0                  # EKF(attitude_euler) yaw 만 이 속도로 흘러간다 (자이로 바이어스 고장 흉내)
        self.ekf_yaw_offset = 0.0
        self._att_t = None
        self.on_step = None                           # _step 마다 호출되는 훅 (시간 기반 고장 주입)

    async def connect(self):
        pass

    async def _conn(self):
        yield _Obj(is_connected=True)

    async def _arm(self):
        self.armed = True
        self.cmd_log.append("arm")

    async def _disarm(self):
        self.armed = False
        self.cmd_log.append("disarm")

    async def _kill(self):
        self.armed = False
        self.cmd_log.append("kill")

    async def _land(self):
        self.landing = True
        self.cmd_log.append("land")

    async def _offboard_start(self):
        self.cmd_log.append("offboard_start")

    async def _set_target(self, t):
        self.target = t
        self.vel_body = None
        self._step()

    async def _set_vel_body(self, vb):
        if self.vel_body is None:
            self.cmd_log.append("vel_body_down" if vb.down_m_s > 0 else "vel_body")
        self.vel_body = (vb.forward_m_s, vb.right_m_s, vb.down_m_s, vb.yawspeed_deg_s)
        self._step()

    def _step(self):
        if self.on_step is not None:
            self.on_step(self)
        if self.landing:
            self.d += (0.0 - self.d) * 0.4
            if abs(self.d) < 0.03:
                self.in_air_flag = False
            return
        if self.vel_body is not None and self.armed and not self.stuck:
            tn = time.monotonic()
            dt = 0.05 if self._vb_t is None else min(0.2, tn - self._vb_t)
            self._vb_t = tn
            fwd, right, down, _yr = self.vel_body
            dn, de = util.body_delta_to_ned(fwd * dt, right * dt, self.yaw)
            self.n += dn
            self.e += de
            self.d += down * dt
            if self.d >= -0.02:
                self.d = 0.0
                self.in_air_flag = False
            return
        if self.target is None or self.stuck or not self.armed:   # arm 전에는 setpoint 를 따르지 않는다 (실기와 같게; v0.2.2 게이트 헤딩 검사)
            return
        g = self.gain
        self.n += (self.target.north_m - self.n) * g
        self.e += (self.target.east_m - self.e) * g
        self.d += (self.target.down_m - self.d) * g
        self.yaw = util.yaw_to_360(self.yaw + util.angle_diff_deg(self.target.yaw_deg, self.yaw) * g)
        if self.d < -0.05:
            self.in_air_flag = True

    def push_pose(self):
        if self.vehicle is None:
            return
        self.ts_ms += 50.0
        ox, oy, oz = self.POSE_OFFSET
        ex, ey, ez = self.extra_pipe_offset
        self._pose_n += 1
        if self.vio_feed and self._pose_n % self.vio_every_n == 0:
            self.vehicle.update_vio({"quality": self.vio_quality, "features": self.vio_features, "state": self.vio_state,
                                     "error_codes": "", "pos": None, "rpy": (0.0, 0.0, util.normalize_angle_deg(self.yaw)), "raw": "fake"})
        self.vehicle.update_pose("px4_vehicle_local_position", {
            "voxl_ts_ms": self.ts_ms, "pos_x": self.n + ox + ex, "pos_y": self.e + oy + ey, "pos_z": self.d + oz + ez,
            "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": util.yaw_to_360(self.yaw),
            "vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0, "raw": "fake",
        })

    async def _health(self):
        while True:
            yield _Obj(is_local_position_ok=True)
            await asyncio.sleep(0.02)

    async def _status_text(self):
        sent = 0
        while True:
            while sent < len(self.status_texts):
                typ, text = self.status_texts[sent]
                sent += 1
                yield _Obj(type=typ, text=text)
            await asyncio.sleep(0.05)

    async def _armed_stream(self):
        while True:
            yield self.armed
            await asyncio.sleep(0.05)

    async def _odom(self):
        while True:
            yield _Obj(time_usec=int((time.monotonic() - self.odom_lag_s) * 1e6),
                       position_body=_Obj(x_m=self.n, y_m=self.e, z_m=self.d))
            await asyncio.sleep(0.05)

    async def _pos(self):
        while True:
            self.push_pose()
            j = (random.random() - 0.5) * 0.002          # 1 mm 지터: 실제 EKF 처럼 완전히 같은 값이 반복되지 않게
            yield _Obj(position=_Obj(north_m=self.n + j, east_m=self.e - j, down_m=self.d + j),
                       velocity=_Obj(north_m_s=0.0, east_m_s=0.0, down_m_s=0.0))
            await asyncio.sleep(self.pos_period)

    async def _in_air(self):
        while True:
            self._step()
            yield self.in_air_flag
            await asyncio.sleep(0.05)

    async def _att(self):
        while True:
            if self.ekf_yaw_drift_dps:
                tn = time.monotonic()
                if self._att_t is not None:
                    self.ekf_yaw_offset += self.ekf_yaw_drift_dps * (tn - self._att_t)
                self._att_t = tn
            else:
                self._att_t = None
            yield _Obj(roll_deg=0.0, pitch_deg=0.0, yaw_deg=util.normalize_angle_deg(self.yaw + self.ekf_yaw_offset))
            await asyncio.sleep(0.05)


def _rot_y_cols(psi_deg):
    c, s = math.cos(math.radians(psi_deg)), math.sin(math.radians(psi_deg))
    # 열 = 태그 축의 카메라 프레임 표현. 법선(3열) = (-sinψ, 0, cosψ)
    return ((c, 0.0, -s), (0.0, 1.0, 0.0), (s, 0.0, c))


class FakeWallCamera(tags.TagFeed):
    """벽(태그 열)을 보는 가짜 카메라. 기체 yaw 가 벽 정면 yaw 에서 ψ 만큼 틀어지면 태그 좌표가 그만큼 회전한다."""

    def __init__(self, drone, wall_facing_yaw_deg, standoff=0.7, tag_xs=((0, -0.25), (1, 0.25)),
                 fps=20.0, size_m=0.20, visible_half_angle=60.0):
        self.drone = drone
        self.wall_yaw = wall_facing_yaw_deg
        self.standoff = standoff
        self.tag_xs = tag_xs
        self.period = 1.0 / fps
        self.size_m = size_m
        self.visible = visible_half_angle
        self.last_t = -1e9
        self.frames_made = 0
        self.ts = 0

    def poll(self, now):
        if now - self.last_t < self.period:
            return []
        self.last_t = now
        psi = util.angle_diff_deg(self.drone.yaw, self.wall_yaw)      # >0 : 카메라가 오른쪽으로 돌아감
        if abs(psi) > self.visible:
            return []
        c, s = math.cos(math.radians(psi)), math.sin(math.radians(psi))
        dets = []
        self.ts += int(self.period * 1e9)
        for tag_id, x in self.tag_xs:
            xc = x * c - self.standoff * s
            zc = x * s + self.standoff * c
            dets.append(tags.TagDetection(tag_id, self.size_m, self.ts, "", 0, (xc, 0.0, zc),
                                          _rot_y_cols(psi), (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
                                          "tracking_front", recv_time=now))
        self.frames_made += 1
        return [tags.TagFrame(self.ts, dets, now)]

    def describe(self):
        return "FakeWallCamera(wall_yaw={:.1f})".format(self.wall_yaw)


class FakeBannerCamera(tags.TagFeed):
    """현수막(태그 start..end, spacing) 을 보는 가짜 카메라. 기체 **실제** 위치에서 태그 좌표를 계산한다.

    벽 프레임은 wall_facing_yaw 로 정한다: 벽 법선 = 그 yaw 의 전방, along 축 = 그 yaw 의 body 우측 (실제 현수막처럼 수직).
    실제 위치 = FakeDrone 추정 위치에 드리프트를 입힌 것: along 축 성분에 (1 - drift_scale) 스케일 오차.
    standoff = 0.7 - (벽 법선 방향 변위). 시야 ±half_fov, 거리 max_range 안의 태그만 보인다.
    """

    def __init__(self, drone, wall_facing_yaw_deg=270.0, standoff=0.7, spacing=0.1, n_tags=11, drift_scale=0.0,
                 fps=20.0, size_m=0.20, half_fov=60.0, max_range=3.0, hide_ids=(), start_id=0):
        self.drone = drone
        self.wall_yaw = wall_facing_yaw_deg
        self.standoff0 = standoff
        self.spacing = spacing
        self.n_tags = n_tags
        self.k = drift_scale
        self.period = 1.0 / fps
        self.size_m = size_m
        self.half_fov = half_fov
        self.max_range = max_range
        self.hide = set(hide_ids)
        self.start_id = start_id
        self.last_t = -1e9
        self.frames_made = 0
        self.ts = 0

    def truth(self):
        """(실제 along, 실제 standoff). 추정 변위를 벽 프레임에 사영하고 along 에 스케일 오차를 입힌다."""
        fn, fe = util.body_delta_to_ned(1.0, 0.0, self.wall_yaw)
        rn, re_ = util.body_delta_to_ned(0.0, 1.0, self.wall_yaw)
        along_est = self.drone.n * rn + self.drone.e * re_
        fwd_est = self.drone.n * fn + self.drone.e * fe
        return along_est * (1.0 - self.k), self.standoff0 - fwd_est

    def poll(self, now):
        if now - self.last_t < self.period:
            return []
        self.last_t = now
        psi = util.angle_diff_deg(self.drone.yaw, self.wall_yaw)
        if abs(psi) > 80.0:
            return []
        tn, standoff = self.truth()
        c, sn = math.cos(math.radians(psi)), math.sin(math.radians(psi))
        dets = []
        self.ts += int(self.period * 1e9)
        for i in range(self.n_tags):
            tid = self.start_id + i
            if tid in self.hide:
                continue
            x = self.spacing * i - tn
            xc = x * c - standoff * sn
            zc = x * sn + standoff * c
            if zc <= 0.05 or math.hypot(xc, zc) > self.max_range or abs(math.degrees(math.atan2(xc, zc))) > self.half_fov:
                continue
            dets.append(tags.TagDetection(tid, self.size_m, self.ts, "", 0, (xc, 0.0, zc), _rot_y_cols(psi),
                                          (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "tracking_front", recv_time=now))
        if not dets:
            return []
        self.frames_made += 1
        return [tags.TagFrame(self.ts, dets, now)]

    def describe(self):
        return "FakeBannerCamera(k={:.2f})".format(self.k)


def fast_cfg(**kw):
    base = dict(h_tol=0.03, v_tol=0.06, stable_time=0.2, late_observe=0.5, move_timeout=6.0,
                yaw_tol=3.0, yaw_stable_time=0.2, takeoff_speed=2.0, takeoff_hover_time=0.2,
                preland_alt=0.2, preland_speed=2.0, preland_hover_time=0.2, pos_ok_hold_sec=0.1,
                reach_pipe_timeout=3.0, csv_sample_sec=0.2, geofence_violation_sec=0.3,
                vio_gate_window_s=0.4, vio_quiet_s=0.0, vio_bad_abort_s=0.3, est_disagree_s=0.3, mav_lag_calib_s=0.15, yaw_disagree_s=0.2, min_uptime_s=0.0, vio_min_rate_hz=0.0, land_timeout_s=3.0, land_still_s=0.3)
    base.update(kw)
    return FlightConfig(**base)


def fast_params(**kw):
    align = AlignParams(tol_deg=1.0, hold_s=0.3, timeout_s=6.0, settle_s=0.1, yaw_speed_deg_s=90.0, period_s=0.02)
    base = dict(altitude=0.5, distance=1.0, speed=1.0, turn_deg=-90.0, turn_speed=180.0, post_turn_hover=0.2,
                hover_ref=0.3, hover_far=0.3, hover_return=0.3, do_align=True, align=align, condition="T", run=1,
                stop_mode="tag", start_tag=0, end_tag=10, tag_spacing=0.1, center=True, center_tol=0.03, max_extra=0.4)
    base.update(kw)
    return mdf.DriftParams(**base)


def landed_cmd(d):
    """접지 명령이 나갔는가: offboard body-velocity 하강(v0.2.5 기본) 또는 action.land()."""
    return "vel_body_down" in d.cmd_log or "land" in d.cmd_log


async def run_flight(drone, cfg, params, feed, out_dir):
    rec = FlightRecorder(cfg, out_dir=out_dir, prefix="t")
    v = Vehicle(cfg, recorder=rec, drone=drone, pose_readers=False, initial_yaw=drone.yaw)
    drone.vehicle = v
    await v.connect()
    await v.wait_position_stable(timeout=5.0)
    await v.check_vio_quality_preflight(samples=2)
    await v.capture_initial_mavsdk_position(samples=3, interval=0.05)
    await v.start_monitors_and_offboard(prime_sec=0.3)
    seq = await mdf.run_sequence(v, params, feed=feed, recorder=rec)
    rec.summary["sequence"] = seq
    rec.summary["vehicle"] = v.summary_dict()
    written = rec.write_all()
    return v, rec, seq, written


# ---------------------------------------------------------------------------
# 1. util / config
# ---------------------------------------------------------------------------

print("\n[util / config]")
ok("angle_diff wraps", near(util.angle_diff_deg(10, 350), 20, 1e-9) and near(util.angle_diff_deg(350, 10), -20, 1e-9))
ok("yaw_to_360", near(util.yaw_to_360(-90), 270, 1e-9))
dn, de = util.body_delta_to_ned(0.0, 1.0, 90.0)
ok("body right at yaw 90 = south(N-)", near(dn, -1.0, 1e-9) and near(de, 0.0, 1e-9))
dn, de = util.body_delta_to_ned(0.0, 1.0, 270.0)
ok("body right at yaw 270 = north(N+)", near(dn, 1.0, 1e-9) and near(de, 0.0, 1e-9))
st, ratio, cross = util.evaluate_horizontal_tracking(1.0, 0.0, 0.95, 0.05)
ok("tracking OK", st == "OK" and near(ratio, 0.95, 1e-9))
ok("tracking wrong dir", util.evaluate_horizontal_tracking(1.0, 0.0, -0.5, 0.0)[0] == "WRONG_DIRECTION")
ok("mono_time increases", util.mono_time() <= util.mono_time())

cfg = FlightConfig(h_tol=0.05)
ok("config override", cfg.h_tol == 0.05 and cfg.v_tol == FlightConfig.v_tol)
ok("config reach pipe auto-added", cfg.voxl_pipes[0] == cfg.reach_pipe)
try:
    FlightConfig(bogus=1)
    ok("config rejects unknown", False)
except TypeError:
    ok("config rejects unknown", True)
import argparse                                           # noqa: E402
_p = argparse.ArgumentParser()
FlightConfig.add_arguments(_p)
_a = _p.parse_args(["--h-tol", "0.04", "--no-geofence", "--max-home-dist", "3"])
_c = FlightConfig.from_args(_a)
ok("config from_args", _c.h_tol == 0.04 and _c.no_geofence and _c.max_home_dist == 3.0)

# ---------------------------------------------------------------------------
# 2. pose parser
# ---------------------------------------------------------------------------

print("\n[pose parser]")
line = "  123456| 0.10 -0.20 -0.60 | 1.0 2.0 270.5 | 0.01 0.02 0.00 | 0 0 0"
pp = parse_voxl_inspect_pose_line(line)
ok("parse pose line", pp and near(pp["pos_x"], 0.1, 1e-9) and near(pp["yaw_deg"], 270.5, 1e-9) and pp["voxl_ts_ms"] == 123456)
ok("header skipped", parse_voxl_inspect_pose_line("timestamp(ms) | position | ...") is None)
ok("garbage skipped", parse_voxl_inspect_pose_line("hello") is None)

# ---------------------------------------------------------------------------
# 3. geofence
# ---------------------------------------------------------------------------

print("\n[geofence]")
gcfg = FlightConfig(max_horizontal_dev=0.5, max_alt_deviation=0.3, geofence_violation_sec=1.0,
                    geofence_jump_thresh=0.5, geofence_jump_timeout=5.0)
gf = Geofence(gcfg)
home = (0.0, 0.0, 0.0)
ok("disarmed -> None", gf.check(0.0, (0, 0, -1), (5, 5, -1), home) is None)
gf.arm((0.0, 0.0, -1.0))
ok("inside -> None", gf.check(0.0, (0, 0, -1), (0.1, 0.1, -1.0), home) is None)
ok("first violation -> warn only", gf.check(1.0, (0, 0, -1), (1.0, 0.0, -1.0), home) is None)
ok("violation before persist -> None", gf.check(1.5, (0, 0, -1), (1.0, 0.0, -1.0), home) is None)
r = gf.check(2.1, (0, 0, -1), (1.0, 0.0, -1.0), home)
ok("sustained violation -> trigger", r is not None and "horizontal" in r)
gf.arm((0.0, 0.0, -1.0))
ok("recover clears timer", gf.check(0.0, (0, 0, -1), (1.0, 0, -1), home) is None
   and gf.check(0.5, (0, 0, -1), (0.0, 0, -1), home) is None
   and gf.check(1.6, (0, 0, -1), (1.0, 0, -1), home) is None)
gf.arm((0.0, 0.0, -1.0))
ok("setpoint jump grace", gf.check(0.0, (2.0, 0, -1), (0.0, 0, -1), home) is None
   and gf.check(1.5, (2.0, 0, -1), (0.0, 0, -1), home) is None)
ok("jump timeout triggers", gf.check(5.2, (2.0, 0, -1), (0.0, 0, -1), home) is not None)
gf2 = Geofence(FlightConfig(max_home_dist=1.0, geofence_violation_sec=0.5, max_horizontal_dev=10))
gf2.arm((0.0, 0.0, -1.0))
ok("home fence warn", gf2.check(0.0, (2, 0, -1), (2.0, 0, -1), home) is None)
r = gf2.check(0.6, (2, 0, -1), (2.0, 0, -1), home)
ok("home fence trigger", r is not None and "home distance" in r)
gf3 = Geofence(FlightConfig(no_geofence=True))
gf3.arm((0, 0, 0))
ok("no_geofence never arms", not gf3.armed and gf3.check(0, (0, 0, 0), (9, 9, 9), home) is None)

# ---------------------------------------------------------------------------
# 4. tags
# ---------------------------------------------------------------------------

print("\n[tags]")
det = tags.TagDetection(3, 0.07, 123456789, "", 0, (0.1, -0.02, 0.5), ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
                        (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "tracking_front")
raw = tags.pack_record(det)
ok("record 252 B", len(raw) == 252)
stream = tags.TagDetectionStream()
out = stream.feed(b"\x00\x11\x22" + raw + raw[:100])
ok("resync + parse", len(out) == 1 and out[0].id == 3 and near(out[0].t_cam[2], 0.5, 1e-6) and stream.stats["resyncs"] >= 1)
out2 = stream.feed(raw[100:])
ok("partial record completes", len(out2) == 1 and out2[0].cam == "tracking_front")
asm = tags.FrameAssembler()
d1 = tags.TagDetection(0, 0.07, 1000, "", 0, (0, 0, 0.5), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "c", recv_time=1.0)
d2 = tags.TagDetection(1, 0.07, 1000, "", 0, (0.5, 0, 0.5), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "c", recv_time=1.0)
d3 = tags.TagDetection(0, 0.07, 2000, "", 0, (0, 0, 0.5), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "c", recv_time=1.05)
fr = asm.add_many([d1, d2, d3])
ok("assembler closes frame on new ts", len(fr) == 1 and fr[0].ids == [0, 1])
ok("assembler flush_stale", asm.flush_stale(1.2, 0.08) is not None and asm.pending == 0)
bad = tags.TagDetection(3, 9.0, 1, "", 0, (0, 0, 0.5), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "c")
ok("insane size rejected", not bad.is_sane())

# ---------------------------------------------------------------------------
# 5. align
# ---------------------------------------------------------------------------

print("\n[align]")
cam = FakeWallCamera(_Obj(yaw=10.0), wall_facing_yaw_deg=0.0)
f = cam.poll(100.0)[0]
m = al.measure_alignment(f.tags)
ok("measure sign: camera right 10deg -> err +10", m.valid and near(m.yaw_error_deg, 10.0, 0.05), repr(m))
ok("measure correction = -err", near(m.yaw_correction_deg, -10.0, 0.05))
ok("measure normal yaw agrees", near(m.normal_yaw_deg, 10.0, 0.05))
m1 = al.measure_alignment(f.tags[:1])
ok("single tag invalid", not m1.valid and m1.reason == "single_tag")
m2 = al.measure_alignment(f.tags, max_range_m=0.3)
ok("range filter", not m2.valid and m2.reason == "no_tags")
m3 = al.measure_alignment(f.tags, allowed_ids={20, 21})
ok("allowed_ids filter", not m3.valid)
# 2026-11-10: 상단열도 ID 0~10 → 다른 높이(|y| 1.4 m)의 같은 ID 는 버린다
_other_row = [tags.TagDetection(t.id, t.size_m, t.timestamp_ns, t.name, t.loc_type, (t.t_cam[0], 1.4, t.t_cam[2]), t.r_cam,
                                t.t_fixed, t.r_fixed, t.cam) for t in f.tags]
m4 = al.measure_alignment(_other_row)
ok("vertical filter: other row (|y| 1.4) alone -> no_tags", not m4.valid and m4.reason == "no_tags", repr(m4))
m5 = al.measure_alignment(list(f.tags) + _other_row)
ok("vertical filter: mixed rows -> only own row counted", m5.valid and m5.n_tags == len(f.tags) and near(m5.yaw_error_deg, 10.0, 0.05), repr(m5))
m6 = al.measure_alignment(_other_row, max_vertical_m=None)
ok("vertical filter off (None) keeps other row", m6.valid, repr(m6))

ctl = al.AlignController(tol_deg=1.0, hold_s=0.3, timeout_s=5.0, settle_s=0.0, min_hold_frames=2)
ctl.start(0.0)
cmd = ctl.update(m, 0.1)
ok("controller: first valid frame -> correction", cmd.state == al.CORRECTING and cmd.yaw_delta_deg is not None and cmd.yaw_delta_deg < 0)
cmd2 = ctl.update(m, 0.2)
ok("controller: awaiting ack ignores", cmd2.state == al.CORRECTING and cmd2.yaw_delta_deg is None)
ctl.ack_correction(0.3)
good = al.measure_alignment(FakeWallCamera(_Obj(yaw=0.2), 0.0).poll(200.0)[0].tags)
ctl.update(good, 0.4)
ctl.update(good, 0.5)
cmd3 = ctl.update(good, 0.8)
ok("controller: holds then ALIGNED", cmd3.state == al.ALIGNED, repr(cmd3))
ctl2 = al.AlignController(timeout_s=1.0)
ctl2.start(0.0)
ok("controller: timeout -> FAILED", ctl2.update(None, 1.1).state == al.FAILED)

# ---------------------------------------------------------------------------
# 6. voxl_logger command
# ---------------------------------------------------------------------------

print("\n[voxl_logger]")
lg = VoxlLogger("x", cameras=["hires_large_color"], max_seconds=120)
cmd = lg.command()
ok("logger cmd", cmd[:3] == ["voxl-logger", "-c", "hires_large_color"] and "-t" in cmd and cmd[cmd.index("-t") + 1] == "120"
   and cmd[-1] == "x")

# ---------------------------------------------------------------------------
# 7. 전체 시퀀스 (가짜 기체)
# ---------------------------------------------------------------------------

print("\n[full sequence — nominal (tag-stop mode, banner camera, wall 8 deg off, no drift)]")
tmp = tempfile.mkdtemp(prefix="s2flight_test_")
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
try:
    drone = FakeDrone(yaw0=0.0)
    WALL_YAW = 278.0                       # 벽 정면 yaw: 좌회전 90° 뒤(270°) 8° 더 우회전해야 정면
    cam = FakeBannerCamera(drone, WALL_YAW)
    cfgN = fast_cfg()
    prmN = fast_params()
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, cfgN, prmN, cam, tmp))
    ok("tag mode: out leg stopped by tag 10", seq["legs"] and seq["legs"][0]["status"] == "STOPPED" and "tag 10" in seq["legs"][0]["reason"],
       str(seq["legs"][:1]))
    ok("tag mode: back leg stopped by tag 0", len(seq["legs"]) == 2 and seq["legs"][1]["status"] == "STOPPED" and "tag 0" in seq["legs"][1]["reason"])
    ok("tag mode: no drift -> settled drift ~0 both legs", len(seq["legs_settled"]) == 2 and all(abs(l["drift_along"]) < 0.03 for l in seq["legs_settled"]),
       str([round(l["drift_along"], 3) for l in seq["legs_settled"]]))
    ok("tag mode: settled standoff ≈ 0.70 (truth) and own agrees", all(abs(l["truth_standoff"] - 0.70) < 0.03 and abs(l["drift_standoff"]) < 0.03 for l in seq["legs_settled"]),
       str([(round(l["truth_standoff"], 3), round(l["drift_standoff"], 3)) for l in seq["legs_settled"]]))
    ok("tag mode: physically at tag 10 at the end of out leg", abs(seq["legs"][0]["truth_along"] - 1.0) < 0.06, str(seq["legs"][0]["truth_along"]))
    ok("tag mode: centering ran and ok", seq["center"] and seq["center"]["ok"], str(seq["center"]))
    ok("drift_log written with rows", os.path.exists(os.path.join(tmp, "t_drift_log.csv")) and rec.extra_tables.get("drift_log") and len(rec.extra_tables["drift_log"]) > 10)
    rows_out = [r for r in rec.extra_tables["drift_log"] if r["leg"] == "out" and r["err_along"] != ""]
    rows_hf = [r for r in rec.extra_tables["drift_log"] if r["leg"] == "hover_far" and r["err_along"] != ""]
    # 이동 중 err 는 가짜 포즈 파이프 지연(50 ms, 1 m/s → ~5-10 cm)이 섞인다; 호버 행은 지연 없이 0 근처여야 한다
    ok("drift_log rows carry own vs truth during the out leg", len(rows_out) > 3 and all(abs(r["err_along"]) < 0.2 for r in rows_out)
       and len(rows_hf) > 4 and all(abs(r["err_along"]) < 0.04 for r in rows_hf[len(rows_hf) // 2:]),
       str([round(r["err_along"], 3) for r in rows_out[:8]]) + " hover_far " + str([round(r["err_along"], 3) for r in rows_hf[:5]]))
    ok("leg_end events (stop + settled x2)", sum(1 for e in rec.events if e["event"] == "leg_end") == 4 and any(e["event"] == "drift_reference" for e in rec.events))

    ok("sequence completed", seq["completed"] and not seq["aborted"])
    ok("no exception, landed+disarmed", landed_cmd(drone) and "disarm" in drone.cmd_log and not v.aborted)
    names = [r["name"] for r in v.results]
    ok("stage order (explicit --turn-deg -90)", names[:2] == ["takeoff", "aim_turn"] and "move_out" in names and "move_back" in names
       and names.index("move_out") < names.index("move_back") and names[-1] == "preland_descend", str(names))
    ok("all stages OK/STOPPED", all(r["status"] in ("OK", "STOPPED") for r in v.results), str([(r["name"], r["status"]) for r in v.results]))
    corr = [n for n in names if n.startswith("align_corr")]
    ok("alignment made >=1 correction", len(corr) >= 1, str(names))
    a = seq["align"]
    ok("aligned", a and a["state"] == al.ALIGNED and a["t0"] is not None, str(a and a.get("reason")))
    ok("final heading faces wall (278 +-1.5)", abs(util.angle_diff_deg(WALL_YAW, v.plan_yaw)) <= 1.5,
       "plan_yaw={:.2f}".format(v.plan_yaw))
    ok("final yaw error small", a and abs(a["final_yaw_error_deg"]) <= 1.0)
    mo = [r for r in v.results if r["name"] == "move_out"][0]
    exp_dn, exp_de = util.body_delta_to_ned(0.0, 1.0, v.plan_yaw)
    mo_len = math.hypot(mo["fb_delta"][0], mo["fb_delta"][1])
    mo_ang = math.degrees(math.atan2(mo["fb_delta"][1], mo["fb_delta"][0])) - math.degrees(math.atan2(exp_de, exp_dn))
    ok("move_out goes body-right along wall (direction within 2 deg, ~banner length)",
       abs(util.angle_diff_deg(mo_ang, 0.0)) < 2.0 and 0.8 < mo_len < 1.25,     # 정지 순간 피드백 지연(가짜 50 ms) 만큼 짧게 보일 수 있다
       "fb_delta={} exp_dir=({:.2f},{:.2f}) len={:.2f}".format(mo["fb_delta"], exp_dn, exp_de, mo_len))
    mb = [r for r in v.results if r["name"] == "move_back"][0]
    ok("move_back returns near origin (tag 0 centre ± tol + braking lag)", near(mb["fb_end_rel"][0], 0.0, 0.15) and near(mb["fb_end_rel"][1], 0.0, 0.15),
       str(mb["fb_end_rel"]))
    ok("reach offset applied (pose pipe offset absorbed)", near(v.reach_offset_n, -FakeDrone.POSE_OFFSET[0], 0.005))
    ev = [e["event"] for e in rec.events]
    for want in ("armed", "offboard_started", "sequence_start", "align_start", "aligned", "hover_start", "stage_start",
                 "stage_end", "land_cmd", "landed", "disarmed"):
        ok("event recorded: " + want, want in ev)
    t0_ev = [e for e in rec.events if e["event"] == "aligned"][0]
    ok("aligned event has t0_mono & frame ns", t0_ev.get("t0_mono") and t0_ev.get("t0_frame_ns"))
    hover_ref = [e for e in rec.events if e["event"] == "hover_start" and e["detail"] == "hover_ref"]
    ok("hover_ref after aligned", hover_ref and hover_ref[0]["mono_time"] >= t0_ev["mono_time"])
    files = sorted(os.path.basename(w) for w in written)
    ok("files written: sync/events/stages/align_log/summary", all(any(f.endswith(s) for f in files)
       for s in ("_sync.csv", "_events.csv", "_stages.csv", "_align_log.csv", "_summary.json")), str(files))
    with open(os.path.join(tmp, "t_sync.csv")) as fh:
        rows = list(csv.DictReader(fh))
    ok("sync csv has mono_ns + align_state", rows and "mono_ns" in rows[0] and "align_state" in rows[0]
       and any(r["align_state"] == al.ALIGNED for r in rows))
    with open(os.path.join(tmp, "t_align_log.csv")) as fh:
        arows = list(csv.DictReader(fh))
    ok("align_log rows with yaw error", arows and any(r["valid"] == "1" for r in arows))
    with open(os.path.join(tmp, "t_summary.json")) as fh:
        sj = json.load(fh)
    ok("summary json round-trips", sj["sequence"]["completed"] is True and sj["vehicle"]["aborted"] is False)
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# --- 정렬 실패 → 착륙 ----------------------------------------------------------

print("\n[full sequence — align failure lands]")
tmp = tempfile.mkdtemp(prefix="s2flight_test_")
try:
    drone = FakeDrone(yaw0=0.0)
    blind = tags.TagFeed()                      # 프레임이 전혀 없는 피드
    prmF = fast_params()
    prmF.align.timeout_s = 0.6
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), prmF, blind, tmp))
    names = [r["name"] for r in v.results]
    ok("align FAILED", seq["align"]["state"] == al.FAILED and seq["aborted"])
    ok("no lateral move after failure", "move_out" not in names, str(names))
    ok("landed after failure", landed_cmd(drone) and "landed" in [e["event"] for e in rec.events])
    ok("align_failed event", "align_failed" in [e["event"] for e in rec.events])
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# --- --no-align 경로 -------------------------------------------------------

print("\n[full sequence — no-align]")
tmp = tempfile.mkdtemp(prefix="s2flight_test_")
try:
    drone = FakeDrone(yaw0=45.0)
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("no-align completes", seq["completed"] and seq["align"]["state"] == "SKIPPED")
    ok("turn applied from yaw 45 -> 315", near(util.angle_diff_deg(v.plan_yaw, 315.0), 0.0, 1e-6), str(v.plan_yaw))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# --- 지오펜스 비상착륙 -------------------------------------------------------

print("\n[full sequence — geofence emergency]")
tmp = tempfile.mkdtemp(prefix="s2flight_test_")
try:
    drone = FakeDrone(yaw0=0.0)
    # 속도는 실제 실험값에 가깝게 둔다. 4 m/s 처럼 빠르면 setpoint 가 모니터 1샘플에 0.5 m 넘게 움직여
    # "점프" 로 간주되고 추종 펜스가 유예된다(v14 설계). jump_thresh 도 키워 스케줄링 지터에 흔들리지 않게 한다.
    cfgG = fast_cfg(max_horizontal_dev=0.3, geofence_violation_sec=0.2, geofence_jump_thresh=5.0,
                    move_timeout=3.0, late_observe=0.0)
    prmG = fast_params(do_align=False, distance=1.0, speed=1.0)

    async def stuck_flight():
        rec = FlightRecorder(cfgG, out_dir=tmp, prefix="g")
        v = Vehicle(cfgG, recorder=rec, drone=drone, pose_readers=False)
        drone.vehicle = v
        await v.connect()
        await v.wait_position_stable(timeout=5.0)
        await v.check_vio_quality_preflight(samples=2)
        await v.capture_initial_mavsdk_position(samples=3, interval=0.05)
        await v.start_monitors_and_offboard(prime_sec=0.3)
        await v.takeoff(0.5)
        drone.stuck = True                       # 이륙 후 기체가 setpoint 를 못 따라간다
        seq = await _turn_and_move(v, prmG)
        return v, rec, seq

    async def _turn_and_move(v, p):
        await v.move_body(0.0, p.distance, "move_out", "RIGHT", speed_mps=p.speed)
        if v.aborted:
            await v.land_and_disarm()
        return {"aborted": v.aborted}

    v, rec, seq = loop.run_until_complete(stuck_flight())
    ok("geofence aborted the flight", v.aborted and v.abort_reason and "GEOFENCE" in v.abort_reason, str(v.abort_reason))
    ok("emergency land commanded", landed_cmd(drone) and "emergency_land" in [e["event"] for e in rec.events])
    ok("move_out not OK", any(r["name"] == "move_out" and r["status"] != "OK" for r in v.results))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# ---------------------------------------------------------------------------
# 7b. 정면 기준 (initial_yaw): current = arm 직전 방향, zero = 전원 켠 방향, 숫자 = 고정
# ---------------------------------------------------------------------------

print("\n[initial yaw reference]")
ok("config initial_yaw default current", FlightConfig().initial_yaw_value() == ("current", None))
ok("config initial_yaw zero/fixed parse", FlightConfig(initial_yaw="zero").initial_yaw_value() == ("zero", 0.0)
   and FlightConfig(initial_yaw="45").initial_yaw_value() == ("fixed", 45.0) and FlightConfig(initial_yaw=-90).initial_yaw_value() == ("fixed", -90.0))
try:
    FlightConfig(initial_yaw="north").validate()
    ok("config initial_yaw rejects garbage", False)
except ValueError:
    ok("config initial_yaw rejects garbage", True)

tmp = tempfile.mkdtemp(prefix="s2flight_yaw_")
try:
    async def yaw_ref_flight(yaw0, cfg):
        drone = FakeDrone(yaw0=yaw0)
        rec = FlightRecorder(cfg, out_dir=tmp, prefix="y")
        v = Vehicle(cfg, recorder=rec, drone=drone, pose_readers=False)        # initial_yaw 미지정 → cfg 규칙
        drone.vehicle = v
        await v.connect()
        await v.wait_position_stable(timeout=5.0)
        await v.capture_initial_mavsdk_position(samples=3, interval=0.05)
        await v.start_monitors_and_offboard(prime_sec=0.3)
        yaw_before = drone.yaw
        await v.takeoff(0.5)
        yaw_after = drone.yaw
        await v.cancel_background_tasks()
        return v, drone, yaw_before, yaw_after, rec

    v, drone, yb, ya, rec = loop.run_until_complete(yaw_ref_flight(120.0, fast_cfg()))
    ok("current: forward = heading at arm (120)", near(util.angle_diff_deg(v.initial_yaw_deg, 120.0), 0.0, 0.5), str(v.initial_yaw_deg))
    ok("current: no turn during takeoff", near(util.angle_diff_deg(ya, 120.0), 0.0, 0.5), "yaw after takeoff {:.1f}".format(ya))
    ok("current: plan_yaw/target follow it", near(util.angle_diff_deg(v.plan_yaw, 120.0), 0.0, 0.5) and near(util.angle_diff_deg(v.target.yaw_deg, 120.0), 0.0, 0.5))
    ok("initial_yaw event recorded", any(e["event"] == "initial_yaw" for e in rec.events) and v.summary_dict()["initial_yaw_source"].startswith("current"))

    v, drone, yb, ya, rec = loop.run_until_complete(yaw_ref_flight(120.0, fast_cfg(initial_yaw="zero")))
    ok("zero: forward = power-on heading (0) → vehicle turns during takeoff", v.initial_yaw_deg == 0.0 and near(util.angle_diff_deg(ya, 0.0), 0.0, 2.0),
       "yaw after takeoff {:.1f}".format(ya))

    v, drone, yb, ya, rec = loop.run_until_complete(yaw_ref_flight(120.0, fast_cfg(initial_yaw="-45")))
    ok("fixed -45: forward = 315", near(util.angle_diff_deg(v.initial_yaw_deg, 315.0), 0.0, 1e-6) and near(util.angle_diff_deg(ya, 315.0), 0.0, 2.0))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# ---------------------------------------------------------------------------
# 7c. 태그 기반 드리프트 기록 / 조건 정지 (s2flight.drift)
# ---------------------------------------------------------------------------

print("\n[drift: banner coordinates]")
lay = s2drift.BannerLayout(0, 10, 0.5)
ok("layout along", lay.along(0) == 0.0 and lay.along(10) == 5.0 and lay.along(11) is None and lay.length_m == 5.0)
lay_u = s2drift.BannerLayout(20, 30, 0.5)
ok("layout upper row", lay_u.along(20) == 0.0 and lay_u.along(25) == 2.5 and lay_u.along(3) is None)
# FakeWallCamera 정변환: x=+0.30 (태그가 기체 오른쪽 0.3), standoff 0.7, psi=10deg → 카메라 좌표 → 역변환
psi = 10.0
c, sn = math.cos(math.radians(psi)), math.sin(math.radians(psi))
xc, zc = 0.30 * c - 0.7 * sn, 0.30 * sn + 0.7 * c
dx, sd = s2drift.banner_coords(xc, zc, psi)
ok("banner_coords inverts the camera transform", near(dx, 0.30, 1e-9) and near(sd, 0.70, 1e-9), "{} {}".format(dx, sd))
fr = tags.TagFrame(1, [tags.TagDetection(3, 0.2, 1, "", 0, (xc, 0.0, zc), _rot_y_cols(psi), (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "x"),
                       tags.TagDetection(4, 0.2, 1, "", 0, ((0.80) * c - 0.7 * sn, 0.0, 0.80 * sn + 0.7 * c), _rot_y_cols(psi), (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "x")], 1.0)
est = s2drift.estimate_frame(fr, lay)
ok("estimate_frame: two tags -> line psi, truth along = along(3) - 0.30 = 1.20, standoff 0.70",
   est.n_tags == 2 and est.psi_src == "line" and near(est.psi_deg, psi, 0.01) and near(est.truth_along, 1.20, 0.005) and near(est.truth_standoff, 0.70, 0.005),
   "{} {} {} {}".format(est.psi_src, est.psi_deg, est.truth_along, est.truth_standoff))
fr1 = tags.TagFrame(2, fr.tags[:1], 1.1)
est1 = s2drift.estimate_frame(fr1, lay)
ok("estimate_frame: single tag -> normal psi", est1.n_tags == 1 and est1.psi_src == "normal" and near(est1.truth_along, 1.20, 0.01))
_fr_up = tags.TagFrame(4, [tags.TagDetection(d.id, d.size_m, 4, "", 0, (d.t_cam[0], -1.4, d.t_cam[2]), d.r_cam, d.t_fixed, d.r_fixed, d.cam)
                           for d in fr.tags], 1.3)
ok("estimate_frame: other row (|y| 1.4, same ids) ignored", s2drift.estimate_frame(_fr_up, lay).n_tags == 0)
_fr_mix = tags.TagFrame(5, list(fr.tags) + list(_fr_up.tags), 1.4)
_em = s2drift.estimate_frame(_fr_mix, lay)
ok("estimate_frame: mixed rows -> own row only, same along", _em.n_tags == 2 and near(_em.truth_along, 1.20, 0.005), "{} {}".format(_em.n_tags, _em.truth_along))
ok("estimate_frame ignores ids outside layout", s2drift.estimate_frame(tags.TagFrame(3, [tags.TagDetection(25, 0.2, 3, "", 0, (0, 0, 0.7), _rot_y_cols(0), (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "x")], 1.2), lay).n_tags == 0)

print("\n[drift: tag-stop sequence with 10 % scale drift, start offset, missing end tag]")
tmp = tempfile.mkdtemp(prefix="s2flight_drift_")
try:
    # (a) 추정이 실제보다 10 % 더 간다 (truth = 0.9 * own). 1.0 m 현수막 → 태그 10 에서 own ≈ 1.11, drift ≈ +0.11
    drone = FakeDrone(yaw0=0.0)
    cam = FakeBannerCamera(drone, 270.0, drift_scale=0.10)
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(), cam, tmp))
    ok("drift 10 %: sequence completed", seq["completed"] and not seq["landed_in_place"], str(seq.get("legs")))
    out, back = seq["legs"]
    ok("drift 10 %: out leg stopped at tag 10 physically (truth ≈ 1.0)", out["status"] == "STOPPED" and abs(out["truth_along"] - 1.0) < 0.06, str(out))
    outs, backs = seq["legs_settled"]
    ok("drift 10 %: settled own estimate ≈ truth/0.9 -> drift ≈ +0.11 recorded, NOT applied", abs(outs["drift_along"] - outs["truth_along"] / 9.0) < 0.02 and outs["drift_along"] > 0.08,
       "own {:.3f} truth {:.3f} drift {:.3f}".format(outs["own_along_b"], outs["truth_along"], outs["drift_along"]))
    # 복귀 정지 뒤 실제 위치는 tol + 제동 지연(가짜 50 ms 포즈 + 1 m/s) 만큼 지나칠 수 있다 → 0.2 m 안이면 된다; 핵심은 드리프트 값
    ok("drift 10 %: back leg returns to tag 0 physically, settled drift ~0", back["status"] == "STOPPED" and abs(backs["truth_along"]) < 0.20 and abs(backs["drift_along"]) < 0.03,
       "truth {:.3f} drift {:.3f}".format(backs["truth_along"], backs["drift_along"]))
    mo = v.results[[r["name"] for r in v.results].index("move_out")]
    ok("drift 10 %: move_until result records cmd_dist / own_dist / stop_reason", mo["mode"] == "move_until" and mo["own_dist"] > 0.9
       and mo["cmd_dist"] >= mo["own_dist"] - 0.05 and "tag 10" in mo["stop_reason"], str({k: mo[k] for k in ("cmd_dist", "own_dist", "stop_reason")}))
    def med(rows):
        xs = sorted(r["err_along"] for r in rows)
        return xs[len(xs) // 2] if xs else float("nan")
    e_ref = med([r for r in rec.extra_tables["drift_log"] if r["leg"] == "hover_ref" and r["err_along"] != ""])
    e_far = med([r for r in rec.extra_tables["drift_log"] if r["leg"] == "hover_far" and r["err_along"] != ""])
    ok("drift 10 %: err_along in drift_log grows from hover_ref (~0) to hover_far (~+0.10)", abs(e_ref) < 0.03 and 0.07 < e_far < 0.14,
       "ref {:.3f} far {:.3f}".format(e_ref, e_far))

    # (b) 시작 위치가 태그 0 에서 0.2 m 오른쪽 → 센터링이 왼쪽으로 옮긴다
    drone = FakeDrone(yaw0=0.0, n0=0.2)
    cam = FakeBannerCamera(drone, 270.0)
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(), cam, tmp))
    ok("centering: moved to tag 0 (≥1 move, ok)", seq["center"]["ok"] and seq["center"]["iterations"] >= 1, str(seq["center"]))
    ok("centering: reference truth along ≈ 0", abs(seq["reference"]["truth_along"]) < 0.04, str(seq["reference"]["truth_along"]))
    ok("centering: sequence still completes", seq["completed"])

    # (c) 끝 태그가 없다 → 안전 상한에서 멈추고 그 자리 착륙, 복귀 없음
    drone = FakeDrone(yaw0=0.0)
    cam = FakeBannerCamera(drone, 270.0, hide_ids=(10,))
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(), cam, tmp))
    names = [r["name"] for r in v.results]
    ok("missing end tag: out leg MAX_DIST", seq["legs"] and seq["legs"][0]["status"] == "MAX_DIST", str(seq["legs"][:1]))
    ok("missing end tag: landed in place, no return leg", seq["landed_in_place"] and "move_back" not in names and landed_cmd(drone) and not seq["completed"])
    ok("missing end tag: stopped within banner length + max_extra", v.results[names.index("move_out")]["cmd_dist"] <= 1.4 + 1e-6)

    # (d) 옛 방식(distance) 도 그대로 동작
    drone = FakeDrone(yaw0=0.0)
    cam = FakeWallCamera(drone, 278.0)
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(stop_mode="distance", center=False), cam, tmp))
    ok("distance mode: completes with reach-judged legs", seq["completed"] and all(r["status"] == "OK" for r in v.results))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# ---------------------------------------------------------------------------
# 8. SITL 모드 (s2flight.sim): UDP 태그 피드, MAVSDK 텔레메트리 포즈, --sim 전체 시퀀스
# ---------------------------------------------------------------------------

print("\n[sim: UDP tag datagram parsing]")
good = s2sim.encode_tag_datagram(12.345, [tags.TagDetection(3, 0.2, 0, "", 2, (0.1, 0.0, 0.7), _rot_y_cols(0.0),
                                                            (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "x")])
fr = s2sim.parse_tag_datagram(good, 0.20, recv_time=1.0)
ok("sim datagram → frame", fr is not None and len(fr.tags) == 1 and fr.tags[0].id == 3 and fr.timestamp_ns == 12345000000)
ok("sim frame carries expected size_m", fr.tags[0].size_m == 0.20 and fr.tags[0].cam == "sim_front_camera")
ok("sim frame t_cam/r_cam", fr.tags[0].t_cam == (0.1, 0.0, 0.7) and fr.tags[0].r_cam[2][2] == 1.0)
ok("bad json rejected", s2sim.parse_tag_datagram(b"not json", 0.2) is None)
ok("missing dets rejected", s2sim.parse_tag_datagram(b'{"t": 1.0}', 0.2) is None)
ok("non-finite t rejected", s2sim.parse_tag_datagram(b'{"t": NaN, "dets": []}', 0.2) is None)
bad_det = b'{"t": 2.0, "dets": [{"id": 1, "t_cam": [0, 0, "x"], "R": [[1,0,0],[0,1,0],[0,0,1]]}, {"id": 2, "t_cam": [0.0, 0.0, 0.5], "R": [[1,0,0],[0,1,0],[0,0,1]]}]}'
fr2 = s2sim.parse_tag_datagram(bad_det, 0.2)
ok("malformed detection skipped, good one kept", fr2 is not None and [t.id for t in fr2.tags] == [2])
feed = s2sim.UdpTagFeed(port=0, size_m=0.20)
feed.feed_datagram(good, 1.0)
feed.feed_datagram(b'{"t": 3.0, "dets": []}', 1.1)
feed.feed_datagram(b"garbage", 1.2)
polled = feed.poll(1.3)
ok("UdpTagFeed drops empty frames by default, counts stats", len(polled) == 1 and feed.stats["datagrams"] == 3
   and feed.stats["empty"] == 1 and feed.stats["bad"] == 1 and feed.stats["dets"] == 1, str(feed.stats))
ok("poll clears the buffer", feed.poll(1.4) == [])
ok("measure_alignment accepts sim detections",
   al.measure_alignment(s2sim.parse_tag_datagram(s2sim.encode_tag_datagram(1.0, [
       tags.TagDetection(0, 0.2, 0, "", 2, (-0.25, 0.0, 0.7), _rot_y_cols(0.0), (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "x"),
       tags.TagDetection(1, 0.2, 0, "", 2, (0.25, 0.0, 0.7), _rot_y_cols(0.0), (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "x")]), 0.2).tags).valid)

print("\n[sim: telemetry → pose dict]")
pv = _Obj(position=_Obj(north_m=1.0, east_m=2.0, down_m=-0.5), velocity=_Obj(north_m_s=0.1, east_m_s=0.0, down_m_s=0.0))
pd = s2sim.telemetry_to_pose(pv, _Obj(roll_deg=1.0, pitch_deg=2.0, yaw_deg=-90.0), None, now=5.0)
ok("telemetry_to_pose fields", pd["pos_x"] == 1.0 and pd["pos_y"] == 2.0 and pd["pos_z"] == -0.5 and pd["yaw_deg"] == -90.0
   and pd["vel_x"] == 0.1 and math.isnan(pd["ang_z"]) and pd["mono_time"] == 5.0 and pd["raw"] == s2sim.SIM_RAW)
parsed_keys = set(parse_voxl_inspect_pose_line("100.0| 1 2 3 | 4 5 6 | 7 8 9 | 0 0 0").keys())
ok("sim pose dict has every voxl pose key", parsed_keys <= set(pd.keys()), str(parsed_keys - set(pd.keys())))

print("\n[sim: Vehicle(sim=True) reach pipe from MAVSDK telemetry]")
tmp = tempfile.mkdtemp(prefix="s2flight_sim_")
try:
    async def sim_pose_check():
        drone = FakeDrone(yaw0=0.0)            # drone.vehicle 을 안 묶으므로 push_pose 는 안 돈다 → 포즈는 sim 경로에서만 온다
        cfgS = fast_cfg(sim=True)
        v = Vehicle(cfgS, recorder=None, drone=drone, pose_readers=False)
        await v.connect()
        await v.wait_position_stable(timeout=5.0)
        await v.capture_initial_mavsdk_position(samples=3, interval=0.05)
        await v.start_monitors_and_offboard(prime_sec=0.3)
        got = dict(v.voxl_latest)
        ready = v.reach_ready
        off = (v.reach_offset_n, v.reach_offset_e, v.reach_offset_d)
        await v.cancel_background_tasks()
        return v, drone, got, ready, off

    v, drone, got, ready, off = loop.run_until_complete(sim_pose_check())
    ok("sim vehicle flag from cfg", v.sim is True)
    ok("reach pipe ready via telemetry", ready and v.reach_pipe in got, str(list(got)))
    ok("all cfg pipes populated with the same sim pose",
       all(p in got for p in v.cfg.voxl_pipes) and got[v.cfg.voxl_pipes[-1]]["raw"] == s2sim.SIM_RAW)
    ok("sim reach offset ~0 (telemetry == command frame)", all(abs(o) < 0.01 for o in off), str(off))
    ok("no voxl-inspect-pose errors recorded", not v.voxl_reader_errors, str(v.voxl_reader_errors))

    print("\n[full sequence — SITL mode (--sim, UDP tag feed, 8 deg wall misalignment)]")

    class UdpWallSender(object):
        """FakeWallCamera 의 프레임을 sim_tag_detect.py 형식 JSON 으로 UDP 전송 (시뮬 검출기 대역)."""

        def __init__(self, cam, port):
            self.cam = cam
            self.port = port
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sent = 0
            self.running = True

        async def run(self):
            t = 0.0
            while self.running:
                for f in self.cam.poll(util.mono_time()):
                    t += 0.05
                    self.sock.sendto(s2sim.encode_tag_datagram(t, f.tags), ("127.0.0.1", self.port))
                    self.sent += 1
                await asyncio.sleep(0.01)
            self.sock.close()

    async def sim_full():
        drone = FakeDrone(yaw0=0.0)
        cam = FakeBannerCamera(drone, 278.0, drift_scale=0.05)
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        feedS = s2sim.UdpTagFeed(port=port, size_m=0.20)
        sender = UdpWallSender(cam, port)
        rec = FlightRecorder(fast_cfg(sim=True), out_dir=tmp, prefix="s")
        v = Vehicle(rec.cfg, recorder=rec, drone=drone, pose_readers=False, initial_yaw=drone.yaw)
        feedS.start(asyncio.get_event_loop())
        send_task = asyncio.ensure_future(sender.run())
        try:
            await v.connect()
            await v.wait_position_stable(timeout=5.0)
            await v.check_vio_quality_preflight(samples=2)
            await v.capture_initial_mavsdk_position(samples=3, interval=0.05)
            await v.start_monitors_and_offboard(prime_sec=0.3)
            seq = await mdf.run_sequence(v, fast_params(), feed=feedS, recorder=rec)
        finally:
            sender.running = False
            await send_task
            feedS.stop()
            await v.cancel_background_tasks()
        return v, drone, seq, feedS, sender

    v, drone, seq, feedS, sender = loop.run_until_complete(sim_full())
    ok("sim sequence completed", seq["completed"] and not seq["aborted"], str(seq.get("align")))
    ok("sim alignment via UDP feed", seq["align"]["state"] == al.ALIGNED and feedS.stats["frames"] > 5, str(feedS.stats))
    ok("sim yaw corrected toward the wall", near(util.angle_diff_deg(drone.yaw, 278.0), 0.0, 2.0), "yaw={:.1f}".format(drone.yaw))
    ok("sim all stages OK/STOPPED, landed", all(r["status"] in ("OK", "STOPPED") for r in v.results) and landed_cmd(drone),
       str([(r["name"], r["status"]) for r in v.results]))
    ok("sim tag-stop legs via UDP, drift 5 % measured", len(seq["legs"]) == 2 and all(l["status"] == "STOPPED" for l in seq["legs"])
       and abs(seq["legs_settled"][0]["drift_along"] - seq["legs_settled"][0]["truth_along"] * 0.05 / 0.95) < 0.02 and seq["legs_settled"][0]["drift_along"] > 0.03,
       str([(l["status"], round(l["drift_along"], 3)) for l in seq["legs"]] + [round(l["drift_along"], 3) for l in seq["legs_settled"]]))
    ok("sim pose source used for reach", v.voxl_latest[v.reach_pipe]["raw"] == s2sim.SIM_RAW)
    ok("sim mode: MAVSDK position monitor fed by sim_pose_task (mav_updates > 0, tracks fake)",
       v.mav_update_count > 20 and abs(v.mav_z - drone.d) < 0.1, "updates %d mav_z %.3f fake %.3f" % (v.mav_update_count, v.mav_z, drone.d))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# ---------------------------------------------------------------------------
# 11. offboard start retry (2026-11-10: SITL 과부하에서 NO_SETPOINT_SET 1회)
# ---------------------------------------------------------------------------

print("\n[offboard start retry]")
tmp = tempfile.mkdtemp(prefix="s2f_")
try:
    class FlakyDrone(FakeDrone):
        def __init__(self, fail_n, **kw):
            FakeDrone.__init__(self, **kw)
            self.fail_n = fail_n

        async def _offboard_start(self):
            self.cmd_log.append("offboard_start")
            if self.fail_n > 0:
                self.fail_n -= 1
                raise vehicle.OffboardError("NO_SETPOINT_SET")

    async def start_only(drone, cfg):
        rec = FlightRecorder(cfg, out_dir=tmp, prefix="r")
        v = Vehicle(cfg, recorder=rec, drone=drone, pose_readers=False, initial_yaw=drone.yaw)
        drone.vehicle = v
        await v.connect()
        await v.wait_position_stable(timeout=5.0)
        await v.capture_initial_mavsdk_position(samples=3, interval=0.05)
        err = None
        try:
            await v.start_monitors_and_offboard(prime_sec=0.3)
        except vehicle.OffboardError as e:
            err = e
        finally:
            await v.cancel_background_tasks()
        return v, err

    d1 = FlakyDrone(2)
    v1, e1 = loop.run_until_complete(start_only(d1, fast_cfg(offboard_start_retries=3, offboard_retry_sec=0.05)))
    ok("retry: 2 rejections then success -> offboard_started, 3 start calls", e1 is None and d1.cmd_log.count("offboard_start") == 3
       and any(ev.get("event") == "offboard_started" for ev in v1.rec.events) and not v1.aborted, str(d1.cmd_log))
    ok("retry events recorded", sum(1 for ev in v1.rec.events if ev.get("event") == "offboard_retry") == 2)
    d2 = FlakyDrone(5)
    v2, e2 = loop.run_until_complete(start_only(d2, fast_cfg(offboard_start_retries=3, offboard_retry_sec=0.05)))
    ok("retry exhausted -> OffboardError + emergency stop (land)", e2 is not None and d2.cmd_log.count("offboard_start") == 4
       and "land" in d2.cmd_log and v2.aborted, str(d2.cmd_log))
    d3 = FlakyDrone(1)
    v3, e3 = loop.run_until_complete(start_only(d3, fast_cfg(offboard_start_retries=0)))
    ok("retries=0 keeps v14 behaviour (first rejection -> abort)", e3 is not None and d3.cmd_log.count("offboard_start") == 1 and v3.aborted, str(d3.cmd_log))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# ---------------------------------------------------------------------------
# 11b. VIO gate / in-flight VIO monitor / estimator disagreement / takeoff sanity (2026-11-10 incident)
# ---------------------------------------------------------------------------

print("\n[vio safety]")
from s2flight import vio as s2vio  # noqa: E402
_pl = s2vio.parse_voxl_inspect_vio_line
ok("vio parse: header -> None", _pl(" dt(ms) |    T_imu_wrt_vio (m)   |Roll Pitch Yaw (deg)|features|quality| state| error_codes ") is None)
_pv = _pl("   66.7 |   -0.05    0.05   -0.03|  -2.2    0.4  -70.9|    11  |   96% | OKAY | ")
ok("vio parse: sample -> quality 96 features 11 OKAY pos", _pv["quality"] == 96 and _pv["features"] == 11 and _pv["state"] == "OKAY" and _pv["pos"] == (-0.05, 0.05, -0.03), repr(_pv))
ok("vio parse: rpy (-2.2, 0.4, -70.9)", _pv["rpy"] == (-2.2, 0.4, -70.9), repr(_pv.get("rpy")))
_pf = _pl("   66.7 |    0.00    0.00    0.02|   0.0    0.0    0.0|     0  |    0% | FAILED | 0x8 ")
ok("vio parse: failed -> quality 0 features 0 FAILED 0x8", _pf["quality"] == 0 and _pf["features"] == 0 and _pf["state"] == "FAILED" and _pf["error_codes"] == "0x8")
_h = s2vio.VioHistory()
_h.add({"quality": 96, "features": 11, "state": "OKAY", "mono_time": 0.0})
_h.add({"quality": -1, "features": 0, "state": "OKAY", "mono_time": 1.0})
_h.add({"quality": -1, "features": 0, "state": "OKAY", "mono_time": 2.0})
ok("vio history: bad_duration counts from first bad sample", near(_h.bad_duration(2.5), 1.5, 1e-9) and _h.n_bad_episodes == 1)
_h.add({"quality": 80, "features": 9, "state": "OKAY", "mono_time": 3.0})
ok("vio history: recovers", _h.bad_duration(3.1) == 0.0 and _h.summarize(3.0, 10.0)["min_quality"] == -1)

tmp = tempfile.mkdtemp(prefix="s2f_")
try:
    # (a) 기본 FakeDrone (quality 96) 이면 게이트 통과 + 요약에 PASS
    drone = FakeDrone()
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("vio gate PASS on healthy fake drone; flight completes", v.vio_gate_result and v.vio_gate_result["passed"] is True and seq["completed"],
       str(v.vio_gate_result))
    ok("vio gate event recorded", any(e["event"] == "vio_gate" and e["detail"] == "PASS" for e in rec.events))
    ok("sync csv carries vio_quality column", any(r.get("vio_quality") not in (None, "") for r in rec.sync_rows) if hasattr(rec, "sync_rows") else True)

    # (b) 품질 낮음 → PreflightAbort, arm 안 함
    drone = FakeDrone()
    drone.vio_quality = 10
    err = None
    try:
        loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    except vehicle.PreflightAbort as e:
        err = e
    ok("vio gate FAIL on low quality -> PreflightAbort, never armed", err is not None and "quality" in str(err) and "arm" not in drone.cmd_log, str(err))

    # (c) 특징점 0 → 거부
    drone = FakeDrone()
    drone.vio_features = 0
    err = None
    try:
        loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    except vehicle.PreflightAbort as e:
        err = e
    ok("vio gate FAIL on zero features", err is not None and "features" in str(err) and "arm" not in drone.cmd_log, str(err))

    # (d) 지상 정지인데 파이프가 흐름 → 거부 (사고: 3 s 에 0.45 m)
    drone = FakeDrone()
    def _drift(d):
        d.extra_pipe_offset = (0.0, 0.0, d.extra_pipe_offset[2] + 0.015)     # 매 스텝 15 mm (20 Hz) → 0.4 s 창에 ~12 cm (한계 3 cm)
    drone.on_step = _drift
    err = None
    try:
        loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    except vehicle.PreflightAbort as e:
        err = e
    ok("vio gate FAIL on static feedback drift", err is not None and ("drifted" in str(err) or "disagree" in str(err)) and "arm" not in drone.cmd_log, str(err))

    # (e) 비행 중 VIO 상실 → 비상 착륙
    drone = FakeDrone()
    _state = {"t": None}
    def _lose(d):
        if d.in_air_flag and _state["t"] is None:
            _state["t"] = time.monotonic()
        if _state["t"] is not None and time.monotonic() - _state["t"] > 0.3:
            d.vio_quality = -1
    drone.on_step = _lose
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("in-flight VIO loss -> emergency land with VIO LOST reason", v.aborted and "VIO LOST" in (v.abort_reason or "") and landed_cmd(drone), str(v.abort_reason))
    ok("vio_bad event recorded", any(e["event"] == "vio_bad" for e in rec.events))

    # (f) 비행 중 파이프 vs MAVSDK 불일치 → 비상 착륙
    drone = FakeDrone()
    _state2 = {"t": None}
    def _split(d):
        if d.in_air_flag and _state2["t"] is None:
            _state2["t"] = time.monotonic()
        if _state2["t"] is not None and time.monotonic() - _state2["t"] > 0.3:
            d.extra_pipe_offset = (0.0, 0.0, 0.6)          # 파이프 고도만 0.6 m 어긋남
    drone.on_step = _split
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("in-flight estimator disagreement -> emergency land", v.aborted and "DISAGREEMENT" in (v.abort_reason or "") and landed_cmd(drone), str(v.abort_reason))

    # (g) 이륙 직전 오프셋 → 이륙 거부 (게이트 뒤에 갑자기 흘렀을 때)
    drone = FakeDrone()
    async def _takeoff_refused():
        rec2 = FlightRecorder(fast_cfg(), out_dir=tmp, prefix="tr")
        vv = Vehicle(rec2.cfg, recorder=rec2, drone=drone, pose_readers=False, initial_yaw=drone.yaw)
        drone.vehicle = vv
        await vv.connect()
        await vv.wait_position_stable(timeout=5.0)
        await vv.check_vio_quality_preflight(samples=2)
        await vv.capture_initial_mavsdk_position(samples=3, interval=0.05)
        await vv.start_monitors_and_offboard(prime_sec=0.3)
        drone.extra_pipe_offset = (0.0, 0.0, -0.4)           # 게이트 통과 직후 파이프가 0.4 m 점프
        await asyncio.sleep(0.2)
        okk = await vv.takeoff(0.5)
        await asyncio.sleep(0.3)
        await vv.cancel_background_tasks()
        return vv, okk
    vv, okk = loop.run_until_complete(_takeoff_refused())
    ok("takeoff refused when on-ground offset > limit", okk is False and "TAKEOFF REFUSED" in (vv.abort_reason or ""), str(vv.abort_reason))

    # (h) --no-vio-gate 면 게이트 생략 (요약에 disabled)
    drone = FakeDrone()
    drone.vio_quality = 10
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(vio_gate=False), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("--no-vio-gate skips the gate (disabled) and flies", v.vio_gate_result["passed"] is None and seq["completed"])
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# ---------------------------------------------------------------------------
# 11c. MAVSDK telemetry lag (2026-11-10 second incident): consumer must keep up; lag gate; fresh-only decisions
# ---------------------------------------------------------------------------

print("\n[mavsdk lag]")
tmp = tempfile.mkdtemp(prefix="s2f_")
try:
    # (a) 60 Hz 위치 스트림을 소비 루프가 따라가는가: 비행 후 MAVSDK 위치가 실제(fake)와 일치, 지연 ~0, 가짜 불일치 없음
    drone = FakeDrone()
    drone.pos_period = 1.0 / 60.0
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("60 Hz position stream: flight completes, no false disagreement", seq["completed"] and not v.aborted, str(v.abort_reason))
    ok("60 Hz position stream: MAVSDK position tracks the vehicle at the end (|d| < 0.05 m)",
       abs(v.mav_z - drone.d) < 0.05 and abs(v.mav_x - drone.n) < 0.05, "mav %.3f/%.3f fake %.3f/%.3f" % (v.mav_x, v.mav_z, drone.n, drone.d))
    ok("lag monitor: odometry present -> lag known and ~0", v.mav_lag_known and v.mav_lag_s < 0.1, "lag %.3f known %s" % (v.mav_lag_s, v.mav_lag_known))
    ok("gate result records mav_lag_s", v.vio_gate_result.get("mav_lag_s") is not None)

    # (b) 지연 1.0 s 인 odometry -> 게이트 거부 (arm 없음)
    drone = FakeDrone()
    drone.odom_lag_s = 0.0
    _st = {"t0": None}
    def _grow_lag(d):
        # 기준선은 처음 값으로 잡히므로, 그 뒤 지연을 1.0 s 로 키운다
        if _st["t0"] is None:
            _st["t0"] = time.monotonic()
        elif time.monotonic() - _st["t0"] > 0.25:
            d.odom_lag_s = 1.0
    drone.on_step = _grow_lag
    err = None
    try:
        loop.run_until_complete(run_flight(drone, fast_cfg(vio_gate_window_s=0.8), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    except vehicle.PreflightAbort as e:
        err = e
    ok("MAVSDK lag 1.0 s -> gate FAIL, never armed", err is not None and "lag" in str(err) and "arm" not in drone.cmd_log, str(err))

    # (c) 비행 중 MAVSDK 지연이 커지면 불일치 판정을 보류한다 (파이프만 갈라져도 지연 중엔 착륙하지 않음) + 지연 > abort 면 착륙
    drone = FakeDrone()
    _st2 = {"t": None}
    def _lag_inflight(d):
        if d.in_air_flag and _st2["t"] is None:
            _st2["t"] = time.monotonic()
        if _st2["t"] is not None and time.monotonic() - _st2["t"] > 0.2:
            d.odom_lag_s = 3.0                      # 소비 지연 3 s 흉내 (> abort 2.0)
    drone.on_step = _lag_inflight
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("in-flight MAVSDK lag > abort -> emergency land with LAG reason", v.aborted and "LAG" in (v.abort_reason or ""), str(v.abort_reason))
    ok("mav_lag event recorded", any(e["event"] == "mav_lag" for e in rec.events))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# ---------------------------------------------------------------------------
# 11d. external disarm (kill switch) must stop the sequence immediately
# ---------------------------------------------------------------------------

print("\n[external disarm]")
tmp = tempfile.mkdtemp(prefix="s2f_")
try:
    drone = FakeDrone()
    _k = {"t": None}
    def _kill_mid(d):
        if d.in_air_flag and _k["t"] is None:
            _k["t"] = time.monotonic()
        if _k["t"] is not None and time.monotonic() - _k["t"] > 0.3 and d.armed:
            d.armed = False                       # 조종자 킬 스위치
            d.cmd_log.append("KILL_SWITCH")
    drone.on_step = _kill_mid
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("external disarm -> aborted with EXTERNAL DISARM reason", v.aborted and "EXTERNAL DISARM" in (v.abort_reason or ""), str(v.abort_reason))
    ok("external disarm -> no stages continued after the kill",
       all(r["status"] not in ("OK", "REACHED_POOR_DELTA") for r in v.results[1:]) if len(v.results) > 1 else True,
       str([(r["name"], r["status"]) for r in v.results]))
    ok("external_disarm event recorded", any(e["event"] == "external_disarm" for e in rec.events))
    # 정상 비행에서는 우리가 명령한 disarm 이 외부 disarm 으로 오인되지 않는다 (이전 섹션들의 completed 비행들이 증거) -> 명시 확인
    drone = FakeDrone()
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("own land/disarm is not flagged as external disarm", seq["completed"] and not any(e["event"] == "external_disarm" for e in rec.events))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# ---------------------------------------------------------------------------
# 11e. PX4 STATUSTEXT gate: recent 'Preflight Fail' (post-crash bias) refuses arm
# ---------------------------------------------------------------------------

print("\n[px4 statustext gate]")
tmp = tempfile.mkdtemp(prefix="s2f_")
try:
    drone = FakeDrone()
    drone.status_texts.append(("CRITICAL", "Preflight Fail: High Accelerometer Bias"))
    err = None
    try:
        loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    except vehicle.PreflightAbort as e:
        err = e
    ok("recent 'Preflight Fail' statustext -> gate FAIL, never armed", err is not None and "PX4 reported" in str(err) and "arm" not in drone.cmd_log, str(err))
    drone = FakeDrone()
    drone.status_texts.append(("INFO", "Takeoff detected"))
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("benign statustext does not block; messages kept in summary", seq["completed"] and any("Takeoff detected" in m[2] for m in v.summary_dict()["px4_msgs"]))
    drone = FakeDrone()
    drone.status_texts.append(("CRITICAL", "Preflight Fail: High Accelerometer Bias"))
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(px4_msg_quiet_s=0.0), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("--px4-msg-quiet-sec 0 disables the statustext gate", seq["completed"])
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# ---------------------------------------------------------------------------
# 11f. EKF heading drift defence (2026-11-12: gyro bias z -3.5 deg/s, heading spinning while static)
# ---------------------------------------------------------------------------

print("\n[ekf heading drift]")
ok("yaw_range_deg wraps (359, 1, 0.5) -> 2", near(s2vio.yaw_range_deg([359.0, 1.0, 0.5]), 2.0, 1e-9))
ok("yaw_range_deg ignores NaN / single", s2vio.yaw_range_deg([float("nan"), 10.0]) == 0.0)
_hy = s2vio.VioHistory()
for _i in range(5):
    _hy.add({"quality": 90, "features": 10, "state": "OKAY", "rpy": (0.0, 0.0, 100.0 + 0.1 * _i), "mono_time": float(_i)})
ok("VioHistory yaw_range + latest_yaw", near(_hy.summarize(4.0, 10.0)["yaw_range"], 0.4, 1e-6) and near(_hy.latest_yaw(), 100.4, 1e-9))
_gb = s2vio.parse_px4_gyro_bias("    gyro_bias: [0.00306, 0.00682, -0.06156]\n    gyro_bias_limit: 0.15000\n")
ok("parse px4 gyro bias -> deg/s (z -3.53)", _gb is not None and near(_gb[2], -3.527, 0.01) and near(_gb[0], 0.175, 0.01), repr(_gb))
ok("parse px4 gyro bias: missing -> None", s2vio.parse_px4_gyro_bias("nothing") is None)
_ab = s2vio.parse_px4_accel_bias("    gyro_bias: [0.0, 0.0, 0.0]\n    accel_bias: [0.00243, 0.00091, 0.39500]\n")
ok("parse px4 accel bias (z 0.395)", _ab is not None and near(_ab[2], 0.395, 1e-6), repr(_ab))
_hr = s2vio.VioHistory()
for _i in range(31):
    _hr.add({"quality": 90, "features": 10, "state": "OKAY", "mono_time": _i / 30.0})
ok("VioHistory rate_hz ~30", near(_hr.summarize(1.0, 10.0)["rate_hz"], 30.0, 0.5), str(_hr.summarize(1.0, 10.0)["rate_hz"]))
ok("read_uptime_s: None or float", s2vio.read_uptime_s() is None or isinstance(s2vio.read_uptime_s(), float))

tmp = tempfile.mkdtemp(prefix="s2f_")
try:
    # (a) 지상에서 EKF yaw 가 25 deg/s 로 흐르면 게이트 거부 (0.4 s 창 → ~8 deg ≫ 1.0; 8 deg/s 는 타이밍에 따라 2.5 deg 로 아슬아슬했다)
    drone = FakeDrone()
    drone.ekf_yaw_drift_dps = 25.0
    err = None
    try:
        loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    except vehicle.PreflightAbort as e:
        err = e
    ok("gate FAIL on static EKF heading drift, never armed", err is not None and "heading drifted" in str(err) and "arm" not in drone.cmd_log, str(err))

    # (b) 부팅 직후 (uptime 30 s) → 거부
    _orig_up = vehicle.read_uptime_s
    vehicle.read_uptime_s = lambda: 30.0
    try:
        drone = FakeDrone()
        err = None
        try:
            loop.run_until_complete(run_flight(drone, fast_cfg(min_uptime_s=120.0), fast_params(do_align=False, stop_mode="distance"), None, tmp))
        except vehicle.PreflightAbort as e:
            err = e
        ok("gate FAIL when booted 30 s ago (min 120)", err is not None and "booted only" in str(err) and "arm" not in drone.cmd_log, str(err))
    finally:
        vehicle.read_uptime_s = _orig_up

    # (c) EKF 자이로 바이어스 -3.5 deg/s → 거부
    _orig_gb = vehicle.read_px4_sensor_bias

    async def _fake_gb(timeout_s=6.0):
        return ((0.1, 0.2, -3.5), (0.0, 0.0, 0.1)), None
    vehicle.read_px4_sensor_bias = _fake_gb
    try:
        drone = FakeDrone()
        err = None
        try:
            loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
        except vehicle.PreflightAbort as e:
            err = e
        ok("gate FAIL on EKF gyro bias 3.5 deg/s", err is not None and "gyro bias" in str(err) and "arm" not in drone.cmd_log, str(err))

        async def _fake_ab(timeout_s=6.0):
            return ((0.1, 0.0, 0.05), (0.01, -0.03, 0.395)), None
        vehicle.read_px4_sensor_bias = _fake_ab
        drone = FakeDrone()
        err = None
        try:
            loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
        except vehicle.PreflightAbort as e:
            err = e
        ok("gate FAIL on EKF accel bias 0.395 m/s2 (ceiling crash state)", err is not None and "accel bias" in str(err) and "arm" not in drone.cmd_log, str(err))
    finally:
        vehicle.read_px4_sensor_bias = _orig_gb

    # (c2) 특징점 6 (2026-11-12 이륙 지점) → 거부 (기본 vio_min_features 8)
    drone = FakeDrone()
    drone.vio_features = 6
    err = None
    try:
        loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    except vehicle.PreflightAbort as e:
        err = e
    ok("gate FAIL on VIO features 6 (< 8)", err is not None and "features" in str(err) and "arm" not in drone.cmd_log, str(err))

    # (c3) VIO 출력률 저하 (1/4) → 거부
    drone = FakeDrone()
    drone.vio_every_n = 4
    err = None
    try:
        loop.run_until_complete(run_flight(drone, fast_cfg(vio_min_rate_hz=10.0), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    except vehicle.PreflightAbort as e:
        err = e
    ok("gate FAIL on low VIO status rate", err is not None and "rate" in str(err) and "arm" not in drone.cmd_log, str(err))

    # (d) 비행 중 EKF yaw 만 60 deg/s 로 흐름 → HEADING DISAGREEMENT 착륙
    drone = FakeDrone()

    def _drift_after_offboard(d):
        if d.vehicle is not None and d.vehicle.offboard_active:
            d.ekf_yaw_drift_dps = 60.0
    drone.on_step = _drift_after_offboard
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("in-flight EKF-vs-VIO heading divergence -> emergency land", v.aborted and "HEADING DISAGREEMENT" in (v.abort_reason or "") and landed_cmd(drone),
       "%s %s" % (v.abort_reason, drone.cmd_log[-3:]))
    ok("heading base recorded at offboard start", v.summary_dict()["yaw_ekf_vio_base_deg"] is not None)

    # (d2) attitude 스트림이 죽어 yaw 샘플이 없으면 게이트 FAIL (검사가 꺼진 채 통과 금지 — 2026-11-12 SITL V6)
    drone = FakeDrone()

    async def _never():
        await asyncio.sleep(3600)
        yield None
    drone.telemetry.attitude_euler = _never
    err = None
    try:
        loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    except vehicle.PreflightAbort as e:
        err = e
    ok("gate FAIL when no EKF yaw samples (dead attitude feed)", err is not None and "no EKF yaw samples" in str(err) and "arm" not in drone.cmd_log, str(err))

    # (e0) 기본 turn_deg 0 → aim_turn 단계 없음, settle 호버 뒤 바로 진행 (2026-11-12 정면 이륙)
    drone = FakeDrone()
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance", turn_deg=0.0), None, tmp))
    _names = [e["detail"] for e in rec.events if e["event"] == "stage_start"]
    ok("turn_deg 0: no aim_turn stage, flight completes", seq["completed"] and "aim_turn" not in _names and "post_takeoff_settle" in " ".join(
        str(e.get("detail", "")) for e in rec.events) or (seq["completed"] and "aim_turn" not in _names), str(_names[:4]))
    ok("DriftParams default turn_deg is 0", mdf.DriftParams().turn_deg == 0.0)

    # (t1) v0.2.5 접지: 정상 비행은 PX4 LAND 없이 body-velocity 하강으로 접지, action.land 호출 없음
    drone = FakeDrone()
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    _ev = [e["event"] for e in rec.events]
    ok("offboard touchdown: completed, landed, no action.land", seq["completed"] and v.landed_flag and "vel_body_down" in drone.cmd_log and "land" not in drone.cmd_log
       and "touchdown_start" in _ev and "landed" in _ev and not drone.in_air_flag, "%s %s" % (drone.cmd_log[-4:], v.landed_flag))
    # (t2) --land-mode px4 → 종전 action.land
    drone = FakeDrone()
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(land_mode="px4"), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("land_mode px4: action.land used", seq["completed"] and "land" in drone.cmd_log and "vel_body_down" not in drone.cmd_log, str(drone.cmd_log[-4:]))
    # (t3) 비상 착륙도 body-velocity 접지 (VIO 상실)
    drone = FakeDrone()

    def _lose_vio(d):
        if d.vehicle is not None and d.vehicle.offboard_active and d.d < -0.3:
            d.vio_quality = -1
    drone.on_step = _lose_vio
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("emergency: body-velocity touchdown, landed, no action.land", v.aborted and "VIO LOST" in (v.abort_reason or "") and "vel_body_down" in drone.cmd_log
       and "land" not in drone.cmd_log and not drone.in_air_flag and v.landed_flag, "%s %s" % (v.abort_reason, drone.cmd_log[-4:]))
    # (t4) 접지 감지가 안 되면 PX4 LAND 폴백
    drone = FakeDrone()
    drone.stuck_after_vel = True

    def _stick(d):
        if d.vel_body is not None and getattr(d, "stuck_after_vel", False):
            d.stuck = True            # 하강 명령을 무시 → 타임아웃 → action.land 폴백 (landing 흉내는 _land 가 처리)
    drone.on_step = _stick
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("touchdown timeout -> PX4 LAND fallback", "touchdown_timeout" in [e["event"] for e in rec.events] and "land" in drone.cmd_log, str(drone.cmd_log[-4:]))

    # (t5) v0.2.6: 접지 뒤 바닥에서 VIO 가 끊겨도 비상 착륙이 다시 발동하지 않는다; 접지 판정은 2 s 안
    drone = FakeDrone()

    def _vio_dies_on_ground(d):
        if d.vehicle is not None and d.vehicle.landed_flag:
            d.vio_quality = -1
    drone.on_step = _vio_dies_on_ground
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    _evn = [e["event"] for e in rec.events]
    _t_start = [e["mono_time"] for e in rec.events if e["event"] == "touchdown_start"]
    _t_land = [e["mono_time"] for e in rec.events if e["event"] == "landed"]
    ok("post-landing VIO loss does not trigger emergency; completed", seq["completed"] and not v.aborted and "emergency_land" not in _evn, str(_evn[-6:]))
    ok("touchdown declared within 2 s of start", bool(_t_start and _t_land) and 0 < (_t_land[0] - _t_start[0]) < 2.0,
       "%.2f s" % ((_t_land[0] - _t_start[0]) if (_t_start and _t_land) else -1))
    ok("in_air monitor fed", v.in_air_latest is not None)
    # (t6) disarm 거부 시 강제 disarm(kill) 로 모터 정지
    drone = FakeDrone()
    _orig_disarm = drone.action.disarm

    async def _refuse_disarm():
        if drone.in_air_flag or drone.vel_body is not None:
            raise RuntimeError("Disarm rejected: not landed")
        await _orig_disarm()
    drone.action.disarm = _refuse_disarm
    drone.refuse_first = True
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("touchdown: disarm or force-disarm issued right after landed", seq["completed"] and ("disarm" in drone.cmd_log or "kill" in drone.cmd_log), str(drone.cmd_log[-4:]))

    # (e) 정상 기체: 통과, 요약에 yaw 샘플 수
    drone = FakeDrone()
    v, rec, seq, written = loop.run_until_complete(run_flight(drone, fast_cfg(), fast_params(do_align=False, stop_mode="distance"), None, tmp))
    ok("healthy: completes, gate records yaw drift ~0 and samples>0", seq["completed"] and v.vio_gate_result["ekf_yaw_range_deg"] < 0.5 and v.vio_gate_result["yaw_samples"] > 0 and v.mav_yaw_count > 0,
       str(v.vio_gate_result))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# ---------------------------------------------------------------------------
# 12. ASCII stdout (vehicle ssh locale) -> force_utf8_stdio
# ---------------------------------------------------------------------------

print("\n[utf8 stdio]")
import io as _io
_saved_out, _saved_err = sys.stdout, sys.stderr
try:
    _raw = _io.BytesIO()
    sys.stdout = _io.TextIOWrapper(_raw, encoding="ascii", errors="strict")
    _changed = util.force_utf8_stdio()
    print("Starling 2 \u2014 \ud55c\uae00 ok")
    sys.stdout.flush()
    _bytes = _raw.getvalue()
finally:
    sys.stdout, sys.stderr = _saved_out, _saved_err
ok("force_utf8_stdio wraps ASCII stdout and prints em-dash/Korean", _changed and "\u2014".encode("utf-8") in _bytes and "\ud55c\uae00".encode("utf-8") in _bytes, repr(_bytes[:60]))
try:
    sys.stdout = _io.TextIOWrapper(_io.BytesIO(), encoding="utf-8")
    sys.stderr = _io.TextIOWrapper(_io.BytesIO(), encoding="utf-8")
    _noop = util.force_utf8_stdio() is False
finally:
    sys.stdout, sys.stderr = _saved_out, _saved_err
ok("force_utf8_stdio is a no-op on UTF-8 stdout/stderr", _noop)

# ---------------------------------------------------------------------------

n_pass = sum(1 for _, c in results if c)
print("\n" + "=" * 55)
print("결과: {} / {} 통과".format(n_pass, len(results)))
if n_pass != len(results):
    print("실패:", [n for n, c in results if not c])
    sys.exit(1)
