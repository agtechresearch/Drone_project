"""Vehicle — MAVSDK offboard 기체 제어 (v14 DroneController 를 분해해 옮김, 동작 동일).

쓰는 순서
    v = Vehicle(cfg, recorder)
    await v.connect(); await v.wait_position_stable(); await v.check_vio_quality_preflight()
    await v.capture_initial_mavsdk_position()
    await v.start_monitors_and_offboard()          # 스트림·감시 시작 → arm → offboard → 지오펜스 ARM
    await v.takeoff(alt)
    await v.turn(-90, "aim", "LEFT90")             # 상대 yaw
    await v.move_body(0.0, 5.0, "move_right", "RIGHT", speed_mps=0.5)
    await v.hover(5.0, "hover_far")
    await v.preland_and_land()

좌표
    명령 프레임은 MAVSDK NED 절대값이고, "rel" 은 이륙 전 초기 위치 기준이다.
    도달 판정은 --reach-pipe(px4_vehicle_local_position) 포즈를 초기 오프셋으로 명령 프레임에
    맞춘 값(feedback)으로만 한다. MAVSDK 텔레메트리는 비교·지오펜스용이다.
    계획 위치(plan_n/e/alt/yaw)는 "명령한 대로 갔다면 지금 있어야 할 곳" 이다. move_body 는
    계획 위치에서 상대 이동한다(v14 execute_mission 의 current_plan_* 와 같다).

MAVSDK 객체는 생성자 drone= 으로 바꿔 끼울 수 있다(테스트용 가짜). 포즈 리더도 pose_readers=False
로 끄고 update_pose() 로 직접 넣을 수 있다.
"""

import asyncio
import collections
import math

from .geofence import Geofence
from .pose import pose_reader_task
from .sim import sim_pose_task, sim_vio_task
from .vio import VioHistory, vio_reader_task, journal_recent_vio_events
from .util import (angle_diff_deg, body_delta_to_ned, evaluate_horizontal_tracking,
                   horizontal_norm, log, mono_time, normalize_angle_deg, wallclock, yaw_to_360)

try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw
except ImportError:                        # 기체 밖(테스트)에서는 가짜를 넣는다
    System = None

    class OffboardError(Exception):
        pass

    class PositionNedYaw(object):
        def __init__(self, north_m, east_m, down_m, yaw_deg):
            self.north_m = north_m
            self.east_m = east_m
            self.down_m = down_m
            self.yaw_deg = yaw_deg

        def __repr__(self):
            return "PositionNedYaw(N{:+.3f} E{:+.3f} D{:+.3f} yaw{:+.1f})".format(
                self.north_m, self.east_m, self.down_m, self.yaw_deg)


class PreflightAbort(RuntimeError):
    """arm 전에 안전 조건이 안 맞아 비행을 시작하지 않음 (비상정지 아님, 모터 안 돌았음)."""


class Vehicle(object):
    def __init__(self, cfg, recorder=None, drone=None, pose_readers=True, initial_yaw=None, sim=None):
        self.cfg = cfg
        self.rec = recorder
        # 정면 기준 yaw. initial_yaw= 를 주면 그 값(테스트·호출자 지정), 아니면 start_monitors_and_offboard 가
        # cfg.initial_yaw 규칙(current/zero/숫자)대로 capture_initial_yaw() 로 정한다.
        self.initial_yaw_deg = None if initial_yaw is None else yaw_to_360(initial_yaw)
        self.initial_yaw_source = None if initial_yaw is None else "arg"
        self.sim = bool(getattr(cfg, "sim", False) if sim is None else sim)   # SITL: 포즈 = MAVSDK 텔레메트리 (s2flight.sim)
        if drone is None:
            if System is None:
                raise RuntimeError("mavsdk 가 없다. 기체에서 실행하거나 drone= 에 가짜를 넣을 것")
            drone = System(mavsdk_server_address=cfg.server_address, port=cfg.port)
        self.drone = drone
        self.pose_readers = pose_readers

        self.current_target_yaw = yaw_to_360(0.0 if initial_yaw is None else initial_yaw)
        self.target = PositionNedYaw(0.0, 0.0, 0.0, self.current_target_yaw)

        self.streaming = False
        self.monitoring = False
        self.aborted = False
        self.abort_reason = None

        self.stream_task = None
        self.mav_task = None
        self.health_task = None
        self.voxl_tasks = []
        self.sync_task = None

        self.initial_x = self.initial_y = self.initial_z = 0.0
        self.mav_x = self.mav_y = self.mav_z = 0.0
        self.mav_vn = self.mav_ve = self.mav_vd = 0.0
        self.mav_update_count = 0
        self._last_mav_update_time = 0.0
        self._last_mav_value = (0.0, 0.0, 0.0)

        self.voxl_latest = {}
        self.voxl_initial = {}
        self.voxl_reader_errors = {}

        self.reach_ready = False
        self.reach_pipe = cfg.reach_pipe
        self.reach_offset_n = self.reach_offset_e = self.reach_offset_d = 0.0
        self.reach_initial_raw = None

        self.geofence = Geofence(cfg)
        # VIO 상태 (2026-11-10 사고 대책): 게이트·비행 중 감시·추정기 불일치
        self.vio = VioHistory()
        self.vio_task = None
        self.vio_mon_task = None
        self.vio_gate_result = None
        self.offboard_active = False
        self._est_disagree_since = None
        self._vio_was_bad = False
        # MAVSDK 지연 감시 (v0.2.1)
        self.mav_lag_s = 0.0
        self.mav_lag_known = False
        self.mav_lag_task = None
        self.mav_stream_task = None
        self._mav_lag_base = None
        self.armed_task = None
        self.px4_armed = None            # telemetry.armed() 최신값 (None = 모름)
        self.status_task = None
        self.px4_msgs = collections.deque(maxlen=200)   # (mono_time, type, text) PX4 STATUSTEXT
        self.expect_armed = False        # 우리가 arm 했고 아직 disarm 명령을 안 냈다
        self._mav_lag_warned = False
        self._mav_proc_last = 0.0
        self._emergency_lock = asyncio.Lock()
        self._emergency_triggered = False
        self._stream_fail_count = 0

        self.active_stage = None
        self.active_phase = "IDLE"
        self.results = []
        self.snapshots = {}

        # 계획 위치 (명령대로 갔다면 있어야 할 곳), rel 프레임
        self.plan_n = 0.0
        self.plan_e = 0.0
        self.plan_alt = 0.0
        self.plan_yaw = self.current_target_yaw
        self._extra_sync = {}

    # ------------------------------------------------------------------
    # 공용 도우미
    # ------------------------------------------------------------------

    @staticmethod
    def now():
        return mono_time()

    def set_sync_field(self, key, value):
        """동기 CSV 에 실험 고유 열을 덧붙인다 (예: align_state)."""
        self._extra_sync[key] = value

    def extra_sync_fields(self):
        return dict(self._extra_sync)

    def event(self, name, detail="", **fields):
        if self.rec is not None:
            return self.rec.event(name, detail, **fields)
        log("[event] {} {}".format(name, detail))
        return None

    def mav_rel_position(self):
        return (self.mav_x - self.initial_x, self.mav_y - self.initial_y, self.initial_z - self.mav_z)

    def rel_target(self):
        return (self.target.north_m - self.initial_x, self.target.east_m - self.initial_y,
                self.initial_z - self.target.down_m)

    def update_pose(self, pipe_name, parsed):
        """포즈 리더(또는 테스트)가 파이프 포즈를 넣는다."""
        if "mono_time" not in parsed:
            parsed["mono_time"] = self.now()
        if "wallclock" not in parsed:
            parsed["wallclock"] = wallclock()
        self.voxl_latest[pipe_name] = parsed
        if pipe_name not in self.voxl_initial:
            self.voxl_initial[pipe_name] = {
                "pos_x": parsed["pos_x"], "pos_y": parsed["pos_y"], "pos_z": parsed["pos_z"],
                "voxl_ts_ms": parsed.get("voxl_ts_ms", float("nan")), "mono_time": parsed["mono_time"],
            }
            log("[voxl] {} initial pose: x={:+.3f} y={:+.3f} z={:+.3f} ts_ms={:.0f}".format(
                pipe_name, parsed["pos_x"], parsed["pos_y"], parsed["pos_z"],
                parsed.get("voxl_ts_ms", float("nan"))))

    def update_vio(self, parsed):
        """VIO 상태 리더(또는 시뮬·테스트)가 샘플을 넣는다. 불량 전이를 로그·이벤트로 남긴다."""
        if "mono_time" not in parsed:
            parsed["mono_time"] = self.now()
        bad = self.vio.add(parsed, min_quality=self.cfg.vio_inflight_min_quality)
        self._extra_sync["vio_quality"] = parsed.get("quality", -1)
        self._extra_sync["vio_features"] = parsed.get("features", -1)
        self._extra_sync["vio_state"] = parsed.get("state", "")
        if bad != self._vio_was_bad:
            self._vio_was_bad = bad
            msg = "quality={} features={} state={} err={}".format(parsed.get("quality"), parsed.get("features"),
                                                                  parsed.get("state"), parsed.get("error_codes"))
            log("[vio] {} {}".format("BAD" if bad else "ok", msg))
            self.event("vio_bad" if bad else "vio_ok", msg)

    def feedback_state(self):
        """reach 파이프 포즈를 명령 N/E/D 프레임으로 맞춘 값. 준비 전이면 None."""
        latest = self.voxl_latest.get(self.reach_pipe)
        if latest is None or not self.reach_ready:
            return None
        now = self.now()
        cmd_n = latest["pos_x"] + self.reach_offset_n
        cmd_e = latest["pos_y"] + self.reach_offset_e
        cmd_d = latest["pos_z"] + self.reach_offset_d
        return {
            "source": self.reach_pipe,
            "cmd_n": cmd_n, "cmd_e": cmd_e, "cmd_d": cmd_d,
            "rel_n": cmd_n - self.initial_x, "rel_e": cmd_e - self.initial_y, "rel_u": self.initial_z - cmd_d,
            "vn": latest.get("vel_x", float("nan")), "ve": latest.get("vel_y", float("nan")),
            "vd": latest.get("vel_z", float("nan")),
            "raw_x": latest["pos_x"], "raw_y": latest["pos_y"], "raw_z": latest["pos_z"],
            "yaw_deg": latest.get("yaw_deg", float("nan")),
            "roll_deg": latest.get("roll_deg", float("nan")),
            "pitch_deg": latest.get("pitch_deg", float("nan")),
            "age_sec": now - latest["mono_time"],
            "wallclock": latest["wallclock"],
        }

    def feedback_yaw(self):
        fb = self.feedback_state()
        return fb.get("yaw_deg", float("nan")) if fb else float("nan")

    # ------------------------------------------------------------------
    # 연결 / 프리플라이트
    # ------------------------------------------------------------------

    async def connect(self):
        log("[1/7] Connecting...")
        await self.drone.connect()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                log("       Connected!")
                if self.status_task is None:
                    self.status_task = asyncio.ensure_future(self._status_text_monitor())
                return
            await asyncio.sleep(0.1)

    async def wait_position_stable(self, hold_sec=None, timeout=60.0):
        hold_sec = self.cfg.pos_ok_hold_sec if hold_sec is None else hold_sec
        log("[2/7] Waiting for MAVSDK local position estimate health ({:.1f}s)...".format(hold_sec))
        ok_start = None
        deadline = self.now() + timeout
        async for health in self.drone.telemetry.health():
            now = self.now()
            if now > deadline:
                raise RuntimeError("MAVSDK/PX4 local position health did not stabilize")
            if health.is_local_position_ok:
                if ok_start is None:
                    ok_start = now
                elif now - ok_start >= hold_sec:
                    log("       Local position health OK.")
                    return
            else:
                ok_start = None
            await asyncio.sleep(0.2)

    async def check_vio_quality_preflight(self, samples=5):
        log("[3/7] Checking local-position health ({} samples)...".format(samples))
        count = 0
        async for health in self.drone.telemetry.health():
            if not health.is_local_position_ok:
                raise RuntimeError("Local position degraded during pre-check")
            count += 1
            if count >= samples:
                break
            await asyncio.sleep(0.5)
        log("       Local position health stable.")

    async def capture_initial_mavsdk_position(self, samples=10, interval=0.3):
        """초기 NED 를 잡는다. 스트림을 쉬지 않고 소화하면서 samples*interval 초 동안 모은다
        (v0.2.1: 메시지마다 sleep 하면 gRPC 버퍼에 밀려 과거 값을 읽게 된다 — 2026-11-10 2차 사고)."""
        window = max(0.5, samples * interval)
        log("[4/7] Capturing initial MAVSDK NED position ({:.1f}s window)...".format(window))
        await self._request_mav_rates()
        xs, ys, zs = [], [], []
        t0 = self.now()
        agen = self.drone.telemetry.position_velocity_ned()
        try:
            async for pos in agen:
                xs.append(pos.position.north_m)
                ys.append(pos.position.east_m)
                zs.append(pos.position.down_m)
                if self.now() - t0 >= window and len(xs) >= 3:
                    break
        finally:
            aclose = getattr(agen, "aclose", None)
            if aclose is not None:
                try:
                    await aclose()          # 구독을 닫는다 — 열어 두면 뒤의 구독이 데이터를 못 받을 수 있다
                except Exception:  # noqa: BLE001
                    pass
        samples = len(xs)

        self.initial_x = sum(xs) / len(xs)
        self.initial_y = sum(ys) / len(ys)
        self.initial_z = sum(zs) / len(zs)
        self.mav_x, self.mav_y, self.mav_z = self.initial_x, self.initial_y, self.initial_z

        x_range, y_range, z_range = max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)
        log("       Initial MAVSDK NED: N={:+.3f} E={:+.3f} D={:+.3f}".format(
            self.initial_x, self.initial_y, self.initial_z))
        log("       Initial MAVSDK ranges: N={:.6f} E={:.6f} D={:.6f}".format(x_range, y_range, z_range))

        if x_range < 1e-9 and y_range < 1e-9 and z_range < 1e-9:
            raise RuntimeError("MAVSDK NED data appears STALE. Shake drone 10s and retry.")
        if x_range > 0.05 or y_range > 0.05 or z_range > 0.05:
            raise RuntimeError("Initial MAVSDK position UNSTABLE: N_r={:.3f} E_r={:.3f} D_r={:.3f}".format(
                x_range, y_range, z_range))

        self.target = PositionNedYaw(self.initial_x, self.initial_y, self.initial_z, self.current_target_yaw)
        self.plan_n = self.plan_e = 0.0
        self.plan_alt = 0.0
        self.plan_yaw = self.current_target_yaw

    def set_initial_yaw(self, yaw_deg, source):
        """정면(yaw 명령 기준)을 정한다. 스트림 시작 전에 불러야 이륙하며 돌지 않는다."""
        self.initial_yaw_deg = yaw_to_360(yaw_deg)
        self.initial_yaw_source = source
        self.current_target_yaw = self.initial_yaw_deg
        self.target = PositionNedYaw(self.target.north_m, self.target.east_m, self.target.down_m, self.current_target_yaw)
        self.plan_yaw = self.current_target_yaw
        self.event("initial_yaw", source, yaw_deg=self.initial_yaw_deg)
        log("       Heading reference: yaw {:+.1f} deg ({}) — body 'forward' and turn() are relative to this".format(
            normalize_angle_deg(self.initial_yaw_deg), source))

    async def capture_initial_yaw(self, samples=10, interval=0.1):
        """cfg.initial_yaw 규칙대로 정면을 정한다. current 면 MAVSDK attitude_euler 를 평균(원형 평균)한다."""
        mode, value = self.cfg.initial_yaw_value()
        if mode == "zero":
            self.set_initial_yaw(0.0, "zero (VIO power-on heading, v14 behaviour)")
            return self.initial_yaw_deg
        if mode == "fixed":
            self.set_initial_yaw(value, "fixed {:+.1f} deg (--initial-yaw)".format(value))
            return self.initial_yaw_deg
        log("[4b/7] Capturing current heading as 'forward' ({} samples)...".format(samples))
        sx = sy = 0.0
        yaws = []
        async for att in self.drone.telemetry.attitude_euler():
            y = float(att.yaw_deg)
            if y != y:                       # NaN
                continue
            yaws.append(y)
            sx += math.cos(math.radians(y))
            sy += math.sin(math.radians(y))
            if len(yaws) >= samples:
                break
            await asyncio.sleep(interval)
        if not yaws:
            raise RuntimeError("attitude_euler 에서 yaw 를 받지 못했다 (--initial-yaw zero 로 v14 동작 가능)")
        mean = math.degrees(math.atan2(sy, sx))
        spread = max(abs(angle_diff_deg(y, mean)) for y in yaws)
        if spread > 5.0:
            raise RuntimeError("Heading UNSTABLE while capturing 'forward': spread {:.1f} deg over {} samples "
                               "(기체를 놓고 가만히 둘 것)".format(spread, len(yaws)))
        self.set_initial_yaw(mean, "current heading at arm ({} samples, spread {:.2f} deg)".format(len(yaws), spread))
        return self.initial_yaw_deg

    async def wait_reach_pipe_ready(self, timeout=None):
        timeout = self.cfg.reach_pipe_timeout if timeout is None else timeout
        log("       Waiting for reach feedback pipe '{}'...".format(self.reach_pipe))
        deadline = self.now() + timeout
        while self.now() < deadline:
            latest = self.voxl_latest.get(self.reach_pipe)
            if latest is not None:
                self.reach_initial_raw = dict(latest)
                self.reach_offset_n = self.initial_x - latest["pos_x"]
                self.reach_offset_e = self.initial_y - latest["pos_y"]
                self.reach_offset_d = self.initial_z - latest["pos_z"]
                self.reach_ready = True
                log("       Reach feedback ready: {} raw=(x{:+.3f},y{:+.3f},z{:+.3f})".format(
                    self.reach_pipe, latest["pos_x"], latest["pos_y"], latest["pos_z"]))
                log("       Alignment offset to command NED: dN{:+.3f} dE{:+.3f} dD{:+.3f}".format(
                    self.reach_offset_n, self.reach_offset_e, self.reach_offset_d))
                return
            await asyncio.sleep(0.05)
        raise RuntimeError("Reach feedback pipe '{}' did not produce valid pose within {:.1f}s".format(
            self.reach_pipe, timeout))

    async def preflight_vio_gate(self, window_s=None):
        """arm 직전 VIO 게이트 (2026-11-10 사고 대책 P1/P4). 조건 하나라도 실패하면 PreflightAbort.

        1) 최근 vio_quiet_s 안에 open-vins 저널에 BAD 전이/리셋 없음 (journalctl 없으면 생략)
        2) window_s 동안: VIO 샘플 있음, 최소 품질 >= vio_min_quality, 상태 모두 OKAY, 최소 특징점 >= vio_min_features
        3) 같은 창에서 reach 파이프와 MAVSDK 상대 위치 변동 <= vio_static_drift_m (지상 정지인데 흐르면 추정기가 이미 깨진 것)
        4) 창 끝에서 파이프 vs MAVSDK 상대 위치 차 <= est_disagree 의 절반
        """
        cfg = self.cfg
        window_s = cfg.vio_gate_window_s if window_s is None else window_s
        if not cfg.vio_gate:
            log("       [vio-gate] DISABLED by --no-vio-gate — VIO 품질을 보지 않고 arm 한다 (권장 안 함)")
            self.vio_gate_result = {"passed": None, "reason": "disabled"}
            self.event("vio_gate", "disabled")
            return
        log("       [vio-gate] window {:.1f}s: quality>={} features>={} state OKAY, static drift<={:.2f}m, journal quiet {:.0f}s".format(
            window_s, cfg.vio_min_quality, cfg.vio_min_features, cfg.vio_static_drift_m, cfg.vio_quiet_s))
        journal = None
        if cfg.vio_quiet_s > 0 and not self.sim:
            journal = await journal_recent_vio_events(cfg.vio_quiet_s)
            if journal is None:
                log("       [vio-gate] journalctl unavailable — skipping quiet-period check")
            elif journal:
                for l in journal[-4:]:
                    log("       [vio-gate] journal: {}".format(l[-110:]))
                self._gate_fail("open-vins BAD/reset within last {:.0f}s ({} lines) — wait and re-check".format(
                    cfg.vio_quiet_s, len(journal)))
        t0 = self.now()
        fb_pts, mav_pts = [], []
        while self.now() - t0 < window_s:
            fb = self.feedback_state()
            if fb is not None:
                fb_pts.append((fb["rel_n"], fb["rel_e"], fb["rel_u"]))
            mav_pts.append(self.mav_rel_position())
            if self.aborted:
                self._gate_fail("vehicle aborted during gate: {}".format(self.abort_reason))
            await asyncio.sleep(0.1)
        now = self.now()
        summ = self.vio.summarize(now, window_s + 0.5)
        def _range(pts):
            if len(pts) < 2:
                return 0.0
            return max(max(p[i] for p in pts) - min(p[i] for p in pts) for i in range(3))
        fb_rng, mav_rng = _range(fb_pts), _range(mav_pts)
        dv = dh = float("nan")
        if fb_pts:
            f, m = fb_pts[-1], mav_pts[-1]
            dv = abs(f[2] - m[2])
            dh = math.sqrt((f[0] - m[0]) ** 2 + (f[1] - m[1]) ** 2)
        result = {"passed": True, "window_s": window_s, "vio_n": summ["n"], "min_quality": summ["min_quality"],
                  "mean_quality": summ["mean_quality"], "min_features": summ["min_features"], "n_not_ok": summ["n_not_ok"],
                  "fb_range_m": fb_rng, "mav_range_m": mav_rng, "disagree_v_m": dv, "disagree_h_m": dh,
                  "journal_events": (len(journal) if journal is not None else None)}
        self.vio_gate_result = result
        log("       [vio-gate] samples={} quality min {} mean {:.0f} features min {} not_ok {} | drift fb {:.3f} mav {:.3f} m | fb-mav dV {:.3f} dH {:.3f} m".format(
            summ["n"], summ["min_quality"], summ["mean_quality"], summ["min_features"], summ["n_not_ok"], fb_rng, mav_rng, dv, dh))
        if summ["n"] == 0:
            self._gate_fail("no VIO status samples (voxl-inspect-vio -n {} not running?)".format(cfg.vio_pipe), result)
        if summ["min_quality"] < cfg.vio_min_quality:
            self._gate_fail("VIO quality min {} < {}".format(summ["min_quality"], cfg.vio_min_quality), result)
        if summ["n_not_ok"] > 0:
            self._gate_fail("VIO state not OKAY in {} samples".format(summ["n_not_ok"]), result)
        if summ["min_features"] >= 0 and summ["min_features"] < cfg.vio_min_features:
            self._gate_fail("VIO features min {} < {} (featureless view?)".format(summ["min_features"], cfg.vio_min_features), result)
        if not fb_pts:
            self._gate_fail("no reach-pipe feedback during gate window", result)
        if fb_rng > cfg.vio_static_drift_m:
            self._gate_fail("feedback pipe drifted {:.3f} m in {:.1f}s while on ground (limit {:.2f})".format(fb_rng, window_s, cfg.vio_static_drift_m), result)
        if mav_rng > cfg.vio_static_drift_m:
            self._gate_fail("MAVSDK position drifted {:.3f} m in {:.1f}s while on ground (limit {:.2f})".format(mav_rng, window_s, cfg.vio_static_drift_m), result)
        if dv > cfg.est_disagree_v_m / 2.0 or dh > cfg.est_disagree_h_m / 2.0:
            self._gate_fail("pipe vs MAVSDK disagree dV {:.3f} dH {:.3f} m before arm".format(dv, dh), result)
        bad_msgs = self.recent_bad_px4_msgs(cfg.px4_msg_quiet_s) if cfg.px4_msg_quiet_s > 0 else []
        result["px4_bad_msgs"] = len(bad_msgs)
        if bad_msgs:
            for t, typ, txt in bad_msgs[-4:]:
                log("       [vio-gate] px4 {:.0f}s ago {}: {}".format(self.now() - t, typ, txt))
            self._gate_fail("PX4 reported '{}' {:.0f}s ago ({} msgs in {:.0f}s) — reboot/recalibrate and wait".format(
                bad_msgs[-1][2][:60], self.now() - bad_msgs[-1][0], len(bad_msgs), cfg.px4_msg_quiet_s), result)
        result["mav_lag_s"] = self.mav_lag_s if self.mav_lag_known else None
        if self.mav_lag_known and self.mav_lag_s > cfg.mav_lag_warn_s:
            self._gate_fail("MAVSDK telemetry lag {:.2f}s > {:.2f}s (consumer falling behind)".format(self.mav_lag_s, cfg.mav_lag_warn_s), result)
        log("       [vio-gate] PASS")
        self.event("vio_gate", "PASS", **dict((k, v) for k, v in result.items() if k != "passed"))

    def _gate_fail(self, reason, result=None):
        if result is not None:
            result["passed"] = False
            result["reason"] = reason
            self.vio_gate_result = result
        else:
            self.vio_gate_result = {"passed": False, "reason": reason}
        log("!!! [vio-gate] FAIL: {} — NOT ARMING !!!".format(reason))
        self.event("vio_gate", "FAIL: " + reason)
        raise PreflightAbort("VIO gate: " + reason)

    # ------------------------------------------------------------------
    # 감시 태스크
    # ------------------------------------------------------------------

    async def _request_mav_rates(self):
        """PX4 에 텔레메트리 전송률을 cfg.mav_rate_hz 로 요청한다 (기체 onboard 링크 기본 30 Hz). 실패해도 계속."""
        tel = self.drone.telemetry
        for name in ("set_rate_position_velocity_ned", "set_rate_odometry"):
            fn = getattr(tel, name, None)
            if fn is None:
                continue
            try:
                await asyncio.wait_for(fn(float(self.cfg.mav_rate_hz)), timeout=2.0)
            except Exception as e:  # noqa: BLE001
                log("       [mav] {}({}) failed: {}".format(name, self.cfg.mav_rate_hz, e))

    def mav_fresh(self):
        """MAVSDK 위치를 판정에 써도 되는가 (지연 < mav_lag_warn_s). 지연을 아직 모르면 True."""
        return (not self.mav_lag_known) or self.mav_lag_s <= self.cfg.mav_lag_warn_s

    def update_mav_position(self, pos):
        """MAVSDK position_velocity_ned 한 건 반영 (실기: _mavsdk_position_stream, SITL: sim_pose_task 가 호출)."""
        self.mav_x, self.mav_y, self.mav_z = pos.position.north_m, pos.position.east_m, pos.position.down_m
        self.mav_vn, self.mav_ve, self.mav_vd = pos.velocity.north_m_s, pos.velocity.east_m_s, pos.velocity.down_m_s
        new_value = (self.mav_x, self.mav_y, self.mav_z)
        if new_value != self._last_mav_value:
            self._last_mav_update_time = self.now()
            self._last_mav_value = new_value
            self.mav_update_count += 1

    async def _mavsdk_position_stream(self):
        """position_velocity_ned 스트림을 **쉬지 않고** 소화한다 (v0.2.1). 메시지마다 sleep 하면 gRPC 가 버퍼링해 위치가
        초당 수백 ms 씩 과거로 밀린다 (2026-11-10: 30 Hz 입력 vs 18 Hz 소비 → 이륙 때 2~3 s, 첫 사고 때 6~8 s 지연).
        SITL 에서는 sim_pose_task 가 같은 스트림을 이미 구독하므로(두 번째 구독은 데이터를 못 받는다 — 2026-11-10 발견) 이 태스크를 띄우지 않고
        sim_pose_task 가 update_mav_position 을 호출한다."""
        async for pos in self.drone.telemetry.position_velocity_ned():
            if not self.monitoring:
                break
            self.update_mav_position(pos)

    async def _mavsdk_position_monitor(self):
        """주기(0.05 s) 처리 루프: 기록·지오펜스·추정기 불일치. 스트림 소비와 분리돼 있어 입력률과 무관하게 돈다."""
        while self.monitoring:
            await asyncio.sleep(0.05)
            if not self.monitoring:
                break
            now = self.now()
            if self.rec is not None:
                self.rec.stage_sample(self)

            # 지오펜스 입력: 실시간인 reach 파이프(명령 프레임)를 우선, 없으면 신선한 MAVSDK
            fb = self.feedback_state()
            actual = None
            if fb is not None and fb["age_sec"] <= 0.5:
                actual = (fb["cmd_n"], fb["cmd_e"], fb["cmd_d"])
            elif self.mav_fresh() and now - self._last_mav_update_time <= 0.5:
                actual = (self.mav_x, self.mav_y, self.mav_z)
            if actual is not None:
                reason = self.geofence.check(
                    now,
                    (self.target.north_m, self.target.east_m, self.target.down_m),
                    actual,
                    (self.initial_x, self.initial_y, self.initial_z))
                if reason:
                    asyncio.ensure_future(self.trigger_emergency_land(reason))
            self._check_estimator_disagreement()

    async def _status_text_monitor(self):
        """PX4 STATUSTEXT(사전점검 실패·킬·failsafe 메시지)를 모은다. mavsdk_server 가 터미널에만 뿌리던 내용을 기록·판정에 쓴다."""
        tel = self.drone.telemetry
        fn = getattr(tel, "status_text", None)
        if fn is None:
            return
        try:
            async for st in fn():
                text = str(getattr(st, "text", "")).strip()
                typ = str(getattr(st, "type", "")).split(".")[-1]
                if not text:
                    continue
                self.px4_msgs.append((self.now(), typ, text))
                if typ in ("CRITICAL", "ALERT", "EMERGENCY", "ERROR", "WARNING"):
                    log("       [px4] {}: {}".format(typ, text))
                    if self.rec is not None:
                        self.event("px4_msg", "{}: {}".format(typ, text))
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001
            log("       [px4] status_text monitor stopped: {!r}".format(e))

    def recent_bad_px4_msgs(self, seconds):
        now = self.now()
        pats = tuple(self.cfg.px4_msg_bad_patterns)
        return [(t, typ, txt) for (t, typ, txt) in self.px4_msgs if now - t <= seconds and any(p.lower() in txt.lower() for p in pats)]

    async def _armed_monitor(self):
        """PX4 arm 상태 감시 (2026-11-10 1차 사고 재시도: 킬 스위치로 disarm 된 뒤에도 코드가 30 s 동안 시퀀스를 계속했다).
        우리가 arm 한 상태(expect_armed)에서 PX4 가 disarm 이라고 하면 즉시 중단한다. 착륙/disarm 을 우리가 명령한 뒤에는 정상."""
        tel = self.drone.telemetry
        armed_fn = getattr(tel, "armed", None)
        if armed_fn is None:
            log("       [mav] telemetry.armed unavailable — external-disarm monitor off")
            return
        try:
            async for armed in armed_fn():
                if not self.monitoring:
                    break
                self.px4_armed = bool(armed)
                if self.expect_armed and not self.px4_armed and not self.aborted:
                    log("!!! PX4 reports DISARMED while we expect armed (kill switch / failsafe?) — aborting sequence !!!")
                    self.event("external_disarm", "PX4 armed=False during sequence")
                    self.expect_armed = False
                    self.streaming = False
                    self.aborted = True
                    self.abort_reason = "EXTERNAL DISARM (kill switch / failsafe) — sequence stopped"
                    self.geofence.disarm()
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001
            log("       [mav] armed monitor stopped: {!r}".format(e))

    async def _mavsdk_lag_monitor(self):
        """odometry.time_usec(PX4 시각) 와 수신 시각의 오프셋으로 소비 지연을 잰다. 기준선 = 지금까지의 최소 오프셋.
        지연 > warn: MAVSDK 기반 판정 보류(+이벤트). 지연 > abort(offboard 중): 비상 착륙. SITL 은 시뮬 시계라 건너뛴다."""
        if self.sim:
            return
        tel = self.drone.telemetry
        odom_fn = getattr(tel, "odometry", None)
        if odom_fn is None:
            log("       [mav] telemetry.odometry unavailable — lag monitor off")
            return
        t_start = self.now()
        try:
            async for od in odom_fn():
                if not self.monitoring:
                    break
                tu = getattr(od, "time_usec", None)
                if tu is None or tu <= 0:
                    continue
                now = self.now()
                off = now - tu / 1e6
                if self._mav_lag_base is None or off < self._mav_lag_base:
                    self._mav_lag_base = off
                self.mav_lag_s = max(0.0, off - self._mav_lag_base)
                if now - t_start >= self.cfg.mav_lag_calib_s:
                    self.mav_lag_known = True
                self._extra_sync["mav_lag_s"] = round(self.mav_lag_s, 3)
                if self.mav_lag_known and self.mav_lag_s > self.cfg.mav_lag_warn_s and not self._mav_lag_warned:
                    self._mav_lag_warned = True
                    log("       [mav] telemetry lag {:.2f}s > {:.2f}s — MAVSDK-based checks suspended".format(self.mav_lag_s, self.cfg.mav_lag_warn_s))
                    self.event("mav_lag", "{:.2f}s".format(self.mav_lag_s))
                elif self._mav_lag_warned and self.mav_lag_s <= self.cfg.mav_lag_warn_s / 2.0:
                    self._mav_lag_warned = False
                    log("       [mav] telemetry lag recovered ({:.2f}s)".format(self.mav_lag_s))
                if self.mav_lag_known and self.offboard_active and self.mav_lag_s > self.cfg.mav_lag_abort_s:
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "MAVSDK TELEMETRY LAG {:.1f}s (monitors blind)".format(self.mav_lag_s)))
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001
            log("       [mav] lag monitor stopped: {!r}".format(e))

    def _check_estimator_disagreement(self):
        """reach 파이프(VIO 경로) 와 MAVSDK(EKF) 상대 위치가 갈라지면 비상 착륙 (P3). 사고 때 1 s 만에 0.58 m 벌어졌다."""
        cfg = self.cfg
        if not cfg.vio_gate or not self.offboard_active or self.aborted:
            return
        if not self.mav_fresh() or self.now() - self._last_mav_update_time > 0.5:
            self._est_disagree_since = None       # 지연/정지된 MAVSDK 와 비교하면 가짜 불일치가 난다 (2026-11-10 2차 사고)
            return
        fb = self.feedback_state()
        if fb is None or fb["age_sec"] > 0.5:
            return
        mn, me, mu = self.mav_rel_position()
        dv = abs(fb["rel_u"] - mu)
        dh = math.sqrt((fb["rel_n"] - mn) ** 2 + (fb["rel_e"] - me) ** 2)
        self._extra_sync["est_dv"] = round(dv, 3)
        self._extra_sync["est_dh"] = round(dh, 3)
        now = self.now()
        if dv > cfg.est_disagree_v_m or dh > cfg.est_disagree_h_m:
            if self._est_disagree_since is None:
                self._est_disagree_since = now
                log("       [est] pipe vs MAVSDK disagree dV {:.2f} dH {:.2f} m — watching {:.1f}s".format(dv, dh, cfg.est_disagree_s))
            elif now - self._est_disagree_since >= cfg.est_disagree_s:
                asyncio.ensure_future(self.trigger_emergency_land(
                    "ESTIMATOR DISAGREEMENT: pipe vs MAVSDK dV {:.2f} dH {:.2f} m for {:.1f}s".format(dv, dh, now - self._est_disagree_since)))
        else:
            self._est_disagree_since = None

    async def _vio_monitor(self):
        """비행 중 VIO 품질·신선도 감시 (P2). offboard 시작 뒤부터 동작."""
        cfg = self.cfg
        while self.monitoring and not self.aborted:
            if cfg.vio_gate and self.offboard_active:
                now = self.now()
                if self.vio.n > 0 and self.vio.age(now) > cfg.vio_stale_abort_s:
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "VIO STATUS STALE {:.1f}s (voxl-inspect-vio died?)".format(self.vio.age(now))))
                elif self.vio.bad_duration(now) >= cfg.vio_bad_abort_s:
                    l = self.vio.latest or {}
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "VIO LOST: quality={} state={} features={} for {:.1f}s".format(
                            l.get("quality"), l.get("state"), l.get("features"), self.vio.bad_duration(now))))
                fb = self.feedback_state()
                if fb is not None and fb["age_sec"] > cfg.feedback_stale_abort_s:
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "FEEDBACK PIPE STALE {:.1f}s ({})".format(fb["age_sec"], self.reach_pipe)))
            await asyncio.sleep(0.1)

    async def _health_monitor(self):
        consecutive_bad = 0
        async for health in self.drone.telemetry.health():
            if not self.monitoring:
                break
            if not health.is_local_position_ok:
                consecutive_bad += 1
                if consecutive_bad >= 3:
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "local position health lost (3 consecutive bad samples)"))
            else:
                consecutive_bad = 0
            await asyncio.sleep(0.3)

    async def _setpoint_streamer(self):
        period = 1.0 / self.cfg.setpoint_hz
        while self.streaming:
            try:
                await self.drone.offboard.set_position_ned(self.target)
                self._stream_fail_count = 0
            except Exception as e:
                self._stream_fail_count += 1
                log("       [streamer] send failed ({}/{}): {}".format(
                    self._stream_fail_count, self.cfg.max_stream_fail, e))
                if self._stream_fail_count >= self.cfg.max_stream_fail:
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "setpoint stream failed {} times".format(self.cfg.max_stream_fail)))
                    break
            await asyncio.sleep(period)

    async def _sync_sampler(self):
        period = max(0.2, float(self.cfg.csv_sample_sec))
        log("[csv] synchronized sampler running every {:.2f}s".format(period))
        while self.monitoring and not self.aborted:
            self.rec.sync_sample(self)
            await asyncio.sleep(period)

    def _is_running(self):
        return self.monitoring and not self.aborted

    # ------------------------------------------------------------------
    # arm / offboard
    # ------------------------------------------------------------------

    async def start_monitors_and_offboard(self, prime_sec=3.0):
        """스트림·감시 시작 → reach 파이프 정렬 → arm → offboard 시작 → 지오펜스 ARM."""
        if self.initial_yaw_deg is None:
            await self.capture_initial_yaw()
        log("[5/7] Starting monitors + priming setpoint stream ({:.0f}s)...".format(prime_sec))
        self.monitoring = True
        self.streaming = True

        self.stream_task = asyncio.ensure_future(self._setpoint_streamer())
        self.mav_task = asyncio.ensure_future(self._mavsdk_position_monitor())
        if not self.sim:
            self.mav_stream_task = asyncio.ensure_future(self._mavsdk_position_stream())
        self.mav_lag_task = asyncio.ensure_future(self._mavsdk_lag_monitor())
        self.armed_task = asyncio.ensure_future(self._armed_monitor())
        self.health_task = asyncio.ensure_future(self._health_monitor())
        if self.sim:
            self.voxl_tasks.append(asyncio.ensure_future(
                sim_pose_task(self, self.cfg.voxl_pipes, self._is_running, self.voxl_reader_errors)))
        elif self.pose_readers:
            for pipe in self.cfg.voxl_pipes:
                self.voxl_tasks.append(asyncio.ensure_future(
                    pose_reader_task(pipe, self.update_pose, self._is_running, self.voxl_reader_errors)))
        if self.rec is not None:
            self.sync_task = asyncio.ensure_future(self._sync_sampler())
        if self.cfg.vio_gate:
            if self.sim:
                self.vio_task = asyncio.ensure_future(sim_vio_task(self, self._is_running))
            elif self.pose_readers:
                self.vio_task = asyncio.ensure_future(vio_reader_task(self.cfg.vio_pipe, self.update_vio, self._is_running,
                                                                      self.voxl_reader_errors))
            self.vio_mon_task = asyncio.ensure_future(self._vio_monitor())

        await asyncio.sleep(min(1.0, prime_sec / 3.0))
        await self.wait_reach_pipe_ready()
        t_gate = self.now()
        await self.preflight_vio_gate()          # 실패하면 PreflightAbort (arm 안 함)
        await asyncio.sleep(max(0.0, prime_sec - min(1.0, prime_sec / 3.0) - (self.now() - t_gate)))

        log("[6/7] Arming...")
        await self.drone.action.arm()
        self.event("armed")
        self.expect_armed = True
        await asyncio.sleep(1.0)

        if self.sim:
            log("       (SITL) starting offboard without RC")
        else:
            log("       *** RC must be in OFFBOARD mode ***")
        log("       Starting offboard...")
        try:
            attempts = 1 + max(0, int(getattr(self.cfg, "offboard_start_retries", 0)))
            for i in range(attempts):
                try:
                    await self.drone.offboard.start()
                    break
                except OffboardError as e:
                    if i + 1 >= attempts:
                        raise
                    result = getattr(getattr(e, "_result", None), "result", e)
                    log("       Offboard start rejected ({}), retry {}/{} in {:.1f}s (armed on ground, stream running)".format(
                        result, i + 1, attempts - 1, self.cfg.offboard_retry_sec))
                    self.event("offboard_retry", str(result))
                    await asyncio.sleep(self.cfg.offboard_retry_sec)
            log("       Offboard active!")
            self.offboard_active = True
            self.geofence.arm((self.target.north_m, self.target.east_m, self.target.down_m))
            if self.geofence.armed:
                log("       [geofence] ARMED: {}".format(self.geofence.describe()))
            else:
                log("       [geofence] DISABLED by no_geofence")
            self.event("offboard_started")
        except OffboardError as e:
            result = getattr(getattr(e, "_result", None), "result", e)
            log("       Offboard FAILED: {}".format(result))
            await self.emergency_stop()
            raise

    # ------------------------------------------------------------------
    # 도달 판정
    # ------------------------------------------------------------------

    async def _wait_until_reached(self, name, timeout):
        cfg = self.cfg
        log("       [wait] reaching {} using {} (h_tol={:.2f}m, v_tol={:.2f}m, timeout={:.1f}s)".format(
            name, self.reach_pipe, cfg.h_tol, cfg.v_tol, timeout))
        self.active_phase = "WAIT"
        start_time = self.now()
        in_tol_since = None
        last_print = -1.0
        while True:
            if self.aborted:
                return False, "ABORTED", self.now() - self.active_stage["t_start"]
            now = self.now()
            elapsed = now - start_time
            stage_elapsed = now - self.active_stage["t_start"]
            fb = self.feedback_state()
            if fb is None:
                h_dist = v_dist = float("inf")
            else:
                h_dist = horizontal_norm(fb["cmd_n"] - self.target.north_m, fb["cmd_e"] - self.target.east_m)
                v_dist = abs(fb["cmd_d"] - self.target.down_m)

            if elapsed - last_print >= 1.0:
                last_print = elapsed
                mav_rel_n, mav_rel_e, mav_rel_u = self.mav_rel_position()
                if fb is None:
                    log("       [{}] t={:.1f}s feedback not ready | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                        name, elapsed, mav_rel_n, mav_rel_e, mav_rel_u))
                else:
                    stale = " FB_STALE!" if fb["age_sec"] > cfg.feedback_stale_warn_sec else ""
                    mav_stale = " MAV_STALE!" if now - self._last_mav_update_time > cfg.mav_stale_warn_sec else ""
                    log("       [{}] t={:.1f}s fb_h={:.3f} fb_v={:.3f} fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) "
                        "age={:.2f}s{} | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f}){}".format(
                            name, elapsed, h_dist, v_dist, fb["rel_n"], fb["rel_e"], fb["rel_u"],
                            fb["age_sec"], stale, mav_rel_n, mav_rel_e, mav_rel_u, mav_stale))

            if h_dist <= cfg.h_tol and v_dist <= cfg.v_tol:
                if in_tol_since is None:
                    in_tol_since = now
                elif now - in_tol_since >= cfg.stable_time:
                    log("       [{}] REACHED by {} (stable {:.1f}s)".format(name, self.reach_pipe, cfg.stable_time))
                    return True, "REACHED", stage_elapsed
            else:
                in_tol_since = None

            if elapsed > timeout:
                log("!!! TIMEOUT {} after {:.1f}s by {}, h={:.3f}m v={:.3f}m !!!".format(
                    name, timeout, self.reach_pipe, h_dist, v_dist))
                return False, "TIMEOUT", stage_elapsed
            await asyncio.sleep(0.1)

    async def _observe_late_reach(self, name, duration):
        cfg = self.cfg
        if duration <= 0.0 or self.aborted:
            return False, "NO_LATE_OBSERVE", None
        log("       [late] keeping same target for {:.1f}s, still judged by {}...".format(duration, self.reach_pipe))
        self.active_phase = "LATE_OBSERVE"
        start_time = self.now()
        in_tol_since = None
        while True:
            if self.aborted:
                return False, "ABORTED", self.now() - self.active_stage["t_start"]
            now = self.now()
            elapsed = now - start_time
            stage_elapsed = now - self.active_stage["t_start"]
            fb = self.feedback_state()
            if fb is None:
                h_dist = v_dist = float("inf")
            else:
                h_dist = horizontal_norm(fb["cmd_n"] - self.target.north_m, fb["cmd_e"] - self.target.east_m)
                v_dist = abs(fb["cmd_d"] - self.target.down_m)
            if h_dist <= cfg.h_tol and v_dist <= cfg.v_tol:
                if in_tol_since is None:
                    in_tol_since = now
                elif now - in_tol_since >= cfg.stable_time:
                    log("       [{}] LATE_REACHED by {} after +{:.1f}s".format(name, self.reach_pipe, elapsed))
                    return True, "LATE_REACHED", stage_elapsed
            else:
                in_tol_since = None
            if elapsed > duration:
                log("       [{}] LATE_NOT_REACHED after {:.1f}s".format(name, duration))
                return False, "LATE_NOT_REACHED", stage_elapsed
            await asyncio.sleep(0.1)

    def hold_current_feedback_position(self, label):
        fb = self.feedback_state()
        if fb is None:
            self.target = PositionNedYaw(self.mav_x, self.mav_y, self.mav_z, self.current_target_yaw)
            n, e, u = self.mav_rel_position()
            log("       [hold_current {}] feedback unavailable, fallback MAVSDK rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                label, n, e, u))
        else:
            self.target = PositionNedYaw(fb["cmd_n"], fb["cmd_e"], fb["cmd_d"], self.current_target_yaw)
            log("       [hold_current {}] new target from {} rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                label, self.reach_pipe, fb["rel_n"], fb["rel_e"], fb["rel_u"]))
        n, e, u = self.rel_target()
        self.plan_n, self.plan_e, self.plan_alt = n, e, u

    # ------------------------------------------------------------------
    # 동작 원시명령
    # ------------------------------------------------------------------

    async def hover(self, duration, name, hold_current=False):
        if self.aborted:
            return False
        if hold_current:
            self.hold_current_feedback_position(name)
        log("[hover {}] for {:.1f}s ({})".format(name, duration, "hold_current_feedback" if hold_current else "hold_target"))
        self.event("hover_start", name, duration_s=duration)
        self.active_phase = "HOVER"
        start = self.now()
        last_print = -1.0
        while True:
            if self.aborted:
                return False
            now = self.now()
            elapsed = now - start
            if elapsed > duration:
                self.event("hover_end", name)
                self.active_phase = "IDLE"
                return True
            if elapsed - last_print >= 1.0:
                last_print = elapsed
                fb = self.feedback_state()
                n, e, u = self.mav_rel_position()
                if fb:
                    log("       [hover {}] t={:.1f}s fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) yaw={:+.1f} | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                        name, elapsed, fb["rel_n"], fb["rel_e"], fb["rel_u"], fb["yaw_deg"], n, e, u))
                else:
                    log("       [hover {}] t={:.1f}s feedback not ready | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                        name, elapsed, n, e, u))
            await asyncio.sleep(0.2)

    async def _ramp_setpoint_to(self, name, start_n, start_e, start_d, end_n, end_e, end_d, speed_mps):
        """setpoint 를 직선으로 옮겨 속도를 흉내낸다 (MAVSDK PositionNedYaw 에는 속도 필드가 없다)."""
        if speed_mps is None or speed_mps <= 0:
            self.target = PositionNedYaw(end_n, end_e, end_d, self.current_target_yaw)
            return 0.0
        dn, de, dd = end_n - start_n, end_e - start_e, end_d - start_d
        dist = math.sqrt(dn * dn + de * de + dd * dd)
        if dist < 1e-6:
            self.target = PositionNedYaw(end_n, end_e, end_d, self.current_target_yaw)
            return 0.0
        duration = dist / max(0.05, float(speed_mps))
        log("       [ramp {}] moving setpoint {:.2f}m at {:.2f}m/s -> {:.1f}s".format(name, dist, speed_mps, duration))
        self.active_phase = "RAMP_SETPOINT"
        start_time = self.now()
        last_print = -1.0
        period = 1.0 / max(5.0, self.cfg.setpoint_hz)
        while True:
            if self.aborted:
                return self.now() - start_time
            now = self.now()
            elapsed = now - start_time
            frac = min(1.0, elapsed / duration)
            self.target = PositionNedYaw(start_n + dn * frac, start_e + de * frac, start_d + dd * frac,
                                         self.current_target_yaw)
            if elapsed - last_print >= 1.0 or frac >= 1.0:
                last_print = elapsed
                fb = self.feedback_state()
                tn, te, tu = self.rel_target()
                if fb:
                    log("       [ramp {}] t={:.1f}/{:.1f}s target_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                        name, elapsed, duration, tn, te, tu, fb["rel_n"], fb["rel_e"], fb["rel_u"]))
                else:
                    log("       [ramp {}] t={:.1f}/{:.1f}s target_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) feedback not ready".format(
                        name, elapsed, duration, tn, te, tu))
            if frac >= 1.0:
                break
            await asyncio.sleep(period)
        self.target = PositionNedYaw(end_n, end_e, end_d, self.current_target_yaw)
        return duration

    async def go_to_relative(self, n_rel, e_rel, alt, name, label, speed_mps=None):
        """초기 위치 기준 (N, E, 고도) 로 이동. 도달하면 True."""
        if self.aborted:
            return False
        cfg = self.cfg
        fb_start = self.feedback_state()
        if fb_start is None:
            raise RuntimeError("Reach feedback not available before stage '{}'".format(name))
        mav_start_rel = self.mav_rel_position()

        end_x = self.initial_x + n_rel
        end_y = self.initial_y + e_rel
        end_z = self.initial_z - alt

        expected_dn = end_x - fb_start["cmd_n"]
        expected_de = end_y - fb_start["cmd_e"]
        expected_du = fb_start["cmd_d"] - end_z
        dist = math.sqrt(expected_dn ** 2 + expected_de ** 2 + expected_du ** 2)
        timeout = max(dist * 8.0 + 8.0, cfg.move_timeout)

        log("[goto {}] [START] [{}] target rel N{:+.2f} E{:+.2f} U{:+.2f} expected_by_{} dN{:+.2f} dE{:+.2f} dU{:+.2f} dist={:.2f}m speed={}".format(
            name, label, n_rel, e_rel, alt, self.reach_pipe, expected_dn, expected_de, expected_du, dist,
            "direct" if speed_mps is None or speed_mps <= 0 else "{:.2f}m/s".format(speed_mps)))
        self.event("stage_start", name, label=label, target_rel_n=n_rel, target_rel_e=e_rel, target_rel_u=alt,
                   speed_mps=speed_mps)

        self.active_stage = {
            "name": name, "t_start": self.now(), "expected_label": label, "fb_start": dict(fb_start),
            "mav_start_rel": mav_start_rel, "target_rel": (n_rel, e_rel, alt),
            "expected_move": (expected_dn, expected_de, expected_du), "speed_mps": speed_mps,
        }
        self.active_phase = "RAMP_SETPOINT" if speed_mps and speed_mps > 0 else "WAIT"

        if speed_mps is not None and speed_mps > 0:
            await self._ramp_setpoint_to(name, fb_start["cmd_n"], fb_start["cmd_e"], fb_start["cmd_d"],
                                         end_x, end_y, end_z, speed_mps)
        else:
            self.target = PositionNedYaw(end_x, end_y, end_z, self.current_target_yaw)

        self.active_phase = "WAIT"
        reached, wait_reason, wait_reach_time = await self._wait_until_reached(name, timeout)
        late_reached, late_reason, late_reach_time = False, "NOT_NEEDED", None
        if not reached and wait_reason == "TIMEOUT" and cfg.late_observe > 0:
            late_reached, late_reason, late_reach_time = await self._observe_late_reach(name, cfg.late_observe)

        fb_end = self.feedback_state()
        if fb_end is None:
            raise RuntimeError("Reach feedback not available after stage '{}'".format(name))
        mav_end_rel = self.mav_rel_position()

        actual_dn = fb_end["cmd_n"] - fb_start["cmd_n"]
        actual_de = fb_end["cmd_e"] - fb_start["cmd_e"]
        actual_du = fb_start["cmd_d"] - fb_end["cmd_d"]
        mav_d = tuple(mav_end_rel[i] - mav_start_rel[i] for i in range(3))

        track_status, ratio, cross = evaluate_horizontal_tracking(expected_dn, expected_de, actual_dn, actual_de)
        if reached:
            status = "OK" if track_status in ("OK", "N/A") else "REACHED_POOR_DELTA"
            reach_time = wait_reach_time
        elif late_reached:
            status = "LATE_REACHED" if track_status in ("OK", "N/A") else "LATE_REACHED_POOR_DELTA"
            reach_time = late_reach_time
        else:
            status = track_status if track_status != "OK" else "TIMEOUT"
            reach_time = None

        result = {
            "name": name, "dir": label, "target_rel": (n_rel, e_rel, alt),
            "expected_move": (expected_dn, expected_de, expected_du),
            "fb_delta": (actual_dn, actual_de, actual_du), "mav_delta": mav_d,
            "ratio": ratio, "cross": cross, "status": status, "speed_mps": speed_mps,
            "wait_reason": wait_reason, "late_reason": late_reason, "reach_time": reach_time,
            "fb_end_rel": (fb_end["rel_n"], fb_end["rel_e"], fb_end["rel_u"]),
        }
        self.results.append(result)
        log("[goto {}] [END] {} by {} - fb_delta dN={:+.3f} dE={:+.3f} dU={:+.3f} ratio={} cross={} | mav_delta dN={:+.3f} dE={:+.3f} dU={:+.3f} wait={} late={}".format(
            name, status, self.reach_pipe, actual_dn, actual_de, actual_du,
            "n/a" if ratio is None else "{:.2f}".format(ratio),
            "n/a" if cross is None else "{:.2f}m".format(cross),
            mav_d[0], mav_d[1], mav_d[2], wait_reason, late_reason))
        self.event("stage_end", name, status=status, fb_rel_n=fb_end["rel_n"], fb_rel_e=fb_end["rel_e"],
                   fb_rel_u=fb_end["rel_u"], reach_time=reach_time)

        ok = status in ("OK", "LATE_REACHED")
        if ok:
            self.plan_n, self.plan_e, self.plan_alt = n_rel, e_rel, alt
        elif name != "takeoff":
            self.hold_current_feedback_position("after_failed_{}".format(name))

        self.active_phase = "IDLE"
        self.active_stage = None
        return ok

    async def move_until(self, forward_m, right_m, name, label, stop_fn, speed_mps, max_dist_m, timeout_s=None,
                         alt=None, tick_fn=None, max_dist_grace_s=3.0):
        """도달 판정 없는 조건 정지 이동. body 방향(forward_m, right_m 은 방향만, 크기 무시)으로 setpoint 를 speed_mps 로
        계속 밀다가 stop_fn(now) 이 참(정지 사유 문자열)을 돌려주면 **그 순간의 피드백 위치**에 setpoint 를 고정한다.

        max_dist_m 만큼 밀고도(여유 max_dist_grace_s 뒤) 조건이 안 되면 MAX_DIST, timeout_s 넘으면 TIMEOUT 으로 멈춘다.
        tick_fn(now) 은 매 틱 호출(피드 폴링 등). 반환 True = 조건으로 멈춤(STOPPED). 결과는 results 에 남는다.
        """
        if self.aborted:
            return False
        fb_start = self.feedback_state()
        if fb_start is None:
            raise RuntimeError("Reach feedback not available before stage '{}'".format(name))
        mav_start_rel = self.mav_rel_position()
        dn_u, de_u = body_delta_to_ned(forward_m, right_m, self.plan_yaw)
        norm = math.sqrt(dn_u * dn_u + de_u * de_u)
        if norm < 1e-9:
            raise ValueError("move_until: 방향이 0 이다")
        dn_u, de_u = dn_u / norm, de_u / norm
        target_alt = self.plan_alt if alt is None else float(alt)
        end_d = self.initial_z - target_alt
        start_n, start_e = fb_start["cmd_n"], fb_start["cmd_e"]
        speed = max(0.05, float(speed_mps))
        max_dist = max(0.0, float(max_dist_m))
        timeout = (max_dist / speed + 10.0) if timeout_s is None else float(timeout_s)
        period = 1.0 / max(5.0, self.cfg.setpoint_hz)

        log("[until {}] [START] [{}] dir body(fwd{:+.1f}, right{:+.1f}) @yaw{:+.1f} -> NED unit (N{:+.3f},E{:+.3f}) speed {:.2f} m/s "
            "max {:.2f} m timeout {:.0f}s — stops on condition, no reach judgement".format(
                name, label, forward_m, right_m, self.plan_yaw, dn_u, de_u, speed, max_dist, timeout))
        self.event("stage_start", name, label=label, mode="move_until", speed_mps=speed, max_dist_m=max_dist, dir_n=dn_u, dir_e=de_u)
        self.active_stage = {
            "name": name, "t_start": self.now(), "expected_label": label, "fb_start": dict(fb_start),
            "mav_start_rel": mav_start_rel, "target_rel": (float("nan"), float("nan"), target_alt),
            "expected_move": (dn_u * max_dist, de_u * max_dist, 0.0), "speed_mps": speed,
        }
        self.active_phase = "MOVE_UNTIL"
        t_start = self.now()
        status, reason = "TIMEOUT", ""
        max_reached_at = None
        last_print = -1.0
        dist_cmd = 0.0
        while True:
            if self.aborted:
                status, reason = "ABORTED", str(self.abort_reason)
                break
            now = self.now()
            elapsed = now - t_start
            dist_cmd = min(max_dist, speed * elapsed)
            self.target = PositionNedYaw(start_n + dn_u * dist_cmd, start_e + de_u * dist_cmd, end_d, self.current_target_yaw)
            if tick_fn is not None:
                tick_fn(now)
            r = stop_fn(now)
            if r:
                status, reason = "STOPPED", str(r)
                break
            if dist_cmd >= max_dist:
                if max_reached_at is None:
                    max_reached_at = now
                elif now - max_reached_at > max_dist_grace_s:
                    status, reason = "MAX_DIST", "condition not met within {:.2f} m (+{:.1f}s)".format(max_dist, max_dist_grace_s)
                    break
            if elapsed > timeout:
                status, reason = "TIMEOUT", "condition not met within {:.0f}s".format(timeout)
                break
            if elapsed - last_print >= 1.0:
                last_print = elapsed
                fb = self.feedback_state()
                if fb:
                    own = (fb["cmd_n"] - start_n) * dn_u + (fb["cmd_e"] - start_e) * de_u
                    log("       [until {}] t={:.1f}s cmd {:.2f} m, own(fb) {:.2f} m along dir, fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                        name, elapsed, dist_cmd, own, fb["rel_n"], fb["rel_e"], fb["rel_u"]))
                else:
                    log("       [until {}] t={:.1f}s cmd {:.2f} m, feedback not ready".format(name, elapsed, dist_cmd))
            await asyncio.sleep(period)

        if status != "ABORTED":
            self.hold_current_feedback_position("{}_{}".format(name, status.lower()))
        fb_end = self.feedback_state() or fb_start
        mav_end_rel = self.mav_rel_position()
        actual_dn = fb_end["cmd_n"] - fb_start["cmd_n"]
        actual_de = fb_end["cmd_e"] - fb_start["cmd_e"]
        actual_du = fb_start["cmd_d"] - fb_end["cmd_d"]
        own_dist = actual_dn * dn_u + actual_de * de_u
        mav_d = tuple(mav_end_rel[i] - mav_start_rel[i] for i in range(3))
        elapsed = self.now() - t_start
        result = {
            "name": name, "dir": label, "mode": "move_until", "target_rel": (float("nan"), float("nan"), target_alt),
            "expected_move": (dn_u * dist_cmd, de_u * dist_cmd, 0.0),
            "fb_delta": (actual_dn, actual_de, actual_du), "mav_delta": mav_d,
            "ratio": None, "cross": None, "status": status, "speed_mps": speed,
            "wait_reason": status, "late_reason": "N/A", "reach_time": elapsed,
            "fb_end_rel": (fb_end["rel_n"], fb_end["rel_e"], fb_end["rel_u"]),
            "cmd_dist": dist_cmd, "own_dist": own_dist, "stop_reason": reason,
        }
        self.results.append(result)
        log("[until {}] [END] {} after {:.1f}s: setpoint pushed {:.2f} m, own(fb) moved {:.3f} m along dir | {}".format(
            name, status, elapsed, dist_cmd, own_dist, reason))
        self.event("stage_end", name, status=status, stop_reason=reason, cmd_dist=dist_cmd, own_dist=own_dist,
                   fb_rel_n=fb_end["rel_n"], fb_rel_e=fb_end["rel_e"], fb_rel_u=fb_end["rel_u"], reach_time=elapsed)
        self.active_phase = "IDLE"
        self.active_stage = None
        return status == "STOPPED"

    async def takeoff(self, alt, speed_mps=None, hover_time=None, name="takeoff"):
        speed_mps = self.cfg.takeoff_speed if speed_mps is None else speed_mps
        hover_time = self.cfg.takeoff_hover_time if hover_time is None else hover_time
        log(">>> TAKEOFF to U{:.2f}m <<<".format(alt))
        if self.cfg.vio_gate:
            # P4: 지상인데 파이프/MAVSDK 가 원점에서 벗어나 있으면 추정기가 이미 흘렀다 → 이륙하지 않는다 (사고 때 파이프 U-0.45)
            fb = self.feedback_state()
            mn, me, mu = self.mav_rel_position()
            off_mav = math.sqrt(mn * mn + me * me + mu * mu)
            off_fb = math.sqrt(fb["rel_n"] ** 2 + fb["rel_e"] ** 2 + fb["rel_u"] ** 2) if fb is not None else float("inf")
            lim = self.cfg.takeoff_fb_max_offset_m
            if off_fb > lim or off_mav > lim:
                reason = "TAKEOFF REFUSED: on-ground offset pipe {:.2f} m / MAVSDK {:.2f} m > {:.2f} (estimator drifted)".format(off_fb, off_mav, lim)
                log("!!! {} !!!".format(reason))
                await self.trigger_emergency_land(reason)
                return False
        ok = await self.go_to_relative(self.plan_n, self.plan_e, alt, name, "UP", speed_mps=speed_mps)
        if hover_time > 0:
            await self.hover(hover_time, "post_" + name)
        return ok

    async def move_body(self, forward_m, right_m, name, label, speed_mps=None, alt=None):
        """계획 위치에서 기체 프레임(현재 계획 yaw 기준) 상대 이동."""
        dn, de = body_delta_to_ned(forward_m, right_m, self.plan_yaw)
        target_alt = self.plan_alt if alt is None else alt
        log("       [move_body {}] fwd{:+.2f} right{:+.2f} @yaw{:+.1f} -> dN{:+.2f} dE{:+.2f} -> N{:+.2f} E{:+.2f} U{:+.2f}".format(
            name, forward_m, right_m, self.plan_yaw, dn, de, self.plan_n + dn, self.plan_e + de, target_alt))
        return await self.go_to_relative(self.plan_n + dn, self.plan_e + de, target_alt, name, label, speed_mps=speed_mps)

    async def move_world(self, dn, de, name, label, speed_mps=None, alt=None):
        target_alt = self.plan_alt if alt is None else alt
        return await self.go_to_relative(self.plan_n + dn, self.plan_e + de, target_alt, name, label, speed_mps=speed_mps)

    async def change_alt(self, alt, name, label, speed_mps=None):
        alt = max(self.cfg.min_command_alt, float(alt))
        return await self.go_to_relative(self.plan_n, self.plan_e, alt, name, label, speed_mps=speed_mps)

    async def _ramp_yaw_to(self, name, start_yaw, target_yaw, yaw_speed_deg_s):
        diff = angle_diff_deg(target_yaw, start_yaw)
        if yaw_speed_deg_s is None or yaw_speed_deg_s <= 0 or abs(diff) < 1e-6:
            self.current_target_yaw = yaw_to_360(target_yaw)
            self.target = PositionNedYaw(self.target.north_m, self.target.east_m, self.target.down_m, self.current_target_yaw)
            return 0.0
        duration = abs(diff) / max(1.0, float(yaw_speed_deg_s))
        log("       [yaw_ramp {}] yaw {:+.1f} -> {:+.1f} deg at {:.1f}deg/s -> {:.1f}s".format(
            name, start_yaw, yaw_to_360(target_yaw), yaw_speed_deg_s, duration))
        self.active_phase = "RAMP_YAW"
        start_time = self.now()
        last_print = -1.0
        period = 1.0 / max(5.0, self.cfg.setpoint_hz)
        while True:
            if self.aborted:
                return self.now() - start_time
            now = self.now()
            elapsed = now - start_time
            frac = min(1.0, elapsed / duration)
            yaw = yaw_to_360(start_yaw + diff * frac)
            self.current_target_yaw = yaw
            self.target = PositionNedYaw(self.target.north_m, self.target.east_m, self.target.down_m, yaw)
            if elapsed - last_print >= 1.0 or frac >= 1.0:
                last_print = elapsed
                fb_yaw = self.feedback_yaw()
                log("       [yaw_ramp {}] t={:.1f}/{:.1f}s target_yaw={:+.1f} fb_yaw={}".format(
                    name, elapsed, duration, yaw, "n/a" if math.isnan(fb_yaw) else "{:+.1f}".format(fb_yaw)))
            if frac >= 1.0:
                break
            await asyncio.sleep(period)
        self.current_target_yaw = yaw_to_360(target_yaw)
        self.target = PositionNedYaw(self.target.north_m, self.target.east_m, self.target.down_m, self.current_target_yaw)
        return duration

    async def _wait_until_yaw_reached(self, name, target_yaw, timeout):
        cfg = self.cfg
        log("       [wait_yaw] reaching {} target_yaw={:+.1f} tol={:.1f}deg timeout={:.1f}s".format(
            name, yaw_to_360(target_yaw), cfg.yaw_tol, timeout))
        start = self.now()
        in_tol_since = None
        last_print = -1.0
        valid_seen = False
        while True:
            if self.aborted:
                return False, "ABORTED", None
            now = self.now()
            elapsed = now - start
            fb_yaw = self.feedback_yaw()
            valid = not math.isnan(fb_yaw)
            err = abs(angle_diff_deg(target_yaw, fb_yaw)) if valid else float("nan")
            if elapsed - last_print >= 0.5:
                last_print = elapsed
                log("       [{} yaw] t={:.1f}s target={:+.1f} fb_yaw={} err={}".format(
                    name, elapsed, yaw_to_360(target_yaw),
                    "n/a" if not valid else "{:+.1f}".format(fb_yaw),
                    "n/a" if not valid else "{:.1f}deg".format(err)))
            if valid:
                valid_seen = True
                if err <= cfg.yaw_tol:
                    if in_tol_since is None:
                        in_tol_since = now
                    elif now - in_tol_since >= cfg.yaw_stable_time:
                        return True, "REACHED", elapsed
                else:
                    in_tol_since = None
            if (not valid_seen) and elapsed >= min(2.0, timeout):
                return True, "NO_YAW_FEEDBACK_ASSUMED", elapsed
            if elapsed > timeout:
                return False, "TIMEOUT", elapsed
            await asyncio.sleep(0.1)

    async def go_yaw(self, target_yaw, name, label, yaw_speed_deg_s=None):
        """절대 yaw(NED, deg) 로 회전. 위치 setpoint 는 유지."""
        if self.aborted:
            return False
        fb_start = self.feedback_state()
        mav_start_rel = self.mav_rel_position()
        start_yaw = self.current_target_yaw
        target_yaw = yaw_to_360(target_yaw)
        diff = angle_diff_deg(target_yaw, start_yaw)
        speed = self.cfg.default_yaw_speed if yaw_speed_deg_s is None else float(yaw_speed_deg_s)
        timeout = max(abs(diff) / max(1.0, speed) + 5.0, 6.0)
        log("[yaw {}] [START] [{}] yaw {:+.1f} -> {:+.1f} diff={:+.1f}deg speed={:.1f}deg/s".format(
            name, label, start_yaw, target_yaw, diff, speed))
        self.event("stage_start", name, label=label, yaw_start=start_yaw, yaw_target=target_yaw)

        self.active_stage = {
            "name": name, "t_start": self.now(), "expected_label": label,
            "fb_start": dict(fb_start) if fb_start else None, "mav_start_rel": mav_start_rel,
            "target_rel": self.rel_target(), "expected_move": (0.0, 0.0, 0.0), "speed_mps": None,
            "yaw_target": target_yaw,
        }
        await self._ramp_yaw_to(name, start_yaw, target_yaw, speed)
        self.active_phase = "WAIT_YAW"
        ok, reason, reach_time = await self._wait_until_yaw_reached(name, target_yaw, timeout)
        mav_end_rel = self.mav_rel_position()
        yaw_end = self.feedback_yaw()
        yaw_error = None if math.isnan(yaw_end) else abs(angle_diff_deg(target_yaw, yaw_end))
        status = "OK" if ok else "YAW_TIMEOUT"
        self.results.append({
            "name": name, "dir": label, "target_rel": self.rel_target(), "expected_move": (0.0, 0.0, 0.0),
            "fb_delta": (0.0, 0.0, 0.0),
            "mav_delta": tuple(mav_end_rel[i] - mav_start_rel[i] for i in range(3)),
            "ratio": None, "cross": None, "status": status, "speed_mps": None,
            "wait_reason": reason, "late_reason": "NOT_NEEDED", "reach_time": reach_time,
            "yaw_start": start_yaw, "yaw_target": target_yaw, "yaw_end": yaw_end, "yaw_error": yaw_error,
            "yaw_speed_deg_s": speed,
        })
        log("[yaw {}] [END] {} target_yaw={:+.1f} fb_yaw={} err={} reason={}".format(
            name, status, target_yaw, "n/a" if math.isnan(yaw_end) else "{:+.1f}".format(yaw_end),
            "n/a" if yaw_error is None else "{:.1f}deg".format(yaw_error), reason))
        self.event("stage_end", name, status=status, yaw_target=target_yaw, yaw_fb=yaw_end)
        if ok:
            self.plan_yaw = target_yaw
        self.active_phase = "IDLE"
        self.active_stage = None
        return ok

    async def turn(self, delta_deg, name, label, yaw_speed_deg_s=None):
        """계획 yaw 기준 상대 회전 (양수 = 시계방향/우회전)."""
        return await self.go_yaw(self.plan_yaw + float(delta_deg), name, label, yaw_speed_deg_s)

    # ------------------------------------------------------------------
    # 착륙 / 비상
    # ------------------------------------------------------------------

    def make_position_snapshot(self, label):
        fb = self.feedback_state()
        n, e, u = self.mav_rel_position()
        snap = {"label": label, "wallclock": wallclock(), "mono_time": self.now(),
                "mav_abs_n": self.mav_x, "mav_abs_e": self.mav_y, "mav_abs_d": self.mav_z,
                "mav_rel_n": n, "mav_rel_e": e, "mav_rel_u": u, "fb_available": fb is not None}
        if fb:
            snap.update({"fb_cmd_n": fb["cmd_n"], "fb_cmd_e": fb["cmd_e"], "fb_cmd_d": fb["cmd_d"],
                         "fb_rel_n": fb["rel_n"], "fb_rel_e": fb["rel_e"], "fb_rel_u": fb["rel_u"],
                         "fb_yaw_deg": fb["yaw_deg"], "fb_age_sec": fb["age_sec"]})
        self.snapshots[label] = snap
        if snap["fb_available"]:
            log("       [{}] fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) yaw={:+.1f} | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                label, snap["fb_rel_n"], snap["fb_rel_e"], snap["fb_rel_u"], snap["fb_yaw_deg"], n, e, u))
        else:
            log("       [{}] feedback unavailable | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(label, n, e, u))
        return snap

    async def preland(self):
        """착륙 전 저고도 하강 (v14 preland). 끝나면 land 직전 스냅샷을 남긴다."""
        cfg = self.cfg
        self.make_position_snapshot("mission_end_before_preland")
        if (not cfg.skip_preland) and cfg.preland_alt > 0.0 and cfg.preland_alt < self.plan_alt:
            log(">>> PRELAND DESCENT (U{:.2f}m) <<<".format(cfg.preland_alt))
            ok = await self.go_to_relative(self.plan_n, self.plan_e, cfg.preland_alt, "preland_descend",
                                           "PRELAND_DESCEND", speed_mps=cfg.preland_speed)
            await self.hover(cfg.preland_hover_time, "preland_stabilize")
            if self.aborted:
                return False
            if not ok:
                self.hold_current_feedback_position("preland_failed")
        elif cfg.skip_preland:
            log("       [preland] skipped by skip_preland")
        else:
            log("       [preland] skipped: preland_alt {:.2f}m is not lower than altitude {:.2f}m".format(
                cfg.preland_alt, self.plan_alt))
        self.make_position_snapshot("before_land")
        return True

    async def land_and_disarm(self):
        self.expect_armed = False          # 착륙·disarm 은 우리가 명령한 것 (외부 disarm 감시 해제)
        log("[7/7] Landing...")
        self.geofence.disarm()                  # 착륙은 setpoint 를 따르지 않으므로 먼저 끈다
        self.event("land_cmd")
        if not self.aborted:
            self.streaming = False
            await asyncio.sleep(0.2)
            try:
                await self.drone.action.land()
                log("       Land command sent.")
            except Exception as e:
                log("       Land command warning: {}".format(e))

        start = self.now()
        async for in_air in self.drone.telemetry.in_air():
            elapsed = self.now() - start
            fb = self.feedback_state()
            n, e, u = self.mav_rel_position()
            if fb:
                log("       [land] t={:.1f}s in_air={} fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                    elapsed, in_air, fb["rel_n"], fb["rel_e"], fb["rel_u"], n, e, u))
            else:
                log("       [land] t={:.1f}s in_air={} feedback not ready | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                    elapsed, in_air, n, e, u))
            if not in_air:
                log("       *** LANDED ***")
                self.event("landed")
                break
            if elapsed > self.cfg.land_wait_timeout:
                log("       [land] timeout, breaking")
                self.event("land_timeout")
                break
            await asyncio.sleep(0.5)

        self.make_position_snapshot("land_end")
        await self.cancel_background_tasks()
        log("[disarm]")
        try:
            await self.drone.action.disarm()
            log("       Disarmed.")
            self.event("disarmed")
        except Exception as e:
            log("       Disarm note: {}".format(e))

    async def preland_and_land(self):
        await self.preland()
        await self.land_and_disarm()

    async def trigger_emergency_land(self, reason="unknown"):
        async with self._emergency_lock:
            if self._emergency_triggered:
                return
            self._emergency_triggered = True
        self.expect_armed = False          # 이후 PX4 disarm 은 우리가 유도한 것
        self.geofence.disarm()
        log("!!! EMERGENCY LAND TRIGGERED: {} !!!".format(reason))
        self.event("emergency_land", reason)
        self.streaming = False
        self.aborted = True
        self.abort_reason = reason
        await asyncio.sleep(0.2)
        try:
            await self.drone.action.land()
            log("       Land command sent.")
        except Exception as e:
            log("       Emergency land failed: {}, trying kill".format(e))
            try:
                await self.drone.action.kill()
                log("       Kill command sent.")
            except Exception as ke:
                log("       Kill also failed: {}".format(ke))

    async def emergency_stop(self):
        log("!!! EMERGENCY STOP !!!")
        await self.trigger_emergency_land("emergency_stop called")
        await asyncio.sleep(0.5)
        try:
            await self.drone.action.kill()
        except Exception:
            pass
        await self.cancel_background_tasks()

    async def cancel_background_tasks(self):
        self.streaming = False
        self.monitoring = False
        tasks = [self.stream_task, self.mav_task, self.mav_stream_task, self.mav_lag_task, self.armed_task, self.status_task, self.health_task, self.sync_task, self.vio_task, self.vio_mon_task] + list(self.voxl_tasks)
        for task in tasks:
            if task and not task.done():
                task.cancel()
                try:
                    await asyncio.wait_for(task, timeout=2.0)
                except (asyncio.CancelledError, asyncio.TimeoutError):
                    pass
                except Exception:
                    pass

    # ------------------------------------------------------------------
    # 요약
    # ------------------------------------------------------------------

    def print_summary(self):
        log("")
        log("=" * 120)
        log("POST-FLIGHT SUMMARY (reach judged by {})".format(self.reach_pipe))
        log("=" * 120)
        log("Command frame initial MAVSDK NED: N={:+.3f} E={:+.3f} D={:+.3f}".format(
            self.initial_x, self.initial_y, self.initial_z))
        for label in ("mission_end_before_preland", "before_land", "land_end"):
            snap = self.snapshots.get(label)
            if snap and snap.get("fb_available"):
                log("{:<28} fb_rel=(N{:+.3f},E{:+.3f},U{:+.3f}) | mav_rel=(N{:+.3f},E{:+.3f},U{:+.3f})".format(
                    label, snap["fb_rel_n"], snap["fb_rel_e"], snap["fb_rel_u"],
                    snap["mav_rel_n"], snap["mav_rel_e"], snap["mav_rel_u"]))
            elif snap:
                log("{:<28} feedback n/a | mav_rel=(N{:+.3f},E{:+.3f},U{:+.3f})".format(
                    label, snap["mav_rel_n"], snap["mav_rel_e"], snap["mav_rel_u"]))
        a, b = self.snapshots.get("before_land"), self.snapshots.get("land_end")
        if a and b and a.get("fb_available") and b.get("fb_available"):
            dn, de = b["fb_rel_n"] - a["fb_rel_n"], b["fb_rel_e"] - a["fb_rel_e"]
            log("Landing drift by {}: dN={:+.3f} dE={:+.3f} horizontal={:.3f}m".format(
                self.reach_pipe, dn, de, horizontal_norm(dn, de)))
        log("")
        log("{:<22} {:<16} {:<20} {:<20} {:<8} {:<22}".format("Stage", "Dir", "Expected", "FB_Delta", "Speed", "Status"))
        log("-" * 120)
        for r in self.results:
            if "yaw_target" in r:
                exp = "yaw->{:+.0f}".format(r["yaw_target"])
                ye = r.get("yaw_end")
                fbd = "fb_yaw={}".format("n/a" if ye is None or math.isnan(ye) else "{:+.1f}".format(ye))
            else:
                exp = "dN{:+.2f},dE{:+.2f},dU{:+.2f}".format(*r["expected_move"])
                fbd = "dN{:+.2f},dE{:+.2f},dU{:+.2f}".format(*r["fb_delta"])
            sp = r.get("speed_mps")
            speed = "direct" if sp is None or sp <= 0 else "{:.2f}".format(sp)
            log("{:<22} {:<16} {:<20} {:<20} {:<8} {:<22}".format(r["name"], r["dir"], exp, fbd, speed, r["status"]))
        log("-" * 120)
        if self.abort_reason:
            log("ABORTED: {}".format(self.abort_reason))
        if self.voxl_reader_errors:
            for pipe, err in self.voxl_reader_errors.items():
                log("[voxl] last reader note for {}: {}".format(pipe, err))
        log("=" * 120)

    def summary_dict(self):
        return {
            "initial_ned": (self.initial_x, self.initial_y, self.initial_z),
            "initial_yaw_deg": self.initial_yaw_deg, "initial_yaw_source": self.initial_yaw_source,
            "reach_pipe": self.reach_pipe,
            "aborted": self.aborted, "abort_reason": self.abort_reason,
            "vio_gate": self.vio_gate_result, "vio_samples": self.vio.n, "vio_bad_episodes": self.vio.n_bad_episodes,
            "mav_lag_s": self.mav_lag_s, "mav_lag_known": self.mav_lag_known, "mav_updates": self.mav_update_count,
            "px4_armed_last": self.px4_armed,
            "px4_msgs": [(round(t, 2), typ, txt) for (t, typ, txt) in list(self.px4_msgs)[-40:]],
            "results": self.results, "snapshots": self.snapshots,
            "config": self.cfg.as_dict(),
        }
