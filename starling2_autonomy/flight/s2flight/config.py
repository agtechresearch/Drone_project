"""FlightConfig — 비행 공통 파라미터. 기본값은 v14 와 같다.

실험 스크립트는
    cfg = FlightConfig()                 # 기본값
    FlightConfig.add_arguments(parser)   # CLI 플래그 (v14 와 같은 이름)
    cfg = FlightConfig.from_args(args)
로 쓴다. 파이썬 3.6 이라 dataclass 없이 평범한 클래스다.
"""


class FlightConfig(object):
    # 도달 판정
    h_tol = 0.02
    v_tol = 0.06
    stable_time = 0.8
    late_observe = 5.0
    move_timeout = 15.0
    yaw_tol = 5.0
    yaw_stable_time = 0.5
    default_yaw_speed = 30.0        # deg/s, go_yaw 에 속도를 안 주면

    # 이착륙
    takeoff_speed = 0.25
    takeoff_hover_time = 3.0
    preland_alt = 0.25
    preland_speed = 0.20
    preland_hover_time = 2.0
    skip_preland = False
    land_wait_timeout = 15.0
    min_command_alt = 0.15

    # 스트림 / 감시
    setpoint_hz = 20.0
    pos_ok_hold_sec = 5.0
    max_stream_fail = 5
    # offboard.start() 가 NO_SETPOINT_SET 등으로 거부되면 이만큼 재시도 (지상 arm 상태, 2026-11-10 SITL 과부하에서 1회 발생)
    offboard_start_retries = 3
    offboard_retry_sec = 0.5
    mav_stale_warn_sec = 1.5
    feedback_stale_warn_sec = 0.8

    # VIO 게이트 / 비행 중 감시 (2026-11-10 이륙 사고 대책 P1~P4, s2flight.vio). 끄려면 --no-vio-gate (권장 안 함)
    vio_gate = True
    vio_pipe = "ov"                 # voxl-inspect-vio 파이프 (open-vins 원출력). vvhub_aligned_vio 도 가능
    vio_min_quality = 50             # 게이트: 창 안 최소 품질 %
    vio_min_features = 3             # 게이트: 창 안 최소 특징점 수 (사고 때 0)
    vio_gate_window_s = 3.0          # 게이트 관찰 창 s (정지 상태)
    vio_quiet_s = 30.0               # 게이트: 최근 이 시간 안에 open-vins 저널에 BAD/리셋이 있으면 거부 (journalctl 없으면 생략)
    vio_static_drift_m = 0.03        # 게이트: 창 안 파이프·MAVSDK 위치 변동 상한 m (사고 때 3 s 에 0.45 m)
    est_disagree_v_m = 0.3           # 비행 중: 파이프 vs MAVSDK 수직 불일치 상한 m
    est_disagree_h_m = 0.5           # 비행 중: 수평 불일치 상한 m
    est_disagree_s = 0.5             # 불일치 지속 시간 → 비상 착륙
    vio_inflight_min_quality = 1     # 비행 중: 품질이 이보다 낮거나(0, -1) 상태 != OKAY 면 불량
    vio_bad_abort_s = 0.5            # 불량 지속 시간 → 비상 착륙 (사고 때 3.0 s 무효)
    vio_stale_abort_s = 1.0          # VIO 상태 샘플이 이보다 오래 안 오면 비상 착륙
    feedback_stale_abort_s = 1.0     # reach 파이프가 이보다 오래 안 오면 비상 착륙 (기존 0.8 s 는 경고만)
    takeoff_fb_max_offset_m = 0.15   # 이륙 직전 파이프/MAVSDK 상대 위치가 원점에서 이보다 멀면 이륙 거부 (사고 때 0.45 m)

    # MAVSDK 텔레메트리 지연 (2026-11-10 2차 사고: 소비 루프가 30 Hz 스트림을 18 Hz 로만 읽어 위치가 초당 0.4 s 씩 과거로 밀림)
    mav_rate_hz = 20.0               # PX4 에 요청하는 position_velocity_ned / odometry 전송률
    mav_lag_warn_s = 0.3             # odometry time_usec 기준 지연이 이보다 크면 MAVSDK 기반 판정(지오펜스·불일치) 보류 + 게이트 거부
    mav_lag_abort_s = 2.0            # 비행 중 지연이 이보다 크면 비상 착륙 (감시가 눈을 감은 상태)
    mav_lag_calib_s = 2.0            # 지연 기준선(최소 오프셋) 수집 시간

    # PX4 STATUSTEXT 감시 (2026-11-10: 추락 뒤 재부팅 없이 날아 'High Accelerometer Bias'·'Attitude failure' 사전점검 실패가 떠 있었다)
    px4_msg_quiet_s = 30.0           # 게이트: 최근 이 시간 안에 아래 패턴의 PX4 메시지가 있으면 arm 거부. 0 = 끔
    px4_msg_bad_patterns = ("Preflight Fail", "Accelerometer Bias", "estimate error", "Attitude failure",
                            "Flight termination", "Kill switch", "velocity unstable", "height estimate not stable")

    # 지오펜스 (v14)
    no_geofence = False
    max_horizontal_dev = 2.5
    max_alt_deviation = 1.5
    geofence_violation_sec = 1.0
    geofence_jump_thresh = 0.5
    geofence_jump_timeout = 30.0
    max_home_dist = 0.0
    max_alt_agl = 0.0

    # 피드백 파이프
    reach_pipe = "px4_vehicle_local_position"
    reach_pipe_timeout = 8.0
    voxl_pipes = ("px4_vehicle_local_position", "vvhub_body_wrt_local")

    # 기록
    csv_sample_sec = 1.0

    # MAVSDK 서버
    server_address = "localhost"
    port = 50051

    # SITL (drone_sim_gz): 포즈를 voxl-inspect-pose 대신 MAVSDK 텔레메트리에서 받는다 (s2flight.sim)
    sim = False

    # 정면(yaw 0 명령) 기준. "current" = arm 직전 기체가 보고 있는 방향 (2026-11-09 결정: 전원을 다른 곳에서 켜고
    # 옮겨 오면 v14 식 "전원 켠 방향" 은 실험장에서 엉뚱한 쪽이라 이륙하며 돌아버림). "zero" = VIO 전원 켠 방향(v14 동작).
    # 숫자 = NED yaw 도(deg) 고정.
    initial_yaw = "current"

    def __init__(self, **kw):
        for k, v in kw.items():
            if not hasattr(FlightConfig, k):
                raise TypeError("FlightConfig 에 없는 항목: {}".format(k))
            setattr(self, k, v)
        self.voxl_pipes = list(self.voxl_pipes)
        if self.reach_pipe not in self.voxl_pipes:
            self.voxl_pipes.insert(0, self.reach_pipe)

    def validate(self):
        if self.h_tol <= 0 or self.v_tol <= 0:
            raise ValueError("h_tol, v_tol 은 양수여야 한다")
        if self.stable_time < 0:
            raise ValueError("stable_time 은 0 이상이어야 한다")
        if self.preland_alt < 0 or self.preland_hover_time < 0 or self.takeoff_hover_time < 0:
            raise ValueError("preland_alt, preland_hover_time, takeoff_hover_time 은 0 이상이어야 한다")
        if self.setpoint_hz < 5.0:
            raise ValueError("setpoint_hz 는 5 이상이어야 한다 (PX4 offboard 는 2 Hz 미만이면 끊긴다)")
        self.initial_yaw_value()        # 형식 검사
        return self

    def initial_yaw_value(self):
        """initial_yaw 해석: ("current", None) | ("zero", 0.0) | ("fixed", deg)."""
        v = self.initial_yaw
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            return "fixed", float(v)
        s = str(v).strip().lower()
        if s == "current":
            return "current", None
        if s == "zero":
            return "zero", 0.0
        try:
            return "fixed", float(s)
        except ValueError:
            raise ValueError("initial_yaw 는 current / zero / 숫자(deg) 중 하나여야 한다: {!r}".format(v))

    def as_dict(self):
        return dict((k, getattr(self, k)) for k in dir(FlightConfig)
                    if not k.startswith("_") and not callable(getattr(FlightConfig, k)))

    # -- argparse ------------------------------------------------------------

    @staticmethod
    def add_arguments(p):
        g = p.add_argument_group("비행 공통 (s2flight.FlightConfig, 기본값 = v14)")
        g.add_argument("--h-tol", type=float, default=FlightConfig.h_tol, help="수평 도달 허용오차 m")
        g.add_argument("--v-tol", type=float, default=FlightConfig.v_tol, help="수직 도달 허용오차 m")
        g.add_argument("--stable-time", type=float, default=FlightConfig.stable_time,
                       help="허용오차 안에 이만큼 머물러야 REACHED")
        g.add_argument("--late-observe", type=float, default=FlightConfig.late_observe)
        g.add_argument("--yaw-tol", type=float, default=FlightConfig.yaw_tol)
        g.add_argument("--yaw-stable-time", type=float, default=FlightConfig.yaw_stable_time)
        g.add_argument("--takeoff-speed", type=float, default=FlightConfig.takeoff_speed)
        g.add_argument("--takeoff-hover-time", type=float, default=FlightConfig.takeoff_hover_time)
        g.add_argument("--preland-alt", type=float, default=FlightConfig.preland_alt)
        g.add_argument("--preland-speed", type=float, default=FlightConfig.preland_speed)
        g.add_argument("--preland-hover-time", type=float, default=FlightConfig.preland_hover_time)
        g.add_argument("--skip-preland", action="store_true")
        g.add_argument("--max-horizontal-dev", type=float, default=FlightConfig.max_horizontal_dev,
                       help="지오펜스: setpoint 대비 수평 추종 오차 상한 m")
        g.add_argument("--max-alt-deviation", type=float, default=FlightConfig.max_alt_deviation)
        g.add_argument("--geofence-violation-sec", type=float, default=FlightConfig.geofence_violation_sec)
        g.add_argument("--geofence-jump-thresh", type=float, default=FlightConfig.geofence_jump_thresh)
        g.add_argument("--geofence-jump-timeout", type=float, default=FlightConfig.geofence_jump_timeout)
        g.add_argument("--max-home-dist", type=float, default=FlightConfig.max_home_dist,
                       help="지오펜스: 홈 기준 수평 반경 m (0 = 끔)")
        g.add_argument("--max-alt-agl", type=float, default=FlightConfig.max_alt_agl,
                       help="지오펜스: 홈 기준 고도 상한 m (0 = 끔)")
        g.add_argument("--no-geofence", action="store_true")
        g.add_argument("--reach-pipe", default=FlightConfig.reach_pipe)
        g.add_argument("--reach-pipe-timeout", type=float, default=FlightConfig.reach_pipe_timeout)
        g.add_argument("--voxl-pipes", nargs="*", default=list(FlightConfig.voxl_pipes),
                       help="voxl-inspect-pose -n 으로 같이 기록할 포즈 파이프")
        g.add_argument("--csv-sample-sec", type=float, default=FlightConfig.csv_sample_sec)
        s = p.add_argument_group("VIO 게이트 / 비행 중 감시 (2026-11-10 사고 대책, s2flight.vio)")
        s.add_argument("--no-vio-gate", dest="vio_gate", action="store_false", default=FlightConfig.vio_gate,
                       help="VIO 게이트·감시를 끈다 (권장 안 함; 시뮬 디버그용)")
        s.add_argument("--vio-pipe", dest="vio_pipe", default=FlightConfig.vio_pipe)
        s.add_argument("--vio-min-quality", dest="vio_min_quality", type=int, default=FlightConfig.vio_min_quality)
        s.add_argument("--vio-min-features", dest="vio_min_features", type=int, default=FlightConfig.vio_min_features)
        s.add_argument("--vio-gate-window", dest="vio_gate_window_s", type=float, default=FlightConfig.vio_gate_window_s)
        s.add_argument("--vio-quiet-sec", dest="vio_quiet_s", type=float, default=FlightConfig.vio_quiet_s,
                       help="최근 이 시간 안 open-vins BAD/리셋이 있으면 arm 거부. 0 = 저널 검사 생략")
        s.add_argument("--vio-static-drift", dest="vio_static_drift_m", type=float, default=FlightConfig.vio_static_drift_m)
        s.add_argument("--est-disagree-v", dest="est_disagree_v_m", type=float, default=FlightConfig.est_disagree_v_m)
        s.add_argument("--est-disagree-h", dest="est_disagree_h_m", type=float, default=FlightConfig.est_disagree_h_m)
        s.add_argument("--est-disagree-sec", dest="est_disagree_s", type=float, default=FlightConfig.est_disagree_s)
        s.add_argument("--vio-bad-abort-sec", dest="vio_bad_abort_s", type=float, default=FlightConfig.vio_bad_abort_s)
        s.add_argument("--vio-stale-abort-sec", dest="vio_stale_abort_s", type=float, default=FlightConfig.vio_stale_abort_s)
        s.add_argument("--feedback-stale-abort-sec", dest="feedback_stale_abort_s", type=float, default=FlightConfig.feedback_stale_abort_s)
        s.add_argument("--takeoff-fb-max-offset", dest="takeoff_fb_max_offset_m", type=float, default=FlightConfig.takeoff_fb_max_offset_m)
        s.add_argument("--mav-rate-hz", dest="mav_rate_hz", type=float, default=FlightConfig.mav_rate_hz)
        s.add_argument("--mav-lag-warn-sec", dest="mav_lag_warn_s", type=float, default=FlightConfig.mav_lag_warn_s)
        s.add_argument("--mav-lag-abort-sec", dest="mav_lag_abort_s", type=float, default=FlightConfig.mav_lag_abort_s)
        s.add_argument("--px4-msg-quiet-sec", dest="px4_msg_quiet_s", type=float, default=FlightConfig.px4_msg_quiet_s,
                       help="최근 이 시간 안 PX4 'Preflight Fail' 등 메시지가 있으면 arm 거부. 0 = 끔")
        g.add_argument("--offboard-start-retries", type=int, default=FlightConfig.offboard_start_retries,
                       help="offboard 시작 거부(NO_SETPOINT_SET 등) 시 재시도 횟수. 0 = 즉시 비상정지(v14 동작)")
        g.add_argument("--offboard-retry-sec", type=float, default=FlightConfig.offboard_retry_sec)
        g.add_argument("--server-address", default=FlightConfig.server_address)
        g.add_argument("--port", type=int, default=FlightConfig.port,
                       help="mavsdk_server 포트. SITL 인스턴스 N 은 50051+N")
        g.add_argument("--sim", action="store_true",
                       help="SITL(drone_sim_gz): 포즈를 MAVSDK 텔레메트리에서 받고 voxl 도구를 쓰지 않는다. 실기 경로는 그대로")
        g.add_argument("--initial-yaw", default=FlightConfig.initial_yaw,
                       help="정면 기준: current = arm 직전 기체가 보는 방향(기본), zero = 전원 켠 방향(v14 동작), 숫자 = NED yaw deg")
        return g

    @classmethod
    def from_args(cls, args):
        kw = {}
        for k in cls().as_dict():
            if hasattr(args, k):
                kw[k] = getattr(args, k)
        return cls(**kw).validate()
