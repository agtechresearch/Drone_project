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
    mav_stale_warn_sec = 1.5
    feedback_stale_warn_sec = 0.8

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
