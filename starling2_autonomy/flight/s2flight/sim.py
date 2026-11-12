"""SITL 대응 입력 어댑터 — drone_sim_gz(gz Harmonic + PX4 SITL) 에서 실기 전용 입력을 대신한다.

실기에서만 있는 것                      시뮬에서 대신하는 것 (이 모듈)
    voxl-inspect-pose -n <pipe>       →  sim_pose_task: MAVSDK 텔레메트리를 같은 dict 형식으로 vehicle.update_pose() 에 넣는다
    /run/mpa/tag_detections (MPA)     →  UdpTagFeed: sim_tag_detect.py 가 보내는 UDP JSON 을 TagFrame 으로 바꾼다
    voxl-logger                       →  없음 (진입 스크립트가 --sim 이면 띄우지 않는다)

비행 로직(vehicle/align_stage)은 입력이 어디서 오는지 모른다. v13 의 `--sim` (sim_pose_reader) 와 같은 접근이고,
UDP JSON 형식은 drone_sim_gz/v3_marker/scripts/tag_detect.py 와 같다:
    {"t": <sim 초>, "dets": [{"id": int, "t_cam": [x 우, y 하, z 전], "R": 3x3, "margin": f, "hamming": i}, ...]}
t_cam/R 은 voxl-tag-detector 와 같은 카메라 광학 프레임(T_tag_wrt_cam, R_tag_to_cam) 이라 align.measure_alignment 가 그대로 쓴다.

기체 파이썬 3.6 제약(asyncio.run·dataclass 금지)을 그대로 지킨다. 시뮬 컨테이너(starling-flight)도 3.6.9 다.
"""

import asyncio
import json
import math
import socket

from . import tags
from .util import log, mono_time, wallclock

DEFAULT_TAG_PORT = 14662          # tag_detect.py 기본값. run_sim_drift.sh 는 14680 + 인스턴스 번호를 쓴다
SIM_RAW = "[sim] MAVSDK telemetry"
_I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _finite(*vals):
    for v in vals:
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            return False
        if math.isnan(v) or math.isinf(v):
            return False
    return True


# ---------------------------------------------------------------------------
# 포즈: MAVSDK 텔레메트리 → voxl-inspect-pose dict
# ---------------------------------------------------------------------------

def telemetry_to_pose(pv, att=None, av=None, now=None):
    """position_velocity_ned(+attitude_euler, +attitude_angular_velocity_body) → pose.parse_voxl_inspect_pose_line 과 같은 dict."""
    now = mono_time() if now is None else now
    nan = float("nan")
    return {
        "voxl_ts_ms": now * 1000.0,
        "pos_x": pv.position.north_m,
        "pos_y": pv.position.east_m,
        "pos_z": pv.position.down_m,
        "roll_deg": att.roll_deg if att is not None else nan,
        "pitch_deg": att.pitch_deg if att is not None else nan,
        "yaw_deg": att.yaw_deg if att is not None else nan,
        "vel_x": pv.velocity.north_m_s,
        "vel_y": pv.velocity.east_m_s,
        "vel_z": pv.velocity.down_m_s,
        "ang_x": av.roll_rad_s if av is not None else nan,
        "ang_y": av.pitch_rad_s if av is not None else nan,
        "ang_z": av.yaw_rad_s if av is not None else nan,
        "raw": SIM_RAW,
        "mono_time": now,
        "wallclock": wallclock(),
    }


async def sim_pose_task(vehicle, pipe_names, is_running, errors, period_s=0.05):
    """pose.pose_reader_task 의 SITL 대체. 요청된 모든 파이프 이름에 같은 MAVSDK 포즈를 넣는다.

    vehicle     : Vehicle (drone.telemetry 와 update_pose 를 쓴다)
    pipe_names  : cfg.voxl_pipes (reach_pipe 포함)
    is_running  : False 가 되면 종료
    errors      : dict, 구독 오류 메모
    """
    pipes = [p for p in pipe_names if p]
    if not pipes:
        return
    log("[sim] MAVSDK telemetry pose source for pipes: {}".format(", ".join(pipes)))
    telemetry = vehicle.drone.telemetry
    state = {"pv": None, "att": None, "av": None}

    async def _sub(key, stream_fn):
        try:
            async for msg in stream_fn():
                if not is_running():
                    break
                state[key] = msg
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001 — 구독 하나가 죽어도 포즈 공급은 계속
            errors["_sim_" + key] = str(e)

    subs = [asyncio.ensure_future(_sub("pv", telemetry.position_velocity_ned)),
            asyncio.ensure_future(_sub("att", telemetry.attitude_euler))]
    av_fn = getattr(telemetry, "attitude_angular_velocity_body", None)
    if av_fn is not None:
        subs.append(asyncio.ensure_future(_sub("av", av_fn)))

    try:
        while is_running():
            pv = state["pv"]
            if pv is not None:
                upd = getattr(vehicle, "update_mav_position", None)
                if upd is not None:
                    upd(pv)                 # SITL: MAVSDK 위치도 여기서 갱신 (두 번째 구독은 데이터를 못 받는다)
                parsed = telemetry_to_pose(pv, state["att"], state["av"])
                for p in pipes:
                    vehicle.update_pose(p, dict(parsed))
            await asyncio.sleep(period_s)
    except asyncio.CancelledError:
        raise
    finally:
        for t in subs:
            if not t.done():
                t.cancel()
        for t in subs:
            try:
                await t
            except Exception:  # noqa: BLE001 — CancelledError 포함
                pass


# ---------------------------------------------------------------------------
# 태그: UDP JSON → TagFrame
# ---------------------------------------------------------------------------

def parse_tag_datagram(data, size_m, recv_time=None, cam="sim_front_camera", loc_type=2):
    """UDP JSON 한 개 → TagFrame (검출 없음이면 빈 프레임, 형식이 틀리면 None)."""
    recv_time = mono_time() if recv_time is None else recv_time
    try:
        msg = json.loads(data.decode("utf-8") if isinstance(data, bytes) else data)
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(msg, dict):
        return None
    t = msg.get("t")
    dets = msg.get("dets")
    if not _finite(t) or not isinstance(dets, list):
        return None
    ts_ns = int(round(t * 1e9))
    out = []
    for d in dets:
        try:
            tag_id = d["id"]
            t_cam = tuple(float(v) for v in d["t_cam"])
            r_cam = tuple(tuple(float(v) for v in row) for row in d["R"])
        except (KeyError, TypeError, ValueError):
            continue
        if isinstance(tag_id, bool) or not isinstance(tag_id, int):
            continue
        if len(t_cam) != 3 or len(r_cam) != 3 or any(len(r) != 3 for r in r_cam):
            continue
        if not _finite(*t_cam) or not all(_finite(*r) for r in r_cam):
            continue
        out.append(tags.TagDetection(tag_id, size_m, ts_ns, "", loc_type, t_cam, r_cam,
                                     (0.0, 0.0, 0.0), _I3, cam, recv_time=recv_time))
    return tags.TagFrame(ts_ns, out, recv_time)


class UdpTagFeed(tags.TagFeed):
    """시뮬 태그 검출기(UDP JSON)를 받아 프레임으로 내보낸다. MpaTagFeed 와 같은 인터페이스."""

    def __init__(self, port=DEFAULT_TAG_PORT, host="127.0.0.1", size_m=0.20, cam="sim_front_camera",
                 keep_empty=False):
        self.port = int(port)
        self.host = host
        self.size_m = size_m
        self.cam = cam
        self.keep_empty = keep_empty       # True 면 검출 0개 프레임도 내보낸다 (MPA 는 검출이 있을 때만 레코드가 온다)
        self.sock = None
        self._loop = None
        self._task = None                  # add_reader 가 없는 루프(Windows Proactor)에서는 폴링 태스크로 받는다
        self._frames = []
        self.n_datagrams = 0
        self.n_frames = 0
        self.n_dets = 0
        self.n_empty = 0
        self.n_bad = 0
        self.first_size_m = size_m

    def start(self, loop):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.setblocking(False)
        self._loop = loop
        try:
            loop.add_reader(self.sock.fileno(), self._on_readable)
            how = "add_reader"
        except NotImplementedError:
            self._task = asyncio.ensure_future(self._poll_task(), loop=loop)
            how = "polling"
        log("[tags] UdpTagFeed listening on {}:{} (size_m={:.3f}, {})".format(self.host, self.port, self.size_m, how))

    async def _poll_task(self, period_s=0.01):
        try:
            while self.sock is not None:
                self._on_readable()
                await asyncio.sleep(period_s)
        except asyncio.CancelledError:
            pass

    def _on_readable(self):
        while self.sock is not None:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return
            self.feed_datagram(data, mono_time())

    def feed_datagram(self, data, now):
        """테스트나 다른 전송로에서 데이터그램을 직접 넣을 때도 쓴다."""
        self.n_datagrams += 1
        frame = parse_tag_datagram(data, self.size_m, recv_time=now, cam=self.cam)
        if frame is None:
            self.n_bad += 1
            return None
        if not frame.tags:
            self.n_empty += 1
            if not self.keep_empty:
                return None
        self.n_frames += 1
        self.n_dets += len(frame.tags)
        self._frames.append(frame)
        return frame

    def poll(self, now):
        out = self._frames
        self._frames = []
        return out

    def stop(self):
        if self._task is not None:
            self._task.cancel()
            self._task = None
        elif self._loop is not None and self.sock is not None:
            try:
                self._loop.remove_reader(self.sock.fileno())
            except Exception:  # noqa: BLE001
                pass
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self._loop = None

    def describe(self):
        return "UdpTagFeed(udp://{}:{})".format(self.host, self.port)

    @property
    def stats(self):
        return {"port": self.port, "datagrams": self.n_datagrams, "frames": self.n_frames, "dets": self.n_dets,
                "empty": self.n_empty, "bad": self.n_bad, "first_size_m": self.first_size_m}


def encode_tag_datagram(t_s, detections):
    """TagDetection 목록 → tag_detect.py 형식 JSON bytes (테스트용 송신, 시뮬 검출기와 같은 형식)."""
    dets = [{"id": d.id, "t_cam": list(d.t_cam), "R": [list(r) for r in d.r_cam], "margin": 60.0, "hamming": 0}
            for d in detections]
    return json.dumps({"t": t_s, "dets": dets}).encode("utf-8")


async def sim_vio_task(vehicle, is_running, period_s=0.1, status_file=None):
    """vio.vio_reader_task 의 SITL 대체: 기본 quality 100 / OKAY / features 50 을 넣는다.

    status_file (기본 환경변수 S2_SIM_VIO_FILE) 에 JSON {"quality": -1, "state": "FAILED", "features": 0} 을 써 두면
    0.5 s 안에 그 값으로 바뀐다 → 비행 중 VIO 상실 시나리오를 SITL 에서 검증할 수 있다 (run_sim_drift.sh VIO_FAIL_AT).
    """
    import json
    import os
    path = os.environ.get("S2_SIM_VIO_FILE", "") if status_file is None else status_file
    log("[sim] VIO status source: constant OK{}".format(" (override file {})".format(path) if path else ""))
    override = {}
    last_read = -1.0
    while is_running():
        now = mono_time()
        if path and now - last_read >= 0.5:
            last_read = now
            try:
                with open(path) as f:
                    txt = f.read().strip()
                override = json.loads(txt) if txt else {}
            except (OSError, ValueError):
                override = {}
        parsed = {"quality": 100, "features": 50, "state": "OKAY", "error_codes": "", "pos": None, "raw": "sim"}
        parsed.update(override)
        parsed["mono_time"] = now
        parsed["wallclock"] = wallclock()
        vehicle.update_vio(parsed)
        await asyncio.sleep(period_s)
