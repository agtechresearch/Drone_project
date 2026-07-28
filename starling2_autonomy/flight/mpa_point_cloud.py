#!/usr/bin/env python3
"""
MPA 포인트클라우드 파이프 구독기 (voa_pc_out / tof_pc)

VOXL의 MPA(Modal Pipe Architecture) 포인트클라우드 파이프를 **순수 파이썬으로**
구독한다. libmodal_pipe.so 링크도, numpy도, 외부 패키지도 필요 없다.

왜 직접 구현하는가
  - libmodal-pipe의 python/pympa.py는 66행이고 IMU·카메라 전용이다.
    포인트클라우드 지원이 없고 자동생성 pympa_types 모듈과 .so를 요구한다.
  - voxl-inspect-points는 텍스트 요약만 출력한다. 700점 × 20Hz의 좌표 전체를
    텍스트로 받아 파싱하는 것은 비행 루프에 넣을 물건이 아니다.
  → 바이너리 FIFO를 직접 읽는 것이 맞다. 근거는 docs/07 §1-A.

와이어 포맷 (libmodal-pipe/library/include/pipe_interfaces/point_cloud_metadata_t.h)
    typedef struct point_cloud_metadata_t {
        uint32_t magic_number;   // 0x564F584C ("VOXL")
        int64_t  timestamp_ns;
        uint32_t n_points;
        uint32_t format;         // 0 = FLOAT_XYZ (12B/pt)
        uint32_t id;
        char     server_name[32];
        uint32_t reserved;
    } __attribute__((packed));   // 총 60바이트
  헤더 뒤에 n_points × (포맷별 크기) 바이트가 이어진다.

  주의: magic 0x564F584C는 카메라·ToF·IMU·pose_4dof와 **공용**이다.
        magic만으로는 자료형을 구분할 수 없고 파이프 이름에 의존해야 한다.

구독 프로토콜 (libmodal-pipe/library/src/client.c)
    1. 베이스 디렉토리 /run/mpa/
    2. 서버 생존 확인: /run/mpa/<파이프>/request 존재 여부
    3. 클라이언트 이름 = "<내이름>" + 8자리 0패딩 난수
    4. 그 이름을 request FIFO에 write (NULL 종료 문자 포함)
    5. 서버가 /run/mpa/<파이프>/<그이름> 으로 mkfifo 생성
    6. 그 FIFO를 열어 read

좌표계 (docs/04 — voa_manager.c / geometry.c 직독)
    voa_pc_out은 **level 프레임**이다.
      원점 = 기체 본체, 롤·피치만 제거(yaw는 헤딩 유지)
      X = 전방, Y = 우측, Z = 하방
    이미 기체 상대이고 헤딩 정렬돼 있으므로 포즈를 읽어 변환할 필요가 없다.
    전방 장애물 = X가 크고 |Y|가 작은 점.

단독 실행 (기체에서 진단용)
    python3 mpa_point_cloud.py voa_pc_out --seconds 5
    python3 mpa_point_cloud.py tof_pc --seconds 3 --raw

라이브러리로 쓸 때
    from mpa_point_cloud import MpaPointCloudClient
    with MpaPointCloudClient("voa_pc_out") as c:
        for frame in c.frames():
            print(frame.n_points, frame.nearest_forward())
"""

import argparse
import errno
import os
import random
import select
import struct
import sys
import time

# ---------------------------------------------------------------------------
# 와이어 포맷 상수
# ---------------------------------------------------------------------------

POINT_CLOUD_MAGIC = 0x564F584C          # "VOXL" — 다른 자료형과 공용이다
HEADER_FMT = "<IqIII32sI"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 60, "헤더가 60바이트가 아니다: {}".format(HEADER_SIZE)

MAGIC_BYTES = struct.pack("<I", POINT_CLOUD_MAGIC)

# format 값 -> (점당 바이트, 이름, XYZ를 담고 있는가)
# XYZ가 없는 포맷(XY 계열)은 z를 0.0으로 채운다.
POINT_FORMATS = {
    0: (12, "FLOAT_XYZ", True),
    1: (16, "FLOAT_XYZC", True),        # +confidence(float32)
    2: (15, "FLOAT_XYZRGB", True),      # +RGB(3바이트)
    3: (19, "FLOAT_XYZCRGB", True),
    4: (8,  "FLOAT_XY", False),
    5: (12, "FLOAT_XYC", False),
}

# 깨진 헤더로 거대 할당을 하지 않기 위한 상한.
# 실측: voa_pc_out ~700점, tof_pc 43200점(240x180). 여유를 크게 잡아도 충분하다.
MAX_POINTS = 2000000

DEFAULT_BASE_DIR = "/run/mpa"


class PipeClosed(Exception):
    """파이프가 닫혔다(서버 종료 또는 EOF)."""
    pass


class PipeTimeout(Exception):
    """정해진 시간 안에 데이터가 오지 않았다."""
    pass


# ---------------------------------------------------------------------------
# 프레임
# ---------------------------------------------------------------------------

class PointCloudFrame(object):
    """포인트클라우드 한 프레임.

    points는 [(x, y, z), ...] 리스트다. voa_pc_out 기준으로 level 프레임이며
    X 전방 / Y 우측 / Z 하방, 단위는 미터다.
    """

    __slots__ = ("timestamp_ns", "n_points", "format", "id",
                 "server_name", "points", "recv_time")

    def __init__(self, timestamp_ns, n_points, fmt, pipe_id, server_name,
                 points, recv_time):
        self.timestamp_ns = timestamp_ns
        self.n_points = n_points
        self.format = fmt
        self.id = pipe_id
        self.server_name = server_name
        self.points = points
        self.recv_time = recv_time

    @property
    def format_name(self):
        info = POINT_FORMATS.get(self.format)
        return info[1] if info else "UNKNOWN({})".format(self.format)

    def bounds(self):
        """(xmin, xmax, ymin, ymax, zmin, zmax). 점이 없으면 None."""
        if not self.points:
            return None
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        zs = [p[2] for p in self.points]
        return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

    def nearest_forward(self, half_width=1.0, half_height=1.0, min_x=0.05):
        """전방 직육면체 통로 안에서 가장 가까운 점까지의 거리(X, 미터).

        level 프레임이므로 회전 변환 없이 그대로 판정할 수 있다.
          - |Y| <= half_width   : 좌우 통로 폭
          - |Z| <= half_height  : 위아래 통로 높이 (Z는 하방이 양수)
          - X   >= min_x        : 기체 바로 앞 노이즈 제외

        해당 점이 없으면 None(= 그 통로에는 장애물 없음)을 돌려준다.

        주의: 이건 진단·프로토타이핑용 요약이다. 실제 회피 판정 로직은
        운용 환경을 정한 뒤 별도로 설계한다(docs/02 단계1-B).
        """
        best = None
        for x, y, z in self.points:
            if x < min_x:
                continue
            if abs(y) > half_width or abs(z) > half_height:
                continue
            if best is None or x < best:
                best = x
        return best

    def sector_min_x(self, n_sectors=5, fov_deg=90.0, half_height=1.0,
                     min_range=0.05):
        """전방 부채꼴을 좌→우 n등분해 각 구역의 최근접 거리를 돌려준다.

        회피 방향을 고르려면 "앞이 막혔다"만으로는 부족하고 어느 쪽이 비었는지가
        필요하다. 반환값은 길이 n_sectors 리스트이고, 비어 있는 구역은 None이다.
        인덱스 0이 가장 왼쪽(Y 음수 방향)이다.
        """
        import math
        half_fov = math.radians(fov_deg) * 0.5
        sectors = [None] * n_sectors
        step = (2.0 * half_fov) / n_sectors
        for x, y, z in self.points:
            if abs(z) > half_height:
                continue
            rng = math.hypot(x, y)
            if rng < min_range or x <= 0.0:
                continue
            ang = math.atan2(y, x)          # 우측이 양수
            if abs(ang) > half_fov:
                continue
            idx = int((ang + half_fov) / step)
            if idx < 0:
                idx = 0
            elif idx >= n_sectors:
                idx = n_sectors - 1
            if sectors[idx] is None or rng < sectors[idx]:
                sectors[idx] = rng
        return sectors

    def __repr__(self):
        return "<PointCloudFrame {} pts, {}, ts={}ns, from '{}'>".format(
            self.n_points, self.format_name, self.timestamp_ns, self.server_name)


# ---------------------------------------------------------------------------
# 스트림 파서 (I/O와 분리 — 기체 없이 테스트 가능)
# ---------------------------------------------------------------------------

class PointCloudStream(object):
    """바이트 스트림에서 포인트클라우드 프레임을 뽑아낸다.

    read_fn(n) 콜러블 하나만 있으면 동작한다. 실제 FIFO든 파일이든 테스트용
    가짜든 상관없다. read_fn은 최대 n바이트를 돌려주고, 빈 bytes면 EOF로 본다.

    FIFO 스트림은 부분 read가 흔하고, 서버 재시작이나 버퍼 오버런으로 프레임
    경계가 깨질 수 있다. 그래서 헤더 magic을 매번 검증하고 어긋나면
    재동기화한다.

    한계 (알고 쓸 것):
      프로토콜에 체크섬도 프레임 종료 구분자도 없다. 그래서 **헤더가 온전한 채
      페이로드만 잘린 프레임은 탐지할 수 없다.** 파서는 헤더의 n_points를 믿고
      그만큼 읽으므로, 그 한 프레임은 다음 프레임의 앞부분을 좌표로 오인한 채
      나온다. 그 다음 read_frame()에서 재동기화로 복구된다.

      막으려면 프레임 끝 직후에 다음 magic이 있는지 미리 확인하면 되지만,
      그러면 다음 프레임이 도착할 때까지(20Hz 기준 최대 50ms) 현재 프레임을
      내보내지 못한다. 회피 판단에서 50ms 지연이 오염된 프레임 하나보다 더
      나쁘다고 보고 하지 않았다.

      → 회피 로직은 단일 프레임을 신뢰하지 말고 연속 몇 프레임으로 판단할 것.
        오염 발생 여부는 stats["resyncs"]로 확인할 수 있다.
    """

    def __init__(self, read_fn, max_points=MAX_POINTS, chunk_size=65536):
        self._read = read_fn
        self._buf = bytearray()
        self._max_points = max_points
        self._chunk = chunk_size
        self.stats = {
            "frames": 0,
            "points": 0,
            "resyncs": 0,
            "dropped_bytes": 0,
            "bad_headers": 0,
        }

    # -- 내부 --------------------------------------------------------------

    def _fill(self, need):
        """버퍼에 최소 need 바이트가 쌓일 때까지 읽는다."""
        while len(self._buf) < need:
            want = max(self._chunk, need - len(self._buf))
            chunk = self._read(want)
            if not chunk:
                raise PipeClosed("스트림 종료 (버퍼에 {}바이트 남음)".format(len(self._buf)))
            self._buf += chunk

    def _resync(self):
        """버퍼 앞에서 다음 magic 위치까지 버린다."""
        idx = self._buf.find(MAGIC_BYTES, 1)
        if idx < 0:
            # magic이 없다. 다음 read에서 경계에 걸칠 수 있으니 끝 3바이트는 남긴다.
            keep = min(len(self._buf), len(MAGIC_BYTES) - 1)
            drop = len(self._buf) - keep
            if drop > 0:
                del self._buf[0:drop]
                self.stats["dropped_bytes"] += drop
        else:
            del self._buf[0:idx]
            self.stats["dropped_bytes"] += idx
        self.stats["resyncs"] += 1

    @staticmethod
    def _unpack_points(data, n_points, fmt):
        """페이로드에서 [(x,y,z), ...]를 뽑는다."""
        stride, _name, has_z = POINT_FORMATS[fmt]

        if fmt == 0:
            # 가장 흔한 경로(voa_pc_out). 한 번에 언팩하는 게 제일 빠르다.
            flat = struct.unpack_from("<{}f".format(n_points * 3), data, 0)
            return [(flat[i], flat[i + 1], flat[i + 2])
                    for i in range(0, n_points * 3, 3)]

        points = []
        if has_z:
            for i in range(n_points):
                x, y, z = struct.unpack_from("<3f", data, i * stride)
                points.append((x, y, z))
        else:
            # XY 계열에는 높이 정보가 없다. z=0.0으로 채우되 이 사실을 잊지 말 것.
            for i in range(n_points):
                x, y = struct.unpack_from("<2f", data, i * stride)
                points.append((x, y, 0.0))
        return points

    # -- 공개 --------------------------------------------------------------

    def read_frame(self):
        """다음 프레임 하나를 돌려준다. 스트림이 끝나면 PipeClosed."""
        while True:
            self._fill(HEADER_SIZE)

            magic = struct.unpack_from("<I", self._buf, 0)[0]
            if magic != POINT_CLOUD_MAGIC:
                self._resync()
                continue

            (_magic, ts_ns, n_points, fmt, pipe_id,
             server_raw, _reserved) = struct.unpack_from(HEADER_FMT, self._buf, 0)

            # magic이 맞아도 우연일 수 있다. 내용이 말이 되는지 본다.
            if fmt not in POINT_FORMATS or n_points > self._max_points:
                self.stats["bad_headers"] += 1
                del self._buf[0:1]      # 이 magic은 가짜다. 한 바이트 밀고 다시 찾는다
                self.stats["dropped_bytes"] += 1
                self._resync()
                continue

            stride = POINT_FORMATS[fmt][0]
            payload = n_points * stride
            self._fill(HEADER_SIZE + payload)

            data = bytes(self._buf[HEADER_SIZE:HEADER_SIZE + payload])
            del self._buf[0:HEADER_SIZE + payload]

            points = self._unpack_points(data, n_points, fmt) if n_points else []
            server_name = server_raw.split(b"\0", 1)[0].decode("utf-8", "replace")

            self.stats["frames"] += 1
            self.stats["points"] += n_points

            return PointCloudFrame(ts_ns, n_points, fmt, pipe_id,
                                   server_name, points, time.time())

    def frames(self):
        """프레임 제너레이터. 스트림이 끝나면 조용히 종료한다."""
        while True:
            try:
                yield self.read_frame()
            except PipeClosed:
                return


# ---------------------------------------------------------------------------
# 실제 MPA 파이프 구독 (기체 전용 — 리눅스 FIFO)
# ---------------------------------------------------------------------------

class MpaPointCloudClient(object):
    """/run/mpa/<파이프>를 구독한다.

    사용:
        with MpaPointCloudClient("voa_pc_out") as c:
            for frame in c.frames():
                ...
    """

    def __init__(self, pipe_name, client_name="pypc", base_dir=DEFAULT_BASE_DIR,
                 read_timeout=2.0):
        self.pipe_name = pipe_name
        self.base_dir = base_dir
        self.pipe_dir = os.path.join(base_dir, pipe_name)
        self.read_timeout = read_timeout
        # 서버는 클라이언트를 이름으로 구분한다. 같은 이름이 겹치면 안 되므로
        # client.c와 동일하게 8자리 난수를 붙인다.
        self.client_name = "{}{:08d}".format(client_name, random.randint(0, 99999999))
        self.fifo_path = os.path.join(self.pipe_dir, self.client_name)
        self._fd = None
        self._stream = None

    # -- 접속 --------------------------------------------------------------

    def _request_subscription(self):
        request_path = os.path.join(self.pipe_dir, "request")
        if not os.path.exists(request_path):
            raise PipeClosed(
                "파이프 서버가 없다: {} (서버가 죽었거나 파이프 이름이 틀렸다. "
                "'ls {}'로 확인할 것)".format(request_path, self.base_dir))

        # 논블로킹으로 연다. 읽는 쪽(서버)이 없으면 ENXIO로 즉시 실패하므로
        # 죽은 서버에 영원히 매달리지 않는다.
        try:
            fd = os.open(request_path, os.O_WRONLY | os.O_NONBLOCK)
        except OSError as e:
            if e.errno == errno.ENXIO:
                raise PipeClosed(
                    "request FIFO에 서버가 붙어있지 않다: {}".format(request_path))
            raise
        try:
            os.write(fd, self.client_name.encode("utf-8") + b"\0")
        finally:
            os.close(fd)

    def _wait_for_fifo(self, timeout):
        """서버가 우리 이름으로 mkfifo 해줄 때까지 기다린다."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if os.path.exists(self.fifo_path):
                return True
            time.sleep(0.01)
        return False

    def connect(self, timeout=5.0):
        self._request_subscription()
        if not self._wait_for_fifo(timeout):
            raise PipeTimeout(
                "서버가 {:.1f}초 안에 클라이언트 FIFO를 만들지 않았다: {}".format(
                    timeout, self.fifo_path))

        # O_NONBLOCK으로 열면 쓰는 쪽이 아직 없어도 즉시 성공한다.
        # 실제 대기는 select로 한다.
        self._fd = os.open(self.fifo_path, os.O_RDONLY | os.O_NONBLOCK)
        self._stream = PointCloudStream(self._read)
        return self

    # -- 읽기 --------------------------------------------------------------

    def _read(self, n):
        """select로 기다렸다가 읽는다. 타임아웃이면 PipeTimeout."""
        deadline = time.time() + self.read_timeout
        while True:
            remain = deadline - time.time()
            if remain <= 0:
                raise PipeTimeout(
                    "{:.1f}초 동안 '{}'에서 데이터가 오지 않았다".format(
                        self.read_timeout, self.pipe_name))
            try:
                ready, _, _ = select.select([self._fd], [], [], remain)
            except select.error:
                continue
            if not ready:
                continue
            try:
                chunk = os.read(self._fd, n)
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    continue
                raise
            if chunk:
                return chunk
            # 빈 read = 쓰는 쪽이 전부 닫혔다. 서버가 재시작 중일 수 있으니
            # 타임아웃까지는 기다려 본다.
            time.sleep(0.01)

    def read_frame(self):
        if self._stream is None:
            raise PipeClosed("connect()를 먼저 호출할 것")
        return self._stream.read_frame()

    def frames(self):
        if self._stream is None:
            raise PipeClosed("connect()를 먼저 호출할 것")
        return self._stream.frames()

    @property
    def stats(self):
        return self._stream.stats if self._stream else {}

    # -- 정리 --------------------------------------------------------------

    def close(self):
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
        # 서버가 알아서 지우지만, 서버가 먼저 죽으면 남는다. 남은 FIFO는
        # /run/mpa를 지저분하게 만들 뿐이므로 우리 것은 우리가 치운다.
        try:
            if os.path.exists(self.fifo_path):
                os.unlink(self.fifo_path)
        except OSError:
            pass
        self._stream = None

    def __enter__(self):
        if self._fd is None:
            self.connect()
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def list_pipes(base_dir=DEFAULT_BASE_DIR):
    """구독 가능한 파이프 이름 목록."""
    try:
        names = os.listdir(base_dir)
    except OSError:
        return []
    return sorted(n for n in names
                  if os.path.exists(os.path.join(base_dir, n, "request")))


def read_pipe_info(pipe_name, base_dir=DEFAULT_BASE_DIR):
    """파이프의 info 파일(JSON 메타)을 읽는다. 없으면 None."""
    path = os.path.join(base_dir, pipe_name, "info")
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except (OSError, IOError):
        return None


# ---------------------------------------------------------------------------
# 단독 실행 — 기체 진단용
# ---------------------------------------------------------------------------

def _main(argv=None):
    ap = argparse.ArgumentParser(
        description="MPA 포인트클라우드 파이프를 구독해 요약을 출력한다 "
                    "(voxl-inspect-points 대조용)")
    ap.add_argument("pipe", nargs="?", default="voa_pc_out",
                    help="파이프 이름 (기본: voa_pc_out)")
    ap.add_argument("--seconds", type=float, default=5.0,
                    help="구독 시간 (기본 5초, 0이면 무한)")
    ap.add_argument("--base-dir", default=DEFAULT_BASE_DIR)
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="데이터 대기 타임아웃 (기본 3초)")
    ap.add_argument("--raw", action="store_true",
                    help="프레임마다 첫 3개 점의 좌표를 출력")
    ap.add_argument("--sectors", type=int, default=5,
                    help="전방 부채꼴 분할 수 (기본 5)")
    ap.add_argument("--list", action="store_true",
                    help="구독 가능한 파이프 목록만 출력하고 종료")
    args = ap.parse_args(argv)

    if args.list:
        pipes = list_pipes(args.base_dir)
        if not pipes:
            print("구독 가능한 파이프가 없다: {}".format(args.base_dir))
            return 1
        print("구독 가능한 파이프 ({}개):".format(len(pipes)))
        for p in pipes:
            print("  " + p)
        return 0

    info = read_pipe_info(args.pipe, args.base_dir)
    print("파이프 : {}/{}".format(args.base_dir, args.pipe))
    if info:
        print("info   : {}".format(info.replace("\n", " ")[:200]))

    client = MpaPointCloudClient(args.pipe, base_dir=args.base_dir,
                                 read_timeout=args.timeout)
    try:
        client.connect()
    except (PipeClosed, PipeTimeout) as e:
        print("접속 실패: {}".format(e), file=sys.stderr)
        pipes = list_pipes(args.base_dir)
        if pipes:
            print("사용 가능한 파이프: {}".format(", ".join(pipes)), file=sys.stderr)
        return 1

    print("클라이언트: {}".format(client.client_name))
    print("-" * 72)

    t0 = time.time()
    n_frames = 0
    first_ts = None
    last_ts = None

    try:
        while True:
            if args.seconds > 0 and (time.time() - t0) >= args.seconds:
                break
            try:
                frame = client.read_frame()
            except PipeTimeout as e:
                print("타임아웃: {}".format(e), file=sys.stderr)
                break
            except PipeClosed as e:
                print("파이프 종료: {}".format(e), file=sys.stderr)
                break

            n_frames += 1
            if first_ts is None:
                first_ts = frame.timestamp_ns
                print("포맷   : {} ({}B/pt), server='{}'".format(
                    frame.format_name, POINT_FORMATS[frame.format][0],
                    frame.server_name))
                print("-" * 72)
            last_ts = frame.timestamp_ns

            b = frame.bounds()
            near = frame.nearest_forward()
            sectors = frame.sector_min_x(n_sectors=args.sectors)

            line = "[{:4d}] {:5d}pt".format(n_frames, frame.n_points)
            if b:
                line += "  x[{:+5.2f},{:+5.2f}] y[{:+5.2f},{:+5.2f}] z[{:+5.2f},{:+5.2f}]".format(*b)
            line += "  전방최근접 " + ("{:5.2f}m".format(near) if near is not None else "  없음")
            print(line)

            if sectors:
                cells = []
                for s in sectors:
                    cells.append("  --  " if s is None else "{:5.2f}m".format(s))
                print("       구역(좌→우): " + " ".join(cells))

            if args.raw and frame.points:
                for p in frame.points[:3]:
                    print("       pt: x={:+7.3f} y={:+7.3f} z={:+7.3f}".format(*p))

    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        elapsed = time.time() - t0
        st = dict(client.stats)
        client.close()

        print("-" * 72)
        print("경과      : {:.2f}s".format(elapsed))
        print("프레임    : {} ({:.1f} Hz)".format(
            n_frames, n_frames / elapsed if elapsed > 0 else 0.0))
        if n_frames:
            print("평균 점수 : {:.0f}".format(st.get("points", 0) / float(n_frames)))
        if first_ts is not None and last_ts is not None and n_frames > 1:
            span_s = (last_ts - first_ts) / 1e9
            if span_s > 0:
                print("타임스탬프: {:.2f}s 구간, {:.1f} Hz (헤더 기준)".format(
                    span_s, (n_frames - 1) / span_s))
        if st.get("resyncs"):
            print("재동기화  : {}회, {}바이트 폐기, 가짜 헤더 {}회".format(
                st["resyncs"], st["dropped_bytes"], st.get("bad_headers", 0)))
        else:
            print("재동기화  : 없음 (스트림 정상)")

    return 0


if __name__ == "__main__":
    sys.exit(_main())
