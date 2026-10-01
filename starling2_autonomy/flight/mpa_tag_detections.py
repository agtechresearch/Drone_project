#!/usr/bin/env python3
"""
MPA 태그 검출 파이프 구독기 (tag_detections)

voxl-tag-detector 가 내보내는 `tag_detections` 파이프를 **순수 파이썬으로** 구독한다.
기체 파이썬(3.6)에는 cv2 도 apriltag 도 없으므로(docs/10 §4) 온보드에서 태그를
쓰려면 이 파이프를 읽는 길뿐이다. 접속·대기·정리 절차는 `mpa_point_cloud.py` 의
클라이언트를 그대로 상속하고, 바이트 → 레코드 파서만 바꾼다.

와이어 포맷 (libmodal-pipe/library/include/pipe_interfaces/tag_detection_t.h)
    typedef struct tag_detection_t {
        uint32_t magic_number;        // 0x564F584C ("VOXL") — 다른 자료형과 공용
        int32_t  id;                  // 태그 ID (tag36h11 fiducial id)
        float    size_m;              // tag_locations.conf 의 크기 (잘못되면 거리도 그 배율로 틀린다)
        int64_t  timestamp_ns;        // 프레임 시각, CLOCK_MONOTONIC
        char     name[64];
        int32_t  loc_type;            // 0 unknown / 1 fixed / 2 static / 3 dynamic
        float    T_tag_wrt_cam[3];    // 태그 중심 위치, 카메라 프레임 (x 우, y 하, z 전방, m)
        float    R_tag_to_cam[3][3];  // 태그 축 → 카메라 축 회전 (열 = 태그 축의 카메라 프레임 표현)
        float    T_tag_wrt_fixed[3];  // loc_type fixed 일 때만 의미 있음
        float    R_tag_to_fixed[3][3];
        char     cam[64];             // 검출한 카메라 이름 (tracking_front 등)
        int32_t  reserved;
    } __attribute__((packed));        // 총 252 바이트

  검출기는 **태그 하나당 레코드 하나**를 쓴다. 한 프레임에 태그가 둘 보이면
  timestamp_ns 가 같은 레코드가 둘 연달아 온다. 프레임 단위로 쓰려면
  `FrameAssembler` 로 묶는다.

  magic 이 공용이고 체크섬도 없으므로 레코드 경계가 깨지면 magic 만으로는
  모자란다. 그래서 레코드마다 내용의 타당성(크기·거리·회전행렬 정규성)을 본다.

좌표계
    카메라 프레임은 OpenCV 관례다: x 우측, y 아래, z 광축(전방).
    R_tag_to_cam 의 세 번째 열이 태그 평면의 법선(태그 z 축)이다. z 가 태그 안쪽을
    향하는지 바깥쪽인지는 검출기 버전에 따라 다를 수 있어 `tag_align.py` 는
    부호를 접어서(mod 180°) 쓴다. 정렬 yaw 는 법선이 아니라 **태그 두 개의 연결선**
    으로 구한다(단일 태그 PnP 모호성, docs/09 §3.2).

단독 실행 (기체 진단용)
    python3 mpa_tag_detections.py --seconds 10            # 레코드 요약
    python3 mpa_tag_detections.py --seconds 10 --align    # 프레임별 정렬 yaw 오차까지
    python3 mpa_tag_detections.py --list

라이브러리
    from mpa_tag_detections import MpaTagDetectionClient
    with MpaTagDetectionClient() as c:
        for frame in c.frames():           # 같은 timestamp 의 태그 묶음
            print(frame.ids, frame.tags[0].distance_m)

asyncio 미션 루프(v15)에서는 블로킹 read 대신
    loop.add_reader(c.fileno(), on_readable)   # on_readable 안에서 c.read_available()
를 쓴다. read_available() 은 지금 FIFO 에 있는 바이트만 읽어 완성된 레코드를 돌려준다.
"""

import argparse
import collections
import errno
import math
import os
import struct
import sys
import time

import mpa_point_cloud as mpc

# ---------------------------------------------------------------------------
# 와이어 포맷
# ---------------------------------------------------------------------------

TAG_DETECTION_MAGIC = 0x564F584C
TAG_NAME_MAX_LEN = 64
RECORD_FMT = "<Iifq64si3f9f3f9f64si"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
assert RECORD_SIZE == 252, "tag_detection_t 가 252 바이트가 아니다: {}".format(RECORD_SIZE)

MAGIC_BYTES = struct.pack("<I", TAG_DETECTION_MAGIC)

LOC_TYPES = {0: "unknown", 1: "fixed", 2: "static", 3: "dynamic"}

DEFAULT_PIPE = "tag_detections"

# 타당성 한계. 실험장은 통로 폭 1 m, 이격 0.5 m 이고 tracking_front 로 7 cm 태그를
# 3 m 너머에서 잡을 일은 없다. 그래도 '파싱 오류'와 '먼 태그'를 섞지 않도록 넉넉히 둔다.
MAX_TAG_SIZE_M = 5.0
MAX_RANGE_M = 50.0
ROT_NORM_TOL = 0.05

PipeClosed = mpc.PipeClosed
PipeTimeout = mpc.PipeTimeout


def _cstr(raw):
    return raw.split(b"\0", 1)[0].decode("utf-8", "replace")


def _finite(*vals):
    for v in vals:
        if math.isnan(v) or math.isinf(v):
            return False
    return True


class TagDetection(object):
    """태그 하나의 검출 결과 (tag_detection_t 한 레코드)."""

    __slots__ = ("id", "size_m", "timestamp_ns", "name", "loc_type",
                 "t_cam", "r_cam", "t_fixed", "r_fixed", "cam", "recv_time")

    def __init__(self, tag_id, size_m, timestamp_ns, name, loc_type,
                 t_cam, r_cam, t_fixed, r_fixed, cam, recv_time=None):
        self.id = tag_id
        self.size_m = size_m
        self.timestamp_ns = timestamp_ns
        self.name = name
        self.loc_type = loc_type
        self.t_cam = tuple(t_cam)                       # (x, y, z) m
        self.r_cam = tuple(tuple(r) for r in r_cam)     # 3x3, 행 우선
        self.t_fixed = tuple(t_fixed)
        self.r_fixed = tuple(tuple(r) for r in r_fixed)
        self.cam = cam
        self.recv_time = time.time() if recv_time is None else recv_time

    @property
    def loc_type_name(self):
        return LOC_TYPES.get(self.loc_type, "?{}".format(self.loc_type))

    @property
    def distance_m(self):
        x, y, z = self.t_cam
        return math.sqrt(x * x + y * y + z * z)

    @property
    def normal_cam(self):
        """태그 z 축(평면 법선)의 카메라 프레임 표현 = R_tag_to_cam 세 번째 열."""
        r = self.r_cam
        return (r[0][2], r[1][2], r[2][2])

    def is_sane(self):
        """파싱 오류로 나온 쓰레기 레코드인지 가린다."""
        if not _finite(self.size_m, *self.t_cam):
            return False
        if not (0.0 < self.size_m <= MAX_TAG_SIZE_M):
            return False
        if self.loc_type not in LOC_TYPES:
            return False
        if self.distance_m > MAX_RANGE_M:
            return False
        r = self.r_cam
        for c in range(3):
            col = (r[0][c], r[1][c], r[2][c])
            if not _finite(*col):
                return False
            n = math.sqrt(col[0] ** 2 + col[1] ** 2 + col[2] ** 2)
            if abs(n - 1.0) > ROT_NORM_TOL:
                return False
        return True

    def __repr__(self):
        x, y, z = self.t_cam
        return "<Tag id={} cam={} ts={} xyz=({:+.3f},{:+.3f},{:+.3f}) d={:.3f}m>".format(
            self.id, self.cam, self.timestamp_ns, x, y, z, self.distance_m)


def unpack_record(buf, offset=0, recv_time=None):
    """버퍼의 offset 위치에서 레코드 하나를 TagDetection 으로 푼다 (magic 검사는 호출자 몫)."""
    f = struct.unpack_from(RECORD_FMT, buf, offset)
    # f: magic, id, size, ts, name, loc, t_cam[3], r_cam[9], t_fixed[3], r_fixed[9], cam, reserved
    t_cam = f[6:9]
    r_cam = (f[9:12], f[12:15], f[15:18])
    t_fixed = f[18:21]
    r_fixed = (f[21:24], f[24:27], f[27:30])
    return TagDetection(f[1], f[2], f[3], _cstr(f[4]), f[5],
                        t_cam, r_cam, t_fixed, r_fixed, _cstr(f[30]), recv_time)


def pack_record(det):
    """TagDetection → 252 바이트. 검출기가 내보내는 바이트를 흉내낼 때(테스트·재생) 쓴다."""
    flat = [TAG_DETECTION_MAGIC, int(det.id), float(det.size_m), int(det.timestamp_ns),
            det.name.encode("utf-8")[:TAG_NAME_MAX_LEN], int(det.loc_type)]
    flat += [float(v) for v in det.t_cam]
    flat += [float(v) for row in det.r_cam for v in row]
    flat += [float(v) for v in det.t_fixed]
    flat += [float(v) for row in det.r_fixed for v in row]
    flat += [det.cam.encode("utf-8")[:TAG_NAME_MAX_LEN], 0]
    return struct.pack(RECORD_FMT, *flat)


# ---------------------------------------------------------------------------
# 스트림 파서 (I/O 와 분리 — 기체 없이 테스트 가능)
# ---------------------------------------------------------------------------

class TagDetectionStream(object):
    """바이트 스트림에서 tag_detection_t 레코드를 뽑는다.

    두 가지로 쓸 수 있다.
      - 풀(pull): read_fn 을 주고 read_detection() / detections() 로 하나씩 받는다.
      - 푸시(push): feed(bytes) 에 읽은 바이트를 넣으면 완성된 레코드 목록을 돌려준다.
        asyncio add_reader 콜백에서는 이쪽이 맞다.

    레코드는 고정 길이라 포인트클라우드보다 단순하지만 magic 이 공용이라서
    (a) magic 검색으로 재동기화하고 (b) 내용 타당성으로 가짜 magic 을 걸러낸다.
    타당성에서 떨어진 레코드는 stats["bad_records"] 로 세고 한 바이트 밀어 다시 찾는다.
    """

    def __init__(self, read_fn=None, chunk_size=16384):
        self._read = read_fn
        self._buf = bytearray()
        self._chunk = chunk_size
        self._pending = collections.deque()
        self.stats = {
            "records": 0,
            "resyncs": 0,
            "dropped_bytes": 0,
            "bad_records": 0,
        }

    # -- 내부 --------------------------------------------------------------

    def _resync(self):
        idx = self._buf.find(MAGIC_BYTES, 1)
        if idx < 0:
            keep = min(len(self._buf), len(MAGIC_BYTES) - 1)
            drop = len(self._buf) - keep
            if drop > 0:
                del self._buf[0:drop]
                self.stats["dropped_bytes"] += drop
        else:
            del self._buf[0:idx]
            self.stats["dropped_bytes"] += idx
        self.stats["resyncs"] += 1

    def _parse_buffer(self, recv_time=None):
        """버퍼에서 완성된 레코드를 전부 꺼낸다."""
        out = []
        while len(self._buf) >= RECORD_SIZE:
            if self._buf[0:4] != MAGIC_BYTES:
                self._resync()
                continue
            det = unpack_record(self._buf, 0, recv_time)
            if not det.is_sane():
                # magic 이 우연히 맞은 자리다. 한 바이트 밀고 다음 magic 을 찾는다.
                self.stats["bad_records"] += 1
                del self._buf[0:1]
                self.stats["dropped_bytes"] += 1
                self._resync()
                continue
            del self._buf[0:RECORD_SIZE]
            self.stats["records"] += 1
            out.append(det)
        return out

    # -- 공개 --------------------------------------------------------------

    def feed(self, data, recv_time=None):
        """바이트를 넣고 완성된 레코드 목록을 받는다 (푸시 방식)."""
        if data:
            self._buf += data
        return self._parse_buffer(recv_time)

    def read_detection(self):
        """다음 레코드 하나 (풀 방식). 스트림이 끝나면 PipeClosed."""
        if self._read is None:
            raise PipeClosed("read_fn 없이 만든 스트림이다. feed() 를 쓸 것")
        while not self._pending:
            chunk = self._read(self._chunk)
            if not chunk:
                raise PipeClosed("스트림 종료 (버퍼에 {}바이트 남음)".format(len(self._buf)))
            self._pending.extend(self.feed(chunk))
        return self._pending.popleft()

    def detections(self):
        while True:
            try:
                yield self.read_detection()
            except PipeClosed:
                return


# ---------------------------------------------------------------------------
# 프레임 묶기
# ---------------------------------------------------------------------------

class TagFrame(object):
    """같은 timestamp_ns 를 가진 검출 묶음 = 카메라 프레임 하나."""

    __slots__ = ("timestamp_ns", "tags", "recv_time")

    def __init__(self, timestamp_ns, tags, recv_time):
        self.timestamp_ns = timestamp_ns
        self.tags = list(tags)
        self.recv_time = recv_time

    @property
    def ids(self):
        return sorted(t.id for t in self.tags)

    def by_id(self):
        return dict((t.id, t) for t in self.tags)

    def __len__(self):
        return len(self.tags)

    def __repr__(self):
        return "<TagFrame ts={} ids={}>".format(self.timestamp_ns, self.ids)


class FrameAssembler(object):
    """레코드를 timestamp_ns 기준으로 프레임에 묶는다.

    프레임 끝 표시가 없으므로 **다음 프레임의 첫 레코드가 와야** 앞 프레임이 닫힌다.
    즉 프레임 하나만큼(검출기 설정에 따라 33~200 ms) 늦다. 실시간 루프에서는
    flush_stale() 로 일정 시간 새 레코드가 없으면 들고 있던 프레임을 내보낸다.
    """

    def __init__(self):
        self._ts = None
        self._tags = []
        self._recv = None

    def add(self, det):
        """레코드 하나 추가. 닫힌 프레임이 있으면 그것을 돌려주고 없으면 None."""
        closed = None
        if self._ts is not None and det.timestamp_ns != self._ts:
            closed = TagFrame(self._ts, self._tags, self._recv)
            self._tags = []
        self._ts = det.timestamp_ns
        self._recv = det.recv_time
        self._tags.append(det)
        return closed

    def add_many(self, dets):
        out = []
        for d in dets:
            f = self.add(d)
            if f is not None:
                out.append(f)
        return out

    def flush(self):
        if self._ts is None:
            return None
        f = TagFrame(self._ts, self._tags, self._recv)
        self._ts = None
        self._tags = []
        self._recv = None
        return f

    def flush_stale(self, now, max_age_s):
        """마지막 레코드 후 max_age_s 가 지났으면 들고 있던 프레임을 닫는다."""
        if self._ts is not None and self._recv is not None and now - self._recv >= max_age_s:
            return self.flush()
        return None

    @property
    def pending(self):
        return len(self._tags)


# ---------------------------------------------------------------------------
# 실제 MPA 파이프 구독 (기체 전용)
# ---------------------------------------------------------------------------

class MpaTagDetectionClient(mpc.MpaPointCloudClient):
    """/run/mpa/tag_detections 구독. 접속 절차는 포인트클라우드 클라이언트와 같다."""

    def __init__(self, pipe_name=DEFAULT_PIPE, client_name="pytag",
                 base_dir=mpc.DEFAULT_BASE_DIR, read_timeout=2.0):
        mpc.MpaPointCloudClient.__init__(self, pipe_name, client_name, base_dir, read_timeout)

    def _make_stream(self):
        return TagDetectionStream(self._read)

    # 포인트클라우드 이름의 메서드는 쓰지 않도록 막는다.
    def read_frame(self):
        raise NotImplementedError("태그 파이프는 read_detection() / frames() 를 쓸 것")

    def read_detection(self):
        if self._stream is None:
            raise PipeClosed("connect()를 먼저 호출할 것")
        return self._stream.read_detection()

    def detections(self):
        if self._stream is None:
            raise PipeClosed("connect()를 먼저 호출할 것")
        return self._stream.detections()

    def frames(self):
        """프레임(같은 timestamp 묶음) 제너레이터. 한 프레임 늦게 나온다(FrameAssembler 참조)."""
        asm = FrameAssembler()
        for det in self.detections():
            f = asm.add(det)
            if f is not None:
                yield f
        f = asm.flush()
        if f is not None:
            yield f

    def fileno(self):
        if self._fd is None:
            raise PipeClosed("connect()를 먼저 호출할 것")
        return self._fd

    def read_available(self, max_bytes=65536):
        """논블로킹. 지금 FIFO 에 있는 바이트만 읽어 완성된 레코드 목록을 돌려준다.

        asyncio: loop.add_reader(c.fileno(), cb) 의 cb 에서 호출한다.
        읽을 것이 없거나 쓰는 쪽이 닫혔으면 빈 목록. 서버 종료 판정은 호출자가
        '한동안 빈 목록' 으로 한다.
        """
        if self._fd is None or self._stream is None:
            raise PipeClosed("connect()를 먼저 호출할 것")
        try:
            chunk = os.read(self._fd, max_bytes)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return []
            raise
        if not chunk:
            return []
        return self._stream.feed(chunk, time.time())


# ---------------------------------------------------------------------------
# 단독 실행 — 기체 진단용
# ---------------------------------------------------------------------------

def _main(argv=None):
    ap = argparse.ArgumentParser(
        description="tag_detections 파이프를 구독해 태그 검출을 출력한다 (voxl-inspect-tags 대조용)")
    ap.add_argument("pipe", nargs="?", default=DEFAULT_PIPE)
    ap.add_argument("--seconds", type=float, default=10.0, help="구독 시간 (0 이면 무한)")
    ap.add_argument("--base-dir", default=mpc.DEFAULT_BASE_DIR)
    ap.add_argument("--timeout", type=float, default=3.0, help="데이터 대기 타임아웃")
    ap.add_argument("--raw", action="store_true", help="레코드마다 회전행렬·법선까지 출력")
    ap.add_argument("--align", action="store_true",
                    help="프레임마다 tag_align 으로 yaw 오차를 계산해 출력")
    ap.add_argument("--list", action="store_true", help="구독 가능한 파이프 목록만 출력")
    args = ap.parse_args(argv)

    if args.list:
        pipes = mpc.list_pipes(args.base_dir)
        print("구독 가능한 파이프 ({}개): {}".format(len(pipes), ", ".join(pipes) or "없음"))
        return 0 if pipes else 1

    measure = None
    if args.align:
        try:
            import tag_align
            measure = tag_align.measure_alignment
        except ImportError as e:
            print("tag_align 을 불러올 수 없다: {}".format(e), file=sys.stderr)
            return 1

    info = mpc.read_pipe_info(args.pipe, args.base_dir)
    print("파이프 : {}/{}".format(args.base_dir, args.pipe))
    if info:
        print("info   : {}".format(info.replace("\n", " ")[:200]))

    client = MpaTagDetectionClient(args.pipe, base_dir=args.base_dir, read_timeout=args.timeout)
    try:
        client.connect()
    except (PipeClosed, PipeTimeout) as e:
        print("접속 실패: {}".format(e), file=sys.stderr)
        print("voxl-tag-detector 가 떠 있는지 확인: systemctl status voxl-tag-detector", file=sys.stderr)
        return 1
    print("클라이언트: {}".format(client.client_name))
    print("-" * 72)

    t0 = time.time()
    counters = {"frames": 0, "tags": 0}
    seen_ids = collections.Counter()
    ts_span = {"first": None, "last": None}
    asm = FrameAssembler()

    def show_frame(frame):
        counters["frames"] += 1
        line = "[{:4d}] ts={:.3f}s ids={}".format(
            counters["frames"], frame.timestamp_ns / 1e9, frame.ids)
        for t in sorted(frame.tags, key=lambda d: d.id):
            x, y, z = t.t_cam
            line += "\n       id {:2d}: x={:+.3f} y={:+.3f} z={:+.3f} d={:.3f}m size={:.3f} cam={}".format(
                t.id, x, y, z, t.distance_m, t.size_m, t.cam)
            if args.raw:
                n = t.normal_cam
                line += "\n              법선=({:+.3f},{:+.3f},{:+.3f}) R={}".format(
                    n[0], n[1], n[2], [[round(v, 3) for v in r] for r in t.r_cam])
        if measure is not None:
            m = measure(frame.tags)
            if m.valid:
                line += ("\n       정렬: yaw오차 {:+.2f}° (보정 {:+.2f}°) 법선기준 {:+.2f}° "
                         "중심x {:+.3f} 거리 {:.3f} 선폭 {:.1f}mm").format(
                    m.yaw_error_deg, m.yaw_correction_deg, m.normal_yaw_deg,
                    m.center_x_m, m.range_m, m.line_rms_m * 1000.0)
            else:
                line += "\n       정렬: 불가 ({})".format(m.reason)
        print(line)

    try:
        while True:
            if args.seconds > 0 and (time.time() - t0) >= args.seconds:
                break
            try:
                det = client.read_detection()
            except PipeTimeout as e:
                print("타임아웃: {} (태그가 안 보이거나 검출기가 멈춤)".format(e), file=sys.stderr)
                continue
            except PipeClosed as e:
                print("파이프 종료: {}".format(e), file=sys.stderr)
                break
            counters["tags"] += 1
            seen_ids[det.id] += 1
            if ts_span["first"] is None:
                ts_span["first"] = det.timestamp_ns
            ts_span["last"] = det.timestamp_ns
            frame = asm.add(det)
            if frame is not None:
                show_frame(frame)
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        frame = asm.flush()
        if frame is not None:
            show_frame(frame)
        elapsed = time.time() - t0
        st = dict(client.stats)
        client.close()
        n_frames = counters["frames"]
        print("-" * 72)
        print("경과      : {:.2f}s".format(elapsed))
        print("프레임    : {} ({:.1f} Hz)   레코드 {}".format(
            n_frames, n_frames / elapsed if elapsed > 0 else 0.0, counters["tags"]))
        if seen_ids:
            print("ID 별 횟수: {}".format(dict(sorted(seen_ids.items()))))
        if ts_span["first"] is not None and n_frames > 1:
            span_s = (ts_span["last"] - ts_span["first"]) / 1e9
            if span_s > 0:
                print("타임스탬프: {:.2f}s 구간, {:.1f} Hz (헤더 기준)".format(
                    span_s, (n_frames - 1) / span_s))
        if st.get("resyncs") or st.get("bad_records"):
            print("재동기화  : {}회, {}바이트 폐기, 가짜 레코드 {}회".format(
                st.get("resyncs", 0), st.get("dropped_bytes", 0), st.get("bad_records", 0)))
        else:
            print("재동기화  : 없음 (스트림 정상)")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
