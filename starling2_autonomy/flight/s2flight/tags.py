"""tag_detections 파이프 — 레코드 파서, 프레임 묶기, asyncio 피드 (flight/mpa_tag_detections.py 사본).

voxl-tag-detector 가 내보내는 `tag_detections` 파이프를 순수 파이썬으로 읽는다.
기체 파이썬(3.6)에는 cv2 도 apriltag 도 없으므로(docs/10 §4) 온보드에서 태그를 쓰려면 이 길뿐이다.

와이어 포맷 (libmodal-pipe tag_detection_t.h, packed 252 B)
    uint32 magic(0x564F584C, 다른 자료형과 공용) | int32 id | float size_m | int64 timestamp_ns(CLOCK_MONOTONIC)
    | char name[64] | int32 loc_type | float T_tag_wrt_cam[3] | float R_tag_to_cam[3][3]
    | float T_tag_wrt_fixed[3] | float R_tag_to_fixed[3][3] | char cam[64] | int32 reserved
태그 하나당 레코드 하나. 한 프레임의 태그들은 timestamp_ns 가 같다 → FrameAssembler 로 묶는다.
magic 이 공용이고 체크섬이 없어서 레코드마다 내용 타당성(크기·거리·회전행렬 열 길이)을 본다.

카메라 프레임은 OpenCV 관례 (x 우, y 하, z 전방). size_m 이 tag_locations.conf 와 다르면
T_tag_wrt_cam 거리가 그 배율로 틀린다(0.4 m 설정 + 7 cm 태그 = 5.7 배).
"""

import collections
import math
import struct
import time

from . import mpa
from .util import mono_time

TAG_DETECTION_MAGIC = 0x564F584C
TAG_NAME_MAX_LEN = 64
RECORD_FMT = "<Iifq64si3f9f3f9f64si"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
assert RECORD_SIZE == 252, "tag_detection_t 가 252 바이트가 아니다: {}".format(RECORD_SIZE)
MAGIC_BYTES = struct.pack("<I", TAG_DETECTION_MAGIC)

LOC_TYPES = {0: "unknown", 1: "fixed", 2: "static", 3: "dynamic"}
DEFAULT_PIPE = "tag_detections"

MAX_TAG_SIZE_M = 5.0
MAX_RANGE_M = 50.0
ROT_NORM_TOL = 0.05

PipeClosed = mpa.PipeClosed
PipeTimeout = mpa.PipeTimeout


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
        self.t_cam = tuple(t_cam)
        self.r_cam = tuple(tuple(r) for r in r_cam)
        self.t_fixed = tuple(t_fixed)
        self.r_fixed = tuple(tuple(r) for r in r_fixed)
        self.cam = cam
        self.recv_time = mono_time() if recv_time is None else recv_time

    @property
    def loc_type_name(self):
        return LOC_TYPES.get(self.loc_type, "?{}".format(self.loc_type))

    @property
    def distance_m(self):
        x, y, z = self.t_cam
        return math.sqrt(x * x + y * y + z * z)

    @property
    def normal_cam(self):
        r = self.r_cam
        return (r[0][2], r[1][2], r[2][2])

    def is_sane(self):
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
    f = struct.unpack_from(RECORD_FMT, buf, offset)
    t_cam = f[6:9]
    r_cam = (f[9:12], f[12:15], f[15:18])
    t_fixed = f[18:21]
    r_fixed = (f[21:24], f[24:27], f[27:30])
    return TagDetection(f[1], f[2], f[3], _cstr(f[4]), f[5], t_cam, r_cam, t_fixed, r_fixed,
                        _cstr(f[30]), recv_time)


def pack_record(det):
    """TagDetection → 252 바이트 (테스트·재생용)."""
    flat = [TAG_DETECTION_MAGIC, int(det.id), float(det.size_m), int(det.timestamp_ns),
            det.name.encode("utf-8")[:TAG_NAME_MAX_LEN], int(det.loc_type)]
    flat += [float(v) for v in det.t_cam]
    flat += [float(v) for row in det.r_cam for v in row]
    flat += [float(v) for v in det.t_fixed]
    flat += [float(v) for row in det.r_fixed for v in row]
    flat += [det.cam.encode("utf-8")[:TAG_NAME_MAX_LEN], 0]
    return struct.pack(RECORD_FMT, *flat)


class TagDetectionStream(object):
    """바이트 스트림 → tag_detection_t 레코드. feed(bytes) 푸시 또는 read_fn 풀."""

    def __init__(self, read_fn=None, chunk_size=16384):
        self._read = read_fn
        self._buf = bytearray()
        self._chunk = chunk_size
        self._pending = collections.deque()
        self.stats = {"records": 0, "resyncs": 0, "dropped_bytes": 0, "bad_records": 0}

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
        out = []
        while len(self._buf) >= RECORD_SIZE:
            if self._buf[0:4] != MAGIC_BYTES:
                self._resync()
                continue
            det = unpack_record(self._buf, 0, recv_time)
            if not det.is_sane():
                self.stats["bad_records"] += 1
                del self._buf[0:1]
                self.stats["dropped_bytes"] += 1
                self._resync()
                continue
            del self._buf[0:RECORD_SIZE]
            self.stats["records"] += 1
            out.append(det)
        return out

    def feed(self, data, recv_time=None):
        if data:
            self._buf += data
        return self._parse_buffer(recv_time)

    def read_detection(self):
        if self._read is None:
            raise PipeClosed("read_fn 없이 만든 스트림이다. feed() 를 쓸 것")
        while not self._pending:
            chunk = self._read(self._chunk)
            if not chunk:
                raise PipeClosed("스트림 종료 (버퍼에 {}바이트 남음)".format(len(self._buf)))
            self._pending.extend(self.feed(chunk, mono_time()))
        return self._pending.popleft()

    def detections(self):
        while True:
            try:
                yield self.read_detection()
            except PipeClosed:
                return


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

    프레임 끝 표시가 없으므로 다음 프레임의 첫 레코드가 와야 앞 프레임이 닫힌다.
    실시간 루프에서는 flush_stale() 로 일정 시간 새 레코드가 없으면 들고 있던 프레임을 내보낸다.
    """

    def __init__(self):
        self._ts = None
        self._tags = []
        self._recv = None

    def add(self, det):
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
        if self._ts is not None and self._recv is not None and now - self._recv >= max_age_s:
            return self.flush()
        return None

    @property
    def pending(self):
        return len(self._tags)


# ---------------------------------------------------------------------------
# 실제 파이프 클라이언트 + asyncio 피드
# ---------------------------------------------------------------------------

def make_client(pipe_name=DEFAULT_PIPE, base_dir=mpa.DEFAULT_BASE_DIR, read_timeout=2.0):
    """MpaClient(tag_detections). client.stream 이 TagDetectionStream."""
    return mpa.MpaClient(pipe_name, lambda read_fn: TagDetectionStream(read_fn),
                         client_name="pytag", base_dir=base_dir, read_timeout=read_timeout)


class TagFeed(object):
    """정렬 단계가 쓰는 프레임 공급원 인터페이스.

    poll(now) -> 지금까지 닫힌 TagFrame 목록 (논블로킹). start()/stop() 은 자원 관리.
    테스트에서는 이 인터페이스를 흉내낸 가짜 카메라를 넣는다.
    """

    def start(self, loop):
        pass

    def poll(self, now):
        return []

    def stop(self):
        pass

    def describe(self):
        return self.__class__.__name__


class MpaTagFeed(TagFeed):
    """tag_detections 파이프를 asyncio add_reader 로 읽어 프레임으로 내보낸다."""

    def __init__(self, pipe_name=DEFAULT_PIPE, base_dir=mpa.DEFAULT_BASE_DIR, stale_s=0.08):
        self.client = make_client(pipe_name, base_dir)
        self.stale_s = stale_s
        self._asm = FrameAssembler()
        self._frames = collections.deque()
        self._loop = None
        self.n_records = 0
        self.first_size_m = None
        self.cams = collections.Counter()

    def start(self, loop, connect_timeout=5.0):
        self.client.connect(timeout=connect_timeout)
        self._loop = loop
        loop.add_reader(self.client.fileno(), self._on_readable)

    def _on_readable(self):
        try:
            chunk = self.client.read_nonblocking()
        except OSError:
            return
        if not chunk:
            return
        dets = self.client.stream.feed(chunk, mono_time())
        for d in dets:
            self.n_records += 1
            self.cams[d.cam] += 1
            if self.first_size_m is None:
                self.first_size_m = d.size_m
        self._frames.extend(self._asm.add_many(dets))

    def poll(self, now):
        f = self._asm.flush_stale(now, self.stale_s)
        if f is not None:
            self._frames.append(f)
        out = list(self._frames)
        self._frames.clear()
        return out

    def stop(self):
        if self._loop is not None and self.client.connected:
            try:
                self._loop.remove_reader(self.client.fileno())
            except Exception:
                pass
        self.client.close()
        self._loop = None

    def describe(self):
        return "MpaTagFeed({}/{})".format(self.client.base_dir, self.client.pipe_name)

    @property
    def stats(self):
        return dict(self.client.stats, records=self.n_records, first_size_m=self.first_size_m,
                    cams=dict(self.cams))
