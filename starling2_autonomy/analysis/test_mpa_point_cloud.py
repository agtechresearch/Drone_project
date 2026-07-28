#!/usr/bin/env python3
"""mpa_point_cloud 파서 단위 테스트.

기체 없이 실행한다. FIFO 대신 read 콜러블을 주입해 파싱 로직만 검증한다.
FIFO 스트림은 부분 read가 흔하고 서버 재시작·오버런으로 프레임 경계가 깨질 수
있으므로, 정상 경로보다 그런 상황을 더 많이 본다.

    python analysis/test_mpa_point_cloud.py
"""
import math
import pathlib
import struct
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "flight"))

import mpa_point_cloud as mpc  # noqa: E402

results = []


def ok(name, cond):
    results.append((name, bool(cond)))
    print(("  PASS  " if cond else "  FAIL  ") + name)


# ---------------------------------------------------------------------------
# 프레임 생성 헬퍼 - 서버가 내보내는 바이트를 그대로 흉내낸다
# ---------------------------------------------------------------------------

def make_frame(points, fmt=0, ts_ns=123456789, pipe_id=7, server=b"voa"):
    """[(x,y,z), ...] -> 바이트 프레임."""
    n = len(points)
    hdr = struct.pack(mpc.HEADER_FMT, mpc.POINT_CLOUD_MAGIC, ts_ns, n, fmt,
                      pipe_id, server, 0)
    body = b""
    stride = mpc.POINT_FORMATS[fmt][0]
    for p in points:
        if fmt == 0:
            body += struct.pack("<3f", *p)
        elif fmt == 1:                      # XYZC
            body += struct.pack("<4f", p[0], p[1], p[2], 0.9)
        elif fmt == 2:                      # XYZRGB (12 + 3바이트)
            body += struct.pack("<3f", *p) + b"\x10\x20\x30"
        elif fmt == 4:                      # XY (z 없음)
            body += struct.pack("<2f", p[0], p[1])
        else:
            body += b"\x00" * stride
    return hdr + body


def reader_from(data, chunk=None):
    """bytes를 흘려보내는 read_fn. chunk를 주면 그 크기로 잘라서 준다."""
    state = {"pos": 0}

    def _read(n):
        want = n if chunk is None else min(n, chunk)
        pos = state["pos"]
        out = data[pos:pos + want]
        state["pos"] = pos + len(out)
        return out
    return _read


PTS = [(1.0, 0.0, 0.0), (2.5, -0.5, 0.25), (3.0, 1.0, -0.5)]


# ---------------------------------------------------------------------------
print("\n=== 1. 와이어 포맷 상수 ===")

ok("헤더 60바이트 (packed)", mpc.HEADER_SIZE == 60)
ok("magic 0x564F584C == 'VOXL'",
   struct.pack("<I", mpc.POINT_CLOUD_MAGIC) == b"LXOV")   # 리틀엔디안이라 역순
ok("FLOAT_XYZ = 12B/pt", mpc.POINT_FORMATS[0][0] == 12)
ok("포맷 6종 정의됨", len(mpc.POINT_FORMATS) == 6)


# ---------------------------------------------------------------------------
print("\n=== 2. 정상 프레임 파싱 ===")

s = mpc.PointCloudStream(reader_from(make_frame(PTS)))
f = s.read_frame()
ok("점 개수", f.n_points == 3)
ok("포맷 이름", f.format_name == "FLOAT_XYZ")
ok("타임스탬프", f.timestamp_ns == 123456789)
ok("server_name NULL 종료 처리", f.server_name == "voa")
ok("좌표 정확", f.points[1] == (2.5, -0.5, 0.25))
ok("id 필드", f.id == 7)

s = mpc.PointCloudStream(reader_from(make_frame([])))
f = s.read_frame()
ok("빈 프레임(0점) 처리", f.n_points == 0 and f.points == [])


# ---------------------------------------------------------------------------
print("\n=== 3. 연속 프레임 ===")

blob = b"".join(make_frame(PTS, ts_ns=1000 + i) for i in range(5))
s = mpc.PointCloudStream(reader_from(blob))
got = list(s.frames())
ok("5프레임 모두 읽음", len(got) == 5)
ok("타임스탬프 순서 유지", [g.timestamp_ns for g in got] == [1000 + i for i in range(5)])
ok("통계 frames=5", s.stats["frames"] == 5)
ok("통계 points=15", s.stats["points"] == 15)
ok("재동기화 0회 (정상 스트림)", s.stats["resyncs"] == 0)


# ---------------------------------------------------------------------------
print("\n=== 4. 부분 read (FIFO 실전 조건) ===")

# 실제 FIFO는 요청한 만큼 주지 않는다. 1바이트씩 줘도 조립돼야 한다.
s = mpc.PointCloudStream(reader_from(blob, chunk=1))
got = list(s.frames())
ok("1바이트씩 공급해도 5프레임 복원", len(got) == 5)
ok("좌표 무손상", got[0].points == PTS)

s = mpc.PointCloudStream(reader_from(blob, chunk=7))   # 헤더 경계와 어긋나는 크기
ok("7바이트씩 공급해도 5프레임", len(list(s.frames())) == 5)

s = mpc.PointCloudStream(reader_from(blob, chunk=59))  # 헤더보다 1 작게
ok("59바이트씩(헤더-1) 공급해도 5프레임", len(list(s.frames())) == 5)


# ---------------------------------------------------------------------------
print("\n=== 5. 재동기화 - 스트림 앞에 쓰레기 ===")

junk = b"\xde\xad\xbe\xef" * 10
s = mpc.PointCloudStream(reader_from(junk + make_frame(PTS)))
f = s.read_frame()
ok("앞쪽 쓰레기 40바이트 건너뛰고 파싱", f.n_points == 3)
ok("재동기화 기록됨", s.stats["resyncs"] >= 1)
ok("폐기 바이트 40", s.stats["dropped_bytes"] == 40)


# ---------------------------------------------------------------------------
print("\n=== 6. 재동기화 - 프레임 사이 오염 (서버 오버런 상황) ===")

corrupt = make_frame(PTS, ts_ns=1) + b"\x00\x01\x02" * 9 + make_frame(PTS, ts_ns=2)
s = mpc.PointCloudStream(reader_from(corrupt))
got = list(s.frames())
ok("오염 앞뒤 프레임 2개 모두 복구", len(got) == 2)
ok("두 번째 프레임 타임스탬프 정확", got[1].timestamp_ns == 2)

# 페이로드가 잘린 프레임. 헤더가 온전하면 파서는 이게 잘렸는지 알 방법이 없다
# (프로토콜에 체크섬도 프레임 구분자도 없다). 그래서 그 프레임 하나는 오염된
# 채로 나오고, 중요한 건 그 뒤로 스트림이 복구되느냐다.
truncated = (make_frame(PTS, ts_ns=1)[:40]
             + b"".join(make_frame(PTS, ts_ns=100 + i) for i in range(3)))
s = mpc.PointCloudStream(reader_from(truncated))
got = list(s.frames())
ok("잘린 프레임 뒤로 스트림 복구됨",
   len(got) >= 2 and any(g.timestamp_ns >= 100 for g in got))
ok("복구된 프레임의 좌표는 정상",
   all(g.points == PTS for g in got if g.timestamp_ns >= 100))
ok("재동기화 기록됨 (오염 감지는 됐다)", s.stats["resyncs"] >= 1)


# ---------------------------------------------------------------------------
print("\n=== 7. 가짜 헤더 방어 ===")

# magic은 맞는데 format이 말이 안 되는 경우 (우연히 데이터에 VOXL이 나온 상황)
fake = struct.pack(mpc.HEADER_FMT, mpc.POINT_CLOUD_MAGIC, 0, 3, 99, 0, b"x", 0)
s = mpc.PointCloudStream(reader_from(fake + make_frame(PTS, ts_ns=555)))
f = s.read_frame()
ok("잘못된 format(99) 거부 후 진짜 프레임 파싱", f.timestamp_ns == 555)
ok("bad_headers 기록됨", s.stats["bad_headers"] >= 1)

# n_points가 터무니없이 큰 경우 - 이걸 믿으면 수 GB를 할당하려 든다
huge = struct.pack(mpc.HEADER_FMT, mpc.POINT_CLOUD_MAGIC, 0, 10 ** 9, 0, 0, b"x", 0)
s = mpc.PointCloudStream(reader_from(huge + make_frame(PTS, ts_ns=777)))
f = s.read_frame()
ok("과대 n_points(10억) 거부 후 복구", f.timestamp_ns == 777)

s = mpc.PointCloudStream(reader_from(make_frame(PTS)), max_points=2)
try:
    s.read_frame()
    ok("max_points 초과 시 해당 프레임 거부", False)
except mpc.PipeClosed:
    ok("max_points 초과 시 해당 프레임 거부", True)


# ---------------------------------------------------------------------------
print("\n=== 8. 다른 포맷 (voa 외 파이프 대비) ===")

s = mpc.PointCloudStream(reader_from(make_frame(PTS, fmt=1)))
f = s.read_frame()
ok("FLOAT_XYZC(16B) stride 처리", f.n_points == 3 and f.points[1] == (2.5, -0.5, 0.25))
ok("포맷 이름 XYZC", f.format_name == "FLOAT_XYZC")

s = mpc.PointCloudStream(reader_from(make_frame(PTS, fmt=2)))
f = s.read_frame()
ok("FLOAT_XYZRGB(15B) stride 처리", f.points[2] == (3.0, 1.0, -0.5))

s = mpc.PointCloudStream(reader_from(make_frame(PTS, fmt=4)))
f = s.read_frame()
ok("FLOAT_XY(8B)는 z=0으로 채움",
   f.points[1][0] == 2.5 and f.points[1][1] == -0.5 and f.points[1][2] == 0.0)


# ---------------------------------------------------------------------------
print("\n=== 9. 스트림 종료 ===")

s = mpc.PointCloudStream(reader_from(b""))
try:
    s.read_frame()
    ok("빈 스트림 -> PipeClosed", False)
except mpc.PipeClosed:
    ok("빈 스트림 -> PipeClosed", True)

s = mpc.PointCloudStream(reader_from(make_frame(PTS)[:30]))
try:
    s.read_frame()
    ok("헤더 도중 EOF -> PipeClosed", False)
except mpc.PipeClosed:
    ok("헤더 도중 EOF -> PipeClosed", True)

s = mpc.PointCloudStream(reader_from(make_frame(PTS)[:64]))
try:
    s.read_frame()
    ok("페이로드 도중 EOF -> PipeClosed", False)
except mpc.PipeClosed:
    ok("페이로드 도중 EOF -> PipeClosed", True)

s = mpc.PointCloudStream(reader_from(make_frame(PTS) + b"\x01\x02"))
ok("꼬리에 잔여 바이트 있어도 frames()는 정상 종료", len(list(s.frames())) == 1)


# ---------------------------------------------------------------------------
print("\n=== 10. 장애물 요약 (level 프레임: X전방 Y우측 Z하방) ===")

f = mpc.PointCloudFrame(0, 3, 0, 0, "t", PTS, 0.0)
ok("bounds", f.bounds() == (1.0, 3.0, -0.5, 1.0, -0.5, 0.25))

# 통로 반폭 1.0m, 반높이 1.0m -> 세 점 모두 통로 안. 최근접은 x=1.0
ok("nearest_forward = 1.0", abs(f.nearest_forward() - 1.0) < 1e-6)

# 통로를 좁히면 y=1.0인 점이 빠지지만 x=1.0 점은 남는다
ok("반폭 0.4로 좁혀도 최근접 1.0",
   abs(f.nearest_forward(half_width=0.4) - 1.0) < 1e-6)

side_only = mpc.PointCloudFrame(0, 2, 0, 0, "t",
                                [(2.0, 5.0, 0.0), (2.0, -5.0, 0.0)], 0.0)
ok("통로 밖 점만 있으면 None", side_only.nearest_forward() is None)

behind = mpc.PointCloudFrame(0, 1, 0, 0, "t", [(-3.0, 0.0, 0.0)], 0.0)
ok("후방 점은 무시", behind.nearest_forward() is None)

close_noise = mpc.PointCloudFrame(0, 1, 0, 0, "t", [(0.01, 0.0, 0.0)], 0.0)
ok("min_x 미만 근접 노이즈 제외", close_noise.nearest_forward() is None)

tall = mpc.PointCloudFrame(0, 1, 0, 0, "t", [(2.0, 0.0, 3.0)], 0.0)
ok("통로 높이 밖(z=3.0) 제외", tall.nearest_forward() is None)


# ---------------------------------------------------------------------------
print("\n=== 11. 구역 분할 (회피 방향 선택용) ===")

# 좌(-y) 가까움, 정면 멀리, 우(+y) 중간
sect_pts = [
    (2.0, -1.5, 0.0),   # 좌
    (5.0,  0.0, 0.0),   # 정면
    (3.0,  1.5, 0.0),   # 우
]
f = mpc.PointCloudFrame(0, 3, 0, 0, "t", sect_pts, 0.0)
sec = f.sector_min_x(n_sectors=3, fov_deg=120.0)
ok("구역 3개 반환", len(sec) == 3)
ok("좌측 구역이 가장 가까움 (2.5m)", sec[0] is not None and abs(sec[0] - math.hypot(2.0, 1.5)) < 1e-6)
ok("정면 구역 5.0m", sec[1] is not None and abs(sec[1] - 5.0) < 1e-6)
ok("우측 구역 존재", sec[2] is not None)
ok("좌측이 정면보다 가까움", sec[0] < sec[1])

empty = mpc.PointCloudFrame(0, 0, 0, 0, "t", [], 0.0)
ok("점 없으면 전 구역 None", all(v is None for v in empty.sector_min_x()))
ok("점 없으면 bounds None", empty.bounds() is None)

# FOV 밖의 점은 어느 구역에도 안 들어가야 한다
wide = mpc.PointCloudFrame(0, 1, 0, 0, "t", [(0.1, 10.0, 0.0)], 0.0)
ok("FOV 밖 점 제외", all(v is None for v in wide.sector_min_x(fov_deg=60.0)))


# ---------------------------------------------------------------------------
print("\n=== 12. 실전 규모 (voa_pc_out 실측: ~700점 20Hz) ===")

import random as _rnd


def f32(v):
    """와이어에 실리면 float32가 된다. 기대값도 같은 정밀도로 맞춰야 한다."""
    return struct.unpack("<f", struct.pack("<f", v))[0]


_rnd.seed(42)
big = [(_rnd.uniform(0.2, 6.0), _rnd.uniform(-3, 3), _rnd.uniform(-1, 1))
       for _ in range(700)]
big32 = [(f32(x), f32(y), f32(z)) for x, y, z in big]

data = b"".join(make_frame(big, ts_ns=i * 50_000_000) for i in range(20))  # 1초분
s = mpc.PointCloudStream(reader_from(data, chunk=4096))
got = list(s.frames())
ok("700점 x 20프레임 파싱", len(got) == 20 and all(g.n_points == 700 for g in got))
ok("총 14000점", s.stats["points"] == 14000)
ok("마지막 프레임 좌표 무손상 (float32 왕복)", got[-1].points[699] == big32[699])
ok("전체 좌표 무손상", got[0].points == big32)

near = got[0].nearest_forward()
expect = min((p[0] for p in big32
              if p[0] >= 0.05 and abs(p[1]) <= 1.0 and abs(p[2]) <= 1.0),
             default=None)
ok("nearest_forward가 전수 계산과 일치",
   (near is None and expect is None) or abs(near - expect) < 1e-9)


# ---------------------------------------------------------------------------
print("\n" + "=" * 55)
passed = sum(1 for _n, c in results if c)
total = len(results)
print("결과: {} / {} 통과".format(passed, total))
if passed != total:
    print("\n실패 항목:")
    for n, c in results:
        if not c:
            print("  - " + n)
sys.exit(0 if passed == total else 1)
