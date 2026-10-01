#!/usr/bin/env python3
"""mpa_tag_detections 파서 단위 테스트.

기체 없이 실행한다. FIFO 대신 read 콜러블/feed() 로 바이트를 넣어 파싱만 검증한다.
포인트클라우드 테스트와 같은 이유로, 정상 경로보다 부분 read·경계 깨짐·가짜 magic 을 더 본다.

    python analysis/test_mpa_tag_detections.py
"""
import math
import pathlib
import struct
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "flight"))

import mpa_point_cloud as mpc        # noqa: E402
import mpa_tag_detections as mtd     # noqa: E402

if hasattr(sys.stdout, "reconfigure"):          # Windows 콘솔(cp949)에서 기호가 깨지지 않게
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

results = []


def ok(name, cond):
    results.append((name, bool(cond)))
    print(("  PASS  " if cond else "  FAIL  ") + name)


def _raises(fn, exc):
    try:
        fn()
    except exc:
        return True
    except Exception:
        return False
    return False


IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def make_det(tag_id, ts_ns, t_cam=(0.1, -0.2, 0.5), r_cam=IDENTITY, size=0.07,
             cam="tracking_front", name="", loc_type=0):
    return mtd.TagDetection(tag_id, size, ts_ns, name, loc_type, t_cam, r_cam,
                            (0.0, 0.0, 0.0), IDENTITY, cam, recv_time=0.0)


def reader_from(data, chunk=None):
    state = {"pos": 0}

    def _read(n):
        want = n if chunk is None else min(n, chunk)
        pos = state["pos"]
        out = data[pos:pos + want]
        state["pos"] = pos + len(out)
        return out
    return _read


def f32(v):
    return struct.unpack("<f", struct.pack("<f", v))[0]


# ---------------------------------------------------------------------------
print("[1] 레코드 포맷")
ok("tag_detection_t 는 252 바이트", mtd.RECORD_SIZE == 252)
ok("magic 은 포인트클라우드와 같은 VOXL", mtd.TAG_DETECTION_MAGIC == mpc.POINT_CLOUD_MAGIC)

d = make_det(3, 123456789012, t_cam=(0.123, -0.045, 0.512), name="banner_3", loc_type=2)
raw = mtd.pack_record(d)
ok("pack_record 길이 252", len(raw) == 252)
back = mtd.unpack_record(raw)
ok("id / ts / loc_type 왕복", back.id == 3 and back.timestamp_ns == 123456789012 and back.loc_type == 2)
ok("size float32 왕복", back.size_m == f32(0.07))
ok("t_cam float32 왕복", back.t_cam == (f32(0.123), f32(-0.045), f32(0.512)))
ok("r_cam 왕복", back.r_cam == IDENTITY)
ok("name / cam 문자열(NUL 종료) 왕복", back.name == "banner_3" and back.cam == "tracking_front")
ok("loc_type_name", back.loc_type_name == "static")
ok("distance_m", abs(back.distance_m - math.sqrt(0.123 ** 2 + 0.045 ** 2 + 0.512 ** 2)) < 1e-6)
ok("normal_cam = R 세 번째 열", back.normal_cam == (0.0, 0.0, 1.0))
ok("정상 레코드 is_sane", back.is_sane())

# ---------------------------------------------------------------------------
print("[2] 타당성 검사")
bad_nan = make_det(0, 1, t_cam=(float("nan"), 0.0, 0.5))
ok("NaN 위치는 거부", not bad_nan.is_sane())
bad_size = make_det(0, 1, size=0.0)
ok("크기 0 은 거부", not bad_size.is_sane())
bad_far = make_det(0, 1, t_cam=(0.0, 0.0, 99.0))
ok("99 m 는 거부", not bad_far.is_sane())
bad_rot = make_det(0, 1, r_cam=((2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
ok("회전행렬 열 길이 ≠ 1 은 거부", not bad_rot.is_sane())
bad_loc = make_det(0, 1, loc_type=7)
ok("loc_type 7 은 거부", not bad_loc.is_sane())
ok("size 0.4(기체 기본값)도 통과 — 설정 오류는 파서가 아니라 운용자가 잡는다",
   make_det(0, 1, size=0.4).is_sane())

# ---------------------------------------------------------------------------
print("[3] 스트림 — 정상")
dets = [make_det(i % 2, 1000 + (i // 2) * 33, t_cam=(0.25 * (i % 2), 0.0, 0.5)) for i in range(6)]
data = b"".join(mtd.pack_record(x) for x in dets)

s = mtd.TagDetectionStream(reader_from(data))
got = list(s.detections())
ok("6 레코드 한 번에", len(got) == 6 and [g.id for g in got] == [0, 1, 0, 1, 0, 1])
ok("timestamp 보존", [g.timestamp_ns for g in got] == [1000, 1000, 1033, 1033, 1066, 1066])
ok("stats.records = 6, 재동기화 없음", s.stats["records"] == 6 and s.stats["resyncs"] == 0)

s = mtd.TagDetectionStream(reader_from(data, chunk=7))
got = list(s.detections())
ok("7 바이트씩 조각내도 6 레코드", len(got) == 6 and got[-1].timestamp_ns == 1066)

s = mtd.TagDetectionStream(reader_from(data, chunk=1))
ok("1 바이트씩도 6 레코드", len(list(s.detections())) == 6)

s = mtd.TagDetectionStream()
pushed = []
for i in range(0, len(data), 100):
    pushed += s.feed(data[i:i + 100], recv_time=5.0)
ok("feed() 푸시 방식 6 레코드, recv_time 전달", len(pushed) == 6 and pushed[0].recv_time == 5.0)
ok("read_fn 없이 read_detection 은 PipeClosed", _raises(s.read_detection, mpc.PipeClosed))

# ---------------------------------------------------------------------------
print("[4] 스트림 — 경계 깨짐")
garbage = b"\x01\x02\x03\x04\x05xyz" * 5
s = mtd.TagDetectionStream(reader_from(garbage + data))
got = list(s.detections())
ok("앞 쓰레기 뒤 6 레코드 복구", len(got) == 6)
ok("재동기화 횟수 ≥ 1, 폐기 바이트 = 쓰레기 길이",
   s.stats["resyncs"] >= 1 and s.stats["dropped_bytes"] == len(garbage))

torn = data[:100] + data[252:]           # 첫 레코드가 100 바이트에서 잘림
s = mtd.TagDetectionStream(reader_from(torn, chunk=50))
got = list(s.detections())
ok("잘린 첫 레코드는 버리고 나머지 5개", len(got) == 5 and got[0].id == 1)

# magic 은 맞지만 내용이 엉터리인 가짜 레코드 (NaN 위치) 를 끼워 넣는다
fake = bytearray(mtd.pack_record(make_det(9, 1, t_cam=(0.0, 0.0, 0.5))))
struct.pack_into("<f", fake, 20 + 64 + 4, float("nan"))       # T_tag_wrt_cam[0] 자리
s = mtd.TagDetectionStream(reader_from(data[:252] + bytes(fake) + data[252:], chunk=64))
got = list(s.detections())
ok("가짜 레코드(NaN)는 bad_records 로 걸러지고 6개 정상", len(got) == 6 and s.stats["bad_records"] == 1)
ok("가짜 뒤 레코드 내용 무손상", got[1].id == 1 and got[1].timestamp_ns == 1000)

# 쓰레기 안에 magic 바이트가 우연히 들어 있는 경우
tricky = b"\x00" * 10 + mtd.MAGIC_BYTES + b"\xff" * 30
s = mtd.TagDetectionStream(reader_from(tricky + data))
got = list(s.detections())
ok("쓰레기 속 가짜 magic 도 넘기고 6개", len(got) == 6 and s.stats["bad_records"] >= 1)

s = mtd.TagDetectionStream(reader_from(b""))
ok("빈 스트림은 조용히 종료", list(s.detections()) == [])

# ---------------------------------------------------------------------------
print("[5] 프레임 묶기")
asm = mtd.FrameAssembler()
frames = asm.add_many(dets)
ok("6 레코드 → 닫힌 프레임 2개 (마지막은 대기)", len(frames) == 2 and asm.pending == 2)
ok("프레임 ids / timestamp", frames[0].ids == [0, 1] and frames[0].timestamp_ns == 1000
   and frames[1].timestamp_ns == 1033)
last = asm.flush()
ok("flush 로 마지막 프레임", last is not None and last.timestamp_ns == 1066 and asm.pending == 0)
ok("flush 두 번째는 None", asm.flush() is None)
ok("by_id / len", frames[0].by_id()[1].t_cam[0] == f32(0.25) and len(frames[0]) == 2)

asm = mtd.FrameAssembler()
one = make_det(0, 5000)
one.recv_time = 10.0
asm.add(one)
ok("flush_stale: 아직 안 지남", asm.flush_stale(10.1, 0.2) is None)
st = asm.flush_stale(10.3, 0.2)
ok("flush_stale: 지나면 프레임", st is not None and st.timestamp_ns == 5000)

# 한 프레임에 태그 셋
three = [make_det(i, 7000) for i in (0, 1, 2)] + [make_det(0, 7033)]
asm = mtd.FrameAssembler()
fr = asm.add_many(three)
ok("태그 3개 프레임", len(fr) == 1 and fr[0].ids == [0, 1, 2])

# ---------------------------------------------------------------------------
print("[6] 클라이언트 (접속 없이)")
c = mtd.MpaTagDetectionClient(base_dir="/nonexistent")
ok("클라이언트 이름 pytag+8자리", c.client_name.startswith("pytag") and len(c.client_name) == 13)
ok("기본 파이프 tag_detections", c.pipe_name == "tag_detections" and c.pipe_dir.endswith("tag_detections"))
ok("_make_stream 은 TagDetectionStream", isinstance(c._make_stream(), mtd.TagDetectionStream))
ok("접속 전 read_detection 은 PipeClosed", _raises(c.read_detection, mpc.PipeClosed))
ok("접속 전 fileno 는 PipeClosed", _raises(c.fileno, mpc.PipeClosed))
ok("read_frame 은 막혀 있다", _raises(c.read_frame, NotImplementedError))
ok("서버 없는 connect 는 PipeClosed", _raises(c.connect, mpc.PipeClosed))
pc = mpc.MpaPointCloudClient("voa_pc_out", base_dir="/nonexistent")
ok("포인트클라우드 클라이언트 훅은 PointCloudStream 유지", isinstance(pc._make_stream(), mpc.PointCloudStream))

# ---------------------------------------------------------------------------
print("[7] 부하")
many = [make_det(i % 3, 1000 + (i // 3) * 33) for i in range(3000)]      # 1000 프레임 × 3 태그
big = b"".join(mtd.pack_record(x) for x in many)
s = mtd.TagDetectionStream(reader_from(big, chunk=4096))
asm = mtd.FrameAssembler()
nf = len(asm.add_many(s.detections()))
ok("3000 레코드 → 999 닫힌 프레임 + 1 대기", nf == 999 and asm.pending == 3 and s.stats["records"] == 3000)

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
