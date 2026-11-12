"""VIO 상태 리더 (voxl-inspect-vio) — 2026-11-10 사고 대책 P1/P2.

이륙 중 OpenVINS 가 3 s 끊기자 PX4 EKF 가 추측항법으로 고도를 과소 추정해 천장까지 올라갔다(docs/12 사고 기록).
PX4 의 `is_local_position_ok` 는 그동안 계속 OK 였으므로, 비행 코드가 **VIO 자체의 품질**을 직접 봐야 한다.

리더는 `voxl-inspect-vio -n <pipe>` 를 자식 프로세스로 띄워 한 줄씩 읽는다 (pose.py 의 voxl-inspect-pose 리더와 같은 방식).
기본 파이프는 `ov`(voxl-open-vins-server 원출력). 출력 형식(한 줄, -n):

     dt(ms) |    T_imu_wrt_vio (m)   |Roll Pitch Yaw (deg)|features|quality| state| error_codes
       66.7 |   -0.05    0.05   -0.03|  -2.2    0.4  -70.9|    11  |   96% | OKAY |

품질(quality)은 0~100 %, VIO 가 무효면 PX4 쪽 `vehicle_visual_odometry.quality` 는 -1 이 되고 inspect 출력은 0 % 또는 상태가
OKAY 가 아니게 된다. 여기서는 quality 가 없거나 파싱 불가면 -1 로 둔다.
"""

import asyncio
import collections
import math
import re
import time

from .util import log, mono_time, nums, wallclock

STATE_OK = "OKAY"
_STATE_RE = re.compile(r"^[A-Z_]{3,}$")


def parse_voxl_inspect_vio_line(line):
    """`voxl-inspect-vio -n [pipe]` 한 줄 → dict 또는 None (헤더·빈 줄·에러 줄).

    반환 키: quality(int, -1 = 모름), features(int, -1 = 모름), state(str, "" = 모름), error_codes(str),
    pos(x,y,z) 또는 None, raw.
    """
    if not line:
        return None
    text = line.strip()
    if not text or "|" not in text:
        return None
    upper = text.upper()
    if "FEATURES" in upper or "QUALITY" in upper or "WAITING" in upper or "ERROR" in upper and "ERROR_CODES" not in upper:
        return None
    parts = [p.strip() for p in text.split("|")]
    if len(parts) < 5:
        return None

    quality = -1
    features = -1
    state = ""
    error_codes = ""
    pos = None
    q_idx = None
    for i, p in enumerate(parts):
        if "%" in p and q_idx is None:
            q_idx = i
            qn = nums(p.replace("%", " "))
            if qn:
                quality = int(round(qn[0]))
    if q_idx is None:
        return None
    # features = 품질 바로 앞 칸의 정수
    if q_idx >= 1:
        fn = nums(parts[q_idx - 1])
        if len(fn) == 1:
            features = int(round(fn[0]))
    # state = 품질 뒤 첫 영문 칸, error_codes = 그 다음 칸
    for p in parts[q_idx + 1:]:
        if not state and _STATE_RE.match(p):
            state = p
            continue
        if state:
            error_codes = p
            break
    # 위치 = 첫 번째로 숫자 3개가 나오는 칸 (dt 칸 다음), RPY = 그 다음 숫자 3개 칸 (deg, VIO 프레임 yaw — PX4 EV 와 같은 부호)
    rpy = None
    for p in parts[1:q_idx - 1]:
        pn = nums(p)
        if len(pn) == 3:
            if pos is None:
                pos = (pn[0], pn[1], pn[2])
            elif rpy is None:
                rpy = (pn[0], pn[1], pn[2])
                break
    return {"quality": quality, "features": features, "state": state, "error_codes": error_codes,
            "pos": pos, "rpy": rpy, "raw": text}


def stamp(parsed):
    parsed["mono_time"] = mono_time()
    parsed["unix_time"] = time.time()
    parsed["wallclock"] = wallclock()
    return parsed


async def vio_reader_task(pipe_name, sink, is_running, errors):
    """`voxl-inspect-vio -n <pipe>` 를 읽어 sink(parsed) 로 보낸다. 프로세스가 죽으면 백오프 후 재시작."""
    if not pipe_name:
        return
    log("[vio] starting VIO status reader: voxl-inspect-vio -n {}".format(pipe_name))
    backoff = 1.0
    while is_running():
        proc = None
        try:
            proc = await asyncio.create_subprocess_exec(
                "voxl-inspect-vio", "-n", pipe_name,
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
            backoff = 1.0
            while is_running():
                try:
                    raw = await asyncio.wait_for(proc.stdout.readline(), timeout=2.0)
                except asyncio.TimeoutError:
                    continue
                if not raw:
                    break
                line = raw.decode("utf-8", errors="replace").strip()
                parsed = parse_voxl_inspect_vio_line(line)
                if parsed is None:
                    up = line.upper()
                    if "ERROR" in up and "ERROR_CODES" not in up:
                        errors["vio:" + pipe_name] = line
                    continue
                sink(stamp(parsed))
        except FileNotFoundError:
            log("[vio] voxl-inspect-vio not found. VIO status reader disabled (gate will fail unless --no-vio-gate).")
            errors["vio:" + pipe_name] = "voxl-inspect-vio not found"
            return
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001 — 리더는 죽지 않고 재시작한다
            errors["vio:" + pipe_name] = repr(e)
        finally:
            if proc is not None and proc.returncode is None:
                try:
                    proc.terminate()
                    await asyncio.wait_for(proc.wait(), timeout=2.0)
                except Exception:  # noqa: BLE001
                    try:
                        proc.kill()
                    except Exception:  # noqa: BLE001
                        pass
        if is_running():
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2.0, 8.0)


class VioHistory(object):
    """최근 VIO 상태 샘플. 게이트(정지 창 안 최소 품질·특징점·상태)와 비행 중 감시(불량 지속 시간)에 쓴다."""

    def __init__(self, keep_s=60.0):
        self.keep_s = keep_s
        self.samples = collections.deque()
        self.latest = None
        self.n = 0
        self.bad_since = None          # 불량(품질 < 임계 또는 상태 != OKAY)이 시작된 mono_time
        self.n_bad_episodes = 0

    def add(self, parsed, now=None, min_quality=1):
        now = parsed.get("mono_time", mono_time()) if now is None else now
        self.latest = parsed
        self.n += 1
        rpy = parsed.get("rpy")
        yaw = float(rpy[2]) if rpy is not None and len(rpy) >= 3 else float("nan")
        self.samples.append((now, parsed.get("quality", -1), parsed.get("features", -1), parsed.get("state", ""), yaw))
        cutoff = now - self.keep_s
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        bad = self.is_bad(parsed, min_quality)
        if bad and self.bad_since is None:
            self.bad_since = now
            self.n_bad_episodes += 1
        elif not bad:
            self.bad_since = None
        return bad

    @staticmethod
    def is_bad(parsed, min_quality=1):
        if parsed is None:
            return True
        q = parsed.get("quality", -1)
        st = parsed.get("state", "")
        return q < min_quality or (st != "" and st != STATE_OK)

    def age(self, now):
        if self.latest is None:
            return float("inf")
        return now - self.latest.get("mono_time", now)

    def bad_duration(self, now):
        return 0.0 if self.bad_since is None else now - self.bad_since

    def window(self, now, seconds):
        return [s for s in self.samples if s[0] >= now - seconds]

    def latest_yaw(self):
        """최신 VIO yaw (deg) 또는 NaN."""
        l = self.latest
        rpy = l.get("rpy") if l else None
        return float(rpy[2]) if rpy is not None and len(rpy) >= 3 else float("nan")

    def summarize(self, now, seconds):
        """창 안 요약: n, min/mean quality, min features, 비-OKAY 수, yaw 변동 범위(deg)."""
        w = self.window(now, seconds)
        if not w:
            return {"n": 0, "min_quality": -1, "mean_quality": -1.0, "min_features": -1, "n_not_ok": 0, "yaw_range": 0.0, "rate_hz": 0.0}
        qs = [s[1] for s in w]
        fs = [s[2] for s in w if s[2] >= 0]
        span = w[-1][0] - w[0][0]
        return {"n": len(w), "min_quality": min(qs), "mean_quality": sum(qs) / float(len(qs)),
                "min_features": (min(fs) if fs else -1), "n_not_ok": sum(1 for s in w if s[3] not in ("", STATE_OK)),
                "yaw_range": yaw_range_deg([s[4] for s in w]),
                "rate_hz": ((len(w) - 1) / span if span > 0 and len(w) > 1 else 0.0)}


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def yaw_range_deg(yaws):
    """각도 목록의 wrap 을 고려한 변동 범위 (첫 값 기준으로 펼친 max-min). NaN 은 무시."""
    ys = [y for y in yaws if y == y]
    if len(ys) < 2:
        return 0.0
    rel = [wrap180(y - ys[0]) for y in ys]
    return max(rel) - min(rel)


def read_uptime_s(path="/proc/uptime"):
    """부팅 뒤 경과 초. 없으면(비-리눅스) None."""
    try:
        with open(path) as f:
            return float(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        return None


_GB_RE = re.compile(r"gyro_bias:\s*\[([^\]]+)\]")
_AB_RE = re.compile(r"accel_bias:\s*\[([^\]]+)\]")


def parse_px4_accel_bias(text):
    """`px4-listener estimator_sensor_bias` 출력 → (ax, ay, az) m/s², 없으면 None."""
    m = _AB_RE.search(text or "")
    if not m:
        return None
    try:
        vals = [float(v) for v in m.group(1).split(",")]
    except ValueError:
        return None
    return tuple(vals[:3]) if len(vals) >= 3 else None


def parse_px4_gyro_bias(text):
    """`px4-listener estimator_sensor_bias` 출력 → (bx, by, bz) deg/s, 없으면 None."""
    m = _GB_RE.search(text or "")
    if not m:
        return None
    try:
        vals = [float(v) for v in m.group(1).split(",")]
    except ValueError:
        return None
    if len(vals) < 3:
        return None
    return tuple(math.degrees(v) for v in vals[:3])


async def read_px4_sensor_bias(timeout_s=6.0):
    """EKF2 가 추정한 자이로(deg/s)·가속도계(m/s²) 바이어스. ((gyro, accel) 또는 None, 오류 문자열 또는 None).
    2026-11-12: 부팅 31 s 만에 arm 한 비행에서 자이로 z 바이어스 -3.5 deg/s → 정지 상태에서 헤딩이 3.5 deg/s 로 돌아 좌측 밀림·좌회전.
    같은 날 천장 충돌: 직전 비행에서 가속도계 z 바이어스가 한계(0.40 m/s²)까지 커진 채 재부팅 없이 이륙 → VIO 상실 뒤 추측항법이
    0.4 m/s² 로 '하강' 적분 → 추력 상승 → 천장."""
    try:
        proc = await asyncio.create_subprocess_exec("px4-listener", "estimator_sensor_bias", "-n", "1",
                                                    stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
    except (OSError, ValueError) as e:
        return None, "px4-listener unavailable: {!r}".format(e)
    try:
        out, _ = await asyncio.wait_for(proc.communicate(), timeout_s)
    except asyncio.TimeoutError:
        try:
            proc.kill()
        except Exception:  # noqa: BLE001
            pass
        return None, "px4-listener timeout"
    txt = out.decode("utf-8", errors="replace")
    b = parse_px4_gyro_bias(txt)
    if b is None:
        return None, "gyro_bias field not found"
    return (b, parse_px4_accel_bias(txt)), None


async def read_px4_gyro_bias(timeout_s=6.0):
    """하위 호환: 자이로 바이어스만."""
    r, err = await read_px4_sensor_bias(timeout_s)
    return (None, err) if r is None else (r[0], None)


async def journal_recent_vio_events(seconds, unit="voxl-open-vins-server"):
    """최근 N 초 open-vins 저널에서 BAD 전이·리셋 줄을 돌려준다. journalctl 이 없으면 None (검사 생략)."""
    try:
        proc = await asyncio.create_subprocess_exec(
            "journalctl", "-u", unit, "--since", "-{}s".format(int(seconds)), "--no-pager", "-o", "short",
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
    except FileNotFoundError:
        return None
    try:
        out, _ = await asyncio.wait_for(proc.communicate(), timeout=5.0)
    except asyncio.TimeoutError:
        try:
            proc.kill()
        except Exception:  # noqa: BLE001
            pass
        return None
    lines = out.decode("utf-8", errors="replace").splitlines()
    keys = ("→ BAD", "-> BAD", "RESET", "reset", "quality was bad", "not stationary")
    return [l.strip() for l in lines if any(k in l for k in keys)]
