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
    # 위치 = 첫 번째로 숫자 3개가 나오는 칸 (dt 칸 다음)
    for p in parts[1:q_idx - 1]:
        pn = nums(p)
        if len(pn) == 3:
            pos = (pn[0], pn[1], pn[2])
            break
    return {"quality": quality, "features": features, "state": state, "error_codes": error_codes,
            "pos": pos, "raw": text}


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
        self.samples.append((now, parsed.get("quality", -1), parsed.get("features", -1), parsed.get("state", "")))
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

    def summarize(self, now, seconds):
        """창 안 요약: n, min/mean quality, min features, 비-OKAY 수."""
        w = self.window(now, seconds)
        if not w:
            return {"n": 0, "min_quality": -1, "mean_quality": -1.0, "min_features": -1, "n_not_ok": 0}
        qs = [s[1] for s in w]
        fs = [s[2] for s in w if s[2] >= 0]
        return {"n": len(w), "min_quality": min(qs), "mean_quality": sum(qs) / float(len(qs)),
                "min_features": (min(fs) if fs else -1), "n_not_ok": sum(1 for s in w if s[3] not in ("", STATE_OK))}


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
