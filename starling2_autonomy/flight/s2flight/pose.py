"""voxl-inspect-pose 출력 파서와 파이프 리더 태스크 (v14 에서 옮김).

리더는 `voxl-inspect-pose -n <pipe>` 를 자식 프로세스로 띄워 한 줄씩 읽고,
파싱한 포즈를 `sink(pipe_name, parsed)` 로 넘긴다. 프로세스가 죽으면 백오프 후 재시작한다.
"""

import asyncio
import time

from .util import log, mono_time, nums, wallclock


def parse_voxl_inspect_pose_line(line):
    """`voxl-inspect-pose -n <pose_pipe>` 한 줄 파싱.

    기대 형식:  timestamp(ms)| x y z | roll pitch yaw | vx vy vz | wx wy wz
    '|' 로 나눠 두 번째 칸에서만 위치를 읽는다(타임스탬프를 x 로 잘못 읽던 옛 버그 방지).
    """
    if not line:
        return None
    text = line.strip()
    if not text or "|" not in text:
        return None

    upper = text.upper()
    if "TIMESTAMP" in upper or "POSITION" in upper or "ERROR" in upper:
        return None

    parts = [p.strip() for p in text.split("|")]
    if len(parts) < 4:
        return None

    try:
        ts_nums = nums(parts[0])
        pos_nums = nums(parts[1])
        rpy_nums = nums(parts[2])
        vel_nums = nums(parts[3])
        ang_nums = nums(parts[4]) if len(parts) > 4 else []

        if len(ts_nums) < 1 or len(pos_nums) < 3:
            return None

        while len(rpy_nums) < 3:
            rpy_nums.append(float("nan"))
        while len(vel_nums) < 3:
            vel_nums.append(float("nan"))
        while len(ang_nums) < 3:
            ang_nums.append(float("nan"))

        return {
            "voxl_ts_ms": ts_nums[0],
            "pos_x": pos_nums[0],
            "pos_y": pos_nums[1],
            "pos_z": pos_nums[2],
            "roll_deg": rpy_nums[0],
            "pitch_deg": rpy_nums[1],
            "yaw_deg": rpy_nums[2],
            "vel_x": vel_nums[0],
            "vel_y": vel_nums[1],
            "vel_z": vel_nums[2],
            "ang_x": ang_nums[0],
            "ang_y": ang_nums[1],
            "ang_z": ang_nums[2],
            "raw": text,
        }
    except Exception:
        return None


def stamp(parsed):
    """리더가 포즈에 붙이는 수신 시각 필드."""
    parsed["mono_time"] = mono_time()
    parsed["unix_time"] = time.time()
    parsed["wallclock"] = wallclock()
    return parsed


async def pose_reader_task(pipe_name, sink, is_running, errors):
    """`voxl-inspect-pose -n <pipe>` 를 읽어 sink 로 보낸다.

    sink(pipe_name, parsed) : 파싱된 포즈(dict, 수신 시각 포함)를 받는 콜백
    is_running()            : False 가 되면 종료
    errors                  : dict, 마지막 오류 메모 (pipe_name -> str)
    """
    if not pipe_name:
        return
    log("[voxl] starting pose reader: voxl-inspect-pose -n {}".format(pipe_name))
    backoff = 1.0

    while is_running():
        proc = None
        try:
            cmd = ["voxl-inspect-pose", "-n"]
            if pipe_name in ("local", "--local", "-l"):
                cmd.append("-l")
            elif pipe_name in ("fixed", "--fixed", "-f"):
                cmd.append("-f")
            else:
                cmd.append(pipe_name)

            proc = await asyncio.create_subprocess_exec(
                *cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
            backoff = 1.0

            while is_running():
                try:
                    raw = await asyncio.wait_for(proc.stdout.readline(), timeout=2.0)
                except asyncio.TimeoutError:
                    continue
                if not raw:
                    break
                line = raw.decode("utf-8", errors="replace").strip()
                parsed = parse_voxl_inspect_pose_line(line)
                if parsed is None:
                    if "ERROR" in line.upper():
                        errors[pipe_name] = line
                    continue
                sink(pipe_name, stamp(parsed))

        except FileNotFoundError:
            log("[voxl] voxl-inspect-pose not found. Pipe '{}' disabled.".format(pipe_name))
            return
        except asyncio.CancelledError:
            raise
        except Exception as e:
            errors[pipe_name] = str(e)
            log("[voxl] reader error for '{}': {}".format(pipe_name, e))
        finally:
            if proc is not None and proc.returncode is None:
                try:
                    proc.terminate()
                    await asyncio.wait_for(proc.wait(), timeout=1.0)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass

        if is_running():
            await asyncio.sleep(backoff)
            backoff = min(backoff * 1.5, 5.0)
