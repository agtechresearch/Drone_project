"""voxl-logger 제어 — 실험 영상·VIO·IMU 기록을 비행 코드가 시작/정지한다.

기록 형식과 시험 결과는 docs/10 §3. 명령 예:
    voxl-logger -c hires_small_color -v ov -f px4_vehicle_local_position -i imu_apps -t 300 -n <name>

- `-t` 는 상한이다. 비행이 끝나면 SIGINT 로 먼저 끝낸다(voxl-logger 는 SIGINT 에 정상 종료하며
  info.json 을 닫는다). `-t` 없이 띄우면 종료 조건이 불분명해서 반드시 상한을 준다.
- 기록 위치는 /data/voxl-logger/logNNNN/. 시작 직후 가장 최근 폴더를 찾아 기억한다.
- 기체 밖(PC) 에서는 명령이 없으므로 start() 가 False 를 돌려주고 비행은 계속한다
  (`required=True` 면 예외).
"""

import asyncio
import os
import signal

from .util import log

LOG_ROOT = "/data/voxl-logger"


class VoxlLogger(object):
    def __init__(self, name, cameras=("hires_small_color",), vio_pipes=("ov",),
                 pose_pipes=("px4_vehicle_local_position",), imu_pipes=("imu_apps",),
                 max_seconds=300, extra_args=(), log_root=LOG_ROOT):
        self.name = name
        self.cameras = list(cameras)
        self.vio_pipes = list(vio_pipes)
        self.pose_pipes = list(pose_pipes)
        self.imu_pipes = list(imu_pipes)
        self.max_seconds = int(max_seconds)
        self.extra_args = list(extra_args)
        self.log_root = log_root
        self.proc = None
        self.log_dir = None
        self._dirs_before = set()

    def command(self):
        cmd = ["voxl-logger"]
        for c in self.cameras:
            cmd += ["-c", c]
        for v in self.vio_pipes:
            cmd += ["-v", v]
        for p in self.pose_pipes:
            cmd += ["-f", p]
        for i in self.imu_pipes:
            cmd += ["-i", i]
        cmd += ["-t", str(self.max_seconds), "-n", self.name]
        cmd += self.extra_args
        return cmd

    def _list_dirs(self):
        try:
            return set(d for d in os.listdir(self.log_root) if d.startswith("log"))
        except OSError:
            return set()

    async def start(self, required=False, settle_s=1.5):
        cmd = self.command()
        self._dirs_before = self._list_dirs()
        log("[logger] start: {}".format(" ".join(cmd)))
        try:
            self.proc = await asyncio.create_subprocess_exec(
                *cmd, stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)
        except FileNotFoundError:
            msg = "voxl-logger 명령이 없다 (기체가 아니거나 voxl-suite 미설치)"
            if required:
                raise RuntimeError(msg)
            log("[logger] {} — 기록 없이 진행".format(msg))
            self.proc = None
            return False
        await asyncio.sleep(settle_s)
        if self.proc.returncode is not None:
            msg = "voxl-logger 가 {:.1f}s 안에 종료됨 (rc={}). 파이프 이름/권한 확인".format(
                settle_s, self.proc.returncode)
            if required:
                raise RuntimeError(msg)
            log("[logger] {}".format(msg))
            self.proc = None
            return False
        new_dirs = sorted(self._list_dirs() - self._dirs_before)
        if new_dirs:
            self.log_dir = os.path.join(self.log_root, new_dirs[-1])
        log("[logger] running pid={} dir={}".format(self.proc.pid, self.log_dir or "?"))
        return True

    @property
    def running(self):
        return self.proc is not None and self.proc.returncode is None

    async def stop(self, timeout=8.0):
        if self.proc is None:
            return None
        if self.proc.returncode is None:
            log("[logger] stop (SIGINT)")
            try:
                self.proc.send_signal(signal.SIGINT)
                await asyncio.wait_for(self.proc.wait(), timeout=timeout)
            except asyncio.TimeoutError:
                log("[logger] SIGINT 후 {:.0f}s 응답 없음 → terminate".format(timeout))
                try:
                    self.proc.terminate()
                    await asyncio.wait_for(self.proc.wait(), timeout=3.0)
                except Exception:
                    try:
                        self.proc.kill()
                    except Exception:
                        pass
            except ProcessLookupError:
                pass
        rc = self.proc.returncode
        if self.log_dir is None:
            new_dirs = sorted(self._list_dirs() - self._dirs_before)
            if new_dirs:
                self.log_dir = os.path.join(self.log_root, new_dirs[-1])
        log("[logger] stopped rc={} dir={}".format(rc, self.log_dir or "?"))
        return self.log_dir
