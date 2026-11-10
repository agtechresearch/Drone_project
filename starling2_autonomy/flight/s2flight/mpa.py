"""MPA(/run/mpa) 파이프 구독 공용 클라이언트 (flight/mpa_point_cloud.py 의 클라이언트 부분 사본).

libmodal-pipe 의 클라이언트 절차를 순수 파이썬으로 흉내낸다.
  1. /run/mpa/<pipe>/request FIFO 에 클라이언트 이름을 쓴다
  2. 서버가 /run/mpa/<pipe>/<client_name> FIFO 를 만들어 준다
  3. 그 FIFO 를 O_NONBLOCK 으로 열고 select/os.read 로 읽는다
자료형별 파서는 `make_stream(read_fn)` 으로 끼운다 (tags.TagDetectionStream 등).
"""

import errno
import os
import random
import select
import time

DEFAULT_BASE_DIR = "/run/mpa"


class PipeClosed(Exception):
    """파이프가 닫혔다(서버 종료 또는 EOF)."""


class PipeTimeout(Exception):
    """정해진 시간 안에 데이터가 오지 않았다."""


def list_pipes(base_dir=DEFAULT_BASE_DIR):
    """구독 가능한 파이프 이름 목록."""
    try:
        names = os.listdir(base_dir)
    except OSError:
        return []
    return sorted(n for n in names if os.path.exists(os.path.join(base_dir, n, "request")))


def pipe_available(pipe_name, base_dir=DEFAULT_BASE_DIR):
    return os.path.exists(os.path.join(base_dir, pipe_name, "request"))


def read_pipe_info(pipe_name, base_dir=DEFAULT_BASE_DIR):
    path = os.path.join(base_dir, pipe_name, "info")
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except (OSError, IOError):
        return None


class MpaClient(object):
    """/run/mpa/<파이프> 구독. make_stream(read_fn) 이 바이트→레코드 파서를 만든다."""

    def __init__(self, pipe_name, make_stream, client_name="pympa",
                 base_dir=DEFAULT_BASE_DIR, read_timeout=2.0):
        self.pipe_name = pipe_name
        self.base_dir = base_dir
        self.pipe_dir = os.path.join(base_dir, pipe_name)
        self.read_timeout = read_timeout
        self._make_stream = make_stream
        # 서버는 클라이언트를 이름으로 구분한다. client.c 와 같이 8자리 난수를 붙인다.
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
                "'ls {}' 로 확인할 것)".format(request_path, self.base_dir))
        try:
            fd = os.open(request_path, os.O_WRONLY | os.O_NONBLOCK)
        except OSError as e:
            if e.errno == errno.ENXIO:
                raise PipeClosed("request FIFO 에 서버가 붙어있지 않다: {}".format(request_path))
            raise
        try:
            os.write(fd, self.client_name.encode("utf-8") + b"\0")
        finally:
            os.close(fd)

    def _wait_for_fifo(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if os.path.exists(self.fifo_path):
                return True
            time.sleep(0.01)
        return False

    def connect(self, timeout=5.0):
        self._request_subscription()
        if not self._wait_for_fifo(timeout):
            raise PipeTimeout("서버가 {:.1f}초 안에 클라이언트 FIFO 를 만들지 않았다: {}".format(
                timeout, self.fifo_path))
        self._fd = os.open(self.fifo_path, os.O_RDONLY | os.O_NONBLOCK)
        self._stream = self._make_stream(self._read)
        return self

    @property
    def connected(self):
        return self._fd is not None

    @property
    def stream(self):
        if self._stream is None:
            raise PipeClosed("connect() 를 먼저 호출할 것")
        return self._stream

    def fileno(self):
        if self._fd is None:
            raise PipeClosed("connect() 를 먼저 호출할 것")
        return self._fd

    # -- 읽기 --------------------------------------------------------------

    def _read(self, n):
        """select 로 기다렸다가 읽는다 (블로킹). 타임아웃이면 PipeTimeout."""
        deadline = time.time() + self.read_timeout
        while True:
            remain = deadline - time.time()
            if remain <= 0:
                raise PipeTimeout("{:.1f}초 동안 '{}' 에서 데이터가 오지 않았다".format(
                    self.read_timeout, self.pipe_name))
            try:
                ready, _, _ = select.select([self._fd], [], [], remain)
            except (select.error, OSError):
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
            time.sleep(0.01)

    def read_nonblocking(self, max_bytes=65536):
        """지금 FIFO 에 있는 바이트만 (없으면 b''). asyncio add_reader 콜백용."""
        if self._fd is None:
            raise PipeClosed("connect() 를 먼저 호출할 것")
        try:
            return os.read(self._fd, max_bytes)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return b""
            raise

    @property
    def stats(self):
        return self._stream.stats if self._stream is not None else {}

    # -- 정리 --------------------------------------------------------------

    def close(self):
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
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
