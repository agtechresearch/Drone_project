"""전역 핫키 비상 정지 리스너.

pynput의 GlobalHotKeys는 OS 레벨 키보드 훅을 사용하므로
scrcpy 창이든 다른 창이든 포커스와 무관하게 즉시 반응한다.

관리자 권한 불필요 (Windows). Linux에서는 X11 접근이 필요할 수 있다.
"""
from __future__ import annotations
from typing import Callable

try:
    from pynput import keyboard
except ImportError as e:
    raise ImportError(
        "pynput 미설치: pip install pynput"
    ) from e


class EmergencyStop:
    """지정 핫키가 눌리면 on_stop 콜백을 호출.

    pynput 핫키 문법 예시:
      단일 키    : '<f12>', '<esc>', '<space>'
      조합       : '<ctrl>+<shift>+q'
      문자 조합  : '<ctrl>+q'
    """

    def __init__(self, on_stop: Callable[[], None], hotkey: str = '<f12>'):
        self.on_stop = on_stop
        self.hotkey = hotkey
        self._listener: keyboard.GlobalHotKeys | None = None
        self._triggered = False

    def start(self) -> None:
        """리스너를 백그라운드 스레드로 시작."""
        self._listener = keyboard.GlobalHotKeys({
            self.hotkey: self._trigger,
        })
        self._listener.start()

    def _trigger(self) -> None:
        # 여러 번 눌러도 콜백은 한 번만
        if self._triggered:
            return
        self._triggered = True
        try:
            self.on_stop()
        except Exception as e:
            # 콜백 예외로 리스너가 죽지 않도록 방어
            print(f"[emergency] on_stop 예외: {e!r}")

    def stop(self) -> None:
        if self._listener is not None:
            self._listener.stop()
            self._listener = None

    @property
    def triggered(self) -> bool:
        return self._triggered
