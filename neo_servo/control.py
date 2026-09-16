"""DJI Neo 제어: pyautogui로 가상 조이스틱 연속 스트리밍 + 이산 버튼.

핵심 아이디어:
  기존 방식 = mouseDown → drag → mouseUp (이산 명령)
  새 방식   = mouseDown 유지 + moveTo 반복 (연속 서보, RC override 유사)

Neo는 스틱을 놓으면 자체 비전 포지셔닝으로 호버하므로,
아무 명령도 필요없는 상태에서는 mouseUp으로 스틱을 놓아둔다.
"""
from __future__ import annotations
import time
import pyautogui
import pywinctl as pw


# pyautogui 내부 지연 제거 (기본 0.1s → 스틱 스트리밍 불가)
pyautogui.PAUSE = 0
pyautogui.FAILSAFE = False


class NeoController:
    def __init__(self, cfg: dict, logger=None):
        self.cfg = cfg
        self.logger = logger

        # scrcpy 창 찾기
        title = cfg['phone']['window_title']
        wins = pw.getWindowsWithTitle(title)
        if not wins:
            raise RuntimeError(f"scrcpy 창 미발견: '{title}'")
        win = wins[0]

        # 1) 최대화 상태면 복원 (DPI/스냅 등으로 예기치 않게 최대화되는 경우)
        try:
            if getattr(win, 'isMaximized', False):
                if logger:
                    logger.log("[control] 창이 최대화 상태 → 복원(restore)")
                win.restore()
                time.sleep(0.3)
        except Exception as e:
            if logger:
                logger.log(f"[control] 창 복원 실패(무시): {e!r}")

        # 2) config의 window_size로 명시적 리사이즈
        #    (UI 좌표는 이 크기 기준으로 캘리브레이션되어 있으므로 일치시켜야 함)
        expected = tuple(cfg['phone']['window_size'])
        try:
            win.resizeTo(*expected)
            time.sleep(0.3)   # 리사이즈 반영 대기
            if logger:
                logger.log(f"[control] 창 리사이즈 → {expected}")
        except Exception as e:
            if logger:
                logger.log(f"[control] 창 리사이즈 실패(무시): {e!r}")

        # 3) 실제 크기가 요청한 값과 다르면 경고
        actual = (win.width, win.height)
        if actual != expected:
            msg = (f"⚠️ 리사이즈 후에도 크기 불일치: 실제={actual} config={expected}. "
                   f"OS 최소창 제약 등이 원인일 수 있음. UI 좌표 어긋날 수 있으니 확인.")
            if logger:
                logger.log(msg)
            else:
                print(msg)

        self.origin = (win.left, win.top)
        if logger:
            logger.log(f"[control] scrcpy 창 origin={self.origin} size={actual}")

        # UI 절대 좌표 변환
        ui = cfg['ui_layout']
        self.left_center  = self._abs(ui['left_stick_center'])
        self.right_center = self._abs(ui['right_stick_center'])
        self.radius = ui['stick_radius']
        self.buttons = {
            'takeoff':         self._abs(ui['takeoff_button']),
            'takeoff_confirm': self._abs(ui['takeoff_confirm']),
            'landing':         self._abs(ui['landing_button']),
            'capture':         self._abs(ui['capture_button']),
        }

        # 스틱 스트리밍 상태
        self._holding: bool = False
        self._current_stick: str | None = None

    # ---------- 좌표 유틸 ----------
    def _abs(self, rel):
        return (self.origin[0] + rel[0], self.origin[1] + rel[1])

    def _stick_center(self, name: str):
        return self.left_center if name == 'left' else self.right_center

    # ---------- 이산 버튼 액션 ----------
    def _hold_click(self, pos, duration: float):
        pyautogui.moveTo(*pos, _pause=False)
        pyautogui.mouseDown()
        time.sleep(duration)
        pyautogui.mouseUp()
        time.sleep(0.3)

    def takeoff(self):
        self._hold_click(self.buttons['takeoff'], 0.2)
        time.sleep(0.3)
        self._hold_click(self.buttons['takeoff_confirm'], 3)
        time.sleep(2)
        if self.logger: self.logger.log("[control] 이륙 완료")

    def landing(self):
        """착륙 시퀀스는 이륙과 동일: 버튼 클릭 → 확인 슬라이더 3초 홀드.

        DJI Fly에서 이/착륙이 같은 버튼 위치이고 둘 다 확인 슬라이더로 확정.
        """
        # 스틱 잡고 있으면 먼저 놓기
        self._release_stick()
        time.sleep(0.3)   # 스틱 놓음이 앱에 반영될 시간
        self._hold_click(self.buttons['takeoff'], 0.2)
        time.sleep(0.3)
        self._hold_click(self.buttons['takeoff_confirm'], 3)
        time.sleep(2)
        if self.logger:
            self.logger.log("[control] 착륙 완료")

    def capture(self):
        # 촬영은 스틱과 별개 버튼이라 스틱 잠시 놓고 클릭
        was_holding = self._holding
        current = self._current_stick
        self._release_stick()
        time.sleep(0.3)   # 스틱 놓음이 앱에 반영될 시간
        self._hold_click(self.buttons['capture'], 0.2)
        time.sleep(1)
        if self.logger: self.logger.log("[control] 촬영")
        # 촬영 후에는 다시 스틱 잡지 않음 — 컨트롤 루프가 다음 사이클에서 재개

    # ---------- 초기 액션 (mission의 initial_actions 실행용) ----------
    def execute_stick_action(self, stick: str, offset_x: int, offset_y: int,
                              duration: float):
        """지정 스틱을 offset 방향으로 duration 초 동안 유지 (blocking).

        임무 시작 전 초기 상승/전진 등에 사용. control_loop과 별개로
        호출 스레드에서 직접 실행. 호출 전 mission_state='init_actions'로
        설정해서 control_loop 개입을 막아야 함.
        """
        center = self._stick_center(stick)
        self._release_stick()   # 혹시 잡고 있던 것 정리
        time.sleep(0.3)         # 이전 명령 관성 감쇠

        pyautogui.moveTo(*center, _pause=False)
        pyautogui.mouseDown()
        pyautogui.moveTo(center[0] + offset_x, center[1] + offset_y, _pause=False)
        time.sleep(duration)
        pyautogui.mouseUp()
        time.sleep(0.3)

        if self.logger:
            self.logger.log(f"[control] 초기액션 완료: {stick} "
                            f"offset=({offset_x},{offset_y}) dur={duration}s")

    # ---------- 연속 스틱 스트리밍 ----------
    def control_loop(self, state, cfg: dict):
        """Vision 스레드가 갱신한 state의 스틱 명령을 매 주기 반영.

        임무 상태(state.mission_state)에 따라:
          - 'init_actions'/'hardcoded_running': control_loop은 개입 안 함
          - 그 외: state.active_stick / stick_offset 반영

        방향 전환 시 중립 통과:
          이전 명령과 새 명령의 방향이 크게 다르면 (부호 반대 or 축 변경)
          잠시 스틱 놓고(release) 대기해서 드론 관성 감쇠.
        """
        period = 1.0 / cfg['control']['loop_hz']
        transition_delay_s = 0.3     # 방향 전환 시 중립 유지 시간

        last_stick = None
        last_offset = (0, 0)

        while state.running:
            with state.lock:
                ms = state.mission_state
                stick = state.active_stick
                ox, oy = state.stick_offset

            if ms in ('init_actions', 'hardcoded_running'):
                time.sleep(period)
                continue

            # 방향 전환 감지 (이전 명령과 크게 다르면 잠시 중립 통과)
            transition_needed = False
            if stick is not None and last_stick is not None:
                if stick != last_stick:
                    # 다른 스틱으로 전환 (좌↔우스틱)
                    transition_needed = True
                else:
                    # 같은 스틱: 방향 변화 검사
                    # (a) 어느 축이든 부호 반대
                    if (ox * last_offset[0] < 0) or (oy * last_offset[1] < 0):
                        transition_needed = True
                    else:
                        # (b) 주 축 변경 (x→y 또는 y→x)
                        prev_axis = 'y' if abs(last_offset[1]) > abs(last_offset[0]) else 'x'
                        curr_axis = 'y' if abs(oy) > abs(ox) else 'x'
                        prev_nonzero = last_offset != (0, 0)
                        curr_nonzero = (ox, oy) != (0, 0)
                        if prev_nonzero and curr_nonzero and prev_axis != curr_axis:
                            transition_needed = True

            if transition_needed:
                # 잠시 중립 (Neo가 감속하고 새 명령에 반응할 시간)
                self._release_stick()
                if self.logger:
                    self.logger.log(
                        f"[control] 방향 전환: prev={last_stick}{last_offset} "
                        f"→ new={stick}({ox},{oy}), {transition_delay_s}s 중립"
                    )
                time.sleep(transition_delay_s)

            # 스틱 명령 반영
            if stick is None or (ox == 0 and oy == 0):
                self._release_stick()
            else:
                self._update_stick(stick, ox, oy)

            last_stick = stick
            last_offset = (ox, oy)

            time.sleep(period)

        self._release_stick()

    def _update_stick(self, stick: str, ox: int, oy: int):
        center = self._stick_center(stick)
        # 스틱 종류 바뀌면 완전히 놓고 다시 잡기
        if self._current_stick != stick or not self._holding:
            self._release_stick()
            pyautogui.moveTo(*center, _pause=False)
            pyautogui.mouseDown()
            self._holding = True
            self._current_stick = stick
        # drag 유지 상태에서 위치만 갱신
        pyautogui.moveTo(center[0] + ox, center[1] + oy, _pause=False)

    def _release_stick(self):
        if self._holding:
            pyautogui.mouseUp()
            self._holding = False
            self._current_stick = None

    # ---------- 비상 정지 (외부 스레드에서 호출 가능) ----------
    def emergency_release(self):
        """즉시 스틱 해제. 어느 스레드에서 불러도 안전.

        pyautogui.mouseUp()은 idempotent해서 이미 놓은 상태에서 또 불러도 문제없음.
        control_loop의 sleep이 최대 loop_hz 주기만큼 지연될 수 있어서
        여기서 먼저 물리적으로 스틱을 놓아버린다 → Neo 자체 호버.
        """
        try:
            pyautogui.mouseUp()
        except Exception:
            pass
        self._holding = False
        self._current_stick = None
        if self.logger:
            self.logger.log("[control] EMERGENCY: 스틱 즉시 해제")