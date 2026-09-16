"""진입점 - config + mission 로드, 스레드 오케스트레이션, 임무 상태 기계.

임무 상태 흐름:
  idle → takeoff → init_actions → waiting_start → recording → landing → done
"""
from __future__ import annotations
import dpi_setup  # noqa: F401 - Windows DPI awareness (반드시 최상단)
import argparse
import os
import sys
import threading
import time
import yaml

from stream import create_source
from vision import VisionState, build_ui_mask, create_detector, vision_loop
from control import NeoController
from emergency import EmergencyStop


# ============================================================
# 로거
# ============================================================
class Logger:
    def __init__(self, log_dir: str, prefix: str):
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(log_dir, f"{prefix}_{ts}.log")
        self.file = open(self.path, 'a', encoding='utf-8')
        self.start = time.time()

    def log(self, msg: str):
        line = f"[+{time.time()-self.start:7.2f}s] {msg}"
        print(line)
        self.file.write(line + "\n")
        self.file.flush()

    def close(self):
        self.file.close()


def load_yaml(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


# ============================================================
# 임무 상태 기계
# ============================================================
def set_mission_state(state: VisionState, new_state: str, logger: Logger):
    with state.lock:
        old = state.mission_state
        state.mission_state = new_state
    logger.log(f"===== mission_state: {old} → {new_state} =====")


def run_initial_actions(mission_cfg: dict, state: VisionState,
                        controller: NeoController, logger: Logger):
    """이륙 후 초기 스틱 명령 순차 수행."""
    actions = mission_cfg['mission']['post_takeoff'].get('initial_actions', []) or []
    if not actions:
        logger.log("초기 액션 없음")
        return

    set_mission_state(state, 'init_actions', logger)
    for i, act in enumerate(actions, 1):
        if not state.running:
            return
        stick = act['stick']
        ox, oy = act['offset']
        dur = act['duration']
        logger.log(f"초기액션 {i}/{len(actions)}: stick={stick} "
                   f"offset=({ox},{oy}) duration={dur}s")
        controller.execute_stick_action(stick, ox, oy, dur)


def wait_for_marker(state: VisionState, target_id: int, timeout_s: float,
                    logger: Logger) -> bool:
    """특정 마커 ID가 감지될 때까지 대기. True=발견, False=타임아웃/중단."""
    deadline = time.time() + timeout_s
    logger.log(f"마커 ID={target_id} 대기 중 (최대 {timeout_s}s)...")
    while time.time() < deadline and state.running:
        with state.lock:
            if state.marker_seen and state.current_marker_id == target_id:
                logger.log(f"→ 마커 ID={target_id} 발견")
                return True
        time.sleep(0.1)
    return False


def find_role_marker_id(sequence: list, role: str) -> int | None:
    """sequence에서 지정 role의 마커 ID 찾기. 없으면 None."""
    for item in sequence:
        if item.get('role') == role:
            return item['id']
    return None


def check_recording_end(state: VisionState, mission_cfg: dict) -> str | None:
    """녹화 종료 조건 감지. 종료 이유 문자열 반환 or None."""
    sequence = mission_cfg['mission']['sequence']

    with state.lock:
        complete = state.sequence_complete
        cur_id = state.current_marker_id if state.marker_seen else None

    # 시퀀스 완료
    if complete:
        return "시퀀스 완료 (마지막 인덱스 도달)"

    if cur_id is None:
        return None

    # 현재 마커가 role='end' 또는 action='end'
    for item in sequence:
        if item['id'] == cur_id:
            if item.get('role') == 'end' or item.get('action') == 'end':
                return f"종료 마커 감지 (id={cur_id})"
            break

    return None


def check_lost_timeout(state: VisionState, mission_cfg: dict) -> bool:
    lost_timeout = mission_cfg['mission']['safety']['lost_timeout_s']
    with state.lock:
        seen = state.marker_seen
        last_time = state.last_marker_time
    if seen:
        return False
    if last_time == 0.0:
        return False
    return (time.time() - last_time) > lost_timeout


def run_mission(cfg: dict, mission_cfg: dict, state: VisionState,
                controller: NeoController, logger: Logger):
    """임무 흐름 실행 - 시퀀스 기반 상태 기계."""
    sequence = mission_cfg['mission']['sequence']
    settle_time = mission_cfg['mission']['post_takeoff']['settle_time_s']
    start_timeout = mission_cfg['mission'].get('start_wait_timeout_s', 60)
    on_complete = mission_cfg['mission']['on_complete']

    if not sequence:
        logger.log("⚠️ mission.yaml의 sequence가 비어있음 → 임무 중단")
        return

    # ── 1. 이륙 ──
    set_mission_state(state, 'takeoff', logger)
    controller.takeoff()
    time.sleep(settle_time)
    if not state.running: return

    # ── 2. 초기 액션 ──
    run_initial_actions(mission_cfg, state, controller, logger)
    if not state.running: return

    # ── 3. 시작 마커 대기 (role='start' 또는 sequence[0]) ──
    set_mission_state(state, 'waiting_start', logger)
    start_id = find_role_marker_id(sequence, 'start')
    if start_id is None:
        start_id = sequence[0]['id']
        logger.log(f"role='start' 마커 없음 → sequence 첫 마커(id={start_id}) 사용")

    found = wait_for_marker(state, start_id, start_timeout, logger)
    if not found:
        logger.log(f"⚠️ 시작 마커 ID={start_id} 못 찾음 → 임무 중단, 착륙")
        controller.landing()
        return
    if not state.running: return

    # ── 4. 녹화 시작 ──
    controller.capture()
    with state.lock:
        state.is_recording = True
    logger.log("● 녹화 시작")

    # ── 5. Waypoint 진행 (Vision loop이 자동으로 시퀀스 진행 + 스틱 명령 생성) ──
    set_mission_state(state, 'recording', logger)
    logger.log(f"시퀀스 진행 시작 (총 {len(sequence)}개 마커)")

    lost_since_reported = False   # 잃음 상태 로그 중복 방지

    while state.running:
        reason = check_recording_end(state, mission_cfg)
        if reason:
            logger.log(f"녹화 종료 트리거: {reason}")
            break

        # 마커 잃음 상태 관리 (즉시 종료 아님, timeout까지는 hover 유지)
        with state.lock:
            seen = state.marker_seen
            last_time = state.last_marker_time
        lost_timeout = mission_cfg['mission']['safety']['lost_timeout_s']

        if not seen and last_time > 0:
            elapsed_lost = time.time() - last_time
            if elapsed_lost > lost_timeout:
                logger.log(f"⚠️ 마커 잃은 지 {elapsed_lost:.1f}s > {lost_timeout}s "
                           f"→ 임무 중단")
                break
            elif elapsed_lost > 3.0 and not lost_since_reported:
                # 3초 이상 잃으면 한 번 알림 (그 이후는 조용히 hover 유지)
                logger.log(f"마커 잃음 (hover 유지, {lost_timeout}s까지 대기)")
                lost_since_reported = True
        else:
            lost_since_reported = False

        time.sleep(0.1)

    # ── 6. 녹화 종료 ──
    if state.is_recording and on_complete.get('stop_recording', True):
        controller.capture()
        with state.lock:
            state.is_recording = False
        logger.log("■ 녹화 종료")

    # ── 7. 착륙 ──
    set_mission_state(state, 'landing', logger)
    if on_complete.get('auto_landing', True) and state.running:
        controller.landing()
    else:
        logger.log("자동 착륙 비활성 → 호버 상태로 대기")

    set_mission_state(state, 'done', logger)


# ============================================================
# 하드코딩 모드 (마커 탐지 실패 대비 백업)
# ============================================================
def run_mission_hardcoded(cfg: dict, mission_cfg: dict, state: VisionState,
                          controller: NeoController, logger: Logger):
    """이륙 → 첫 마커 정렬 → 녹화 시작 → 하드코딩 스틱 명령 순차 실행 → 착륙."""
    sequence = mission_cfg['mission']['sequence']
    hardcoded = mission_cfg['mission'].get('hardcoded_after_start', []) or []
    settle_time = mission_cfg['mission']['post_takeoff']['settle_time_s']
    start_timeout = mission_cfg['mission'].get('start_wait_timeout_s', 60)
    on_complete = mission_cfg['mission']['on_complete']

    if not hardcoded:
        logger.log("⚠️ hardcoded_after_start 비어있음 → 임무 중단")
        return

    # ── 1. 이륙 ──
    set_mission_state(state, 'takeoff', logger)
    controller.takeoff()
    time.sleep(settle_time)
    if not state.running: return

    # ── 2. 초기 액션 ──
    run_initial_actions(mission_cfg, state, controller, logger)
    if not state.running: return

    # ── 3. 첫 마커(id=0 또는 role=start) 정렬 대기 ──
    set_mission_state(state, 'waiting_start', logger)
    start_id = find_role_marker_id(sequence, 'start')
    if start_id is None:
        start_id = sequence[0]['id'] if sequence else 0
        logger.log(f"role='start' 마커 없음 → id={start_id} 사용")

    found = wait_for_marker(state, start_id, start_timeout, logger)
    if not found:
        logger.log(f"⚠️ 시작 마커 ID={start_id} 못 찾음 → 임무 중단, 착륙")
        controller.landing()
        return
    if not state.running: return

    # ── 4. 녹화 시작 ──
    controller.capture()
    with state.lock:
        state.is_recording = True
    logger.log("● 녹화 시작 (하드코딩 모드)")

    # ── 5. 하드코딩 명령 순차 실행 ──
    set_mission_state(state, 'hardcoded_running', logger)
    total_dur = sum(cmd.get('duration', 0) for cmd in hardcoded)
    logger.log(f"하드코딩 명령 순차 실행 ({len(hardcoded)}개, 예상 총 {total_dur:.1f}s)")

    for i, cmd in enumerate(hardcoded, 1):
        if not state.running:
            logger.log("사용자 중단 → 하드코딩 시퀀스 조기 종료")
            break

        stick = cmd.get('stick', 'none')
        ox, oy = cmd.get('offset', [0, 0])
        dur = float(cmd.get('duration', 1.0))
        comment = cmd.get('comment', '')

        logger.log(f"HC [{i:>2d}/{len(hardcoded)}] {comment} | "
                   f"stick={stick} off=({ox},{oy}) dur={dur}s")

        # hover / 대기 (스틱 조작 없음)
        if stick in ('none', None) or (ox == 0 and oy == 0):
            slept = 0.0
            while slept < dur and state.running:
                time.sleep(0.1)
                slept += 0.1
        else:
            controller.execute_stick_action(stick, ox, oy, dur)

    # ── 6. 녹화 종료 ──
    if state.is_recording and on_complete.get('stop_recording', True):
        controller.capture()
        with state.lock:
            state.is_recording = False
        logger.log("■ 녹화 종료")

    # ── 7. 착륙 ──
    set_mission_state(state, 'landing', logger)
    if on_complete.get('auto_landing', True) and state.running:
        controller.landing()
    else:
        logger.log("자동 착륙 비활성 → 호버 상태로 대기")

    set_mission_state(state, 'done', logger)


# ============================================================
# 로그 요약 생성
# ============================================================
def create_summary(log_path: str) -> str:
    """원본 로그 → 요약 파일. 임무 진행 이벤트만 추출."""
    summary_path = log_path.replace('.log', '_summary.log')

    # 요약에 포함할 키워드 (하나라도 포함하는 줄만 남김)
    keywords = (
        '===== mission_state',
        '시퀀스 진행',
        '● 녹화',
        '■ 녹화',
        '이륙',
        '착륙',
        '⚠️',
        'EMERGENCY',
        'HC [',                # 하드코딩 명령
        '초기액션', '초기 액션',
        '마커 ID=', '마커 발견', '마커 잃음',
        '녹화 종료 트리거',
        '중단', '실패', '오류',
        '요약',
        # 파일 시작 메타
        'config:', 'mission:', 'source=', 'marker=', 'frame shape',
        'camera_params',
        # 시퀀스 시작
        '시퀀스 진행 시작', '하드코딩 명령 순차',
    )
    with open(log_path, 'r', encoding='utf-8') as f_in:
        lines = [ln for ln in f_in if any(kw in ln for kw in keywords)]

    with open(summary_path, 'w', encoding='utf-8') as f_out:
        f_out.writelines(lines)

    return summary_path
def main():
    parser = argparse.ArgumentParser(description="DJI Neo Waypoint Navigation")
    parser.add_argument('--config',  default='config.yaml')
    parser.add_argument('--mission', default='mission.yaml')
    parser.add_argument('--mode', choices=['vision', 'hardcoded'], default='vision',
                        help='vision: 시퀀스 기반 자율(기본). '
                             'hardcoded: 첫 마커 정렬 후 하드코딩 스틱 명령 순차 실행.')
    parser.add_argument('--dry-run', action='store_true',
                        help='이륙/스틱 조작 없이 vision 스레드만 관찰')
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    mission_cfg = load_yaml(args.mission)

    logger = Logger(cfg['logging']['log_dir'], cfg['logging']['log_prefix'])
    logger.log(f"config:  {args.config}")
    logger.log(f"mission: {args.mission}")
    logger.log(f"source={cfg['source']['type']}  family={cfg['apriltag']['family']}")
    logger.log(f"marker={cfg['marker']['physical_size_mm']}mm  "
               f"focal={cfg['camera']['focal_length_px']}px")

    # focal_length_px 미측정이면 종료
    if cfg['camera']['focal_length_px'] in (None, 0, 'null'):
        logger.log("⚠️ camera.focal_length_px 미설정. calibrate_camera.py 먼저 실행.")
        logger.close()
        sys.exit(1)

    # ── 소스, 디텍터 초기화 ──
    source = create_source(cfg)
    detector = create_detector(cfg['apriltag'])

    # 첫 프레임
    frame = None
    for _ in range(20):
        ret, f = source.read()
        if ret and f is not None:
            frame = f
            break
        time.sleep(0.15)
    if frame is None:
        logger.log("첫 프레임 획득 실패 — OBS 가상카메라 확인")
        source.release()
        logger.close()
        sys.exit(1)
    logger.log(f"frame shape: {frame.shape[1]}x{frame.shape[0]}")

    # UI 마스크
    ui_mask = None
    if cfg['source']['type'] == 'scrcpy':
        regions = cfg['ui_layout'].get('mask_regions') or []
        if regions:
            ui_mask = build_ui_mask(frame.shape, regions)
            logger.log(f"UI 마스크 활성 ({len(regions)}개)")
        else:
            logger.log("UI 마스크 비활성")

    # ── 공유 상태, 컨트롤러 ──
    state = VisionState()
    controller = None if args.dry_run else NeoController(cfg, logger=logger)

    # ── 비상 정지 ──
    hotkey = cfg.get('emergency', {}).get('hotkey', '<f12>')
    def emergency_stop():
        logger.log(f"!!!!! 비상 정지 (hotkey={hotkey}) !!!!!")
        if controller is not None:
            controller.emergency_release()
        state.running = False
    es = EmergencyStop(on_stop=emergency_stop, hotkey=hotkey)
    es.start()
    logger.log(f"비상 정지 핫키 활성: {hotkey}")

    # ── 스레드 시작 ──
    vt = threading.Thread(
        target=vision_loop,
        args=(source, detector, ui_mask, state, cfg, mission_cfg, logger),
        daemon=True,
    )
    vt.start()

    ct = None
    if controller is not None:
        ct = threading.Thread(
            target=controller.control_loop,
            args=(state, cfg),
            daemon=True,
        )
        ct.start()

    # ── 임무 실행 ──
    try:
        if args.dry_run:
            logger.log("DRY-RUN: vision 스레드만 60초 관찰 (비상정지 반응 대기)")
            for _ in range(600):
                if not state.running: break
                time.sleep(0.1)
        elif args.mode == 'hardcoded':
            logger.log(f"=== 하드코딩 모드 실행 ===")
            run_mission_hardcoded(cfg, mission_cfg, state, controller, logger)
        else:
            logger.log(f"=== 비전 시퀀스 모드 실행 ===")
            run_mission(cfg, mission_cfg, state, controller, logger)
    except KeyboardInterrupt:
        logger.log("사용자 중단 (Ctrl+C)")
    finally:
        state.running = False
        if controller is not None:
            controller.emergency_release()
        es.stop()
        vt.join(timeout=2)
        if ct is not None:
            ct.join(timeout=2)
        source.release()
        logger.log("종료")
        log_path_saved = logger.path
        logger.close()

        # ── 로그 요약 자동 생성 ──
        try:
            summary_path = create_summary(log_path_saved)
            print(f"[요약 생성 완료] {summary_path}")
        except Exception as e:
            print(f"[요약 생성 실패] {e!r}")


if __name__ == "__main__":
    main()