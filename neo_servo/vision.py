"""AprilTag 감지 + 3D pose + 시퀀스 진행 (mission_state 인식).

★ 중요 개선: mission_state에 따라 vision_loop의 동작이 분기됨.

  - recording:      시퀀스 진행 + 안전 오버라이드 + waypoint action
  - waiting_start:  시작 마커만 감지, 스틱은 항상 hover, sequence_index 진행 안 함
  - 그 외 상태:     감지 로그만, 스틱 hover, sequence_index 그대로

이전 버전은 mission_state 무관하게 항상 진행 로직을 돌려서
이륙 중에 시퀀스가 조기 진행되고 waiting_start에서 스틱 명령이 발행되는 버그가 있었음.
"""
from __future__ import annotations
import threading
import time
from typing import Optional

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
except ImportError as e:
    raise ImportError(
        "AprilTag 탐지를 위해 pupil-apriltags 필요: pip install pupil-apriltags"
    ) from e


# ============================================================
# 공유 상태
# ============================================================
class VisionState:
    def __init__(self):
        self.lock = threading.Lock()

        self.marker_seen: bool = False
        self.current_marker_id: int = -1

        self.dist_mm: float = 0.0
        self.lateral_mm: float = 0.0
        self.vertical_mm: float = 0.0

        self.in_safe_range: bool = False
        self.safety_override: bool = False

        # 스틱 명령
        self.active_stick: Optional[str] = None
        self.stick_offset: tuple[int, int] = (0, 0)

        # 마커 잃음 시 유지할 마지막 waypoint 방향
        self.last_active_waypoint_stick: Optional[str] = None
        self.last_active_waypoint_offset: tuple[int, int] = (0, 0)

        self.sequence_index: int = 0
        self.sequence_complete: bool = False

        self.mission_state: str = 'idle'
        self.is_recording: bool = False

        self.last_marker_time: float = 0.0

        self.running: bool = True


# ============================================================
# Detector / 마스크
# ============================================================
def create_detector(apriltag_cfg: dict) -> Detector:
    d = apriltag_cfg['detector']
    return Detector(
        families=apriltag_cfg['family'],
        nthreads=d['nthreads'],
        quad_decimate=d['quad_decimate'],
        quad_sigma=d['quad_sigma'],
        refine_edges=d['refine_edges'],
        decode_sharpening=d['decode_sharpening'],
    )


def build_ui_mask(frame_shape: tuple, mask_regions: list) -> np.ndarray:
    h, w = frame_shape[:2]
    mask = np.ones((h, w), dtype=np.uint8) * 255
    for (x, y, mw, mh) in mask_regions:
        y2, x2 = min(y + mh, h), min(x + mw, w)
        x, y = max(x, 0), max(y, 0)
        mask[y:y2, x:x2] = 0
    return mask


# ============================================================
# Action → 스틱 명령 매핑
# ============================================================
_ACTION_MAP = {
    'forward':  ('right',  0, -1),
    'backward': ('right',  0, +1),
    'left':     ('right', -1,  0),
    'right':    ('right', +1,  0),
    'ascend':   ('left',   0, -1),
    'descend':  ('left',   0, +1),
    'hover':    (None,     0,  0),
    'end':      (None,     0,  0),
}


def action_to_stick(wp: Optional[dict], radius: int):
    if wp is None:
        return None, (0, 0)
    action = wp.get('action', 'hover')
    speed = int(wp.get('speed', 0))
    speed = min(speed, radius)
    stick, dx, dy = _ACTION_MAP.get(action, (None, 0, 0))
    return stick, (dx * speed, dy * speed)


def find_start_marker_id(sequence: list) -> Optional[int]:
    """role='start'인 첫 항목의 id. 없으면 sequence[0]의 id."""
    for item in sequence:
        if item.get('role') == 'start':
            return item['id']
    return sequence[0]['id'] if sequence else None


def _needs_alignment(wp: dict, sequence: list = None, idx: int = None) -> bool:
    """이 waypoint가 정렬 완료 후에만 지나갈 수 있는지.

    - `require_alignment: true`가 명시된 경우
    - id < 10 (동작 마커: 0=start, 1=상승 등, 9=end 등)
    - **직전 시퀀스 항목이 정렬 필수 마커였다** (동작 마커 이후 첫 경유 마커에서 재정렬)
    """
    if wp.get('require_alignment', False):
        return True
    if wp['id'] < 10:
        return True
    # 직전 인덱스 검사
    if sequence is not None and idx is not None and idx > 0:
        prev = sequence[idx - 1]
        if prev.get('require_alignment', False) or prev['id'] < 10:
            return True
    return False


def _is_aligned(z: float, x: float, y: float, safety: dict) -> bool:
    """모든 축이 tolerance 안에 있으면 정렬 완료."""
    return (safety['min_distance_mm'] <= z <= safety['max_distance_mm'] and
            abs(x) <= safety['lateral_tolerance_mm'] and
            abs(y) <= safety['vertical_tolerance_mm'])


def _apply_deadband(val: int, min_effective: int) -> int:
    """스틱 값이 0이 아니지만 데드밴드보다 작으면 데드밴드로 끌어올림 (부호 유지).

    가상 조이스틱은 반경 대비 일정 크기 이하 명령을 무시함 (Neo 실측 ~25-30px).
    작은 편차에도 실제 반응하도록 min_effective 이상 값을 보장.
    """
    if val == 0:
        return 0
    if abs(val) < min_effective:
        return min_effective if val > 0 else -min_effective
    return val


# ============================================================
# Pose 추출
# ============================================================
def _extract_pose(det) -> dict:
    t = det.pose_t.flatten()
    return {
        'id': int(det.tag_id),
        'x_mm': float(t[0]) * 1000.0,
        'y_mm': float(t[1]) * 1000.0,
        'z_mm': float(t[2]) * 1000.0,
        'center_px': (float(det.center[0]), float(det.center[1])),
        'corners': det.corners,
        'decision_margin': float(det.decision_margin),
    }


def _detect_all(frame, detector, camera_params, tag_size_m, ui_mask):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if ui_mask is not None:
        gray = cv2.bitwise_and(gray, gray, mask=ui_mask)
    return detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=tag_size_m,
    )


# ============================================================
# 상태별 처리
# ============================================================
def _handle_idle_or_moving(state: VisionState, detections, logger):
    """이륙/초기액션/착륙/완료 등: 감지 로그만, 스틱 hover 강제."""
    with state.lock:
        state.marker_seen = False
        state.safety_override = False
        state.active_stick = None
        state.stick_offset = (0, 0)


def _handle_waiting_start(state: VisionState, detections,
                          start_marker_id: int, logger):
    """시작 마커만 감지 확인. 스틱은 hover. 시퀀스 진행 안 함."""
    found_det = None
    for d in detections:
        if int(d.tag_id) == start_marker_id:
            found_det = d
            break

    with state.lock:
        state.active_stick = None
        state.stick_offset = (0, 0)
        state.safety_override = False

        if found_det is not None:
            m = _extract_pose(found_det)
            state.marker_seen = True
            state.current_marker_id = m['id']
            state.dist_mm = m['z_mm']
            state.lateral_mm = m['x_mm']
            state.vertical_mm = m['y_mm']
            state.last_marker_time = time.time()
        else:
            state.marker_seen = False


def _handle_recording(state: VisionState, detections, sequence, seq_ids,
                      safety, radius, kp_lateral, kp_vertical, kp_forward,
                      backoff_px, max_override_px, min_effective_px,
                      continue_lost_s, logger):
    """정상 시퀀스 진행 + 안전 오버라이드 + waypoint action.

    안전 오버라이드 우선순위:
      1. z < min_distance                  : 긴급 후진 (모든 경우)
      2. z > max_distance                  : 전진 (forward/backward 제외)
      3. |y| > y_tolerance                 : 상승/하강 (ascend/descend 제외)
      4. |x| > x_tolerance                 : 좌우 (left/right 제외)
      5. 정상 waypoint action

    마커 잃음 시: continue_lost_s 초 이내면 마지막 waypoint 방향 유지, 이후 hover.
    """
    with state.lock:
        current_idx = state.sequence_index

    if current_idx >= len(sequence):
        with state.lock:
            state.sequence_complete = True
            state.active_stick = None
            state.stick_offset = (0, 0)
        return

    # 감지된 마커들을 시퀀스 인덱스에 매핑
    matches: dict[int, object] = {}
    for d in detections:
        tid = int(d.tag_id)
        for i in range(current_idx, len(sequence)):
            if seq_ids[i] == tid:
                if i not in matches:
                    matches[i] = d
                break

    with state.lock:
        if not matches:
            # 마커 감지 없음. 최근 waypoint 방향 유지 여부 결정.
            state.marker_seen = False
            state.safety_override = False

            now = time.time()
            lost_dur = now - state.last_marker_time if state.last_marker_time > 0 else 9999

            if (lost_dur <= continue_lost_s and
                state.last_active_waypoint_stick is not None):
                # 최근 방향 유지
                state.active_stick = state.last_active_waypoint_stick
                state.stick_offset = state.last_active_waypoint_offset
            else:
                # 시간 초과 or 저장된 방향 없음 → hover
                state.active_stick = None
                state.stick_offset = (0, 0)
            return

        # 처리할 마커 결정
        lowest_idx = min(matches.keys())
        lowest_wp = sequence[lowest_idx]

        if _needs_alignment(lowest_wp, sequence, lowest_idx):
            lowest_det = matches[lowest_idx]
            m_low = _extract_pose(lowest_det)
            if _is_aligned(m_low['z_mm'], m_low['x_mm'], m_low['y_mm'], safety):
                # 정렬 완료 → 다음으로 진행 가능
                best_idx = max(matches.keys())
                best_det = matches[best_idx]
            else:
                # 정렬 미완 → 이 마커에 머무름
                best_idx = lowest_idx
                best_det = lowest_det
        else:
            # 정렬 불필요 → 최대 인덱스로 스킵 가능
            best_idx = max(matches.keys())
            best_det = matches[best_idx]

        if best_idx > current_idx:
            if logger:
                logger.log(
                    f"[vision] 시퀀스 진행: idx {current_idx}→{best_idx} "
                    f"(id {seq_ids[current_idx]}→{seq_ids[best_idx]})"
                )
        state.sequence_index = best_idx

        m = _extract_pose(best_det)
        state.marker_seen = True
        state.current_marker_id = m['id']
        state.dist_mm = m['z_mm']
        state.lateral_mm = m['x_mm']
        state.vertical_mm = m['y_mm']
        state.last_marker_time = time.time()

        z, x, y = m['z_mm'], m['x_mm'], m['y_mm']
        wp = sequence[best_idx]
        action = wp.get('action', 'hover')

        x_moving = {'left', 'right'}
        y_moving = {'ascend', 'descend'}
        z_moving = {'forward', 'backward'}

        # 정렬 필수 마커에서 정렬 미완이면 waypoint action 대신 hover
        force_hover_action = (_needs_alignment(wp, sequence, best_idx) and
                              not _is_aligned(z, x, y, safety))

        # 우선순위 1: 너무 가까움 → 긴급 후진
        if z < safety['min_distance_mm']:
            state.safety_override = True
            state.in_safe_range = False
            state.active_stick = 'right'
            state.stick_offset = (0, +backoff_px)

        # 우선순위 2: 너무 멀음 → 전진 (앞뒤 waypoint에는 미적용)
        elif action not in z_moving and z > safety['max_distance_mm']:
            state.safety_override = True
            state.in_safe_range = False
            over = z - safety['max_distance_mm']
            # 전진 = 우스틱 y-  (음수)
            oy_raw = int(-kp_forward * over)
            oy = int(np.clip(oy_raw, -max_override_px, +max_override_px))
            oy = _apply_deadband(oy, min_effective_px)
            state.active_stick = 'right'
            state.stick_offset = (0, oy)

        # 우선순위 3: y축 이탈 (상하 정렬)
        elif action not in y_moving and abs(y) > safety['vertical_tolerance_mm']:
            state.safety_override = True
            state.in_safe_range = (safety['min_distance_mm'] <= z <=
                                   safety['max_distance_mm'])
            oy_raw = int(kp_vertical * y)
            oy = int(np.clip(oy_raw, -max_override_px, +max_override_px))
            oy = _apply_deadband(oy, min_effective_px)
            state.active_stick = 'left'
            state.stick_offset = (0, oy)

        # 우선순위 4: x축 이탈 (좌우 정렬)
        elif action not in x_moving and abs(x) > safety['lateral_tolerance_mm']:
            state.safety_override = True
            state.in_safe_range = (safety['min_distance_mm'] <= z <=
                                   safety['max_distance_mm'])
            ox_raw = int(kp_lateral * x)
            ox = int(np.clip(ox_raw, -max_override_px, +max_override_px))
            ox = _apply_deadband(ox, min_effective_px)
            state.active_stick = 'right'
            state.stick_offset = (ox, 0)

        # 우선순위 5: 정상 waypoint action
        else:
            state.safety_override = False
            state.in_safe_range = (safety['min_distance_mm'] <= z <=
                                   safety['max_distance_mm'])
            if force_hover_action:
                # 정렬 필수 마커에서 정렬 미완 (안전 오버라이드로도 안 걸린 경우)
                state.active_stick = None
                state.stick_offset = (0, 0)
            else:
                stick, off = action_to_stick(wp, radius)
                state.active_stick = stick
                state.stick_offset = off
                # 마커 잃음 시 유지할 방향 저장 (실제 이동 명령일 때만)
                if stick is not None and (off[0] != 0 or off[1] != 0):
                    state.last_active_waypoint_stick = stick
                    state.last_active_waypoint_offset = off


# ============================================================
# Vision 스레드 메인 루프
# ============================================================
def vision_loop(source, detector, ui_mask, state: VisionState,
                cfg: dict, mission_cfg: dict, logger=None):
    """mission_state별로 처리 분기."""
    marker_size_m = cfg['marker']['physical_size_mm'] / 1000.0
    focal_px = float(cfg['camera']['focal_length_px'])
    safety = mission_cfg['mission']['safety']
    sequence = mission_cfg['mission']['sequence']
    seq_ids = [item['id'] for item in sequence]
    start_marker_id = find_start_marker_id(sequence)
    radius = cfg['ui_layout']['stick_radius']
    period = 1.0 / cfg['control']['vision_hz']

    kp_lateral = safety['kp_lateral']
    kp_vertical = safety.get('kp_vertical', kp_lateral)
    kp_forward = safety.get('kp_forward', kp_lateral)
    backoff_px = safety['backoff_stick_px']
    max_override_px = safety.get('max_override_stick_px', 40)
    min_effective_px = safety.get('min_effective_stick_px', 25)
    continue_lost_s = safety.get('continue_on_lost_s', 10.0)

    camera_params = None

    while state.running:
        t0 = time.time()
        ret, frame = source.read()
        if not ret or frame is None:
            time.sleep(period)
            continue

        # 최초 프레임: camera_params 완성
        if camera_params is None:
            h, w = frame.shape[:2]
            camera_params = [focal_px, focal_px, w / 2.0, h / 2.0]
            if logger:
                logger.log(f"[vision] camera_params fx=fy={focal_px:.0f} "
                           f"cx={w/2:.0f} cy={h/2:.0f}, sequence_len={len(sequence)}, "
                           f"start_marker_id={start_marker_id}")

        detections = _detect_all(frame, detector, camera_params,
                                  marker_size_m, ui_mask)

        # 현재 mission_state 스냅샷
        with state.lock:
            ms = state.mission_state

        # 상태별 분기
        if ms == 'recording':
            _handle_recording(state, detections, sequence, seq_ids,
                              safety, radius, kp_lateral, kp_vertical, kp_forward,
                              backoff_px, max_override_px, min_effective_px,
                              continue_lost_s, logger)
        elif ms == 'waiting_start':
            _handle_waiting_start(state, detections, start_marker_id, logger)
        else:
            # idle / takeoff / init_actions / landing / done
            _handle_idle_or_moving(state, detections, logger)

        # 로그 (recording 상태에서만 상세 로그, 다른 상태는 축약)
        if logger and detections:
            if ms == 'recording':
                with state.lock:
                    if state.marker_seen:
                        logger.log(
                            f"[vision] idx={state.sequence_index:>2d} "
                            f"id={state.current_marker_id:>3d}  "
                            f"z={state.dist_mm:>6.0f}mm "
                            f"x={state.lateral_mm:>+6.0f}mm "
                            f"y={state.vertical_mm:>+6.0f}mm  "
                            f"safety_ov={state.safety_override}  "
                            f"stick={state.active_stick} off={state.stick_offset}"
                        )
            elif ms == 'waiting_start':
                ids = sorted({int(d.tag_id) for d in detections})
                with state.lock:
                    if state.marker_seen:
                        logger.log(f"[vision] waiting_start: start_id={start_marker_id} 감지 "
                                   f"(z={state.dist_mm:.0f}mm x={state.lateral_mm:+.0f}mm)")
                    # 시작 마커가 아닌 다른 마커만 있으면 조용히
            # 그 외 상태에서는 로그 안 남김 (스팸 방지)

        elapsed = time.time() - t0
        if elapsed < period:
            time.sleep(period - elapsed)