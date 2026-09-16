"""AprilTag 탐지 진단 도구.

vision.py의 detect_target()은 target_id로 필터링하므로 "왜 못 찾는지" 알기 어렵다.
이 스크립트는:
  1. 프레임 몇 장을 캡처해서 파일로 저장 (실제로 뭐가 찍히는지 눈으로 확인)
  2. detector에 감지된 모든 마커의 ID/family/좌표/신뢰도를 출력
  3. quad_decimate 여러 값으로도 시도 (탐지율 비교)
  4. 여러 family로도 시도 (인쇄 계열 확인)

사용:
  python diagnose.py
"""
from __future__ import annotations
import dpi_setup  # noqa: F401 - Windows DPI awareness (반드시 최상단)
import os
import sys
import time
import yaml
import cv2

sys.path.insert(0, os.path.dirname(__file__))
from stream import create_source
from pupil_apriltags import Detector


CFG_PATH = 'config.yaml'
OUT_DIR = 'debug'


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(CFG_PATH, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    print(f"config: source={cfg['source']['type']}, family={cfg['apriltag']['family']}, "
          f"target_id={cfg['mission']['start_marker_id']}")

    source = create_source(cfg)

    # ---- 프레임 캡처 (버퍼 안정화 위해 여러 장 읽고 마지막 것 사용) ----
    frame = None
    for _ in range(10):
        ret, f = source.read()
        if ret and f is not None:
            frame = f
        time.sleep(0.1)
    if frame is None:
        print("프레임 획득 실패")
        return

    h, w = frame.shape[:2]
    print(f"프레임 크기: {w}x{h}")

    # ---- 원본과 grayscale 저장 ----
    cv2.imwrite(f'{OUT_DIR}/frame_bgr.png', frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(f'{OUT_DIR}/frame_gray.png', gray)
    print(f"저장: {OUT_DIR}/frame_bgr.png, {OUT_DIR}/frame_gray.png")

    # ---- 1: config의 family + quad_decimate 변주 ----
    family = cfg['apriltag']['family']
    print(f"\n===== family={family}, 여러 quad_decimate로 시도 =====")
    for qd in [1.0, 2.0, 4.0]:
        det = Detector(families=family, nthreads=2, quad_decimate=qd)
        detections = det.detect(gray)
        print(f"  quad_decimate={qd}: 감지 수={len(detections)}")
        for d in detections:
            print(f"    → ID={d.tag_id:3d}  family={d.tag_family.decode() if isinstance(d.tag_family, bytes) else d.tag_family}  "
                  f"center=({d.center[0]:.0f},{d.center[1]:.0f})  "
                  f"decision_margin={d.decision_margin:.1f}")

    # ---- 2: 여러 family로 시도 (인쇄한 계열 확인) ----
    families_to_try = ['tag36h11', 'tag25h9', 'tag16h5', 'tagStandard41h12', 'tagCircle21h7']
    print(f"\n===== 여러 family로 시도 (quad_decimate=1.0 고정) =====")
    for fam in families_to_try:
        try:
            det = Detector(families=fam, nthreads=2, quad_decimate=1.0)
            detections = det.detect(gray)
            print(f"  {fam:20s}: 감지 수={len(detections)}", end='')
            if detections:
                ids = [d.tag_id for d in detections]
                print(f"   IDs={ids}")
            else:
                print()
        except Exception as e:
            print(f"  {fam:20s}: 에러 {e}")

    # ---- 3: 감지된 마커에 표시해서 시각화 저장 ----
    det = Detector(families=family, nthreads=2, quad_decimate=1.0)
    detections = det.detect(gray)
    vis = frame.copy()
    for d in detections:
        pts = d.corners.astype(int)
        cv2.polylines(vis, [pts], True, (0, 255, 0), 3)
        cx, cy = int(d.center[0]), int(d.center[1])
        cv2.putText(vis, f"ID={d.tag_id}", (cx-40, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
    cv2.imwrite(f'{OUT_DIR}/frame_detected.png', vis)
    print(f"\n저장: {OUT_DIR}/frame_detected.png (감지 결과 시각화)")

    source.release()


if __name__ == "__main__":
    main()