"""카메라 초점거리 캘리브레이션 (핀홀 카메라 모델).

원리:
  Z (거리, mm) = f (초점거리, px) × S (마커 실측, mm) / s (화면 마커, px)
  → f = s × Z / S

방법:
  1. AprilTag를 정확히 아는 거리에 놓기
  2. 이 스크립트가 여러 프레임에서 마커 픽셀 크기 자동 측정
  3. 사용자가 다른 거리에서 반복 → 여러 f 값 평균/편차 계산
  4. 결과를 config.yaml의 camera.focal_length_px에 반영

측정 오차:
  줄자 오차 ±5mm, 마커 인쇄 오차 ±1mm 정도면
  30cm/50cm/70cm 3회 측정 시 f 상대오차 3% 이내.
"""
from __future__ import annotations
import dpi_setup  # noqa: F401 - Windows DPI awareness (반드시 최상단)
import os
import sys
import time

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(__file__))
from stream import create_source
from pupil_apriltags import Detector

CFG_PATH = 'config.yaml'


def measure_marker_size_px(source, detector, target_id, n_samples=15):
    """여러 프레임에서 마커 4변 평균 픽셀 크기 반복 측정."""
    sizes = []
    attempts = 0
    while len(sizes) < n_samples and attempts < 80:
        ret, frame = source.read()
        attempts += 1
        if not ret or frame is None:
            time.sleep(0.05)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for d in detector.detect(gray):
            if d.tag_id == target_id:
                c = d.corners
                # 4변 평균 (마커가 기울어져도 안정적)
                sides = [
                    np.linalg.norm(c[0] - c[1]),
                    np.linalg.norm(c[1] - c[2]),
                    np.linalg.norm(c[2] - c[3]),
                    np.linalg.norm(c[3] - c[0]),
                ]
                sizes.append(float(np.mean(sides)))
                break
        time.sleep(0.05)
    if not sizes:
        return None
    return float(np.mean(sizes)), float(np.std(sizes)), len(sizes)


def main():
    with open(CFG_PATH, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    marker_size_mm = cfg['marker']['physical_size_mm']
    target_id = cfg['mission']['start_marker_id']

    print("=" * 56)
    print("카메라 초점거리 캘리브레이션")
    print("=" * 56)
    print(f"마커 실제 크기: {marker_size_mm}mm (config.marker.physical_size_mm)")
    print(f"측정 대상 ID  : {target_id}")
    print()
    print("측정 방법:")
    print("  1. 줄자로 카메라 렌즈 ~ 마커 표면 거리 재기")
    print("  2. 그 거리를 mm로 입력 (엔터)")
    print("  3. 자동으로 15프레임에서 마커 크기 측정")
    print("  4. 3회 이상 다른 거리에서 반복 권장 (예: 300, 500, 700mm)")
    print("  5. 다 마치면 q 입력")
    print()

    source = create_source(cfg)
    # 캘리브레이션은 정밀도가 중요하므로 quad_decimate=1.0 강제
    detector = Detector(
        families=cfg['apriltag']['family'],
        nthreads=cfg['apriltag']['detector']['nthreads'],
        quad_decimate=1.0,
        refine_edges=1,
    )

    measurements = []  # (distance_mm, size_mean, size_std, f_estimate)

    while True:
        raw = input("\n마커까지 거리 mm (q로 종료): ").strip()
        if raw.lower() in ('q', 'quit', 'exit'):
            break
        try:
            distance_mm = float(raw)
        except ValueError:
            print("  → 숫자로 입력. 예: 500")
            continue
        if distance_mm <= 0:
            print("  → 양수여야 함.")
            continue

        print(f"  거리 {distance_mm}mm에서 측정 중...", end='', flush=True)
        result = measure_marker_size_px(source, detector, target_id)
        if result is None:
            print(" 실패")
            print("  → 마커가 화면에 잘 보이고 target_id가 맞는지 확인.")
            continue
        size_mean, size_std, n = result
        f_est = size_mean * distance_mm / marker_size_mm
        measurements.append((distance_mm, size_mean, size_std, f_est))
        print(f"\r  ✓ 거리 {distance_mm:>5.0f}mm  "
              f"마커 {size_mean:>6.1f}±{size_std:>4.1f} px  "
              f"(n={n})  →  f = {f_est:>7.1f} px")

    source.release()

    if len(measurements) < 2:
        print("\n측정 2회 이상 필요. 결과 출력 생략.")
        return

    focals = np.array([m[3] for m in measurements])
    f_mean = float(np.mean(focals))
    f_std  = float(np.std(focals))
    rel = 100 * f_std / f_mean

    print()
    print("=" * 56)
    print("측정 결과 요약")
    print("=" * 56)
    print(f"{'거리(mm)':>10} {'마커(px)':>10} {'std':>6} {'f(px)':>10}")
    for d, s, ss, ff in measurements:
        print(f"{d:>10.0f} {s:>10.1f} {ss:>6.1f} {ff:>10.1f}")
    print("-" * 56)
    print(f"초점거리 f = {f_mean:.1f} ± {f_std:.1f} px")
    print(f"상대오차   = {rel:.1f}%")
    if rel > 10:
        print("⚠️  상대오차 10% 초과 — 줄자 재측정 또는 마커 인쇄 확인 권장.")
    elif rel > 5:
        print("△  상대오차 5-10% — 사용 가능하지만 개선 여지 있음.")
    else:
        print("✓  상대오차 5% 이하 — 양호.")

    print()
    print("config.yaml의 camera 섹션에 반영:")
    print(f"""
camera:
  focal_length_px: {f_mean:.1f}
""")


if __name__ == "__main__":
    main()