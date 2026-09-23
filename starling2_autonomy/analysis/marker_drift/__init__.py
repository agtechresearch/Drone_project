"""marker_drift - AprilTag 기반 드리프트 측정 후처리 파이프라인 (docs/09).

기체 없이 로컬 PC에서 돈다. 구성:
  geometry   좌표계·PnP·프레임 변환 (순수 수학, 테스트 대상)
  markers    마커 배치 파일(YAML) 로드·기본 배치 생성
  intrinsics 카메라 내부 파라미터 로드 (자체 YAML / OpenCV FileStorage / fisheye)
  detect     영상 -> 프레임별 태그 검출 + 카메라 포즈 CSV
  logs       기체 로그(v14 CSV, 일반 포즈 CSV) 로더
  drift      시계열 정합·프레임 정렬·드리프트 지표
  report     그림·조건 비교·표본 수 산정
  calibrate  체커보드 캘리브레이션

좌표계 요약 (자세한 정의는 geometry.py 상단):
  M  마커 프레임: x = 마커 열 방향(진행 방향), y = 벽 안쪽, z = 위. 마커는 y=0 평면에 -y 방향을 향해 붙어 있다.
  T  AprilTag 프레임: x 오른쪽, y 아래, z 태그 안쪽 (AprilTag 공식 규약).
  C  OpenCV 카메라 프레임: x 오른쪽, y 아래, z 전방.
  L  기체 로컬 NED 프레임 (VIO / PX4 local position).
"""

__version__ = "0.1.0"
