"""s2flight — Starling 2 (VOXL 2 / PX4 / MAVSDK) 실험 비행용 공용 패키지.

기체 파이썬 3.6 + mavsdk 0.12 환경에서 돈다. numpy·cv2 없이 표준 라이브러리만 쓴다.

모듈
    util        시계(단조시계)·각도·좌표 도우미, 로그
    config      FlightConfig: 비행 공통 파라미터 + argparse 연결
    geofence    추종 오차 / 홈 기준 지오펜스 (v14 동작 그대로)
    pose        voxl-inspect-pose 파서 + 파이프 리더 태스크
    recorder    동기 CSV·이벤트·단계별 샘플 기록, 파일 저장
    vehicle     Vehicle: 연결·프리플라이트·arm/offboard·호버·이동·회전·착륙·비상
    voxl_logger voxl-logger 기록 시작/정지 (subprocess)
    mpa         MPA(/run/mpa) 파이프 구독 공용 클라이언트
    tags        tag_detections 파이프 레코드 파서·프레임 묶기·asyncio 피드
    align       태그 두 개 연결선 → yaw 오차, 폐루프 정렬 상태기계
    align_stage 정렬 단계 실행기 (Vehicle + TagFeed + AlignController)
    sim         SITL(drone_sim_gz) 입력 어댑터: MAVSDK 텔레메트리 포즈, UDP 태그 피드 (--sim)
    drift       태그 기반 실시간 드리프트 기록(DriftMonitor), 태그 중앙 정지 조건, 시작점 센터링 (마커 드리프트 실험)

출처
    vehicle/geofence/pose/recorder 는 flight/path_flight_phase1_v14.py 의 DroneController 를
    동작 변경 없이 분해해 옮긴 것이다. tags/align/mpa 는 flight/mpa_tag_detections.py,
    flight/tag_align.py, flight/mpa_point_cloud.py 의 사본이다. 원본 파일은 그대로 두었고
    앞으로의 실험 코드는 이 패키지를 쓴다.
"""

__version__ = "0.2.1"
