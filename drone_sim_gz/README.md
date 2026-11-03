# Drone Simulation — gz Harmonic (Starling 2)

[`drone_sim/`](../drone_sim/)(PX4 v1.14 + Gazebo Classic 11, `iris_starling`)의 후속이다. 새 Gazebo(**gz Harmonic**)와 **PX4 v1.15.4**에서 Starling 2를 다시 모델링했다. QVIX 식물공장 재배단 사이 좁은 통로(폭 0.65~0.75 m)를 나는 시험을 한다. 실기 비행 코드 `path_flight_phase1_v13.py`([`drone_sim/flight_code/`](../drone_sim/flight_code/))는 수정하지 않고 그대로 실행한다.

> **시뮬 결과다. 실기에서는 검증하지 않았다.** 센서 잡음 모델은 일반값이고 실측 보정 전이다. 기체 IMU 물리 위치·무게중심·관성은 실측하지 않았다.

## 스택
| 계층 | 버전 |
|---|---|
| Container A `starling-gz` | Ubuntu 22.04, PX4 v1.15.4 SITL, gz Harmonic(gz-sim8), GPU 렌더 (`docker/Dockerfile.gz`) |
| Container B `starling-flight` | Ubuntu 18.04, Python 3.6.9, MAVSDK-Python 0.12.0 = 실기와 같음 (`docker/Dockerfile.flight`) |
| 연결 | 둘 다 `--network host`, MAVLink UDP |
| 추정 | GPS·지자기 없음. gz 오도메트리 → `vehicle_visual_odometry`(VIO 대역, 오차 주입 가능). 실기와 같은 `EKF2_EV_CTRL 7`로 시험 |

## 폴더
| 폴더 | 내용 |
|---|---|
| `gz/` | 기본 `starling2` 모델, QVIX 월드·모델, 에어프레임 `4201_gz_starling2`, 비행 격자. **[`gz/README.md`](gz/README.md)가 정본 설명**(형상·모터·추정·실행·실기 파라미터 반영) |
| `scripts/` | 모델 생성기 `make_gz_model.py`, PX4 설치 `install_into_px4_gz.sh`, SITL 실행·종료(`run_sitl*.sh`, `stop_sitl*.sh`), 격자 경로 컴파일러 `grid_mission.py`, 진실값 도구 |
| `cad_pipeline/` | ModalAI STEP → 기체 메시(STL) 변환 |
| `docker/` | 두 컨테이너의 Dockerfile·빌드·실행 스크립트 |
| `v3_marker/` | 현재 작업. ToF·AprilTag 기반 위치·헤딩 보정, 지도 기반 추정기, 측량 비행, 안전장치, 실기 입력 도구 |

`v3_marker/` 주요 구성 (진입점 목록: [`v3_marker/scripts/ENTRYPOINTS.md`](v3_marker/scripts/ENTRYPOINTS.md))
```
[실기와 같은 구조]  path_flight_phase1_v13.py (수정 안 함)
                       ↑ 피드백 − c          ↓ 목표 + c
                wk_mission.py 보정층 ── loc2d.py (EKF: px, py, yaw; 지도 = 설계도)
                       ↑                            ↑ 태그 / ToF
[시뮬 센서]   wall_sense.py (ToF 면 맞춤)    tag_detect.py (AprilTag)    vio_relay.py (VIO + 오차 주입)
[실기에서는]   VOXL tof_pc                     voxl-tag-detector              OpenVINS
[착륙 후]     survey_map.py (일괄 추정 → 보정 지도) → survey_eval.py (진실값 비교, 시뮬 전용)
```
- 기체 모델 `starling2_tof`(인스턴스 2~5 기본): 실측 무게 297.5 g, IMU를 실기 `EKF2_IMU_POS` 위치에 두고, 실물 카메라·ToF 규격(2026-10-30 기체 자료)을 반영했다. 이전 모델 변형은 `v3_marker/gz/variants/`.
- 실기 PX4 파라미터는 2026-10-27 기체 덤프(`starling2_autonomy/sim/handoff_2026-10-27/`)에서 반영했다.
- 결과 요약 `v3_marker/results/REPORT.md`.

## 포함하지 않은 것
- **기체 메시** `starling2_body.stl`, `starling2_prop_a/b.stl`: ModalAI 원본 CAD에서 만든 파일이라 넣지 않았다. `gz/models/starling2/meshes/`와 `v3_marker/gz/models/starling2_tof/meshes/`가 비어 있다.
  - ModalAI가 배포하는 Starling 2 STEP을 받아 `cad_pipeline/10_pipeline.py`로 만든다. 만든 메시는 `make_gz_model.py`가 읽는 `repo_export/airframe/starling2/meshes/`에 둔다.
- PX4-Autopilot 트리(v1.15.4를 받아 `install_into_px4_gz.sh`로 모델·에어프레임을 넣는다), Python venv, 비행 로그·원시 데이터, 작업 기록.

## 경로 주의
스크립트는 개발 서버의 레이아웃 기준이다. 예를 들어 `~/starling_sim/repo_export/…`, `~/starling_sim/v3_marker/…`, `~/starling_sim/PX4-Autopilot`, 로그는 `/990data/…`를 쓴다. 이 폴더의 대응은 다음과 같다: `gz/`·`scripts/` = `repo_export/gz`·`repo_export/scripts`, `v3_marker/` = `v3_marker`. 다른 곳에서 돌리려면 `docker/run_gz.sh`의 마운트와 스크립트 안의 경로를 맞춰야 한다.

## 빠른 실행 (서버 레이아웃 기준)
```bash
docker/build_gz.sh && docker/build_flight.sh      # 이미지
docker/run_gz.sh && docker/run_flight.sh          # 컨테이너
# starling-gz 안: 모델 설치 후 SITL
scripts/install_into_px4_gz.sh
scripts/run_sitl.sh qvix headless
# v3_marker 비행 1회 (호스트, 인스턴스 2~5)
v3_marker/scripts/wk_fly.sh
```
자세한 순서와 옵션은 `gz/README.md`와 `v3_marker/scripts/ENTRYPOINTS.md`를 본다. 비행 성공은 EKF 값이 아니라 gz 진실 위치로 판정한다(`grid_verify.py`, `loc_compare.py`).
