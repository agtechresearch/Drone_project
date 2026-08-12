# Drone Simulation (Starling 2)

세종대학교 agtechlab의 식물공장 자율비행 드론 프로젝트 중 시뮬레이션 파트다.  
실기 flight_code를 수정 없이 SITL 환경에서 검증하고, 나아가 sim-to-real gap을 측정하는 것이 목표다.

## 개요

- **대상 드론**: ModalAI Starling 2 (VOXL2 + voxl-px4 1.14.0-2.0.133)
- **응용 환경**: 태백 NextOn 식물공장 QVIX R Module (딸기 재배)
- **비행 조건**: GPS-denied 실내, VIO 기반 위치추정
- **flight_code**: MAVSDK-Python 0.12.0 + Python 3.6.9
- **주요 로직**: MAVSDK Offboard PositionNedYaw + voxl-inspect-pose 기반 도달 판정

## 시뮬레이션 아키텍처

두 개의 Docker 컨테이너를 UDP로 연결하여 실기 환경과 동일한 통신 구조를 재현한다. Container A는 QVIX 3D 맵을 로드한 Gazebo 안에서 iris_starling airframe으로 SITL을 돌리고, Container B는 실기와 동일한 flight_code를 실행한다.

```
┌─────────────────────────────────┐         ┌─────────────────────────────────┐
│  Container A: starling-sitl      │  UDP    │  Container B: starling-flight    │
│  Ubuntu 22.04                    │  <---> │  Ubuntu 18.04                    │
│  PX4 v1.14.0 + Gazebo Classic 11 │  14540 │  Python 3.6.9 + MAVSDK 0.12.0    │
│  iris_starling airframe          │        │  flight_code (--sim mode)         │
│  QVIX R Module world             │        │  mavsdk_server                   │
│  VNC (GUI 원격 접근)              │         │                                  │
└─────────────────────────────────┘         └─────────────────────────────────┘
```

- Container A는 실기 하드웨어(voxl-px4) + 태백 QVIX 시설을 함께 대체
- Container B는 실기와 동일한 flight_code 실행 환경
- **flight_code 관점에서 SITL과 실기는 CLI 플래그 (`--sim`) 유무만 다름**

## 스택 확정 근거

실기 `voxl-px4 1.14.0-2.0.133`이 PX4 v1.14.0 베이스에 ModalAI 패치 133을 얹은 것이다. SITL도 이와 정확히 같은 PX4 태그를 써야 파라미터 이름(`EKF2_GPS_CTRL`, `SYS_HAS_GPS` 등)과 MAVLink 메시지 스펙이 일치한다.

| 계층 | 버전 | 이유 |
|---|---|---|
| Host OS | Ubuntu 22.04.5 | PX4 v1.14 공식 지원 |
| ROS2 | Humble | 22.04 페어 |
| Gazebo | Classic 11 | PX4 v1.14 완전 지원, 이미 서버에 설치됨 |
| PX4 | v1.14.0 | voxl-px4 base와 정렬 |
| Python (SITL) | 3.10 | Ubuntu 22.04 기본 |
| Python (flight_code) | 3.6.9 | 실기 환경과 동일 (Ubuntu 18.04 native) |
| MAVSDK-Python | 0.12.0 | 실기와 동일 |

## Starling 2 airframe 이식 (Day 2)

iris를 베이스로 Starling 2 실기 스펙을 반영한 `iris_starling` airframe을 추가했다.

파라미터 출처: ModalAI 포크의 `boards/modalai/voxl2/target/voxl-px4-hitl-set-default-parameters.config`.

| 항목 | 값 |
|---|---|
| 질량 | 0.275 kg |
| 로터 위치 (P0, P2) | (0.15, ±0.25) |
| 로터 위치 (P1, P3) | (-0.15, ±0.19) |
| IMU 오프셋 | (0.027, 0.009, -0.019) m |
| IMU 샘플링 | 800Hz |

Starling 2 전용 airframe 정의는 ModalAI 리포에 없다. VOXL2는 표준 PX4 airframe 방식이 아니라 `target/voxl-px4-start` 실행 스크립트로 모듈을 로드하는 방식이라, 이 정의를 그대로 이식할 수 없었다. 대신 HITL config의 실측 파라미터를 활용하는 우회 방식으로 진행했다.

**미완**: 시각 mesh는 iris 원본 크기 그대로. 물리 파라미터는 실기와 일치하지만 시각 크기가 실기 Starling 2(30cm)보다 커서(약 47cm) QVIX 안 좁은 통로에서 재배대와 충돌한다. Day 5에서 `iris_starling.sdf`의 mesh scale을 조정할 예정.

## flight_code의 이중 모드 지원 (Day 3)

실기 코드에는 세 곳 추가 편집만 반영. 기존 로직은 한 줄도 안 바뀜.

- **`sim_pose_reader` 함수**: MAVSDK telemetry(`position_velocity_ned`, `attitude_euler`, `attitude_angular_velocity_body`)를 구독해서 `self.voxl_latest[pipe_name]`을 실기와 동일한 dict 형식으로 채움. 도달 판정, CSV 로깅 등 다운스트림 코드는 소스가 실기든 SITL이든 구분하지 못하고 그대로 동작.
- **`--sim` CLI 플래그**: 이 플래그가 있으면 `sim_pose_reader`, 없으면 원래 `voxl_pose_reader`.
- **CSV 경로 자동 폴백**: `/home/root`가 없으면 현재 디렉터리. SITL/실기 양쪽 다 자동 대응.

실기 실행 커맨드와 SITL 실행 커맨드의 차이는 `--sim` 플래그 하나뿐:

```bash
# 실기
python3 path_flight_phase1_v13.py --csv auto

# SITL
python3 path_flight_phase1_v13.py --sim --csv auto
```

## QVIX R Module 3D 맵 통합 (Day 4)

Fusion 360으로 태백 NextOn QVIX R Module의 전체 시설을 3D로 재현하고, 이를 Gazebo world로 통합했다.

### 시설 구조

- 전체 크기: 22.4 m × 5.8 m
- 재배대 16개: 상부 그룹 8개 (4열 × 2행) + 딸기방 + 하부 그룹 8개
- 딸기방: 중앙 2 m × 5.8 m (문 두 개)
- 재배대 하나: 4 m × 0.51 m × 5 m (6단 선반)
- 재배대 간 간격: 좌우 0.75 m, 상하 0.3 m

### Gazebo 이식

Fusion의 3D 데이터를 STL로 export한 뒤 Gazebo 모델로 등록:

- `gazebo/qvix_integration/qvix_r_module/`: 모델 정의 (`model.config`, `qvix_r_module.sdf`, `meshes/qvix_r_full.stl`)
- `gazebo/qvix_integration/qvix.world`: sun + ground_plane + qvix_r_module 포함
- PX4 CMake의 `set(worlds ...)` 목록에 `qvix` 추가하여 새 make 타겟 생성:

```bash
make px4_sitl gazebo-classic_iris_starling__qvix
```

### 이륙 위치 표준

미션(도면 2 시나리오: 상부 그룹의 재배대 1, 5 옆면 스캔)에 맞춰 확정된 QVIX pose:

```xml
<pose>0.45 -12.0 0.15 0 0 0</pose>
```

- 드론이 Gazebo (0, 0, 0)에 spawn되면 QVIX 좌표계 상 (X=0.45, Y=12.0, Z=0.1)에 위치
- 왼쪽 벽 안쪽 복도, 딸기방 위쪽 여유 공간에서 이륙
- 정면(파란 프로펠러) = Gazebo Y+ = 상부 그룹 방향
- Step 1 `right 1.35m` 후 재배대 1열-2열 사이 통로 진입

### STL 좌표 실측치

STL 파일을 `numpy-stl`로 직접 분석해 얻은 실제 배치 (mm):

**X축 (재배대 열)**:
- 왼쪽 벽 안쪽 복도: 0 ~ 755
- 재배대 1열: 755 ~ 1280
- 통로 1: 1280 ~ 2015
- 재배대 2열: 2015 ~ 2540
- (반복 패턴, 총 4열)

**Y축 (그룹 배치)**:
- 하부 그룹 앞줄: 500 ~ 4500
- 하부 그룹 뒷줄: 4800 ~ 8800
- 딸기방: 10000 ~ 12000
- 딸기방 위 여유: 12000 ~ 13500
- 상부 그룹 앞줄 (재배대 5, 6, 7, 8): 13500 ~ 17500
- 상부 그룹 뒷줄 (재배대 1, 2, 3, 4): 17900 ~ 21400

## 현재 진행 상황

`docs/milestones.md`를 참고할 것.

- [x] **Day 1 (7/28)**: 시뮬레이션 베이스라인 구축 완료
- [x] **Day 2 (8/3)**: Starling 2 airframe 이식 및 SITL 검증 완료
- [x] **Day 3 (8/3)**: flight_code의 SITL 대응 완료, 미션 20단계 완주 검증
- [x] **Day 4 (8/5, 8/12)**: Fusion QVIX 3D 맵 제작 및 Gazebo 통합, QVIX 안에서 미션 실행 확인 (드론 시각 크기 이슈로 부분 완주)
- [ ] **Day 5**: iris_starling mesh scale 조정 후 미션 완주 검증, sim-to-real gap 분석

## 일정 변경 이력

**당초 계획 (7/28 기준)**: 7/28~8/1 5일 내 sim-to-real gap 분석까지 완료  
**실제 진행**:
- 7/28 Day 1 완료
- QVIX 정밀 재현 트랙에서 시간 소진 후 우선순위 재조정
- 8/3 하루에 Day 2, Day 3 연속 진행
- 8/5 Fusion으로 QVIX 3D 맵 제작
- 8/12 QVIX를 Gazebo에 통합, 미션이 QVIX 시나리오에서 도는 것 확인

**현재 계획**: airframe 이식, flight_code 대응, QVIX 통합 완료. 남은 것은 mesh scale 조정과 sim-to-real gap 분석 (Day 5).

## 리포 구조

```
drone_sim/
├── README.md               # 이 파일
├── .gitignore
├── docker/                 # Container A/B Dockerfile 및 실행 스크립트
│   ├── Dockerfile          # Container A (SITL)
│   ├── Dockerfile.flight   # Container B (flight_code)
│   ├── build.sh
│   ├── build_flight.sh
│   ├── run.sh
│   ├── run_flight.sh
│   └── start_all.sh
├── airframe/               # Starling 2 airframe 정의 (Day 2)
│   ├── 4200_gazebo-classic_iris_starling
│   ├── iris_starling/      # Gazebo 모델 (SDF, model.config)
│   └── voxl-px4-hitl-set-default-parameters.config  # 실기 HITL config (원본)
├── gazebo/                 # QVIX 시설 3D 맵과 world 정의 (Day 4)
│   └── qvix_integration/
│       ├── qvix.world      # sun + ground_plane + qvix_r_module
│       ├── qvix_r_module/  # Gazebo 모델
│       │   ├── model.config
│       │   ├── qvix_r_module.sdf
│       │   └── meshes/qvix_r_full.stl
│       └── cmake_worlds_edit.txt  # CMake 편집 참고
├── flight_code/            # 실기와 공유하는 비행 로직
│   └── path_flight_phase1_v13.py    # --sim 대응 (Day 3)
├── logs/                   # 세션별 실행 로그
│   ├── day1/
│   ├── day3/               # SITL 미션 완주 로그 (empty world)
│   └── day4/               # QVIX 시나리오 실행 로그
└── docs/                   # 셋업 절차, 저널, 마일스톤
    ├── setup.md
    ├── day1_journal.md
    ├── day2_journal.md
    ├── day3_journal.md
    ├── day4_journal.md
    └── milestones.md
```

## 시작하기

`docs/setup.md`를 순서대로 따르면 서버에서 Docker 컨테이너 두 개를 재현 가능하다.

빠른 재개(이미 셋업된 상태에서):

```bash
# 호스트에서 컨테이너 시작
~/starling_sim/docker/start_all.sh

# Container A 진입 후 VNC와 SITL 실행
docker exec -it starling-sitl bash
~/start_vnc.sh
cd ~/PX4-Autopilot

# QVIX world에서 미션 진행 (Day 4 이후 기본)
DISPLAY=:1 XAUTHORITY=/tmp/.Xauthority-dronesim \
  make px4_sitl gazebo-classic_iris_starling__qvix

# empty world만으로 airframe만 검증하려면 (Day 3까지의 방식)
# make px4_sitl gazebo-classic_iris_starling

# 새 터미널에서 Container B 진입 후 flight_code 실행
docker exec -it starling-flight bash
MAVSDK_BIN=/usr/local/lib/python3.6/dist-packages/mavsdk/bin/mavsdk_server
$MAVSDK_BIN -p 50051 udp://:14540 > ~/mavsdk_server.log 2>&1 &
cd ~/workspace
python3 path_flight_phase1_v13.py --sim --csv auto --csv-sample-sec 1.0
```

VNC 브라우저: `http://<서버IP>:6080/vnc.html`

## 별도 트랙 (Week 1 밖으로 분리한 것들)

**HITL 확장**: 실기 Starling 2와 서버를 시리얼/이더넷으로 연결하여 Gazebo 센서를 실기 PX4에 주입. voxl-px4의 HITL 지원 여부부터 확인 필요. Day 5 결과 나온 후 정말 필요한지 판단.

**QVIX 시설의 정밀 재현**: 현재 QVIX 3D 맵은 도면 기반 골격(벽, 재배대 배치, 딸기방)까지만 재현되어 있다. 추가로 필요한 것들:
- AprilTag 위치 및 정확한 텍스처
- 재배 선반의 세부 구조 (레일, 배관)
- 조명 조건 (LED 배치, 광량)
- 딸기 잎과 열매 텍스처 (VIO feature density에 직접 영향)
- 벽/바닥 재질 텍스처

이건 NextOn 방문 계측 후 별도 세션에서 진행.

**수확 드론 및 모바일 충전 스테이션**: Week 1 이후 확장 응용.

## 관련 자료

- ModalAI VOXL2 PX4 HITL 문서: https://docs.modalai.com/voxl2-PX4-hitl/
- PX4 v1.14 GPS-denied simulation 논의: https://discuss.px4.io/t/px4-gps-denied-simulation-gazebo/33432
- Gazebo Classic 11 SDF format: http://sdformat.org/spec?ver=1.6
