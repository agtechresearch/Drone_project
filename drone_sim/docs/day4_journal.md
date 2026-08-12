# Day 4 Journal — 2026-08-12

Fusion에서 만든 QVIX R Module 맵을 Gazebo SITL 환경에 통합.

## 목표

- Fusion에서 완성한 QVIX 3D 맵을 STL로 export
- Gazebo 모델로 이식 (model.config, SDF, world)
- iris_starling airframe이 QVIX 안에서 정상 이륙 및 미션 실행
- 실기 flight_code가 QVIX 시나리오에서 진행

## 결과

**QVIX 통합 성공**, 미션 부분 성공. 실기와 동일한 코드가 실기 시설을 재현한 SITL 환경에서 실행되는 파이프라인 완성.

**미완**: 드론 시각 크기가 실기(275g, 30cm)보다 큰 iris 원본 크기로 남아있어 재배대와의 충돌 발생. Day 5에서 iris_starling mesh scale 조정 필요.

## 진행 순서

1. Fusion에서 QVIX 최종 파일(vervis_r_full)을 STL로 export
2. STL 파일을 서버로 scp 후 Container A로 docker cp
3. Gazebo 모델 폴더 구조 생성:
   - `~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/qvix_r_module/`
   - `meshes/qvix_r_full.stl`
   - `model.config`
   - `qvix_r_module.sdf` (STL 참조, scale 0.001 mm→m)
4. World 파일 생성 (`worlds/qvix.world`)
5. CMake 등록 (`sitl_targets_gazebo-classic.cmake`의 `set(worlds ...)` 목록에 qvix 추가)
6. 새 make 타겟 `gazebo-classic_iris_starling__qvix` 성공적으로 빌드
7. 이륙 위치 및 방향 조정 반복 (드론 spawn 위치와 QVIX 상대 좌표 파악)
8. 최종 위치 확정: QVIX pose = `(0.45, -12.0, 0.15, 0, 0, 0)`
9. Container B의 flight_code(--sim 모드) 실행하여 미션 시작 확인

## QVIX 통합 세부사항

### Gazebo 모델 구조

```
Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/qvix_r_module/
├── model.config
├── qvix_r_module.sdf
└── meshes/
    └── qvix_r_full.stl   (Fusion 원본, 462KB)
```

`qvix_r_module.sdf`:
- `<static>true</static>` (시설은 물리 계산 안 함)
- `<scale>0.001 0.001 0.001</scale>` (mm → m)
- `<collision>` + `<visual>` 둘 다 STL 참조

### World 파일

`worlds/qvix.world`:
- Sun, ground_plane, qvix_r_module 포함
- 표준 물리 설정 (ODE)

### QVIX 좌표계 매핑

STL 실측 분석 결과:
- X: 0 ~ 5800 mm (폭)
- Y: 0 ~ 22400 mm (길이)
- Z: -100 ~ 5000 mm (바닥판 아래에서 재배대 상단까지)

Fusion X → Gazebo X, Fusion Y → Gazebo Y 그대로 매핑.

Yaw=0에서 드론 정면 = PX4 NED N (+X) = Gazebo Y+ 방향.

### 재배대 및 시설 실측 위치 (STL 분석 기반)

**X축 (재배대 열)**:
- 왼쪽 벽 안쪽 복도: X = 0.0 ~ 0.755
- 재배대 1열: X = 0.755 ~ 1.28
- 통로 1: X = 1.28 ~ 2.015
- 재배대 2열: X = 2.015 ~ 2.54
- (반복 패턴)

**Y축 (그룹 배치)**:
- 하부 그룹 앞줄: Y = 0.5 ~ 4.5
- 하부 그룹 뒷줄: Y = 4.8 ~ 8.8
- 딸기방: Y = 10.0 ~ 12.0
- 딸기방 위 여유: Y = 12.0 ~ 13.5
- 상부 그룹 앞줄 (재배대 5,6,7,8): Y = 13.5 ~ 17.5
- 상부 그룹 뒷줄 (재배대 1,2,3,4): Y = 17.9 ~ 21.4

### 미션 시나리오 (도면 2, 상부 그룹)

**촬영 대상**: 재배대 1과 5 (왼쪽 첫 번째 열의 두 재배대)

**이륙 위치**: 딸기방 위 여유 공간, 왼쪽 벽 쪽 복도
- QVIX 좌표: (X=0.45, Y=12.0)

**QVIX pose (Gazebo world)**: `(0.45, -12.0, 0.15, 0, 0, 0)`
- 드론이 Gazebo (0,0,0)에 spawn되면 QVIX 안 (0.45, 12.0) 위치

## 삽질 로그

### 1. Attitude failure (roll) 반복 발생

**증상**: QVIX 로드하고 이륙 시도하면 `Preflight Fail: Attitude failure (roll)`, `vertical velocity unstable`, `velocity estimate error` 반복.

**원인**: QVIX 바닥판이 지면(ground_plane, Z=0)과 겹치거나 드론이 QVIX 벽 안에 spawn되어 물리 충돌.

**해결**:
- QVIX pose Z = 0.15로 올려서 바닥판이 지면과 안 겹치게
- QVIX pose X, Y로 드론과 벽/재배대 사이 안전 거리 확보

### 2. QVIX 좌표계와 Gazebo 매핑 혼란

**증상**: QVIX pose를 조정해도 드론이 예상한 위치에 없음.

**원인**: Gazebo world 좌표계와 PX4 NED 좌표계 매핑 초기 오해. STL의 정확한 좌표계 파악 필요.

**해결**: STL 파일을 Python numpy-stl로 직접 분석하여 재배대 위치, 딸기방 위치 등을 실측치로 확인. Fusion X → Gazebo X, Fusion Y → Gazebo Y가 그대로 매핑됨을 확정.

### 3. 재배대 개수 계산 실수 (전날 이후)

**증상**: 도면 2가 상부 그룹인지 하부 그룹인지 혼동.

**해결**: 도면 확인 결과 도면 2 = 딸기방 위쪽(상부 그룹). Y=13.5~21.4 범위.

### 4. QVIX Yaw 회전 시도 후 사라짐

**증상**: QVIX pose에 Yaw=-1.5708 (90도 회전) 적용하니 화면에서 사라짐.

**원인**: 회전축이 QVIX 원점(왼쪽 아래 코너)이라 회전 후 QVIX 전체가 다른 방향으로 뻗어 화면 밖으로.

**해결**: 회전 대신 위치(X, Y)만 조정해서 드론이 원하는 위치에 오도록. Yaw는 0으로 유지.

### 5. iris_starling mesh scale 미조정

**증상**: 드론 시각 크기가 실기 Starling 2보다 큼. 재배대 통로 진입 시 충돌.

**원인**: Day 2에 airframe 이식 시 물리 파라미터(질량, 관성, 로터 위치)만 수정하고 mesh 시각 크기는 iris 원본 그대로.

**후속 조치 (Day 5)**: `iris_starling.sdf`의 `<mesh><scale>1 1 1</scale></mesh>`를 0.7 정도로 줄이는 것 고려.

## 검증 결과

- QVIX 3D 모델이 Gazebo GUI에 정상 표시됨 (재배대 16개, 벽, 딸기방, 문 자리)
- iris_starling이 QVIX 안 원하는 위치에서 정상 spawn (attitude 정상)
- `commander takeoff` 성공, 이륙 후 원하는 방향(상부 그룹 방향) 향함
- Container B의 flight_code(--sim)가 QVIX 시나리오에서 미션 실행 시작
- Step 4 이후 재배대와의 충돌로 미션 부분 종료 (드론 시각 크기 이슈)

## 이미지 스냅샷

- `starling-sitl:qvix-integrated` — QVIX가 통합된 상태의 SITL 컨테이너

## Day 5 준비 사항

- iris_starling mesh scale 조정 (0.7 정도로 축소하여 실기 크기에 맞춤)
- 미션 완주 검증
- sim-to-real gap 지표 정의 및 측정 시작

## 참고 파일

- `gazebo/qvix.world` — QVIX 통합된 world 파일
- `gazebo/qvix_r_module/model.config` — 모델 정의
- `gazebo/qvix_r_module/qvix_r_module.sdf` — SDF (STL 참조)
- `gazebo/qvix_r_module/meshes/qvix_r_full.stl` — Fusion 원본 3D
