# Day 2 Journal — 2026-08-03

Starling 2 airframe 이식 및 SITL 검증.

## 목표

- ModalAI 실기 스펙(질량, 관성, 로터 배치, IMU 오프셋)을 반영한 airframe을 SITL에 이식
- SITL이 이 airframe으로 정상 동작 확인

## 결과

**목표 달성.** Gazebo GUI에서 iris_starling airframe으로 이륙/착륙 정상 동작.

- 파라미터: `CA_ROTOR0_PY=0.25`, `EKF2_IMU_POS_X=0.027`, `IMU_GYRO_RATEMAX=800` 확인
- 물리: 질량 275g (실기와 동일), 관성 텐서 비례 축소
- 로터 위치: HITL config의 실측치 반영

## 진행 순서

1. ModalAI 포크에서 Starling 2 관련 파일 조사 (Mac에서)
2. Starling 전용 airframe 정의는 없음 확인 → HITL config에서 실측 파라미터 추출
3. voxl-px4-hitl-set-default-parameters.config에서 로터 배치, IMU 오프셋 확보
4. 서버로 파일 전송 (Mac scp)
5. iris 모델을 iris_starling으로 복사 후 Starling 스펙으로 수정
6. airframe 스크립트 작성 (`4200_gazebo-classic_iris_starling`)
7. CMake 등록 (CMakeLists.txt + sitl_targets_gazebo-classic.cmake의 models 목록)
8. 빌드 및 SITL 실행 검증

## 이식한 실기 파라미터

`boards/modalai/voxl2/target/voxl-px4-hitl-set-default-parameters.config`에서 발췌:

### Control Allocation (로터 배치)

```
CA_ROTOR0_PX=0.15,  CA_ROTOR0_PY=0.25,  CA_ROTOR0_KM=+0.05
CA_ROTOR1_PX=-0.15, CA_ROTOR1_PY=-0.19, CA_ROTOR1_KM=+0.05
CA_ROTOR2_PX=0.15,  CA_ROTOR2_PY=-0.25, CA_ROTOR2_KM=-0.05
CA_ROTOR3_PX=-0.15, CA_ROTOR3_PY=0.19,  CA_ROTOR3_KM=-0.05
```

- 대칭이 완벽하지 않음 (PY 0.25 vs 0.19) → 실제 Starling 2 기하 반영
- X축 반경 0.15m, Y축 반경 0.19~0.25m
- 4개 로터로 대각선 wheelbase 대략 0.3m x 0.44~0.5m

### EKF/IMU 오프셋

```
EKF2_IMU_POS_X=0.027
EKF2_IMU_POS_Y=0.009
EKF2_IMU_POS_Z=-0.019
```

IMU가 무게중심에서 (X+27mm, Y+9mm, Z-19mm) 위치. 실기 실측치.

### IMU 샘플링

```
IMU_GYRO_RATEMAX=800    # 실기와 동일 800Hz
```

## SDF 물리 파라미터 수정 (iris → iris_starling)

| 파라미터 | iris 원본 | iris_starling |
|---|---|---|
| 질량 | 1.5 kg | 0.275 kg |
| 관성 Ixx, Iyy | 0.029125 | 0.005333 |
| 관성 Izz | 0.055225 | 0.010115 |
| 크기 | 0.47 x 0.47 x 0.11 m | 0.30 x 0.35 x 0.08 m |
| 로터 위치 | (0.13, ±0.22) 등 | (0.15, ±0.25/±0.19) HITL config 값 |

관성 텐서는 형상 상세 계산 없이 질량 비례로 근사(0.275/1.5 ≈ 0.183). Day 5 sim-to-real gap 측정 후 필요하면 정밀 튜닝.

## 삽질 로그

### 1. ModalAI 포크에 Starling 전용 airframe 정의 없음

**증상**: `grep -rln "Starling"`이 완전히 빈 결과.

**원인**: VOXL2는 표준 PX4 airframe 방식(ROMFS 파일)이 아니라 `target/voxl-px4-start` 실행 스크립트에서 모듈을 직접 로드. Starling airframe SDF는 별도 리포(`voxl-px4-params` 등)에 있을 것으로 추정되나 접근 불가.

**해결**: HITL config에 있는 파라미터(로터 배치, IMU 오프셋 등)를 실측치로 활용. iris 모델을 베이스로 물리 파라미터만 수정.

### 2. PX4-Autopilot이 v1.18-beta로 오염됨

**증상**: `make px4_sitl` 실행 로그에 `PX4_GIT_TAG: v1.18.0-beta1-180-g0e9d76a655` 출력.

**원인**: 어제 clone 시점 이후 어떤 이유로 소스가 최신 main으로 업데이트됨. 파일 mtime이 오늘 날짜로 찍혀 있고, `.claude/`, `CLAUDE.md` 같은 최신 파일 존재.

**해결**: 
- `git reset --hard HEAD` + `git clean -fdx`로 정리
- 서브모듈까지 재귀적으로 정리 (`git submodule foreach --recursive`)
- `git checkout v1.14.0` + `git submodule update --init --recursive`
- **재발 방지**: `git remote remove origin`으로 fetch/pull 원천 차단

### 3. airframe 등록만으로는 unknown target

**증상**: airframe 파일 만들고 `CMakeLists.txt`에 등록해도 `ninja: error: unknown target 'gazebo-classic_iris_starling'`.

**원인**: `src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake`에 하드코딩된 `set(models ...)` 목록에 iris_starling이 없어서 빌드 타깃이 생성되지 않음.

**해결**: 
```cmake
set(models
    ...
    iris
    iris_starling       # 추가
    ...
)
```

### 4. jinja 템플릿 처리 방식 이해

PX4 v1.14.0 Gazebo Classic 방식:
- 모델 폴더에는 `.sdf.jinja` 템플릿만 존재
- 빌드 시 자동으로 `.sdf`로 렌더링 (build 폴더에)
- 시뮬레이터가 그 렌더링된 SDF 사용

즉 jinja를 그대로 두고 안 내용만 수정하는 게 맞음. iris_starling.sdf.jinja를 sed로 편집.

### 5. CMakeLists.txt 들여쓰기

`sed`로 airframe을 등록하니 들여쓰기가 다른 줄들(탭)과 안 맞음(공백). awk로 다시 넣어 통일.

### 6. VNC hosts 초기화

컨테이너 재시작 시 `/etc/hosts`가 초기화되어 `vncserver`가 `hostname: Temporary failure in name resolution` 에러. `start_vnc.sh` 스크립트에 hosts 재설정 추가.

## 검증 결과

Container A에서 `make px4_sitl gazebo-classic_iris_starling` 실행 후:

```
pxh> param show CA_ROTOR0_PY
CA_ROTOR0_PY: 0.2500 (default: 0.0000)

pxh> param show EKF2_IMU_POS_X
EKF2_IMU_POS_X: 0.0270 (default: 0.0000)

pxh> param show IMU_GYRO_RATEMAX
IMU_GYRO_RATEMAX: 800 (default: 400)

pxh> commander takeoff
INFO  [commander] Takeoff detected

pxh> commander land
INFO  [commander] Landing detected
```

Gazebo GUI에서 드론 이륙/착륙 시각 확인.

## 이미지 스냅샷

- `starling-sitl:day2-airframe` — Starling 2 airframe 이식 완료 상태

## Day 3 준비 사항

- flight_code의 `voxl_pose_reader` 함수를 MAVSDK 기반 버전으로 대체
- CLI 플래그로 SITL/실기 모드 전환
- SITL에서 미션 20단계 완주 확인

## 참고 파일

- `docker/` — Container A/B 스크립트 (Day 1과 동일)
- `flight_code/path_flight_phase1_v13.py` — 실기와 공유하는 flight_code (Day 3에 수정 예정)
- `airframe/` — Starling 2 airframe 정의 (Day 2 신규)
  - `4200_gazebo-classic_iris_starling` — airframe 초기화 스크립트
  - `iris_starling/` — Gazebo 모델 (SDF, model.config)
  - `voxl-px4-hitl-set-default-parameters.config` — 실기 HITL config (참고용 원본)
